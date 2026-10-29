#include "DepthCameraGpu.hpp"

#include <Gfx/Graph/RenderList.hpp>
#include <Gfx/Graph/Utils.hpp>

#include <DepthCamera/DepthCameraStream.hpp>

#include <ossia/dataflow/geometry_port.hpp>
#include <ossia/detail/algorithms.hpp>

#include <QDebug>

#include <cmath>
#include <cstring>

namespace Gfx::DepthCamera
{
namespace
{
std::atomic<uint64_t> g_uploaded{0};

// --- shaders ----------------------------------------------------------------
//
// Everything is drawn as one triangle covering the target, with no vertex
// buffer, and every texel is addressed through gl_FragCoord: there is no
// interpolation anywhere, so a texel of an output is computed from exactly the
// raw sample of the same pixel.
//
// Image pixel (u, v), v from the top, lives in row H-1-v of a render target on
// every backend but Vulkan, and in row v there. That is how score's render
// targets are laid out: its ISF prelude reads them through the same
// correction, applied on SPIR-V only (isf_glsl_prelude.hpp), and Metal reads
// mirrored without it. These textures then sample like any other node's
// output. Uploaded textures (the raw frame, the ray table) are in image order
// everywhere.

constexpr auto params_block = R"_(
layout(std140, binding = 0) uniform Params {
  vec4 scale;      // x metres per sample, y near, z far, w temporal weight
  vec4 boxMin;     // xyz, w enabled
  vec4 boxMax;     // xyz, w infrared gain
  ivec4 size;      // w, h, flip, mode (0 metres, 1 preview)
  vec4 cull;       // x edge ratio, y points
  mat4 depthToColor;
  vec4 colorIntr;  // fx fy cx cy
  vec4 colorSize;  // w h model has_color
  vec4 colorDist0; // k1 k2 k3 k4
  vec4 colorDist1; // k5 k6 p1 p2
  vec4 irScale;    // x: sample to 0..1
} p;

ivec2 imagePixel(ivec2 frag) {
  return ivec2(frag.x, p.size.z != 0 ? p.size.y - 1 - frag.y : frag.y);
}
ivec2 targetTexel(ivec2 px) {
  return ivec2(px.x, p.size.z != 0 ? p.size.y - 1 - px.y : px.y);
}
)_";

constexpr auto fullscreen_vs = R"_(#version 450
void main() {
  vec2 c = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
  gl_Position = vec4(c * 2.0 - 1.0, 0.0, 1.0);
}
)_";

QString positionsFs()
{
  return QString(R"_(#version 450
%1
layout(binding = 1) uniform sampler2D rawTex;
layout(binding = 2) uniform sampler2D rays;
layout(binding = 3) uniform sampler2D history;
layout(location = 0) out vec4 outPos;

void main() {
  ivec2 frag = ivec2(gl_FragCoord.xy);
  ivec2 px = imagePixel(frag);
  float m = texelFetch(rawTex, px, 0).r * p.scale.x;
  vec4 r = texelFetch(rays, px, 0);

  bool valid = m > 0.0 && !isnan(m) && !isinf(m) && r.w > 0.0;
  if(p.scale.y > 0.0 && m < p.scale.y) valid = false;
  if(p.scale.z > 0.0 && m > p.scale.z) valid = false;

  if(valid && p.scale.w > 0.0) {
    // Exponential average along the pixel's ray. A jump of more than 5 %%
    // is something moving, not noise, and restarts the average.
    vec4 h = texelFetch(history, frag, 0);
    if(h.w > 0.0 && abs(h.z - m) <= 0.05 * m)
      m = mix(m, h.z, p.scale.w);
  }

  vec3 P = vec3(r.xy * m, m);
  if(valid && p.boxMin.w > 0.0)
    valid = all(greaterThanEqual(P, p.boxMin.xyz)) && all(lessThanEqual(P, p.boxMax.xyz));

  outPos = valid ? vec4(P, 1.0) : vec4(0.0);
}
)_").arg(params_block);
}

QString depthFs()
{
  return QString(R"_(#version 450
%1
layout(binding = 1) uniform sampler2D positions;
layout(location = 0) out vec4 outColor;

void main() {
  vec4 P = texelFetch(positions, ivec2(gl_FragCoord.xy), 0);
  float z = P.w > 0.0 ? P.z : 0.0;
  if(p.size.w == 0)
    outColor = vec4(z, z, z, 1.0);
  else
    outColor = vec4(vec3(z / 4.5), 1.0);
}
)_").arg(params_block);
}

QString normalsFs()
{
  return QString(R"_(#version 450
%1
layout(binding = 1) uniform sampler2D positions;
layout(location = 0) out vec4 outNormal;

vec4 at(ivec2 px) {
  if(px.x < 0 || px.y < 0 || px.x >= p.size.x || px.y >= p.size.y) return vec4(0.0);
  return texelFetch(positions, targetTexel(px), 0);
}

void main() {
  ivec2 px = imagePixel(ivec2(gl_FragCoord.xy));
  vec4 c = at(px);
  if(c.w <= 0.0) { outNormal = vec4(0.0); return; }
  vec4 l = at(px - ivec2(1, 0)), r = at(px + ivec2(1, 0));
  vec4 u = at(px - ivec2(0, 1)), d = at(px + ivec2(0, 1));
  vec3 dx = (r.w > 0.0 ? r.xyz : c.xyz) - (l.w > 0.0 ? l.xyz : c.xyz);
  vec3 dy = (d.w > 0.0 ? d.xyz : c.xyz) - (u.w > 0.0 ? u.xyz : c.xyz);
  vec3 n = cross(dy, dx);
  float len = length(n);
  // dx runs along +X and dy along +Y (down), so cross(dy, dx) faces the camera.
  outNormal = len > 0.0 ? vec4(n / len, 1.0) : vec4(0.0);
}
)_").arg(params_block);
}

QString infraredFs()
{
  return QString(R"_(#version 450
%1
layout(binding = 1) uniform sampler2D rawTex;
layout(location = 0) out vec4 outColor;

void main() {
  float v = texelFetch(rawTex, imagePixel(ivec2(gl_FragCoord.xy)), 0).r
            * p.irScale.x * p.boxMax.w;
  outColor = vec4(v, v, v, 1.0);
}
)_").arg(params_block);
}

QString meshCs()
{
  return QString(R"_(#version 450
layout(local_size_x = 16, local_size_y = 16) in;
%1
layout(binding = 1) uniform sampler2D positions;
layout(std430, binding = 2) buffer Vertices { vec4 verts[]; };
layout(std430, binding = 3) buffer Indices { uint idx[]; };

vec4 at(ivec2 px) {
  px = clamp(px, ivec2(0), p.size.xy - 1);
  return texelFetch(positions, targetTexel(px), 0);
}

bool keep(vec4 a, vec4 b, vec4 c) {
  if(a.w <= 0.0 || b.w <= 0.0 || c.w <= 0.0) return false;
  if(p.cull.x <= 0.0) return true;
  float e = max(max(distance(a.xyz, b.xyz), distance(b.xyz, c.xyz)), distance(c.xyz, a.xyz));
  return e <= p.cull.x * (a.z + b.z + c.z) / 3.0;
}

vec2 colorUV(vec3 P, ivec2 px) {
  if(p.colorSize.w <= 0.0)
    return (vec2(px) + 0.5) / vec2(p.size.xy);
  vec4 q = p.depthToColor * vec4(P, 1.0);
  if(q.z <= 0.0) return vec2(-1.0);
  float x = q.x / q.z, y = q.y / q.z;
  if(p.colorSize.z == 1.0) {
    // Brown-Conrady, as the colour camera reports it.
    float r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
    float radial = (1.0 + p.colorDist0.x * r2 + p.colorDist0.y * r4 + p.colorDist0.z * r6)
                 / (1.0 + p.colorDist0.w * r2 + p.colorDist1.x * r4 + p.colorDist1.y * r6);
    float xd = x * radial + 2.0 * p.colorDist1.z * x * y + p.colorDist1.w * (r2 + 2.0 * x * x);
    float yd = y * radial + p.colorDist1.z * (r2 + 2.0 * y * y) + 2.0 * p.colorDist1.w * x * y;
    x = xd; y = yd;
  }
  return vec2(
      (p.colorIntr.x * x + p.colorIntr.z + 0.5) / p.colorSize.x,
      (p.colorIntr.y * y + p.colorIntr.w + 0.5) / p.colorSize.y);
}

void main() {
  ivec2 px = ivec2(gl_GlobalInvocationID.xy);
  int W = p.size.x, H = p.size.y;
  if(px.x >= W || px.y >= H) return;

  vec4 c = at(px);
  vec3 n = vec3(0.0);
  if(c.w > 0.0) {
    vec4 l = at(px - ivec2(1, 0)), r = at(px + ivec2(1, 0));
    vec4 u = at(px - ivec2(0, 1)), d = at(px + ivec2(0, 1));
    vec3 dx = (r.w > 0.0 ? r.xyz : c.xyz) - (l.w > 0.0 ? l.xyz : c.xyz);
    vec3 dy = (d.w > 0.0 ? d.xyz : c.xyz) - (u.w > 0.0 ? u.xyz : c.xyz);
    vec3 cr = cross(dy, dx);
    float len = length(cr);
    if(len > 0.0) n = cr / len;
  }

  uint i = uint(px.y * W + px.x);
  verts[3u * i] = c;
  verts[3u * i + 1u] = vec4(n, 0.0);
  verts[3u * i + 2u] = vec4(colorUV(c.xyz, px), 0.0, 0.0);

  if(p.cull.y == 0.0 && px.x < W - 1 && px.y < H - 1) {
    uint a = i, b = i + 1u, cc = i + uint(W), dd = i + uint(W) + 1u;
    vec4 Pb = at(px + ivec2(1, 0)), Pc = at(px + ivec2(0, 1)), Pd = at(px + ivec2(1, 1));
    uint q = uint(px.y * (W - 1) + px.x) * 6u;
    // A dropped triangle collapses onto one vertex; the rasterizer skips it.
    bool t1 = keep(c, Pc, Pb), t2 = keep(Pb, Pc, Pd);
    idx[q] = a;       idx[q + 1u] = t1 ? cc : a; idx[q + 2u] = t1 ? b : a;
    idx[q + 3u] = b;  idx[q + 4u] = t2 ? cc : b; idx[q + 5u] = t2 ? dd : b;
  }
}
)_").arg(params_block);
}

constexpr auto blit_fs = R"_(#version 450
layout(std140, binding = 0) uniform Blit { vec4 target; } b; // w, h, replicate
layout(binding = 1) uniform sampler2D src;
layout(location = 0) out vec4 outColor;
void main() {
  vec4 v = texture(src, gl_FragCoord.xy / b.target.xy);
  outColor = b.target.z > 0.0 ? vec4(v.rrr, 1.0) : v;
}
)_";

struct alignas(16) ParamsUBO
{
  float scale[4];
  float boxMin[4];
  float boxMax[4];
  int32_t size[4];
  float cull[4];
  float depthToColor[16];
  float colorIntr[4];
  float colorSize[4];
  float colorDist0[4];
  float colorDist1[4];
  float irScale[4];
};
static_assert(sizeof(ParamsUBO) == 10 * 16 + 64);

/// One vertex of the mesh: position (w = valid), normal, texcoord0.
constexpr int vertex_stride = 48;

struct OwnTarget
{
  QRhiTexture* texture{};
  QRhiTextureRenderTarget* rt{};
  QRhiRenderPassDescriptor* rp{};

  bool create(QRhi& rhi, QRhiTexture::Format fmt, QSize sz, const char* name)
  {
    texture = rhi.newTexture(fmt, sz, 1, QRhiTexture::RenderTarget);
    texture->setName(name);
    if(!texture->create())
      return false;
    rt = rhi.newTextureRenderTarget({QRhiColorAttachment{texture}});
    rp = rt->newCompatibleRenderPassDescriptor();
    rt->setRenderPassDescriptor(rp);
    return rt->create();
  }
  void release()
  {
    for(QRhiResource* r : std::initializer_list<QRhiResource*>{rt, rp, texture})
      if(r)
        r->deleteLater();
    rt = nullptr;
    rp = nullptr;
    texture = nullptr;
  }
};

struct Pass
{
  QRhiGraphicsPipeline* pipeline{};
  QRhiShaderResourceBindings* srb{};
  void release()
  {
    if(pipeline)
      pipeline->deleteLater();
    if(srb)
      srb->deleteLater();
    pipeline = nullptr;
    srb = nullptr;
  }
};

class DepthStreamRenderer final : public score::gfx::NodeRenderer
{
public:
  explicit DepthStreamRenderer(const DepthStreamNode& n)
      : score::gfx::NodeRenderer{n}
  {
  }
  ~DepthStreamRenderer() override = default;

  const DepthStreamNode& model() const noexcept
  {
    return static_cast<const DepthStreamNode&>(node);
  }

  // --- lifecycle -----------------------------------------------------------

  void init(score::gfx::RenderList& renderer, QRhiResourceUpdateBatch& res) override
  {
    initState(renderer, res);
    for(auto* out : node.output)
      for(auto* e : out->edges)
        addOutputPass(renderer, *e, res);
  }

  void initState(score::gfx::RenderList& renderer, QRhiResourceUpdateBatch&) override
  {
    auto& rhi = *renderer.state.rhi;
    m_flip = rhi.backend() != QRhi::Vulkan;
    m_compute = rhi.isFeatureSupported(QRhi::Compute);

    m_ubo = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(ParamsUBO));
    m_ubo->setName("DepthStream::params");
    m_ubo->create();

    m_sampler = rhi.newSampler(
        QRhiSampler::Nearest, QRhiSampler::Nearest, QRhiSampler::None,
        QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge);
    m_sampler->setName("DepthStream::sampler");
    m_sampler->create();

    try
    {
      if(model().kind == DepthStreamNode::Kind::Depth)
      {
        m_shPositions = score::gfx::makeShaders(renderer.state, fullscreen_vs, positionsFs());
        m_shDepth = score::gfx::makeShaders(renderer.state, fullscreen_vs, depthFs());
        m_shNormals = score::gfx::makeShaders(renderer.state, fullscreen_vs, normalsFs());
        if(m_compute)
          m_shMesh = score::gfx::makeCompute(renderer.state, meshCs());
      }
      else
      {
        m_shInfrared = score::gfx::makeShaders(renderer.state, fullscreen_vs, infraredFs());
      }
      m_shBlit = score::gfx::makeShaders(renderer.state, fullscreen_vs, blit_fs);
    }
    catch(const std::exception& e)
    {
      qWarning() << "[depthcam] shader compilation failed:" << e.what();
      m_broken = true;
    }

    m_initialized = true;
  }

  void releaseState(score::gfx::RenderList& r) override
  {
    // The geometry buffers may still be bound downstream for this frame, so
    // they go through the RenderList's deferred retirement.
    if(m_vertices)
      r.releaseBuffer(m_vertices);
    if(m_indices)
      r.releaseBuffer(m_indices);
    m_vertices = m_indices = nullptr;
    m_geometry = {};
    releaseFrame();
    releaseTargets();
    for(auto& e : m_edges)
      e.release();
    m_edges.clear();
    if(m_ubo)
      m_ubo->deleteLater();
    if(m_sampler)
      m_sampler->deleteLater();
    m_ubo = nullptr;
    m_sampler = nullptr;
    m_initialized = false;
  }

  void release(score::gfx::RenderList& r) override { releaseState(r); }

  // --- per-edge output passes ---------------------------------------------

  struct EdgePass
  {
    score::gfx::Edge* edge{};
    int outlet{};
    Pass pass;
    QRhiBuffer* ubo{};
    QRhiTexture* boundTexture{};
    QVector<quint32> boundFormat;
    void release()
    {
      pass.release();
      if(ubo)
        ubo->deleteLater();
      ubo = nullptr;
    }
  };

  int outletOf(const score::gfx::Port* p) const noexcept
  {
    for(std::size_t i = 0; i < node.output.size(); i++)
      if(node.output[i] == p)
        return int(i);
    return -1;
  }

  void addOutputPass(
      score::gfx::RenderList& renderer, score::gfx::Edge& edge,
      QRhiResourceUpdateBatch&) override
  {
    if(hasOutputPassForEdge(edge))
      return;
    EdgePass e;
    e.edge = &edge;
    e.outlet = outletOf(edge.source);
    m_edges.push_back(e);
    // The pipeline itself is built lazily: the texture it samples only exists
    // once a frame has arrived.
  }

  void removeOutputPass(score::gfx::RenderList&, score::gfx::Edge& edge) override
  {
    auto it = ossia::find_if(m_edges, [&](const EdgePass& e) { return e.edge == &edge; });
    if(it != m_edges.end())
    {
      it->release();
      m_edges.erase(it);
    }
  }

  bool hasOutputPassForEdge(score::gfx::Edge& edge) const override
  {
    return ossia::find_if(m_edges, [&](const EdgePass& e) { return e.edge == &edge; })
           != m_edges.end();
  }

  QRhiTexture* outletTexture(int outlet) const noexcept
  {
    if(model().kind == DepthStreamNode::Kind::Infrared)
      return m_irOut.texture;
    switch(outlet)
    {
      case DepthStreamNode::DepthOut:
        return m_depthOut.texture;
      case DepthStreamNode::PositionsOut:
        return m_posOut.texture;
      case DepthStreamNode::NormalsOut:
        return m_normalsOut.texture;
      default:
        return nullptr;
    }
  }

  QRhiTexture* textureForOutput(const score::gfx::Port& output) override
  {
    return outletTexture(outletOf(&output));
  }

  score::gfx::TextureRenderTarget renderTargetForInput(const score::gfx::Port&) override
  {
    return {};
  }

  // --- frame upload --------------------------------------------------------

  void releaseFrame()
  {
    if(m_frame)
    {
      m_frame->use_count--;
      m_frame.reset();
    }
  }

  bool wanted(int outlet) const noexcept
  {
    if(outlet < 0 || outlet >= int(node.output.size()))
      return false;
    return !node.output[outlet]->edges.empty();
  }

  void update(
      score::gfx::RenderList& renderer, QRhiResourceUpdateBatch& res,
      score::gfx::Edge* edge) override
  {
    if(m_broken)
      return;
    if(m_updatedFrame != renderer.frame)
    {
      m_updatedFrame = renderer.frame;
      updateShared(renderer, res);
    }
    if(edge && m_haveFrame)
      prepareEdge(renderer, res, *edge);
  }

  /// Builds or refreshes what drawing into one consumer's target needs. Here
  /// and not in runRenderPass: resource updates are not allowed inside a pass.
  void prepareEdge(
      score::gfx::RenderList& renderer, QRhiResourceUpdateBatch& res,
      score::gfx::Edge& edge)
  {
    auto it = ossia::find_if(m_edges, [&](const EdgePass& e) { return e.edge == &edge; });
    if(it == m_edges.end())
    {
      addOutputPass(renderer, edge, res);
      it = ossia::find_if(m_edges, [&](const EdgePass& e) { return e.edge == &edge; });
      if(it == m_edges.end())
        return;
    }
    auto& e = *it;
    QRhiTexture* src = outletTexture(e.outlet);
    // A STATIC input samples textureForOutput() and has no target here.
    auto rt = renderer.renderTargetForOutput(edge);
    if(!src || !rt.renderTarget || !rt.renderPass)
      return;

    auto& rhi = *renderer.state.rhi;
    const auto format = rt.renderPass->serializedFormat();
    if(!e.pass.pipeline || e.boundTexture != src || e.boundFormat != format)
    {
      e.release();
      e.ubo = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, 16);
      e.ubo->setName("DepthStream::blit");
      e.ubo->create();
      using B = QRhiShaderResourceBinding;
      e.pass = makePass(
          renderer, m_shBlit, rt.renderPass,
          {B::uniformBuffer(0, B::FragmentStage, e.ubo),
           B::sampledTexture(1, B::FragmentStage, src, m_sampler)});
      e.boundTexture = src;
      e.boundFormat = format;
      if(!e.pass.pipeline)
        return;
    }

    const QSize sz = rt.renderTarget->pixelSize();
    // A single-channel texture shows as grey rather than red in an RGBA target;
    // an R32F one keeps its value either way.
    const bool replicate = src->format() == QRhiTexture::R32F
                           || src->format() == QRhiTexture::R16F;
    const float b[4]{float(sz.width()), float(sz.height()), replicate ? 1.f : 0.f, 0.f};
    res.updateDynamicBuffer(e.ubo, 0, sizeof(b), b);
  }

  void updateShared(score::gfx::RenderList& renderer, QRhiResourceUpdateBatch& res)
  {

    auto& frames = const_cast<DepthStreamNode&>(model()).frames;
    if(frames.m_currentFrameIdx > m_frameIdx)
    {
      auto old = m_frame;
      m_frame = frames.currentFrame();
      if(old)
        old->use_count--;
      m_frameIdx = frames.m_currentFrameIdx;
      if(m_frame && m_frame->frame)
        m_haveFrame = upload(renderer, res, *m_frame->frame);
    }

    if(m_haveFrame)
    {
      updateRays(renderer, res);
      updateParams(res);
    }
  }

  bool upload(score::gfx::RenderList& renderer, QRhiResourceUpdateBatch& res, AVFrame& f)
  {
    QRhiTexture::Format fmt{};
    int bpp{};
    switch(f.format)
    {
      case AV_PIX_FMT_GRAY16LE:
        fmt = QRhiTexture::R16;
        bpp = 2;
        m_sampleIsFloat = false;
        break;
      case AV_PIX_FMT_GRAYF32LE:
        fmt = QRhiTexture::R32F;
        bpp = 4;
        m_sampleIsFloat = true;
        break;
      case AV_PIX_FMT_GRAY8:
        fmt = QRhiTexture::R8;
        bpp = 1;
        m_sampleIsFloat = false;
        break;
      default:
        if(!m_warnedFormat)
          qWarning() << "[depthcam] unsupported raw pixel format" << f.format;
        m_warnedFormat = true;
        return false;
    }
    m_sampleBytes = bpp;

    const QSize sz{f.width, f.height};
    if(sz.isEmpty() || !f.data[0])
      return false;

    if(!m_raw || m_raw->pixelSize() != sz || m_raw->format() != fmt)
    {
      releaseTargets();
      auto& rhi = *renderer.state.rhi;
      m_raw = rhi.newTexture(fmt, sz, 1, {});
      m_raw->setName("DepthStream::raw");
      if(!m_raw->create())
      {
        qWarning() << "[depthcam] cannot create a" << sz << "raw texture";
        m_raw->deleteLater();
        m_raw = nullptr;
        return false;
      }
      if(!createTargets(renderer, sz))
        return false;
    }

    // Straight from the frame's own buffer, which stays alive until the frame
    // after this one has been uploaded: the only copy is the one QRhi makes
    // into its staging memory.
    const int stride = f.linesize[0] > 0 ? f.linesize[0] : f.width * bpp;
    QRhiTextureSubresourceUploadDescription sub(QByteArray::fromRawData(
        reinterpret_cast<const char*>(f.data[0]), qsizetype(stride) * f.height));
    sub.setDataStride(quint32(stride));
    res.uploadTexture(m_raw, QRhiTextureUploadEntry{0, 0, sub});
    g_uploaded.fetch_add(uint64_t(stride) * uint64_t(f.height), std::memory_order_relaxed);
    return true;
  }

  bool createTargets(score::gfx::RenderList& renderer, QSize sz)
  {
    auto& rhi = *renderer.state.rhi;
    m_size = sz;
    bool ok = true;
    if(model().kind == DepthStreamNode::Kind::Infrared)
    {
      ok &= m_irOut.create(rhi, QRhiTexture::R32F, sz, "DepthStream::ir");
    }
    else
    {
      ok &= m_posOut.create(rhi, QRhiTexture::RGBA32F, sz, "DepthStream::positions");
      ok &= m_history.create(rhi, QRhiTexture::RGBA32F, sz, "DepthStream::history");
      ok &= m_normalsOut.create(rhi, QRhiTexture::RGBA16F, sz, "DepthStream::normals");
      ok &= m_depthOut.create(
          rhi,
          model().mode == DepthCameraSettings::DepthOutput::Metres ? QRhiTexture::R32F
                                                                   : QRhiTexture::RGBA8,
          sz, "DepthStream::depth");

      m_rays = rhi.newTexture(QRhiTexture::RGBA32F, sz, 1, {});
      m_rays->setName("DepthStream::rays");
      ok &= m_rays->create();
      m_raysVersion = -1;
    }
    if(!ok)
    {
      qWarning() << "[depthcam] cannot create the" << sz << "processing targets";
      releaseTargets();
      return false;
    }
    m_geometryDirty = true;
    return buildPasses(renderer);
  }

  Pass makePass(
      score::gfx::RenderList& renderer, const std::pair<QShader, QShader>& sh,
      QRhiRenderPassDescriptor* rp, std::initializer_list<QRhiShaderResourceBinding> b)
  {
    auto& rhi = *renderer.state.rhi;
    Pass p;
    p.srb = rhi.newShaderResourceBindings();
    p.srb->setBindings(b.begin(), b.end());
    p.srb->create();
    p.pipeline = rhi.newGraphicsPipeline();
    p.pipeline->setShaderStages(
        {{QRhiShaderStage::Vertex, sh.first}, {QRhiShaderStage::Fragment, sh.second}});
    p.pipeline->setVertexInputLayout({});
    p.pipeline->setShaderResourceBindings(p.srb);
    p.pipeline->setRenderPassDescriptor(rp);
    p.pipeline->setTopology(QRhiGraphicsPipeline::Triangles);
    if(!p.pipeline->create())
    {
      qWarning() << "[depthcam] pipeline creation failed";
      p.release();
    }
    return p;
  }

  bool buildPasses(score::gfx::RenderList& renderer)
  {
    using B = QRhiShaderResourceBinding;
    const auto frag = B::FragmentStage;
    const auto all = B::VertexStage | B::FragmentStage | B::ComputeStage;
    if(model().kind == DepthStreamNode::Kind::Infrared)
    {
      m_irPass = makePass(
          renderer, m_shInfrared, m_irOut.rp,
          {B::uniformBuffer(0, all, m_ubo), B::sampledTexture(1, frag, m_raw, m_sampler)});
      return m_irPass.pipeline != nullptr;
    }

    m_posPass = makePass(
        renderer, m_shPositions, m_posOut.rp,
        {B::uniformBuffer(0, all, m_ubo), B::sampledTexture(1, frag, m_raw, m_sampler),
         B::sampledTexture(2, frag, m_rays, m_sampler),
         B::sampledTexture(3, frag, m_history.texture, m_sampler)});
    m_depthPass = makePass(
        renderer, m_shDepth, m_depthOut.rp,
        {B::uniformBuffer(0, all, m_ubo),
         B::sampledTexture(1, frag, m_posOut.texture, m_sampler)});
    m_normalsPass = makePass(
        renderer, m_shNormals, m_normalsOut.rp,
        {B::uniformBuffer(0, all, m_ubo),
         B::sampledTexture(1, frag, m_posOut.texture, m_sampler)});
    return m_posPass.pipeline && m_depthPass.pipeline && m_normalsPass.pipeline;
  }

  void releaseTargets()
  {
    m_posPass.release();
    m_depthPass.release();
    m_normalsPass.release();
    m_irPass.release();
    releaseMesh();
    m_posOut.release();
    m_history.release();
    m_normalsOut.release();
    m_depthOut.release();
    m_irOut.release();
    if(m_rays)
      m_rays->deleteLater();
    if(m_raw)
      m_raw->deleteLater();
    m_rays = m_raw = nullptr;
    for(auto& e : m_edges)
    {
      e.release();
      e.boundTexture = nullptr;
    }
    m_size = {};
  }

  void updateRays(score::gfx::RenderList&, QRhiResourceUpdateBatch& res)
  {
    if(model().kind != DepthStreamNode::Kind::Depth || !m_rays || !model().calib)
      return;
    const auto v = model().calib->version();
    if(v == m_raysVersion)
      return;
    m_raysVersion = v;

    const int w = m_size.width(), h = m_size.height();
    auto table = model().calib->depthRays(w, h);
    if(!table || table->size() < std::size_t(w) * h * 2)
      return;

    // RG would do, but Qt 6.4 has no RG32F.
    QByteArray rgba(qsizetype(w) * h * 4 * sizeof(float), Qt::Uninitialized);
    auto* dst = reinterpret_cast<float*>(rgba.data());
    const float* src = table->data();
    for(std::size_t i = 0, n = std::size_t(w) * h; i < n; i++)
    {
      const float x = src[2 * i], y = src[2 * i + 1];
      const bool ok = std::isfinite(x) && std::isfinite(y);
      dst[4 * i] = ok ? x : 0.f;
      dst[4 * i + 1] = ok ? y : 0.f;
      dst[4 * i + 2] = 0.f;
      dst[4 * i + 3] = ok ? 1.f : 0.f;
    }
    res.uploadTexture(
        m_rays, QRhiTextureUploadEntry{0, 0, QRhiTextureSubresourceUploadDescription{rgba}});
    g_uploaded.fetch_add(uint64_t(rgba.size()), std::memory_order_relaxed);
    m_calibration = model().calib->get();
    m_geometryDirty = true;
  }

  void updateParams(QRhiResourceUpdateBatch& res)
  {
    const auto& prm = *model().params;
    ParamsUBO u{};
    const float unit = model().out.unit_mm.load(std::memory_order_relaxed);
    const float mm = unit > 0.f ? unit : 1.f;
    // An R16 sample reads as raw / 65535; an R32F one as raw.
    u.scale[0] = m_sampleIsFloat ? mm * 1e-3f : float(65535. * mm * 1e-3);
    u.scale[1] = prm.near_m.load();
    u.scale[2] = prm.far_m.load();
    u.scale[3] = std::clamp(prm.temporal.load(), 0.f, 0.99f);
    for(int i = 0; i < 3; i++)
    {
      u.boxMin[i] = prm.box_min[i].load();
      u.boxMax[i] = prm.box_max[i].load();
    }
    u.boxMin[3] = prm.box_enabled.load() ? 1.f : 0.f;
    u.boxMax[3] = prm.ir_gain.load();
    u.size[0] = m_size.width();
    u.size[1] = m_size.height();
    u.size[2] = m_flip ? 1 : 0;
    u.size[3] = model().mode == DepthCameraSettings::DepthOutput::Metres ? 0 : 1;
    u.cull[0] = prm.edge_cull.load();
    u.cull[1] = prm.topology.load() == 1 ? 1.f : 0.f;

    const auto& d = m_calibration;
    const auto m = d.depthToColor ? *d.depthToColor : identityMatrix();
    std::memcpy(u.depthToColor, m.data(), sizeof(u.depthToColor));
    if(const auto* c = d.intrinsics(DEPTHCAM_STREAM_COLOR); c && d.depthToColor)
    {
      u.colorIntr[0] = c->fx;
      u.colorIntr[1] = c->fy;
      u.colorIntr[2] = c->cx;
      u.colorIntr[3] = c->cy;
      u.colorSize[0] = float(c->width);
      u.colorSize[1] = float(c->height);
      u.colorSize[2] = c->model == DEPTHCAM_DISTORTION_BROWN_CONRADY ? 1.f : 0.f;
      u.colorSize[3] = 1.f;
      std::memcpy(u.colorDist0, c->coeffs, 4 * sizeof(float));
      std::memcpy(u.colorDist1, c->coeffs + 4, 4 * sizeof(float));
    }
    // 8- and 16-bit samples are already normalised; a float one carries the
    // 16-bit range, as the Kinect v2 sends it.
    u.irScale[0] = m_sampleIsFloat ? 1.f / 65535.f : 1.f;

    res.updateDynamicBuffer(m_ubo, 0, sizeof(u), &u);

    const bool points = u.cull[1] > 0;
    if(points != m_points)
    {
      m_points = points;
      m_geometryDirty = true;
    }
  }

  // --- processing ----------------------------------------------------------

  void draw(
      QRhiCommandBuffer& cb, QRhiResourceUpdateBatch*& res, OwnTarget& t, const Pass& p)
  {
    if(!p.pipeline || !t.rt)
      return;
    cb.beginPass(t.rt, Qt::transparent, {1.0f, 0}, res);
    res = nullptr;
    cb.setGraphicsPipeline(p.pipeline);
    cb.setShaderResources(p.srb);
    cb.setViewport(QRhiViewport(0, 0, float(m_size.width()), float(m_size.height())));
    cb.draw(3);
    cb.endPass();
  }

  bool meshWanted() const noexcept
  {
    return model().kind == DepthStreamNode::Kind::Depth
           && wanted(DepthStreamNode::MeshOut);
  }

  void runInitialPasses(
      score::gfx::RenderList& renderer, QRhiCommandBuffer& cb,
      QRhiResourceUpdateBatch*& res, score::gfx::Edge& edge) override
  {
    if(m_broken || !m_haveFrame)
      return;

    if(m_passedFrame != renderer.frame)
    {
      m_passedFrame = renderer.frame;
      processFrame(renderer, cb, res);
    }

    // Geometry goes to its consumer every frame: they bind the buffers, which
    // the compute pass above has just rewritten in place.
    if(model().kind == DepthStreamNode::Kind::Depth
       && outletOf(edge.source) == DepthStreamNode::MeshOut)
      pushGeometry(renderer, edge);
  }

  void processFrame(
      score::gfx::RenderList& renderer, QRhiCommandBuffer& cb,
      QRhiResourceUpdateBatch*& res)
  {
    if(model().kind == DepthStreamNode::Kind::Infrared)
    {
      draw(cb, res, m_irOut, m_irPass);
      return;
    }

    const bool depth = wanted(DepthStreamNode::DepthOut);
    const bool normals = wanted(DepthStreamNode::NormalsOut);
    const bool mesh = meshWanted();
    const bool positions = wanted(DepthStreamNode::PositionsOut) || depth || normals || mesh;
    if(!positions)
      return;

    draw(cb, res, m_posOut, m_posPass);
    if(depth)
      draw(cb, res, m_depthOut, m_depthPass);
    if(normals)
      draw(cb, res, m_normalsOut, m_normalsPass);
    if(mesh)
      computeMesh(renderer, cb, res);

    // Keep this frame for the next one's average.
    if(model().params->temporal.load() > 0.f)
    {
      auto* batch = res ? res : renderer.state.rhi->nextResourceUpdateBatch();
      res = nullptr;
      batch->copyTexture(m_history.texture, m_posOut.texture);
      cb.resourceUpdate(batch);
    }
  }

  // --- mesh ------------------------------------------------------------------

  void releaseMesh()
  {
    m_meshPass.release();
    if(m_meshCompute)
      m_meshCompute->deleteLater();
    m_meshCompute = nullptr;
    // The buffers themselves are retired through the RenderList in release()
    // or replaced in ensureMesh(): a consumer may still have them bound.
    m_meshSrbTexture = nullptr;
  }

  bool ensureMesh(score::gfx::RenderList& renderer)
  {
    if(!m_compute || !m_shMesh.isValid())
    {
      if(!m_warnedCompute)
        qWarning() << "[depthcam] the mesh output needs compute shaders, which this "
                      "graphics backend does not offer";
      m_warnedCompute = true;
      return false;
    }
    auto& rhi = *renderer.state.rhi;
    const int w = m_size.width(), h = m_size.height();
    const quint32 vbytes = quint32(w) * quint32(h) * vertex_stride;
    const quint32 ibytes = quint32(std::max(0, w - 1)) * quint32(std::max(0, h - 1)) * 6 * 4;

    if(!m_vertices || m_vertices->size() != vbytes)
    {
      if(m_vertices)
        renderer.releaseBuffer(m_vertices);
      m_vertices = rhi.newBuffer(
          QRhiBuffer::Static, QRhiBuffer::VertexBuffer | QRhiBuffer::StorageBuffer, vbytes);
      m_vertices->setName("DepthStream::mesh_vertices");
      m_vertices->create();
      releaseMesh();
      m_geometryDirty = true;
    }
    if(!m_indices || m_indices->size() != ibytes)
    {
      if(m_indices)
        renderer.releaseBuffer(m_indices);
      m_indices = rhi.newBuffer(
          QRhiBuffer::Static, QRhiBuffer::IndexBuffer | QRhiBuffer::StorageBuffer,
          std::max<quint32>(ibytes, 4));
      m_indices->setName("DepthStream::mesh_indices");
      m_indices->create();
      releaseMesh();
      m_geometryDirty = true;
    }

    if(!m_meshCompute || m_meshSrbTexture != m_posOut.texture)
    {
      using B = QRhiShaderResourceBinding;
      releaseMesh();
      auto* srb = rhi.newShaderResourceBindings();
      srb->setBindings(
          {B::uniformBuffer(0, B::ComputeStage, m_ubo),
           B::sampledTexture(1, B::ComputeStage, m_posOut.texture, m_sampler),
           B::bufferLoadStore(2, B::ComputeStage, m_vertices),
           B::bufferLoadStore(3, B::ComputeStage, m_indices)});
      srb->create();
      auto* cp = rhi.newComputePipeline();
      cp->setShaderStage({QRhiShaderStage::Compute, m_shMesh});
      cp->setShaderResourceBindings(srb);
      if(!cp->create())
      {
        qWarning() << "[depthcam] mesh compute pipeline creation failed";
        cp->deleteLater();
        srb->deleteLater();
        return false;
      }
      m_meshCompute = cp;
      m_meshPass.srb = srb;
      m_meshSrbTexture = m_posOut.texture;
    }
    return true;
  }

  void computeMesh(
      score::gfx::RenderList& renderer, QRhiCommandBuffer& cb,
      QRhiResourceUpdateBatch*& res)
  {
    if(!ensureMesh(renderer) || !m_meshCompute)
      return;
    cb.beginComputePass(res);
    res = nullptr;
    cb.setComputePipeline(m_meshCompute);
    cb.setShaderResources(m_meshPass.srb);
    cb.dispatch((m_size.width() + 15) / 16, (m_size.height() + 15) / 16, 1);
    cb.endComputePass();
  }

  void pushGeometry(score::gfx::RenderList& renderer, score::gfx::Edge& edge)
  {
    if(!m_vertices || !m_indices)
      return;
    auto* sink = edge.sink;
    if(!sink || !sink->node)
      return;
    auto rendered = sink->node->renderedNodes.find(&renderer);
    if(rendered == sink->node->renderedNodes.end())
      return;
    auto it = std::find(sink->node->input.begin(), sink->node->input.end(), sink);
    if(it == sink->node->input.end())
      return;

    if(m_geometryDirty || !m_geometry.meshes)
    {
      m_geometryDirty = false;
      auto list = std::make_shared<ossia::mesh_list>();
      if(m_geometry.meshes)
        list->dirty_index = m_geometry.meshes->dirty_index + 1;

      ossia::geometry g;
      const int w = m_size.width(), h = m_size.height();
      g.buffers.push_back(
          {.data = ossia::geometry::gpu_buffer{m_vertices, int64_t(m_vertices->size())},
           .dirty = false});
      g.bindings.push_back(
          {.byte_stride = vertex_stride,
           .classification = ossia::geometry::binding::per_vertex,
           .step_rate = 1});

      ossia::geometry::attribute pos;
      pos.binding = 0;
      pos.location = 0;
      pos.format = ossia::geometry::attribute::float3;
      pos.byte_offset = 0;
      pos.semantic = ossia::attribute_semantic::position;
      g.attributes.push_back(pos);

      auto nrm = pos;
      nrm.location = 1;
      nrm.byte_offset = 16;
      nrm.semantic = ossia::attribute_semantic::normal;
      g.attributes.push_back(nrm);

      auto uv = pos;
      uv.location = 2;
      uv.format = ossia::geometry::attribute::float2;
      uv.byte_offset = 32;
      uv.semantic = ossia::attribute_semantic::texcoord0;
      g.attributes.push_back(uv);

      g.input.push_back({.buffer = 0, .byte_offset = 0});
      g.vertices = w * h;
      g.cull_mode = ossia::geometry::none;
      g.front_face = ossia::geometry::counter_clockwise;
      if(m_points)
      {
        g.topology = ossia::geometry::points;
        g.indices = 0;
      }
      else
      {
        g.buffers.push_back(
            {.data = ossia::geometry::gpu_buffer{m_indices, int64_t(m_indices->size())},
             .dirty = false});
        g.topology = ossia::geometry::triangles;
        g.indices = std::max(0, w - 1) * std::max(0, h - 1) * 6;
        g.index.buffer = 1;
        g.index.byte_offset = 0;
        g.index.format = decltype(g.index)::uint32;
      }

      // The organised positions too, for a compute shader that wants them as a
      // texture: one texel per vertex.
      g.auxiliary_textures.push_back(
          {.name = "depth_positions", .native_handle = m_posOut.texture,
           .sampler_handle = nullptr});

      const auto& prm = *model().params;
      for(int i = 0; i < 3; i++)
      {
        g.bounds.min[i] = prm.box_enabled ? prm.box_min[i].load() : -4.f;
        g.bounds.max[i] = prm.box_enabled ? prm.box_max[i].load() : 4.f;
      }
      if(!prm.box_enabled)
      {
        g.bounds.min[2] = 0.f;
        g.bounds.max[2] = 8.f;
      }

      list->meshes.push_back(std::move(g));
      m_geometry.meshes = std::move(list);
      if(!m_geometry.filters)
        m_geometry.filters = std::make_shared<ossia::geometry_filter_list>();
    }

    const int port = int(it - sink->node->input.begin());
    rendered->second->process(port, m_geometry, edge.source);
  }

  // --- output to consumers -----------------------------------------------------

  void runRenderPass(
      score::gfx::RenderList& renderer, QRhiCommandBuffer& cb,
      score::gfx::Edge& edge) override
  {
    if(m_broken || !m_haveFrame)
      return;
    auto it = ossia::find_if(m_edges, [&](const EdgePass& e) { return e.edge == &edge; });
    if(it == m_edges.end() || !it->pass.pipeline)
      return;
    // The texture may have been replaced since update(): a size change.
    if(it->boundTexture != outletTexture(it->outlet))
      return;
    auto rt = renderer.renderTargetForOutput(edge);
    if(!rt.renderTarget)
      return;
    const QSize sz = rt.renderTarget->pixelSize();
    cb.setGraphicsPipeline(it->pass.pipeline);
    cb.setShaderResources(it->pass.srb);
    cb.setViewport(QRhiViewport(0, 0, float(sz.width()), float(sz.height())));
    cb.draw(3);
  }

private:
  bool m_flip{};
  bool m_compute{};
  bool m_broken{};
  bool m_warnedFormat{};
  bool m_warnedCompute{};
  bool m_haveFrame{};
  bool m_sampleIsFloat{};
  bool m_points{};
  bool m_geometryDirty{true};
  int m_sampleBytes{};

  int64_t m_updatedFrame{-1};
  int64_t m_passedFrame{-1};
  int64_t m_frameIdx{-1};
  int64_t m_raysVersion{-1};
  std::shared_ptr<score::gfx::RefcountedFrame> m_frame;

  QSize m_size{};
  QRhiBuffer* m_ubo{};
  QRhiSampler* m_sampler{};
  QRhiTexture* m_raw{};
  QRhiTexture* m_rays{};
  OwnTarget m_posOut, m_history, m_normalsOut, m_depthOut, m_irOut;
  Pass m_posPass, m_depthPass, m_normalsPass, m_irPass, m_meshPass;
  QRhiComputePipeline* m_meshCompute{};
  QRhiTexture* m_meshSrbTexture{};
  QRhiBuffer* m_vertices{};
  QRhiBuffer* m_indices{};
  ossia::geometry_spec m_geometry;
  CalibrationData m_calibration;

  std::pair<QShader, QShader> m_shPositions, m_shDepth, m_shNormals, m_shInfrared, m_shBlit;
  QShader m_shMesh;

  ossia::small_vector<EdgePass, 4> m_edges;
};

}

uint64_t depthStreamUploadedBytes() noexcept
{
  return g_uploaded.load(std::memory_order_relaxed);
}

DepthStreamNode::DepthStreamNode(
    Kind k, std::shared_ptr<InputStream> s, StreamOutput& o,
    std::shared_ptr<InputStreamExtractor> extractor, std::shared_ptr<RenderParams> prm,
    std::shared_ptr<Calibration> c, DepthCameraSettings::DepthOutput m)
    : kind{k}
    , mode{m}
    , stream{std::move(s)}
    , out{o}
    , params{std::move(prm)}
    , calib{std::move(c)}
{
  frames.m_decoder = std::move(extractor);
  using namespace score::gfx;
  output.push_back(new Port{this, {}, Types::Image, {}, {}});
  if(kind == Kind::Depth)
  {
    output.push_back(new Port{this, {}, Types::Image, {}, {}});
    output.push_back(new Port{this, {}, Types::Image, {}, {}});
    output.push_back(new Port{this, {}, Types::Geometry, {}, {}});
  }
}

DepthStreamNode::~DepthStreamNode()
{
  if(m_consuming && stream)
    stream->addConsumer(out, -1);
}

score::gfx::NodeRenderer*
DepthStreamNode::createRenderer(score::gfx::RenderList&) const noexcept
{
  return new DepthStreamRenderer{*this};
}

void DepthStreamNode::process(score::gfx::Message&& msg)
{
  ProcessNode::process(std::move(msg));
  if(!renderedNodes.empty())
    if(auto* f = frames.m_decoder->dequeue_frame())
      frames.updateCurrentFrame(f);
  frames.releaseFramesToFree();
}

void DepthStreamNode::renderedNodesChanged()
{
  const bool consuming = !renderedNodes.empty();
  if(consuming != m_consuming)
  {
    m_consuming = consuming;
    if(stream)
      stream->addConsumer(out, consuming ? 1 : -1);
  }
  if(!consuming)
    frames.releaseAllFrames();
}

}
