// The GPU outputs of the depth camera against the synthetic scene, on every
// graphics backend the machine has: metric depth to the last bit for each
// sample type, positions and the mesh against the CPU unprojection, normals,
// infrared, the clip planes, the stage box and the edge cull.
//
// Float textures are read through ISF inputs declared STATIC, which bind the
// node's own texture, and leave as the bits of the float split over two RGBA8
// sinks -- 16 bits each in red and green, alpha 1 so that no compositing can
// touch them. The mesh comes back through BufferSinkNode.
#include <DepthCamera/DepthCameraCalibration.hpp>
#include <DepthCamera/DepthCameraGpu.hpp>
#include <DepthCamera/DepthCameraStream.hpp>

#include <SyntheticBackend.hpp>
#include <SyntheticScene.hpp>

#include <score_test/Gfx.hpp>
#include <score_test/GfxBufferSink.hpp>

#include <QFile>
#include <QTemporaryDir>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_range.hpp>

#include <cmath>
#include <cstring>

using namespace score::test::gfx;
using namespace Gfx::DepthCamera;
namespace syn = depthcam_synthetic;

namespace
{
constexpr int W = 160;
constexpr int H = 120;

QString writeProbe(const QTemporaryDir& dir, int component, bool high)
{
  const QString path
      = dir.filePath(QStringLiteral("probe_%1_%2.fs").arg(component).arg(high ? "hi" : "lo"));
  QFile f(path);
  if(!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
    return {};
  f.write(QStringLiteral(R"(/*{
  "ISFVSN": "2.0",
  "INPUTS": [ { "NAME": "tex", "TYPE": "image", "STATIC": true } ]
}*/
void main() {
  // IMG_TEXEL, not texelFetch: an ISF's gl_FragCoord is GL's on every
  // backend, and the macro reads the row that matches it.
  vec4 v = IMG_TEXEL_PREMULTIPLIED(tex, gl_FragCoord.xy);
  uint b = floatBitsToUint(v[%1]);
  uint h = %2;
  gl_FragColor = vec4(float(h >> 8u) / 255.0, float(h & 255u) / 255.0, 0.0, 1.0);
}
)")
              .arg(component)
              .arg(high ? "b >> 16u" : "b & 0xFFFFu")
              .toUtf8());
  return path;
}

/// What the depth node is asked for and how.
struct Setup
{
  QString uri{"synthetic:clock=manual"};
  DepthCameraSettings::DepthOutput mode{DepthCameraSettings::DepthOutput::Metres};
  bool infrared{};
  std::function<void(RenderParams&)> params;
  /// Texture readbacks: (outlet, component).
  std::vector<std::pair<int, int>> probes;
  bool mesh{};
  bool preview{}; // read the depth outlet straight, RGBA8
};

struct Shots
{
  bool skipped{};
  std::string skip_reason, backend, error;
  /// One W x H float image per probe.
  std::vector<std::vector<float>> floats;
  ReadbackImage preview;
  /// Mesh: interleaved vertices and the index buffer.
  std::vector<float> vertices;
  std::vector<uint32_t> indices;
  int vertexCount{}, indexCount{};
  int64_t frameNumber{};
};

Shots render(score::gfx::GraphicsApi be, const Setup& s)
{
  Shots r;
  QTemporaryDir dir;
  score::test::run_in_gui_app([&](const score::GUIApplicationContext&) {
    DepthCameraSettings set;
    set.device = s.uri;
    set.rgb = false;
    set.depth = !s.infrared;
    set.ir = s.infrared;
    set.pointcloud = false;
    set.depthWidth = W;
    set.depthHeight = H;
    auto stream = std::make_shared<InputStream>(syn::backend(), s.uri, set);
    if(!stream->valid())
    {
      r.error = "could not open the synthetic camera";
      return;
    }
    auto& out = s.infrared ? stream->m_ir : stream->m_depth;
    auto dec = std::make_shared<InputStreamExtractor>(stream, out, QString{});

    auto calib = std::make_shared<Calibration>();
    calib->set(Calibration::query(
        *syn::backend(), *stream->device(), stream->activeStreams(), W, H));
    calib->setRaySource(syn::backend(), stream->device());

    auto params = std::make_shared<RenderParams>();
    if(s.params)
      s.params(*params);

    {
      GfxPipeline p;
      const int node = p.addNode(std::make_unique<DepthStreamNode>(
          s.infrared ? DepthStreamNode::Kind::Infrared : DepthStreamNode::Kind::Depth,
          stream, out, dec, params, calib, s.mode));

      std::vector<std::pair<int, int>> probeSinks; // hi, lo
      for(auto [outlet, comp] : s.probes)
      {
        const int hi = p.addIsf(writeProbe(dir, comp, true));
        const int lo = p.addIsf(writeProbe(dir, comp, false));
        const int shi = p.addSink({W, H});
        const int slo = p.addSink({W, H});
        p.wire(p.nodeImageOut(node, outlet), p.imageIn(hi, 0));
        p.wire(p.nodeImageOut(node, outlet), p.imageIn(lo, 0));
        p.wire(p.imageOut(hi, 0), p.sinkInput(shi));
        p.wire(p.imageOut(lo, 0), p.sinkInput(slo));
        probeSinks.emplace_back(shi, slo);
      }
      int previewSink = -1;
      if(s.preview)
      {
        previewSink = p.addSink({W, H});
        p.wire(p.nodeImageOut(node, 0), p.sinkInput(previewSink));
      }
      AttachedBufferSink mesh{};
      if(s.mesh)
      {
        mesh = attach_buffer_sink(p);
        p.wire(p.nodeGeometryOut(node, 0), mesh.node->input[0]);
      }

      if(!p.create(be))
      {
        r.skipped = p.skipped();
        r.skip_reason = p.skipReason();
        r.backend = p.backend();
        r.error = p.error();
        stream->close();
        return;
      }
      r.backend = p.backend();

      // The camera only starts producing once something draws it.
      if(!stream->start())
      {
        r.error = "could not start the synthetic camera";
        return;
      }
      r.frameNumber = syn::nextFrame(stream->device());
      syn::step(stream->device(), 1);

      if(mesh.node)
        render_with_buffer_sink(p, *mesh.node, 4);
      else
        p.render(4);

      for(auto [shi, slo] : probeSinks)
      {
        const auto hi = p.readback(shi);
        const auto lo = p.readback(slo);
        std::vector<float> img;
        if(hi.valid() && lo.valid() && hi.width == W && hi.height == H)
        {
          img.resize(std::size_t(W) * H);
          for(int y = 0; y < H; y++)
            for(int x = 0; x < W; x++)
            {
              const auto a = hi.at(x, y);
              const auto b = lo.at(x, y);
              const uint32_t bits = (uint32_t(a[0]) << 24) | (uint32_t(a[1]) << 16)
                                    | (uint32_t(b[0]) << 8) | uint32_t(b[1]);
              std::memcpy(&img[std::size_t(y) * W + x], &bits, 4);
            }
        }
        else if(r.error.empty())
          r.error = "empty texture readback";
        r.floats.push_back(std::move(img));
      }
      if(previewSink >= 0)
        r.preview = p.readback(previewSink);

      if(mesh.node)
      {
        auto& h = *mesh.node->harvest;
        if(h.readback_unsupported)
        {
          r.skipped = true;
          r.skip_reason = "no buffer readback";
        }
        else if(!h.meshes.empty())
        {
          const auto& m = h.meshes.front();
          r.vertexCount = m.vertices;
          r.indexCount = m.indices;
          for(const auto& a : m.attributes)
          {
            if(a.name == "position")
              r.vertices = as_floats(a.rb.data);
            else if(a.name == "index")
            {
              r.indices.resize(std::size_t(a.rb.data.size()) / 4);
              std::memcpy(r.indices.data(), a.rb.data.constData(), r.indices.size() * 4);
            }
          }
        }
        else if(r.error.empty())
          r.error = "the mesh never reached the sink";
      }
      if(r.error.empty())
        r.error = p.error();
    }
    calib->setRaySource(nullptr, nullptr);
    stream->close();
  });
  return r;
}

/// The depth a texel must hold: the scene, quantised as the sample type is,
/// with the unit applied in float as the shader does.
double expectedDepth(const syn::Camera& cam, int u, int v, int64_t frame, std::string_view unit)
{
  const double m = syn::depthAt(cam, u, v, frame, false);
  if(unit == "0.1mm")
    return std::round(m * 10000.) / 10000.;
  if(unit == "f32mm")
    return double(float(m * 1000.)) / 1000.;
  if(unit == "f32m")
    return double(float(m));
  return std::round(m * 1000.) / 1000.;
}

#define SKIP_OR_REQUIRE(s)                       \
  do                                             \
  {                                              \
    if((s).skipped)                              \
      SKIP((s).backend + ": " + (s).skip_reason); \
    CAPTURE((s).backend);                        \
    INFO("error=" << (s).error);                 \
    REQUIRE((s).error.empty());                  \
  } while(0)
}

TEST_CASE("Depth is in metres, for every sample type", "[depthcam][gfx]")
{
  const auto backend = GENERATE(from_range(platform_backends()));
  const std::string unit = GENERATE("1mm", "0.1mm", "f32mm", "f32m");
  CAPTURE(backend_name(backend), unit);

  Setup s;
  s.uri = QString::fromStdString("synthetic:clock=manual,unit=" + unit);
  s.probes = {{DepthStreamNode::DepthOut, 0}};
  const auto r = render(backend, s);
  SKIP_OR_REQUIRE(r);
  REQUIRE(r.floats.size() == 1);
  REQUIRE(r.floats[0].size() == std::size_t(W * H));

  const auto cam = syn::depthCamera(W, H, false);
  double worst = 0;
  int zeros = 0;
  for(int v = 0; v < H; v++)
    for(int u = 0; u < W; u++)
    {
      const double want = expectedDepth(cam, u, v, r.frameNumber, unit);
      const double got = r.floats[0][std::size_t(v) * W + u];
      if(want == 0)
      {
        zeros++;
        CHECK(got == 0.);
        continue;
      }
      worst = std::max(worst, std::abs(got - want) / want);
    }
  // A 16-bit sample reads as raw / 65535 and is scaled back: a few ulp.
  CHECK(worst < 3e-7);
  CHECK(zeros == syn::dropout * syn::dropout);

  // The sphere's nearest point, on the axis: 1.5 m.
  const double centre = r.floats[0][std::size_t(H / 2) * W + W / 2];
  CHECK(centre == Catch::Approx(1.5).margin(2e-3));
}

TEST_CASE("The preview is the old grey", "[depthcam][gfx]")
{
  const auto backend = GENERATE(from_range(platform_backends()));
  CAPTURE(backend_name(backend));
  Setup s;
  s.mode = DepthCameraSettings::DepthOutput::Preview;
  s.preview = true;
  const auto r = render(backend, s);
  SKIP_OR_REQUIRE(r);
  REQUIRE(r.preview.valid());
  const auto cam = syn::depthCamera(W, H, false);
  // 0 to 4.5 m, black to white.
  for(auto [u, v] : {std::pair{W / 2, H / 2}, std::pair{W - 3, 5}, std::pair{2, 2}})
  {
    CAPTURE(u, v);
    const double z = expectedDepth(cam, u, v, r.frameNumber, "1mm");
    const auto px = r.preview.at(u, v);
    CHECK(std::abs(int(px[0]) - int(std::lround(z / 4.5 * 255.))) <= 1);
    CHECK(px[0] == px[1]);
  }
}

TEST_CASE("Positions and the mesh match the CPU unprojection", "[depthcam][gfx]")
{
  const auto backend = GENERATE(from_range(platform_backends()));
  const bool distorted = GENERATE(false, true);
  CAPTURE(backend_name(backend), distorted);

  Setup s;
  s.uri = distorted ? "synthetic:clock=manual,distortion=bc" : "synthetic:clock=manual";
  s.probes = {{DepthStreamNode::PositionsOut, 0}, {DepthStreamNode::NormalsOut, 2}};
  s.mesh = true;
  s.params = [](RenderParams& p) { p.edge_cull = 0.05f; };
  const auto r = render(backend, s);
  SKIP_OR_REQUIRE(r);
  REQUIRE(r.vertexCount == W * H);
  REQUIRE(r.vertices.size() >= std::size_t(W * H * 12));
  REQUIRE(r.indexCount == (W - 1) * (H - 1) * 6);
  REQUIRE(r.indices.size() >= std::size_t(r.indexCount));

  const auto cam = syn::depthCamera(W, H, distorted);
  double worst = 0;
  for(int v = 0; v < H; v++)
    for(int u = 0; u < W; u++)
    {
      const auto i = std::size_t(v) * W + u;
      const float* vx = &r.vertices[12 * i];
      const double z = expectedDepth(cam, u, v, r.frameNumber, "1mm");
      if(z == 0)
      {
        CHECK(vx[3] == 0.f);
        continue;
      }
      REQUIRE(vx[3] == 1.f);
      const auto ray = syn::ray(cam, u, v);
      worst = std::max(worst, std::abs(vx[0] - ray[0] * z));
      worst = std::max(worst, std::abs(vx[1] - ray[1] * z));
      worst = std::max(worst, std::abs(vx[2] - z));

      // The positions texture is the same grid, in image order once read
      // back: the dropout is top-left, wherever the backend puts row 0.
      CHECK(r.floats[0][i] == vx[0]);
    }
  CHECK(worst < 2e-5); // metres

  // The wall faces the camera: -Z, in both the mesh and the normals texture.
  for(auto [u, v] : {std::pair{W - 6, H / 3}, std::pair{W - 20, 10}})
  {
    CAPTURE(u, v);
    const auto i = std::size_t(v) * W + u;
    REQUIRE(expectedDepth(cam, u, v, r.frameNumber, "1mm") == 3.0);
    CHECK(r.vertices[12 * i + 6] == Catch::Approx(-1.f).margin(1e-3));
    CHECK(r.floats[1][i] == Catch::Approx(-1.f).margin(1e-3));
  }

  // Triangles: dropped across the box's edge and in the dropout, kept on the
  // wall.
  const auto degenerate = [&](int u, int v, int t) {
    const auto q = (std::size_t(v) * (W - 1) + u) * 6 + 3 * t;
    return r.indices[q] == r.indices[q + 1] && r.indices[q + 1] == r.indices[q + 2];
  };
  CHECK(degenerate(2, 2, 0));
  CHECK_FALSE(degenerate(W - 6, H / 3, 0));
  CHECK_FALSE(degenerate(W - 6, H / 3, 1));
  int culledEdges = 0;
  for(int u = 1; u < W / 2; u++)
  {
    // Along the row through the box's middle, the box's left and right sides.
    const int v = int(std::lround(cam.cy + cam.fy * (-0.45 / syn::box_min[2])));
    const double a = expectedDepth(cam, u, v, r.frameNumber, "1mm");
    const double b = expectedDepth(cam, u + 1, v, r.frameNumber, "1mm");
    if(a > 0 && b > 0 && std::abs(a - b) > 0.5)
    {
      culledEdges++;
      CHECK(degenerate(u, v, 0));
    }
  }
  CHECK(culledEdges >= 2);
}

TEST_CASE("Clip planes and the stage box invalidate what is outside", "[depthcam][gfx]")
{
  const auto backend = GENERATE(from_range(platform_backends()));
  CAPTURE(backend_name(backend));

  Setup s;
  s.probes = {{DepthStreamNode::DepthOut, 0}, {DepthStreamNode::PositionsOut, 3}};
  s.params = [](RenderParams& p) {
    p.far_m = 2.8f; // drops the wall
    p.box_enabled = true;
    // Around the sphere only: the floor, the box and the wall are outside.
    for(int i = 0; i < 3; i++)
    {
      p.box_min[i] = i == 2 ? 1.4f : -0.35f;
      p.box_max[i] = i == 2 ? 2.2f : 0.35f;
    }
  };
  const auto r = render(backend, s);
  SKIP_OR_REQUIRE(r);
  const auto cam = syn::depthCamera(W, H, false);
  for(int v = 0; v < H; v++)
    for(int u = 0; u < W; u++)
    {
      const auto i = std::size_t(v) * W + u;
      const double z = expectedDepth(cam, u, v, r.frameNumber, "1mm");
      const auto ray = syn::ray(cam, u, v);
      const double P[3]{ray[0] * z, ray[1] * z, z};
      const bool inside = z > 0 && z <= 2.8 && std::abs(P[0]) <= 0.35
                          && std::abs(P[1]) <= 0.35 && P[2] >= 1.4 && P[2] <= 2.2;
      // Leave a millimetre either side of the faces for float rounding.
      const bool nearFace = std::abs(std::abs(P[0]) - 0.35) < 1e-3
                            || std::abs(std::abs(P[1]) - 0.35) < 1e-3;
      if(nearFace)
        continue;
      CAPTURE(u, v, z);
      CHECK((r.floats[1][i] == 1.f) == inside);
      CHECK((r.floats[0][i] != 0.f) == inside);
    }
}

TEST_CASE("Infrared is scaled by its gain", "[depthcam][gfx]")
{
  const auto backend = GENERATE(from_range(platform_backends()));
  const bool f32 = GENERATE(false, true);
  CAPTURE(backend_name(backend), f32);
  Setup s;
  s.uri = f32 ? "synthetic:clock=manual,ir=f32" : "synthetic:clock=manual";
  s.infrared = true;
  s.probes = {{0, 0}};
  s.params = [](RenderParams& p) { p.ir_gain = 2.f; };
  const auto r = render(backend, s);
  SKIP_OR_REQUIRE(r);
  for(auto [u, v] : {std::pair{0, 0}, std::pair{W / 2, H / 2}, std::pair{W - 1, H - 1}})
  {
    CAPTURE(u, v);
    const double want = syn::irAt(u, v, r.frameNumber, 150.) / 65535. * 2.;
    CHECK(r.floats[0][std::size_t(v) * W + u] == Catch::Approx(want).epsilon(1e-6));
  }
}
