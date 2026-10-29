// What a depth frame costs the host, CPU path against GPU path, with the
// synthetic camera on a manual clock.
//
//   cpu  depth through score's video node (R16 decoder, the 0-4.5 m filter)
//        and the backend's CPU point cloud, drawn by a raw-raster pipeline:
//        what a device with depth and pointcloud enabled did before the GPU
//        outputs.
//   gpu  the depth node: depth outlet, and the mesh drawn by the same
//        raw-raster pipeline. The point cloud is not consumed, so the backend
//        is told not to compute it.
//
// Reported per frame: CPU time of the render thread, of the host's frame
// callback on the camera thread, and of the backend's point cloud; bytes
// handed to the GPU. The synthetic camera's own rendering of its scene is not
// counted: a real camera does that in hardware.
//
// Not a test: run it, read the table. `depthcam_perf [frames]`.
#include <DepthCamera/DepthCameraCalibration.hpp>
#include <DepthCamera/DepthCameraGpu.hpp>
#include <DepthCamera/DepthCameraPointCloud.hpp>
#include <DepthCamera/DepthCameraStream.hpp>

#include <SyntheticBackend.hpp>

#include <Gfx/Graph/VideoNode.hpp>

#include <score_test/Gfx.hpp>

#include <QFile>
#include <QTemporaryDir>

#include <cstdio>
#include <string>
#include <vector>

using namespace score::test::gfx;
using namespace Gfx::DepthCamera;
namespace syn = depthcam_synthetic;

namespace
{
struct Result
{
  std::string api, path;
  int w{}, h{}, frames{};
  double render_ms{}, callback_ms{}, cloud_ms{};
  double upload_kib{};
  std::string error;
};

QString write(const QTemporaryDir& dir, const char* name, const char* text)
{
  const QString path = dir.filePath(QString::fromUtf8(name));
  QFile f(path);
  if(f.open(QIODevice::WriteOnly | QIODevice::Truncate))
    f.write(text);
  return path;
}

constexpr auto raster_vs = R"(void main()
{
  isf_vertShaderInit();
  gl_PointSize = 1.0;
  gl_Position = clipSpaceCorrMatrix * vec4(position.xy / max(position.z, 0.1), 0.5, 1.0);
  isf_vertShaderFinish();
}
)";
constexpr auto raster_fs = R"(/*{
  "ISFVSN": "2.0",
  "MODE": "RAW_RASTER_PIPELINE",
  "VERTEX_INPUTS": [ { "TYPE": "vec4", "NAME": "position" } ],
  "FRAGMENT_OUTPUTS": [ { "TYPE": "vec4", "NAME": "isf_FragColor" } ],
  "INPUTS": []
}*/
void main() { isf_FragColor = vec4(1.0); }
)";

// One output, so one RenderList: each frame is uploaded once, as in a session
// with one window.
constexpr auto mix_fs = R"(/*{
  "ISFVSN": "2.0",
  "INPUTS": [ { "NAME": "a", "TYPE": "image" }, { "NAME": "b", "TYPE": "image" } ]
}*/
void main() { gl_FragColor = 0.5 * (IMG_THIS_PIXEL(a) + IMG_THIS_PIXEL(b)); }
)";

// What master's device built for the depth texture.
const QString legacy_filter = QStringLiteral(
    "float v = tex.r * 65535. * 1 / 4500.; processed.rgb = vec3(v, v, v); processed.a = 1.;");

Result measure(score::gfx::GraphicsApi api, bool gpu, int w, int h, int frames)
{
  Result r;
  r.path = gpu ? "gpu" : "cpu";
  r.w = w;
  r.h = h;
  r.frames = frames;
  QTemporaryDir dir;
  const QString vs = write(dir, "pts.vs", raster_vs);
  const QString fs = write(dir, "pts.fs", raster_fs);
  const QString mix = write(dir, "mix.fs", mix_fs);

  DepthCameraSettings set;
  set.device = "synthetic:clock=manual";
  set.rgb = false;
  set.depth = true;
  set.pointcloud = !gpu;
  set.depthWidth = w;
  set.depthHeight = h;
  auto stream = std::make_shared<InputStream>(syn::backend(), set.device, set);
  if(!stream->valid())
  {
    r.error = "no synthetic camera";
    return r;
  }

  auto calib = std::make_shared<Calibration>();
  calib->set(Calibration::query(*syn::backend(), *stream->device(), stream->activeStreams(), w, h));
  calib->setRaySource(syn::backend(), stream->device());
  auto params = std::make_shared<RenderParams>();

  {
    GfxPipeline p;
    const int raster = p.addRaster(vs, fs);
    const int mixer = p.addIsf(mix);
    const int sink = p.addSink({w, h});
    p.wire(p.imageOut(mixer, 0), p.sinkInput(sink));
    if(gpu)
    {
      auto dec = std::make_shared<InputStreamExtractor>(stream, stream->m_depth, QString{});
      const int node = p.addNode(std::make_unique<DepthStreamNode>(
          DepthStreamNode::Kind::Depth, stream, stream->m_depth, dec, params, calib,
          DepthCameraSettings::DepthOutput::Preview));
      p.wire(p.nodeImageOut(node, DepthStreamNode::DepthOut), p.imageIn(mixer, 1));
      p.wire(p.nodeGeometryOut(node, 0), p.geometryIn(raster, 0));
    }
    else
    {
      auto depthDec = std::make_shared<InputStreamExtractor>(stream, stream->m_depth, QString{});
      auto pclDec = std::make_shared<InputStreamExtractor>(stream, stream->m_pcl, QString{});
      const int depth
          = p.addNode(std::make_unique<score::gfx::CameraNode>(depthDec, legacy_filter));
      auto cloud = std::make_unique<oscr::DeviceNode<PointCloudMesh>>(pclDec);
      cloud->consuming = [s = stream.get()](bool on) {
        s->addConsumer(s->m_pcl, on ? 1 : -1);
      };
      const int pcl = p.addNode(std::move(cloud));
      p.wire(p.nodeImageOut(depth, 0), p.imageIn(mixer, 1));
      p.wire(p.nodeGeometryOut(pcl, 0), p.geometryIn(raster, 0));
      // score's own video node reports nothing; the colour node does it in
      // the device.
      stream->addConsumer(stream->m_depth, 1);
    }
    p.wire(p.imageOut(raster, 0), p.imageIn(mixer, 0));

    if(!p.create(api))
    {
      r.api = p.backend();
      r.error = p.skipped() ? "skipped: " + p.skipReason() : p.error();
      return r;
    }
    r.api = p.backend();
    stream->start();

    const auto run = [&](int n, bool count) {
      auto& c = syn::counters();
      const auto cb0 = c.host_callback_ns.load(), pc0 = c.pointcloud_ns.load();
      const auto up0 = depthStreamUploadedBytes();
      int64_t render = 0;
      for(int i = 0; i < n; i++)
      {
        syn::step(stream->device(), 1);
        const auto t0 = syn::threadCpuNs();
        p.render(1);
        render += syn::threadCpuNs() - t0;
      }
      if(!count)
        return;
      r.render_ms = double(render) / n * 1e-6;
      r.callback_ms = double(c.host_callback_ns.load() - cb0) / n * 1e-6;
      r.cloud_ms = double(c.pointcloud_ns.load() - pc0) / n * 1e-6;
      if(gpu)
        r.upload_kib = double(depthStreamUploadedBytes() - up0) / n / 1024.;
      else
        // score's R16 decoder uploads the frame from its own buffer
        // (GPUVideoDecoder.cpp, createTextureUpload), and the raster pipeline
        // the cloud's CPU buffer, re-uploaded for each new frame: 2 + 12
        // bytes per pixel.
        r.upload_kib = double(w) * h * (2 + 12) / 1024.;
    };
    run(10, false);
    run(frames, true);
    if(r.error.empty())
      r.error = p.error();
  }
  if(!gpu)
    stream->addConsumer(stream->m_depth, -1);
  calib->setRaySource(nullptr, nullptr);
  stream->close();
  return r;
}
}

int main(int argc, char** argv)
{
  const int frames = argc > 1 ? std::max(1, std::atoi(argv[1])) : 60;
  std::vector<Result> results;
  score::test::run_in_gui_app([&](const score::GUIApplicationContext&) {
    for(auto api : platform_backends())
      for(auto [w, h] : {std::pair{640, 576}, std::pair{1280, 720}})
        for(bool gpu : {false, true})
          results.push_back(measure(api, gpu, w, h, frames));
  });

  std::printf(
      "%-8s %-4s %10s %12s %14s %12s %14s  %s\n", "api", "path", "size", "render ms",
      "callback ms", "cloud ms", "upload KiB", "");
  for(const auto& r : results)
    std::printf(
        "%-8s %-4s %5dx%-4d %12.3f %14.3f %12.3f %14.1f  %s\n", r.api.c_str(),
        r.path.c_str(), r.w, r.h, r.render_ms, r.callback_ms, r.cloud_ms, r.upload_kib,
        r.error.c_str());
  return 0;
}
