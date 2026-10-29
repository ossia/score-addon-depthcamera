// The depth camera plug-in against the synthetic camera, linked in-process:
// streams, units, timestamps, calibration, the metadata tree, the controls and
// their persistence, settings compatibility, and ABI 3 backends. No GPU, no
// camera.
#include <DepthCamera/Backend/BackendRegistry.hpp>
#include <DepthCamera/DepthCameraCalibration.hpp>
#include <DepthCamera/DepthCameraControls.hpp>
#include <DepthCamera/DepthCameraGpu.hpp>
#include <DepthCamera/DepthCameraInfo.hpp>
#include <DepthCamera/DepthCameraSettings.hpp>
#include <DepthCamera/DepthCameraStream.hpp>

#include <SyntheticBackend.hpp>
#include <SyntheticScene.hpp>


#include <ossia/network/base/node_functions.hpp>
#include <ossia/network/base/node_attributes.hpp>
#include <ossia/network/dataspace/dataspace_visitors.hpp>
#include <ossia/network/generic/generic_device.hpp>
#include <ossia/network/local/local.hpp>

#include <QBuffer>
#include <QCoreApplication>
#include <QDataStream>
#include <QDir>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

extern "C" {
#include <libavutil/frame.h>
}

#include <chrono>
#include <cmath>
#include <thread>

using namespace Gfx::DepthCamera;
namespace syn = depthcam_synthetic;
using Catch::Approx;

namespace
{
QCoreApplication& app()
{
  static int argc = 1;
  static char name[] = "depthcam_unit";
  static char* argv[] = {name, nullptr};
  static QCoreApplication a{argc, argv};
  return a;
}

void pump()
{
  for(int i = 0; i < 5; i++)
    app().processEvents();
}

DepthCameraSettings settingsFor(QString uri)
{
  DepthCameraSettings s;
  s.device = std::move(uri);
  s.rgb = false;
  s.ir = false;
  s.depth = true;
  s.pointcloud = false;
  return s;
}

/// An opened stream on the in-process synthetic camera, consuming depth.
struct Rig
{
  std::shared_ptr<InputStream> stream;
  explicit Rig(QString uri, DepthCameraSettings s = {}, bool consume = true)
  {
    if(s.device.isEmpty())
      s = settingsFor(uri);
    s.device = uri;
    stream = std::make_shared<InputStream>(syn::backend(), uri, s);
    if(consume)
      stream->addConsumer(stream->m_depth, 1);
  }
  ~Rig()
  {
    if(stream)
      stream->close();
  }
  void step(int n = 1) { syn::step(stream->device(), n); }
};

const float* depthSamples(AVFrame* f)
{
  return reinterpret_cast<const float*>(f->data[0]);
}

ossia::net::parameter_base* param(ossia::net::node_base& root, std::string_view path)
{
  auto* n = ossia::net::find_node(root, path);
  return n ? n->get_parameter() : nullptr;
}
}

// --- streams ------------------------------------------------------------------

TEST_CASE("Depth frames carry the unit their samples are in", "[depthcam][stream]")
{
  struct Case
  {
    const char* uri;
    float unit;
    AVPixelFormat fmt;
  };
  for(auto c : {Case{"synthetic:unit=1mm,clock=manual", 1.f, AV_PIX_FMT_GRAY16LE},
                Case{"synthetic:unit=0.1mm,clock=manual", 0.1f, AV_PIX_FMT_GRAY16LE},
                Case{"synthetic:unit=f32mm,clock=manual", 1.f, AV_PIX_FMT_GRAYF32LE},
                Case{"synthetic:unit=f32m,clock=manual", 1000.f, AV_PIX_FMT_GRAYF32LE}})
  {
    CAPTURE(c.uri);
    Rig r{c.uri};
    REQUIRE(r.stream->valid());
    // Nothing has arrived yet: the unit is unknown, which is the state the
    // device used to bake into its filter.
    CHECK(r.stream->m_depth.unit_mm.load() == 0.f);
    REQUIRE(r.stream->start());
    r.step();
    CHECK(r.stream->m_depth.unit_mm.load() == c.unit);
    CHECK(r.stream->depth_unit_mm.load() == c.unit);
    AVFrame* f = r.stream->m_depth.queue.dequeue();
    REQUIRE(f);
    CHECK(f->format == c.fmt);
    CHECK(f->width == 640);
    CHECK(f->height == 576);
    r.stream->m_depth.queue.release(f);
  }
}

TEST_CASE("Frames are stamped with device and host time", "[depthcam][stream]")
{
  Rig r{"synthetic:clock=manual,fps=30"};
  REQUIRE(r.stream->start());

  std::vector<StreamStats> seen;
  std::mutex m;
  r.stream->setStatsCallback([&](const StreamOutput& out, const StreamStats& st) {
    if(out.stream == DEPTHCAM_STREAM_DEPTH)
    {
      std::lock_guard l{m};
      seen.push_back(st);
    }
  });

  const int64_t first = syn::nextFrame(r.stream->device());
  r.step(4);

  // The queue keeps only the latest on dequeue; the pts says which it is.
  AVFrame* f = r.stream->m_depth.queue.dequeue();
  REQUIRE(f);
  CHECK(uint64_t(f->pts) == syn::timestampNs(first + 3, 30));
  CHECK(f->best_effort_timestamp > 0);
  r.stream->m_depth.queue.release(f);

  r.stream->setStatsCallback({});
  REQUIRE(seen.size() == 4);
  CHECK(seen.back().frames == 4);
  CHECK(seen.back().dropped == 0);
  CHECK(seen.back().device_time_ms == Approx(3 * 1000. / 30.).epsilon(1e-7));
  // 1/30 s is 33 333 333 ns on an integer clock.
  CHECK(seen.back().interval_ms == Approx(1000. / 30.).epsilon(1e-7));
  CHECK(seen.back().fps == Approx(30.).epsilon(1e-6));
  CHECK(seen.back().host_time_ms >= 0.);
}

TEST_CASE("A full queue counts its drops", "[depthcam][stream]")
{
  Rig r{"synthetic:clock=manual"};
  REQUIRE(r.stream->start());
  StreamStats last;
  r.stream->setStatsCallback([&](const StreamOutput& out, const StreamStats& st) {
    if(out.stream == DEPTHCAM_STREAM_DEPTH)
      last = st;
  });
  r.step(20);
  r.stream->setStatsCallback({});
  CHECK(last.frames == 20);
  CHECK(last.dropped == 4); // 16 queued, the rest refused
}

TEST_CASE("Every frame goes back to the backend exactly once", "[depthcam][stream]")
{
  auto& c = syn::counters();
  const auto delivered0 = c.frames_delivered.load();
  const auto released0 = c.frames_released.load();
  {
    auto s = settingsFor("synthetic:clock=manual");
    s.rgb = true;
    s.ir = true;
    s.pointcloud = true;
    s.imu = true;
    Rig r{"synthetic:clock=manual", s};
    r.stream->addConsumer(r.stream->m_rgb, 1);
    r.stream->addConsumer(r.stream->m_pcl, 1);
    REQUIRE(r.stream->start());
    r.step(25); // overflows the image queues and the point cloud one
    if(auto* f = r.stream->m_depth.queue.dequeue())
      r.stream->m_depth.queue.release(f);
    r.stream->close();
  }
  const auto delivered = c.frames_delivered.load() - delivered0;
  const auto released = c.frames_released.load() - released0;
  CHECK(delivered >= 25 * 4); // depth, colour, IR, cloud, IMU, less the IR not consumed
  CHECK(delivered == released);
}

TEST_CASE("An unconsumed stream is neither queued nor produced", "[depthcam][stream]")
{
  auto s = settingsFor("synthetic:clock=manual");
  s.pointcloud = true;
  Rig r{"synthetic:clock=manual", s, false};
  REQUIRE(r.stream->start());

  auto& c = syn::counters();
  const auto clouds0 = c.pointclouds_computed.load();
  r.step(3);
  // No consumer: the frames went straight back, and the backend was told it
  // need not compute a point cloud at all.
  CHECK(r.stream->m_depth.queue.size() == 0);
  CHECK(r.stream->m_pcl.queue.size() == 0);
  CHECK(c.pointclouds_computed.load() == clouds0);
  CHECK((r.stream->demand() & DEPTHCAM_STREAM_POINTCLOUD) == 0);

  r.stream->addConsumer(r.stream->m_pcl, 1);
  CHECK((r.stream->demand() & DEPTHCAM_STREAM_POINTCLOUD) != 0);
  r.step(2);
  CHECK(c.pointclouds_computed.load() == clouds0 + 2);
  CHECK(r.stream->m_pcl.queue.size() == 2);
  r.stream->addConsumer(r.stream->m_pcl, -1);
}

// --- calibration -----------------------------------------------------------------

TEST_CASE("Calibration is read from the camera", "[depthcam][calibration]")
{
  auto s = settingsFor("synthetic:clock=manual");
  s.rgb = true;
  s.imu = true;
  Rig r{"synthetic:clock=manual", s};
  const auto d = Calibration::query(
      *syn::backend(), *r.stream->device(), r.stream->activeStreams(), 640, 576);

  CHECK_FALSE(d.estimated);
  CHECK(d.model == "Synthetic depth camera");
  CHECK(d.serial == "SYN-0001");
  CHECK(d.firmware == "1.2.3");
  CHECK(d.hardwareRev == "A");
  CHECK(d.depthMin == 0.25f);
  CHECK(d.depthMax == 6.5f);

  const auto dc = syn::depthCamera(640, 576, false);
  const auto* in = d.intrinsics(DEPTHCAM_STREAM_DEPTH);
  REQUIRE(in);
  CHECK(in->width == 640);
  CHECK(in->height == 576);
  CHECK(in->fx == float(dc.fx));
  CHECK(in->fy == float(dc.fy));
  CHECK(in->cx == float(dc.cx));
  CHECK(in->cy == float(dc.cy));
  CHECK(in->model == DEPTHCAM_DISTORTION_NONE);

  const auto cc = syn::colorCamera(1280, 720);
  const auto* ci = d.intrinsics(DEPTHCAM_STREAM_COLOR);
  REQUIRE(ci);
  CHECK(ci->width == 1280);
  CHECK(ci->cx == float(cc.cx));
  CHECK(ci->cy == float(cc.cy));

  REQUIRE(d.depthToColor);
  const auto x = syn::depthToColor();
  const auto& m = *d.depthToColor;
  // Column-major: (row i, column j) at [4j + i].
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
      CHECK(m[4 * j + i] == float(x.r[3 * i + j]));
    CHECK(m[12 + i] == float(x.t[i]));
    CHECK(m[4 * i + 3] == 0.f);
  }
  CHECK(m[15] == 1.f);

  REQUIRE(d.depthToImu);
  CHECK(*d.depthToImu == identityMatrix());
}

TEST_CASE("Depth aligned to colour reports the colour camera", "[depthcam][calibration]")
{
  auto s = settingsFor("synthetic:clock=manual");
  s.align = DepthCameraSettings::AlignMode::DepthToColor;
  s.rgb = true;
  Rig r{"synthetic:clock=manual", s};
  const auto d = Calibration::query(
      *syn::backend(), *r.stream->device(), r.stream->activeStreams(), 0, 0);
  const auto* in = d.intrinsics(DEPTHCAM_STREAM_DEPTH);
  REQUIRE(in);
  CHECK(in->width == 1280);
  CHECK(in->height == 720);
  REQUIRE(d.depthToColor);
  CHECK(*d.depthToColor == identityMatrix());
}

TEST_CASE("The ray table inverts the lens model", "[depthcam][calibration]")
{
  for(bool distorted : {false, true})
  {
    CAPTURE(distorted);
    const auto cam = syn::depthCamera(640, 576, distorted);
    depthcam_intrinsics in{};
    in.width = cam.width;
    in.height = cam.height;
    in.fx = float(cam.fx);
    in.fy = float(cam.fy);
    in.cx = float(cam.cx);
    in.cy = float(cam.cy);
    in.model = cam.model;
    for(int i = 0; i < 6; i++)
      in.coeffs[i] = float(cam.k[i]);
    in.coeffs[6] = float(cam.p[0]);
    in.coeffs[7] = float(cam.p[1]);

    std::vector<float> xy(640 * 576 * 2);
    computeRayTable(in, 640, 576, xy.data());
    double worst = 0;
    for(int v : {0, 1, 100, 287, 288, 450, 575})
      for(int u : {0, 1, 200, 319, 320, 500, 639})
      {
        const auto truth = syn::ray(cam, u, v);
        const auto i = 2 * (std::size_t(v) * 640 + u);
        worst = std::max(worst, std::abs(xy[i] - truth[0]));
        worst = std::max(worst, std::abs(xy[i + 1] - truth[1]));
      }
    // The coefficients went through float on the way: a few ulp of a ray of
    // order 0.5.
    CHECK(worst < 2e-6);
  }
}

TEST_CASE("Intrinsics measured at another size are scaled", "[depthcam][calibration]")
{
  const auto full = syn::depthCamera(1280, 1152, false);
  depthcam_intrinsics in{};
  in.width = 1280;
  in.height = 1152;
  in.fx = float(full.fx);
  in.fy = float(full.fy);
  in.cx = float(full.cx);
  in.cy = float(full.cy);
  std::vector<float> xy(640 * 576 * 2);
  computeRayTable(in, 640, 576, xy.data());
  // A 2x binned pixel covers four full ones; its centre is their centre.
  const auto truth = syn::ray(full, 2 * 100 + 0.5, 2 * 50 + 0.5);
  const auto i = 2 * (std::size_t(50) * 640 + 100);
  CHECK(xy[i] == Approx(truth[0]).margin(1e-6));
  CHECK(xy[i + 1] == Approx(truth[1]).margin(1e-6));
}

TEST_CASE("The backend's own ray table is preferred", "[depthcam][calibration]")
{
  Rig r{"synthetic:clock=manual,distortion=bc,raytable=1"};
  Calibration calib;
  calib.set(Calibration::query(
      *syn::backend(), *r.stream->device(), r.stream->activeStreams(), 640, 576));
  calib.setRaySource(syn::backend(), r.stream->device());
  auto t = calib.depthRays(640, 576);
  REQUIRE(t);
  const auto cam = syn::depthCamera(640, 576, true);
  for(int v : {0, 288, 575})
    for(int u : {0, 320, 639})
    {
      const auto truth = syn::ray(cam, u, v);
      const auto i = 2 * (std::size_t(v) * 640 + u);
      CHECK((*t)[i] == float(truth[0]));
      CHECK((*t)[i + 1] == float(truth[1]));
    }
  // Cached until the calibration changes.
  CHECK(calib.depthRays(640, 576) == t);
  calib.set(calib.get());
  CHECK(calib.depthRays(640, 576) != t);
  calib.setRaySource(nullptr, nullptr);
}

TEST_CASE("A camera without calibration gets a labelled estimate", "[depthcam][calibration]")
{
  // The ABI 3 table: no get_intrinsics at all.
  auto t = BackendRegistry::adoptTable(syn::backend());
  REQUIRE(t);
  depthcam_backend_v1 abi3 = *t;
  abi3.get_intrinsics = nullptr;
  abi3.get_extrinsics = nullptr;
  abi3.get_details = nullptr;
  Rig r{"synthetic:clock=manual"};
  const auto d = Calibration::query(
      abi3, *r.stream->device(), r.stream->activeStreams(), 640, 576);
  CHECK(d.estimated);
  const auto* in = d.intrinsics(DEPTHCAM_STREAM_DEPTH);
  REQUIRE(in);
  CHECK(in->fx == Approx(320. / std::tan(30. * 3.14159265358979 / 180.)).epsilon(1e-6));
}

// --- the metadata tree -------------------------------------------------------------

TEST_CASE("info/, calibration/ and stats/ publish what the camera says", "[depthcam][info]")
{
  app();
  auto s = settingsFor("synthetic:clock=manual");
  s.rgb = true;
  Rig r{"synthetic:clock=manual", s};
  r.stream->addConsumer(r.stream->m_rgb, 1);

  ossia::net::generic_device dev{std::make_unique<ossia::net::multiplex_protocol>(), "cam"};
  auto calib = std::make_shared<Calibration>();
  calib->setRaySource(syn::backend(), r.stream->device());
  {
    InfoTree info{
        r.stream, calib, dev.get_root_node(), "synthetic:clock=manual", "synthetic", "memory"};
    auto& root = dev.get_root_node();

    REQUIRE(param(root, "/info/model"));
    CHECK(ossia::convert<std::string>(param(root, "/info/model")->value())
          == "Synthetic depth camera");
    CHECK(ossia::convert<std::string>(param(root, "/info/serial")->value()) == "SYN-0001");
    CHECK(ossia::convert<std::string>(param(root, "/info/backend")->value()) == "synthetic");
    CHECK(ossia::convert<ossia::vec2f>(param(root, "/info/depth_range")->value())
          == ossia::vec2f{0.25f, 6.5f});
    CHECK(ossia::convert<bool>(param(root, "/info/calibration_estimated")->value()) == false);

    const auto dc = syn::depthCamera(640, 576, false);
    REQUIRE(param(root, "/calibration/depth/intrinsics"));
    CHECK(ossia::convert<ossia::vec4f>(param(root, "/calibration/depth/intrinsics")->value())
          == ossia::vec4f{float(dc.fx), float(dc.fy), float(dc.cx), float(dc.cy)});
    CHECK(ossia::convert<int>(param(root, "/calibration/depth/width")->value()) == 640);
    CHECK(ossia::convert<std::string>(
              param(root, "/calibration/depth/distortion/model")->value())
          == "none");
    CHECK(ossia::convert<int>(param(root, "/calibration/color/width")->value()) == 1280);

    auto* d2c = param(root, "/calibration/depth_to_color");
    REQUIRE(d2c);
    const auto list = ossia::convert<std::vector<ossia::value>>(d2c->value());
    REQUIRE(list.size() == 16);
    const auto x = syn::depthToColor();
    CHECK(ossia::convert<float>(list[12]) == float(x.t[0]));
    // The same matrix by rows, for ISF's vector uniforms.
    const auto row0
        = ossia::convert<ossia::vec4f>(param(root, "/calibration/depth_to_color/row0")->value());
    CHECK(row0[0] == float(x.r[0]));
    CHECK(row0[2] == float(x.r[2]));
    CHECK(row0[3] == float(x.t[0]));

    // OpenGL clip space of the depth camera: the principal point projects to
    // the centre of the image's pixel grid, a point at the far plane to z = 1.
    auto* proj = param(root, "/calibration/depth/projection");
    REQUIRE(proj);
    info.setClipPlanes(0.5f, 5.f);
    const auto pl = ossia::convert<std::vector<ossia::value>>(proj->value());
    REQUIRE(pl.size() == 16);
    float P[16];
    for(int i = 0; i < 16; i++)
      P[i] = ossia::convert<float>(pl[i]);
    const auto project = [&](float X, float Y, float Z) {
      const float cx = P[0] * X + P[4] * Y + P[8] * Z + P[12];
      const float cy = P[1] * X + P[5] * Y + P[9] * Z + P[13];
      const float cz = P[2] * X + P[6] * Y + P[10] * Z + P[14];
      const float cw = P[3] * X + P[7] * Y + P[11] * Z + P[15];
      return std::array<float, 3>{cx / cw, cy / cw, cz / cw};
    };
    // Pixel (0, 0)'s centre is half a pixel in from the corner of NDC (-1, +1).
    const auto r00 = syn::ray(dc, 0, 0);
    const auto c00 = project(float(r00[0] * 5.), float(r00[1] * 5.), 5.f);
    CHECK(c00[0] == Approx(-1. + 1. / 640.).margin(1e-5));
    CHECK(c00[1] == Approx(1. - 1. / 576.).margin(1e-5));
    CHECK(c00[2] == Approx(1.f).margin(1e-5));
    CHECK(project(0, 0, 0.5f)[2] == Approx(-1.f).margin(1e-5));

    REQUIRE(r.stream->start());
    r.step(3);
    CHECK(ossia::convert<float>(param(root, "/info/depth_unit_mm")->value()) == 1.f);
    CHECK(ossia::convert<int>(param(root, "/stats/depth/frames")->value()) >= 1);
    CHECK(ossia::convert<float>(param(root, "/stats/depth/device_time_ms")->value())
          == Approx(2 * 1000. / 30.).epsilon(1e-6));
    CHECK(param(root, "/stats/color/fps"));
    r.stream->stop();
  }
  calib->setRaySource(nullptr, nullptr);
}

// --- controls ---------------------------------------------------------------------

namespace
{
struct ControlRig : Rig
{
  ossia::net::generic_device dev{std::make_unique<ossia::net::multiplex_protocol>(), "cam"};
  std::shared_ptr<RenderParams> params = std::make_shared<RenderParams>();
  std::unique_ptr<ControlTree> tree;
  std::vector<ControlTree::ControlValues> stores;
  int restarts{};

  explicit ControlRig(ControlTree::ControlValues saved = {})
      : Rig{"synthetic:clock=manual"}
  {
    app();
    tree = std::make_unique<ControlTree>(
        *syn::backend(), *stream->device(), stream, dev, dev.get_root_node(), params,
        std::move(saved), [this](const auto& v) { stores.push_back(v); },
        [this] { restarts++; });
    settle();
  }
  ~ControlRig() { tree.reset(); }

  void settle()
  {
    tree->waitIdle();
    pump();
  }
  ossia::net::parameter_base& p(std::string_view id)
  {
    auto* x = tree->parameter(id);
    REQUIRE(x);
    return *x;
  }
  /// As ossia does it: the parameter takes the value, then the protocol is
  /// told.
  void write(std::string_view id, ossia::value v)
  {
    p(id).set_value_quiet(v);
    REQUIRE(tree->write(p(id), v));
    settle();
  }
  double cam(const char* id)
  {
    double v{};
    REQUIRE(syn::backend()->get_control(stream->device(), id, &v));
    return v;
  }
};
}

TEST_CASE("Controls are described with their units and flags", "[depthcam][controls]")
{
  ControlRig r;
  auto& root = r.dev.get_root_node();

  // Every kind the ABI has, plus the host's render/ and actions.
  for(auto path :
      {"/controls/depth/laser_power", "/controls/depth/mode", "/controls/color/exposure",
       "/controls/color/auto_exposure", "/controls/color/gain", "/controls/device/preset",
       "/controls/filters/spatial/enabled", "/controls/sensors/temperature",
       "/controls/advanced/reboot", "/controls/render/near", "/controls/render/far",
       "/controls/render/stage_box/min", "/controls/render/edge_cull",
       "/controls/advanced/store_current_settings",
       "/controls/advanced/reset_to_defaults"})
  {
    CAPTURE(path);
    CHECK(param(root, path));
  }

  auto* laser = ossia::net::find_node(root, "/controls/depth/laser_power");
  const auto desc = ossia::net::get_description(*laser);
  REQUIRE(desc);
  CHECK(desc->find("[mW]") != std::string::npos);
  CHECK(desc->find("step 30") != std::string::npos);

  auto* mode = ossia::net::find_node(root, "/controls/depth/mode");
  const auto tags = ossia::net::get_tags(*mode);
  REQUIRE(tags);
  CHECK(std::find(tags->begin(), tags->end(), "restart") != tags->end());

  auto* spatial = ossia::net::find_node(root, "/controls/filters/spatial/alpha");
  const auto ftags = ossia::net::get_tags(*spatial);
  REQUIRE(ftags);
  CHECK(std::find(ftags->begin(), ftags->end(), "filter") != ftags->end());

  // A unit ossia knows is set as one.
  CHECK(param(root, "/controls/render/near")->get_unit());
  CHECK(param(root, "/controls/render/near")->get_unit()
        == ossia::parse_pretty_unit("distance.m"));

  // Enums read and write by label.
  CHECK(ossia::convert<std::string>(r.p("device/preset").value()) == "Default");
}

TEST_CASE("Writes are read back from the camera", "[depthcam][controls]")
{
  ControlRig r;
  // 155 is not a step of 30: the camera rounds it, and the tree shows that.
  r.write("depth/laser_power", 155);
  CHECK(r.cam("depth/laser_power") == 150.);
  CHECK(ossia::convert<int>(r.p("depth/laser_power").value()) == 150);

  // By label or by value.
  r.write("device/preset", std::string{"High density"});
  CHECK(r.cam("device/preset") == 2.);
  r.write("device/preset", 0);
  CHECK(r.cam("device/preset") == 0.);
}

TEST_CASE("An auto toggle makes what it rules read-only", "[depthcam][controls]")
{
  ControlRig r;
  // Auto exposure starts on.
  CHECK(r.p("color/exposure").get_access() == ossia::access_mode::GET);
  const auto calls0 = syn::counters().set_control_calls.load();
  r.write("color/exposure", 2000);
  // Not even sent: it would be refused.
  CHECK(syn::counters().set_control_calls.load() == calls0);

  r.write("color/auto_exposure", false);
  CHECK(r.p("color/exposure").get_access() == ossia::access_mode::BI);
  r.write("color/exposure", 2000);
  CHECK(r.cam("color/exposure") == 2000.);
  CHECK(ossia::convert<int>(r.p("color/exposure").value()) == 2000);

  r.write("color/auto_exposure", true);
  CHECK(r.p("color/exposure").get_access() == ossia::access_mode::GET);
}

TEST_CASE("A control that needs a restart gets one", "[depthcam][controls]")
{
  ControlRig r;
  REQUIRE(r.stream->start());
  const auto starts0 = syn::counters().starts.load();
  const auto stops0 = syn::counters().stops.load();
  r.write("depth/mode", std::string{"512x512"});
  CHECK(syn::counters().stops.load() == stops0 + 1);
  CHECK(syn::counters().starts.load() == starts0 + 1);
  CHECK(r.restarts == 1);
  CHECK(r.stream->running());

  r.step();
  AVFrame* f = r.stream->m_depth.queue.dequeue();
  REQUIRE(f);
  CHECK(f->width == 512);
  CHECK(f->height == 512);
  r.stream->m_depth.queue.release(f);

  // And the calibration follows.
  const auto d = Calibration::query(
      *syn::backend(), *r.stream->device(), r.stream->activeStreams(), 0, 0);
  CHECK(d.intrinsics(DEPTHCAM_STREAM_DEPTH)->width == 512);
  r.stream->stop();
}

TEST_CASE("A camera reporting a change is re-read", "[depthcam][controls]")
{
  ControlRig r;
  CHECK(ossia::convert<int>(r.p("depth/laser_power").value()) == 150);
  // The preset rewrites the laser power on its own and says so.
  r.write("device/preset", std::string{"High accuracy"});
  r.settle();
  CHECK(r.cam("depth/laser_power") == 300.);
  CHECK(ossia::convert<int>(r.p("depth/laser_power").value()) == 300);
}

TEST_CASE("render/ controls drive the processing parameters", "[depthcam][controls]")
{
  ControlRig r;
  r.write("render/near", 0.5f);
  r.write("render/far", 2.5f);
  r.write("render/stage_box/enabled", true);
  r.write("render/stage_box/min", ossia::vec3f{-0.5f, -0.4f, 1.f});
  r.write("render/topology", std::string{"points"});
  r.write("render/temporal", 2.f); // clamped
  CHECK(r.params->near_m.load() == 0.5f);
  CHECK(r.params->far_m.load() == 2.5f);
  CHECK(r.params->box_enabled.load());
  CHECK(r.params->box_min[1].load() == -0.4f);
  CHECK(r.params->topology.load() == 1);
  CHECK(r.params->temporal.load() == 0.99f);
}

TEST_CASE("What is written is recorded, in order, and replayed", "[depthcam][controls]")
{
  ControlTree::ControlValues record;
  {
    ControlRig r;
    r.write("color/auto_exposure", false);
    r.write("color/exposure", 3000);
    r.write("depth/laser_power", 240);
    r.write("render/near", 0.75f);
    r.write("color/exposure", 2500); // moves to the end
    r.write("advanced/reboot", ossia::impulse{}); // an action is not a setting
    REQUIRE_FALSE(r.stores.empty());
    record = r.stores.back();
    CHECK(record == r.tree->stored());
  }
  REQUIRE(record.size() == 4);
  CHECK(record[0].id == "color/auto_exposure");
  CHECK(record[1].id == "depth/laser_power");
  CHECK(record[2].id == "render/near");
  CHECK(record[3].id == "color/exposure");
  CHECK(record[3].value == std::vector<double>{2500.});
  CHECK(record[2].value[0] == Approx(0.75));

  // A fresh camera: the record is replayed before the stream may start.
  ControlRig r2{record};
  REQUIRE(r2.tree->replayed().wait_for(std::chrono::seconds(5)) == std::future_status::ready);
  CHECK(r2.cam("color/auto_exposure") == 0.);
  CHECK(r2.cam("color/exposure") == 2500.);
  CHECK(r2.cam("depth/laser_power") == 240.);
  CHECK(r2.params->near_m.load() == 0.75f);
  CHECK(ossia::convert<float>(r2.p("render/near").value()) == 0.75f);
  CHECK(r2.p("color/exposure").get_access() == ossia::access_mode::BI);
}

TEST_CASE("Store and reset", "[depthcam][controls]")
{
  ControlRig r;
  // Changed behind the tree's back, as another program would.
  syn::backend()->set_control(r.stream->device(), "color/gain", 0.8);
  r.write("advanced/store_current_settings", ossia::impulse{});
  r.settle();
  auto s = r.tree->stored();
  const auto gain = std::find_if(s.begin(), s.end(), [](auto& c) { return c.id == "color/gain"; });
  REQUIRE(gain != s.end());
  CHECK(gain->value[0] == Approx(0.8));
  // Under auto, the exposure is the camera's choice, not a setting.
  CHECK(std::none_of(s.begin(), s.end(), [](auto& c) { return c.id == "color/exposure"; }));
  CHECK(std::any_of(s.begin(), s.end(), [](auto& c) { return c.id == "render/edge_cull"; }));

  r.write("render/near", 1.f);
  r.write("advanced/reset_to_defaults", ossia::impulse{});
  r.settle();
  CHECK(r.cam("color/gain") == Approx(0.5));
  CHECK(r.params->near_m.load() == 0.f);
  CHECK(r.tree->stored().empty());
  REQUIRE_FALSE(r.stores.empty());
  CHECK(r.stores.back().empty());
}

// --- ABI versions ---------------------------------------------------------------------

TEST_CASE("An ABI 3 table is copied without reading past its end", "[depthcam][abi]")
{
  CHECK(BackendRegistry::tableSize(3) == offsetof(depthcam_backend_v1, get_intrinsics));
  CHECK(BackendRegistry::tableSize(4) == sizeof(depthcam_backend_v1));
  CHECK(BackendRegistry::tableSize(2) == 0);
  CHECK(BackendRegistry::tableSize(5) == 0);

  // Only the ABI 3 prefix exists: a heap block of exactly that size, with a
  // tail an overread would show up in under a sanitizer.
  const std::size_t n = BackendRegistry::tableSize(3);
  std::vector<unsigned char> bytes(n);
  std::memcpy(bytes.data(), syn::backend(), n);
  auto* v3 = reinterpret_cast<depthcam_backend_v1*>(bytes.data());
  v3->abi_version = 3;

  auto t = BackendRegistry::adoptTable(v3);
  REQUIRE(t);
  CHECK(t->abi_version == 3);
  CHECK(t->open == syn::backend()->open);
  CHECK(t->active_streams == syn::backend()->active_streams);
  CHECK(t->get_intrinsics == nullptr);
  CHECK(t->list_controls2 == nullptr);
  CHECK(t->set_stream_demand == nullptr);

  depthcam_backend_v1 future = *syn::backend();
  future.abi_version = DEPTHCAM_ABI_VERSION + 1;
  CHECK_FALSE(BackendRegistry::adoptTable(&future));
}

TEST_CASE("ABI 3 and ABI 4 backends load from disk", "[depthcam][abi]")
{
#if defined(SCORE_DEPTHCAM_TEST_BACKEND_DIR)
  qputenv("SCORE_DEPTHCAM_BACKEND_PATH", SCORE_DEPTHCAM_TEST_BACKEND_DIR);
  auto& reg = BackendRegistry::instance();
  reg.unload();
  reg.load();

  const auto* v4 = reg.find("synthetic");
  const auto* v3 = reg.find("synthetic_abi3");
  REQUIRE(v4);
  REQUIRE(v3);
  CHECK(v4->abi_version == DEPTHCAM_ABI_VERSION);
  CHECK(v3->abi_version == 3);
  CHECK(v4->get_intrinsics);
  CHECK(v3->get_intrinsics == nullptr);

  // And the old one still works end to end, with an estimated calibration.
  auto s = settingsFor("synthetic_abi3:clock=manual");
  InputStream stream{v3, "synthetic_abi3:clock=manual", s};
  REQUIRE(stream.valid());
  stream.addConsumer(stream.m_depth, 1);
  REQUIRE(stream.start());
  // No step(): that is a hook of the in-process copy. A manual clock only
  // ever delivers what it is asked for, so wait for nothing and stop.
  const auto d = Calibration::query(*v3, *stream.device(), stream.activeStreams(), 640, 576);
  CHECK(d.estimated);
  stream.close();
  reg.unload();
#else
  SKIP("no test backend directory");
#endif
}
