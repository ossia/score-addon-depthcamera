// Settings serialization, through score's own visitors, which need an
// application context: old documents and old undo/clipboard blobs still load,
// and new fields survive both formats.
#include <DepthCamera/DepthCameraSettings.hpp>

#include <score/serialization/DataStreamVisitor.hpp>
#include <score/serialization/JSONVisitor.hpp>

#include <score_test/App.hpp>

#include <QDataStream>

#include <catch2/catch_test_macros.hpp>

using namespace Gfx::DepthCamera;

// --- settings -----------------------------------------------------------------------

namespace
{
// As Device::ProtocolFactory does it (score::serialize_dyn): read() straight
// into the stream or the already-open object.
DepthCameraSettings roundTripDataStream(const DepthCameraSettings& in)
{
  QByteArray buf;
  {
    DataStreamReader rd{&buf};
    rd.read(in);
  }
  return DataStreamWriter::unmarshall<DepthCameraSettings>(buf);
}

QByteArray toJson(const DepthCameraSettings& in)
{
  JSONReader r;
  r.stream.StartObject();
  r.read(in);
  r.stream.EndObject();
  return r.toByteArray();
}

DepthCameraSettings fromJson(const QByteArray& json)
{
  const auto doc = readJson(json);
  return JSONWriter::unmarshall<DepthCameraSettings>(doc);
}
}

TEST_CASE("Settings round-trip", "[depthcam][settings]")
{
  score::test::run_in_app([](const score::GUIApplicationContext&) {
  DepthCameraSettings s;
  s.device = "synthetic:unit=0.1mm";
  s.depthOutput = DepthCameraSettings::DepthOutput::Metres;
  s.positions = true;
  s.mesh = true;
  s.depthWidth = 512;
  s.storeControl("color/auto_exposure", {0});
  s.storeControl("render/stage_box/min", {-1, -0.5, 0.25});
  s.storeControl("color/exposure", {1200});

  const auto ds = roundTripDataStream(s);
  CHECK(ds.device == s.device);
  CHECK(ds.depthOutput == s.depthOutput);
  CHECK(ds.positions);
  CHECK(ds.mesh);
  CHECK_FALSE(ds.normals);
  CHECK(ds.depthWidth == 512);
  CHECK(ds.controls == s.controls);

  const auto js = fromJson(toJson(s));
  CHECK(js.device == s.device);
  CHECK(js.depthOutput == s.depthOutput);
  CHECK(js.positions);
  CHECK(js.controls == s.controls);
  });
}

TEST_CASE("Settings from before versioning still load", "[depthcam][settings]")
{
  score::test::run_in_app([](const score::GUIApplicationContext&) {
  // Exactly what the version-0 writer produced.
  QByteArray old;
  {
    QDataStream st{&old, QIODevice::WriteOnly};
    st << QString{"orbbec:sn:CL8L1234"} << true << true << true << false << false
       << int32_t(2) << 1280 << 720 << 30 << 640 << 576 << 30 << true << 0 << 0 << 0
       << int32_t(0xDEADBEEF);
  }
  const auto s = DataStreamWriter::unmarshall<DepthCameraSettings>(old);
  CHECK(s.device == "orbbec:sn:CL8L1234");
  CHECK(s.ir);
  CHECK_FALSE(s.pointcloud);
  CHECK(s.align == DepthCameraSettings::AlignMode::ColorToDepth);
  CHECK(s.colorWidth == 1280);
  CHECK(s.depthHeight == 576);
  CHECK(s.imu);
  // Old documents rendered the preview and keep doing so.
  CHECK(s.depthOutput == DepthCameraSettings::DepthOutput::Preview);
  CHECK(s.controls.empty());

  // Same for JSON without the key.
  const auto j = fromJson(QByteArray{R"({"Device":"k4a:index:0","Depth":true})"});
  CHECK(j.device == "k4a:index:0");
  CHECK(j.depthOutput == DepthCameraSettings::DepthOutput::Preview);

  // A new device defaults to metres.
  CHECK(DepthCameraSettings{}.depthOutput == DepthCameraSettings::DepthOutput::Metres);
  });
}

