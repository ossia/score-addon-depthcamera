// The device inside score: a document gets a Depth Camera Input on the
// synthetic camera, loaded as a backend module the way a user's package is.
// Its tree, a control written through ossia ending up in the document, that
// surviving a save and a reload, and the settings dialog's mode lists.
#include <DepthCamera/DepthCameraSettings.hpp>

#include <Device/Protocol/DeviceInterface.hpp>
#include <Device/Protocol/ProtocolFactoryInterface.hpp>
#include <Device/Protocol/ProtocolList.hpp>
#include <Device/Protocol/ProtocolSettingsWidget.hpp>
#include <Explorer/Commands/Add/LoadDevice.hpp>
#include <Explorer/DeviceList.hpp>
#include <Explorer/DocumentPlugin/DeviceDocumentPlugin.hpp>

#include <score/command/Dispatchers/CommandDispatcher.hpp>

#include <core/command/CommandStack.hpp>
#include <core/document/Document.hpp>

#include <ossia/network/base/device.hpp>
#include <ossia/network/base/node_functions.hpp>
#include <ossia/network/base/parameter.hpp>
#include <ossia/network/common/complex_type.hpp>

#include <score_test/App.hpp>
#include <score_test/Document.hpp>

#include <QApplication>
#include <QComboBox>
#include <QElapsedTimer>

#include <catch2/catch_test_macros.hpp>

#include <set>

using namespace Gfx::DepthCamera;

namespace
{
const Device::ProtocolFactory* depthFactory(const score::GUIApplicationContext& ctx)
{
  for(auto& f : ctx.interfaces<Device::ProtocolFactoryList>())
    if(f.prettyName() == QStringLiteral("Depth Camera Input"))
      return &f;
  return nullptr;
}

Device::DeviceSettings cameraSettings(const Device::ProtocolFactory& f)
{
  Device::DeviceSettings s;
  s.name = "cam";
  s.protocol = f.concreteKey();
  DepthCameraSettings d;
  d.device = "synthetic:clock=manual";
  d.rgb = true;
  d.ir = true;
  d.imu = true;
  d.positions = true;
  d.normals = true;
  d.mesh = true;
  s.deviceSpecificSettings = QVariant::fromValue(d);
  return s;
}

void waitFor(const std::function<bool()>& f, int ms = 3000)
{
  QElapsedTimer t;
  t.start();
  while(!f() && t.elapsed() < ms)
    QApplication::processEvents();
}

const Device::Node* deviceNode(Explorer::DeviceDocumentPlugin& plug, const QString& name)
{
  for(auto& n : plug.rootNode())
    if(n.get<Device::DeviceSettings>().name == name)
      return &n;
  return nullptr;
}

ossia::net::parameter_base* param(Device::DeviceInterface& dev, std::string_view path)
{
  auto* d = dev.getDevice();
  if(!d)
    return nullptr;
  auto* n = ossia::net::find_node(d->get_root_node(), path);
  return n ? n->get_parameter() : nullptr;
}
}

TEST_CASE("A depth camera device in a document", "[depthcam][device]")
{
#if defined(SCORE_DEPTHCAM_TEST_BACKEND_DIR)
  qputenv("SCORE_DEPTHCAM_BACKEND_PATH", SCORE_DEPTHCAM_TEST_BACKEND_DIR);
#endif
  score::test::run_in_app([](const score::GUIApplicationContext& ctx) {
    const auto* factory = depthFactory(ctx);
    REQUIRE(factory);

    auto* doc = score::test::new_document(ctx);
    REQUIRE(doc);
    auto& plug = doc->context().plugin<Explorer::DeviceDocumentPlugin>();
    CommandDispatcher<>{doc->context().commandStack}.submit(
        new Explorer::Command::LoadDevice{plug, Device::Node{cameraSettings(*factory), nullptr}});
    QApplication::processEvents();

    auto* dev = plug.list().findDevice("cam");
    REQUIRE(dev);
    REQUIRE(dev->connected());

    // The streams, the GPU outputs, the metadata and the controls.
    std::set<QString> names;
    if(auto* n = deviceNode(plug, "cam"))
      for(auto& child : *n)
        names.insert(child.displayName());
    for(auto expected : {"rgb", "ir", "depth", "positions", "normals", "mesh", "pointcloud",
                         "imu", "info", "calibration", "stats", "controls"})
    {
      CAPTURE(expected);
      CHECK(names.count(QString::fromLatin1(expected)) == 1);
    }

    auto* fx = param(*dev, "/calibration/depth/intrinsics");
    REQUIRE(fx);
    CHECK(ossia::convert<ossia::vec4f>(fx->value())[0] == 512.f); // 0.8 x 640
    auto* model = param(*dev, "/info/model");
    REQUIRE(model);
    CHECK(ossia::convert<std::string>(model->value()) == "Synthetic depth camera");

    // A write, as an OSC message or the explorer would make it: through the
    // parameter, so through the protocol.
    auto* laser = param(*dev, "/controls/depth/laser_power");
    REQUIRE(laser);
    laser->push_value(240);
    auto* near = param(*dev, "/controls/render/near");
    REQUIRE(near);
    near->push_value(0.6f);

    // The record reaches the document's own copy of the settings.
    const auto saved = [&] {
      const auto* n = deviceNode(plug, "cam");
      return n ? n->get<Device::DeviceSettings>()
                     .deviceSpecificSettings.value<DepthCameraSettings>()
                     .controls
               : std::vector<DepthCameraSettings::ControlValue>{};
    };
    waitFor([&] { return saved().size() == 2; });
    const auto rec = saved();
    REQUIRE(rec.size() == 2);
    CHECK(rec[0].id == "depth/laser_power");
    CHECK(rec[0].value == std::vector<double>{240.});
    CHECK(rec[1].id == "render/near");

    // Saved and loaded again: the controls are replayed on the new device.
    auto* reloaded = score::test::reload_via_json(ctx, *doc);
    REQUIRE(reloaded);
    auto& plug2 = reloaded->context().plugin<Explorer::DeviceDocumentPlugin>();
    auto* dev2 = plug2.list().findDevice("cam");
    REQUIRE(dev2);
    REQUIRE(dev2->connected());
    auto* laser2 = param(*dev2, "/controls/depth/laser_power");
    REQUIRE(laser2);
    waitFor([&] { return ossia::convert<int>(laser2->value()) == 240; });
    CHECK(ossia::convert<int>(laser2->value()) == 240);
    auto* near2 = param(*dev2, "/controls/render/near");
    REQUIRE(near2);
    CHECK(ossia::convert<float>(near2->value()) == 0.6f);
    const auto s2 = deviceNode(plug2, "cam")
                        ->get<Device::DeviceSettings>()
                        .deviceSpecificSettings.value<DepthCameraSettings>();
    CHECK(s2.depthOutput == DepthCameraSettings::DepthOutput::Metres);
    CHECK(s2.mesh);

    // The dialog lists the modes the backend reports.
    std::unique_ptr<Device::ProtocolSettingsWidget> w{
        const_cast<Device::ProtocolFactory*>(factory)->makeSettingsWidget()};
    REQUIRE(w);
    w->setSettings(cameraSettings(*factory));
    int lists = 0, depthModes = 0;
    for(auto* box : w->findChildren<QComboBox*>())
    {
      if(box->isHidden() || box->count() < 2)
        continue;
      for(int i = 0; i < box->count(); i++)
        if(box->itemText(i).startsWith(QStringLiteral("512")))
          depthModes++;
      if(box->itemText(1).contains(QStringLiteral("fps")))
        lists++;
    }
    CHECK(lists == 3);    // colour, infrared, depth
    CHECK(depthModes >= 2); // 512x512 at 15 and 30, for IR and depth
  });
}
