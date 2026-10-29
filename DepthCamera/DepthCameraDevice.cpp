#include "DepthCameraDevice.hpp"

#include <Gfx/GfxApplicationPlugin.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>

#include <DepthCamera/ApplicationPlugin.hpp>
#include <DepthCamera/Backend/BackendRegistry.hpp>
#include <DepthCamera/DepthCameraProtocol.hpp>
#include <DepthCamera/DepthCameraStream.hpp>

#include <Explorer/DocumentPlugin/DeviceDocumentPlugin.hpp>
#include <Explorer/Explorer/DeviceExplorerModel.hpp>
#include <State/Widgets/AddressFragmentLineEdit.hpp>

#include <wobjectimpl.h>

#include <map>

namespace Gfx::DepthCamera
{

class InputDevice final : public Gfx::GfxInputDevice
{
  W_OBJECT(InputDevice)
public:
  InputDevice(const Device::DeviceSettings& settings, const score::DocumentContext& ctx)
      : GfxInputDevice{settings, ctx}
  {
    // Depth cameras expose a point cloud next to their image streams
    m_capas.nodeKinds |= Device::NodeKind::GeometryIn;
  }
  ~InputDevice();

private:
  bool reconnect() override;
  void disconnect() override;
  Device::Node refresh() override;
  ossia::net::device_base* getDevice() const override { return m_dev.get(); }

  std::unique_ptr<ossia::net::device_base> m_dev;
};

}

W_OBJECT_IMPL(Gfx::DepthCamera::InputDevice)

namespace Gfx::DepthCamera
{

InputDevice::~InputDevice()
{
  disconnect();
}

Device::Node InputDevice::refresh()
{
  Device::Node score_device{settings(), nullptr};

  auto dev = getDevice();
  if(!dev)
    return score_device;

  // Device::ToDeviceExplorer rather than GfxInputDevice::refresh().
  //
  // The gfx one copies only the node names, which is all a device with nothing
  // but texture and geometry outputs has to say. The settings tree does have
  // more to say -- a type, a range, an access mode, a current value -- and
  // without those the explorer shows the controls as bare branches with no
  // editor. The shared version reads all of it, and it skips parameters whose
  // type has no editable value, so the streams still come out as before.
  const auto& children = dev->get_root_node().children();
  score_device.reserve(children.size());
  for(const auto& node : children)
    score_device.push_back(Device::ToDeviceExplorer(*node));

  score_device.get<Device::DeviceSettings>().name
      = QString::fromStdString(dev->get_name());
  return score_device;
}

void InputDevice::disconnect()
{
  // Before the base class, which clears the whole node tree: the settings tree
  // holds raw pointers to the parameters that live in it.
  if(auto* dev = static_cast<depthcam_device_impl*>(m_dev.get()))
    dev->releaseControls();

  GfxInputDevice::disconnect();

  // And then let the device go. Device::DeviceInterface::disconnect() only
  // clears the node tree, which leaves an object that reports itself connected
  // (connected() is getDevice() != nullptr) while having no streams at all, and
  // -- the part that actually bites -- still holding the camera open.
  //
  // That is what made an Azure Kinect show no picture until the user hit
  // Reconnect once. reconnect() calls this, then opens the camera before
  // assigning over m_dev, so the old handle was still there: k4a_device_open
  // returns LIBUSB_ERROR_BUSY for a device this process already has open, the
  // new device was never assigned, and reconnect() went on to return
  // connected() == true because the *old* one was still sitting in m_dev.
  // Score believed it had a camera; the camera had no nodes.
  m_dev.reset();
}

bool InputDevice::reconnect()
{
  disconnect();

  try
  {
    const auto set
        = this->settings().deviceSpecificSettings.value<DepthCameraSettings>();

    auto plug = m_ctx.findPlugin<Gfx::DocumentPlugin>();
    if(!plug)
      return false;

    auto& registry = BackendRegistry::instance();
    registry.load();

    // An empty address means "whatever is there": fall back to the first
    // backend that reports a camera, so a document with no address still works
    // on a machine with one camera.
    QString uri = set.device.trimmed();
    if(uri.isEmpty())
    {
      registry.enumerate([&](const depthcam_device_info& d) {
        if(uri.isEmpty() && d.uri)
          uri = QString::fromUtf8(d.uri);
      });
      if(uri.isEmpty())
      {
        qDebug() << "[depthcam] no camera found";
        return false;
      }
    }

    const auto backend_name = BackendRegistry::backendOf(uri);
    const auto* backend = registry.find(backend_name);
    if(!backend)
    {
      qDebug() << "[depthcam] no backend named" << backend_name
               << "is installed; cannot open" << uri;
      return false;
    }

    auto proto = std::make_unique<depthcam_protocol>(backend, uri, set);
    if(!proto->stream || !proto->stream->valid())
      return false;

    // What the user writes to the camera goes into the device's settings, and
    // so into the document; the next open replays it. Both copies: the
    // document saves the explorer's.
    auto onStore = [this](const ControlTree::ControlValues& values) {
      auto s = m_settings.deviceSpecificSettings.value<DepthCameraSettings>();
      if(s.controls == values)
        return;
      s.controls = values;
      m_settings.deviceSpecificSettings = QVariant::fromValue(s);
      if(auto* plug = m_ctx.findPlugin<Explorer::DeviceDocumentPlugin>())
        plug->explorer().updateDevice(m_settings.name, m_settings);
    };

    m_dev = std::make_unique<depthcam_device_impl>(
        set, plug->exec, std::move(proto), this->settings().name.toStdString(),
        std::move(onStore));
  }
  catch(const std::exception& e)
  {
    qDebug() << "[depthcam] could not connect:" << e.what();
  }
  catch(...)
  {
    qDebug() << "[depthcam] could not connect: unknown error";
  }

  return connected();
}

QString InputFactory::prettyName() const noexcept
{
  return QObject::tr("Depth Camera Input");
}

QString InputFactory::category() const noexcept
{
  return StandardCategories::video_in;
}

QUrl InputFactory::manual() const noexcept
{
  return QUrl("https://ossia.io/score-docs/devices/depth-camera-device.html");
}

Device::DeviceInterface* InputFactory::makeDevice(
    const Device::DeviceSettings& settings, const Explorer::DeviceDocumentPlugin& plugin,
    const score::DocumentContext& ctx)
{
  return new InputDevice(settings, ctx);
}

const Device::DeviceSettings& InputFactory::defaultSettings() const noexcept
{
  static const Device::DeviceSettings settings = [] {
    Device::DeviceSettings s;
    s.protocol = static_concreteKey();
    s.name = "Camera";
    s.deviceSpecificSettings = QVariant::fromValue(DepthCameraSettings{});
    return s;
  }();
  return settings;
}

QVariant
InputFactory::makeProtocolSpecificSettings(const VisitorVariant& visitor) const
{
  return makeProtocolSpecificSettings_T<DepthCameraSettings>(visitor);
}

void InputFactory::serializeProtocolSpecificSettings(
    const QVariant& data, const VisitorVariant& visitor) const
{
  serializeProtocolSpecificSettings_T<DepthCameraSettings>(data, visitor);
}

/**
 * @brief Lists the cameras one backend knows about.
 *
 * One enumerator per backend rather than per transport: the device browser turns
 * each into a category header, and a category per USB speed produced eight
 * permanently empty ones.
 */
class DepthCameraEnumerator final : public Device::DeviceEnumerator
{
public:
  DepthCameraEnumerator(const score::GUIApplicationContext& ctx, QString backend)
      : m_plugin{ctx.guiApplicationPlugin<DepthCamera::ApplicationPlugin>()}
      , m_backend{std::move(backend)}
  {
    connect(
        &m_plugin, &ApplicationPlugin::deviceAdded, this,
        [this](const DeviceInfo& dev) {
      if(dev.backend != m_backend)
        return;
      const auto name = m_plugin.uniqueName(dev);
      m_names[dev.uri] = name;
      deviceAdded(name, settingsFor(dev));
    });

    connect(
        &m_plugin, &ApplicationPlugin::deviceRemoved, this, [this](const QString& uri) {
      // Removal is by display name, so we have to remember what we announced:
      // by the time it arrives the camera is gone and cannot be queried.
      if(auto it = m_names.find(uri); it != m_names.end())
      {
        deviceRemoved(it->second);
        m_names.erase(it);
      }
    });
  }

  Device::DeviceSettings settingsFor(const DeviceInfo& dev) const
  {
    Device::DeviceSettings set;
    set.name = m_plugin.uniqueName(dev);
    set.protocol = InputFactory::static_concreteKey();

    DepthCameraSettings specif;
    specif.device = dev.uri;
    set.deviceSpecificSettings = QVariant::fromValue(specif);
    return set;
  }

  void enumerate(std::function<void(const QString&, const Device::DeviceSettings&)> f)
      const override
  {
    for(const auto& dev : m_plugin.devices())
    {
      if(dev.backend != m_backend)
        continue;
      const auto name = m_plugin.uniqueName(dev);
      const_cast<DepthCameraEnumerator*>(this)->m_names[dev.uri] = name;
      f(name, settingsFor(dev));
    }
  }

private:
  const ApplicationPlugin& m_plugin;
  QString m_backend;
  std::map<QString, QString> m_names;
};

Device::DeviceEnumerators
InputFactory::getEnumerators(const score::DocumentContext& ctx) const
{
  auto& registry = BackendRegistry::instance();
  registry.load();

  // Called each time the device browser shows this protocol: backends without
  // hot-plug only see newly plugged cameras by enumerating again.
  if(registry.needsRescanOnDemand())
    ctx.app.guiApplicationPlugin<ApplicationPlugin>().rescan();

  Device::DeviceEnumerators es;
  for(const auto& b : registry.backends())
    es.push_back(
        {b.display_name, new DepthCameraEnumerator{ctx.app, b.name}});
  return es;
}

Device::ProtocolSettingsWidget* InputFactory::makeSettingsWidget()
{
  return new InputSettingsWidget;
}

InputSettingsWidget::InputSettingsWidget(QWidget* parent)
    : Device::ProtocolSettingsWidget{parent}
{
  auto layout = new QFormLayout;

  m_deviceNameEdit = new State::AddressFragmentLineEdit{this};
  checkForChanges(m_deviceNameEdit);
  layout->addRow(tr("Device name"), m_deviceNameEdit);

  // Editable on purpose: picking a camera in the browser fills this in, and
  // typing one by hand is simply the case where enumeration was skipped -- which
  // is how a networked camera, or one that is not broadcasting, gets connected.
  m_address = new QLineEdit{this};
  m_address->setPlaceholderText(
      tr("empty for the first camera, or e.g. orbbec:sn:SERIAL, "
         "orbbec:net:192.168.0.12:8090, freenect2:sn:SERIAL, k4a:index:0"));
  checkForChanges(m_address);
  layout->addRow(tr("Camera"), m_address);

  // Address by host and port, without having to know the URI syntax.
  //
  // Discovery is a GVCP broadcast, and the reply comes back addressed to
  // 255.255.255.255 -- the firmware ignores the "unicast acknowledge" flag -- so
  // any host firewall that drops broadcast input hides every networked camera on
  // the segment while the cameras themselves are answering perfectly well. ufw
  // does exactly that by default. A camera on another subnet is out of reach of
  // a broadcast regardless. Either way, typing the address is the way in.
  {
    m_network = new QCheckBox{tr("Network"), this};

    // Two camera families are reachable over the network and they are addressed
    // differently: an Orbbec Femto Mega speaks GVCP on a port, a Luxonis OAK
    // PoE is reached by address alone over XLink. The uri grammar differs with
    // it, so the family has to be picked rather than guessed.
    m_networkKind = new QComboBox{this};
    m_networkKind->addItem(tr("Orbbec"), QStringLiteral("orbbec"));
    m_networkKind->addItem(tr("Luxonis OAK"), QStringLiteral("depthai"));
    m_networkKind->setMaximumWidth(130);

    m_networkHost = new QLineEdit{this};
    m_networkHost->setPlaceholderText(tr("192.168.0.12"));
    m_networkPort = new QSpinBox{this};
    m_networkPort->setRange(1, 65535);
    m_networkPort->setValue(8090);
    m_networkPort->setPrefix(tr("port "));
    m_networkPort->setMaximumWidth(90);

    auto row = new QWidget{this};
    auto h = new QHBoxLayout{row};
    h->setContentsMargins(0, 0, 0, 0);
    h->addWidget(m_network);
    h->addWidget(m_networkKind);
    h->addWidget(m_networkHost, 1);
    h->addWidget(m_networkPort);
    layout->addRow(row);
  }
  connect(
      m_network, &QCheckBox::toggled, this, &Device::ProtocolSettingsWidget::changed);
  checkForChanges(m_networkHost);
  checkForChanges(m_networkPort);

  connect(m_network, &QCheckBox::toggled, this, [this] { updateEnabledState(); });
  connect(
      m_networkKind, &QComboBox::currentIndexChanged, this,
      [this] { updateEnabledState(); });
  connect(
      m_networkKind, &QComboBox::currentIndexChanged, this,
      &Device::ProtocolSettingsWidget::changed);
  connect(
      m_networkHost, &QLineEdit::textChanged, this, [this] { updateEnabledState(); });
  connect(
      m_networkPort, &QSpinBox::valueChanged, this, [this] { updateEnabledState(); });

  // Say so rather than presenting an empty browser and letting the user wonder.
  if(BackendRegistry::instance().backends().empty())
  {
    auto warn = new QLabel{
        tr("No camera backend installed. Install one in the package manager."),
        this};
    warn->setWordWrap(true);
    layout->addRow(warn);
  }

  // Checkboxes on two rows rather than a group box with one per line. There are
  // six of them and they are all the same kind of thing; stacking them turned a
  // short dialog into a scrolling one.
  const auto checkRow = [&](std::initializer_list<QCheckBox**> boxes,
                            std::initializer_list<const char*> names) {
    auto row = new QWidget{this};
    auto h = new QHBoxLayout{row};
    h->setContentsMargins(0, 0, 0, 0);
    auto name = names.begin();
    for(auto* box : boxes)
    {
      *box = new QCheckBox{tr(*name++), this};
      h->addWidget(*box);
    }
    h->addStretch(1);
    return row;
  };

  layout->addRow(
      tr("Streams"),
      checkRow({&m_rgb, &m_ir, &m_depth}, {"Color", "Infrared", "Depth"}));

  // Not every camera has an IMU -- a Kinect v2 does not, a Kinect v1 has only an
  // accelerometer -- and one that does not simply publishes no nodes for it.
  layout->addRow(
      QString{}, checkRow(
                     {&m_pointcloud, &m_colorPointcloud, &m_imu},
                     {"Point cloud", "Colored", "Accel / gyro"}));

  // What the depth texture holds. Metres keeps the sensor's precision; the
  // preview is the 0-4.5 m grey that documents from before this setting show.
  m_depthOutput = new QComboBox{this};
  m_depthOutput->addItem(
      tr("Metres (32-bit float)"), int(DepthCameraSettings::DepthOutput::Metres));
  m_depthOutput->addItem(
      tr("Preview (grey, 0 to 4.5 m)"), int(DepthCameraSettings::DepthOutput::Preview));
  checkForChanges(m_depthOutput);
  layout->addRow(tr("Depth output"), m_depthOutput);

  // Computed on the GPU from the depth stream: no CPU point cloud involved.
  layout->addRow(
      tr("GPU geometry"),
      checkRow(
          {&m_positions, &m_normals, &m_mesh}, {"Positions", "Normals", "Mesh"}));

  // Alignment decides the point cloud's resolution, which dominates its cost:
  // on a Femto Mega, depth-to-colour is 2.07M points per frame against 369k the
  // other way. Which trade-off is right depends on the piece.
  m_align = new QComboBox{this};
  m_align->addItem(tr("None"), int(DepthCameraSettings::AlignMode::None));
  m_align->addItem(
      tr("Depth to color (color resolution)"),
      int(DepthCameraSettings::AlignMode::DepthToColor));
  m_align->addItem(
      tr("Color to depth (depth resolution)"),
      int(DepthCameraSettings::AlignMode::ColorToDepth));
  checkForChanges(m_align);
  layout->addRow(tr("Alignment"), m_align);

  // One row per stream: width x height @ fps. Nine spin boxes on nine labelled
  // lines in three group boxes was most of the dialog's height for three
  // numbers nobody usually changes.
  const auto makeSpin = [this](int max, const char* suffix) {
    auto sb = new QSpinBox{this};
    sb->setRange(0, max);
    sb->setSpecialValueText(tr("Any"));
    if(suffix)
      sb->setSuffix(tr(suffix));
    sb->setMaximumWidth(110);
    checkForChanges(sb);
    return sb;
  };

  const auto formatRow
      = [&](QSpinBox** w, QSpinBox** h_, QSpinBox** fps, QComboBox** mode) {
    auto row = new QWidget{this};
    auto h = new QHBoxLayout{row};
    h->setContentsMargins(0, 0, 0, 0);
    h->addWidget(*w = makeSpin(8192, nullptr));
    // The codepoint rather than the character: score does not pass /utf-8 to
    // MSVC, which then reads the source in the system codepage and renders the
    // multiplication sign as mojibake.
    h->addWidget(new QLabel{QString{QChar{0x00D7}}, this});
    h->addWidget(*h_ = makeSpin(8192, nullptr));
    h->addWidget(*fps = makeSpin(1000, " fps"));

    // The modes the camera says it has, when its backend can list them: a
    // free spin box lets through combinations the camera refuses at open.
    *mode = new QComboBox{this};
    (*mode)->setVisible(false);
    h->addWidget(*mode, 1);
    QSpinBox* sw = *w;
    QSpinBox* sh = *h_;
    QSpinBox* sf = *fps;
    QComboBox* cb = *mode;
    connect(cb, &QComboBox::activated, this, [sw, sh, sf, cb](int i) {
      const auto v = cb->itemData(i).toList();
      if(v.size() != 3)
        return;
      sw->setValue(v[0].toInt());
      sh->setValue(v[1].toInt());
      sf->setValue(v[2].toInt());
    });
    h->addStretch(1);
    return row;
  };

  layout->addRow(
      tr("Color"), formatRow(&m_colorWidth, &m_colorHeight, &m_colorFps, &m_colorMode));
  layout->addRow(tr("Infrared"), formatRow(&m_irWidth, &m_irHeight, &m_irFps, &m_irMode));
  layout->addRow(
      tr("Depth"), formatRow(&m_depthWidth, &m_depthHeight, &m_depthFps, &m_depthMode));

  connect(m_address, &QLineEdit::editingFinished, this, [this] { refreshProfiles(); });

  for(auto* cb : {m_rgb, m_ir, m_depth, m_pointcloud, m_colorPointcloud, m_imu,
                  m_positions, m_normals, m_mesh})
  {
    checkForChanges(cb);
    connect(cb, &QCheckBox::toggled, this, [this] { updateEnabledState(); });
  }
  connect(
      m_align, &QComboBox::currentIndexChanged, this,
      [this] { updateEnabledState(); });

  setLayout(layout);
  setSettings(InputFactory{}.defaultSettings());
}

namespace
{
const char* formatName(int fmt)
{
  switch(fmt)
  {
    case DEPTHCAM_FMT_RGB24:
      return "RGB";
    case DEPTHCAM_FMT_BGR24:
      return "BGR";
    case DEPTHCAM_FMT_RGBA:
    case DEPTHCAM_FMT_BGRA:
    case DEPTHCAM_FMT_RGB0:
    case DEPTHCAM_FMT_BGR0:
      return "RGBA";
    case DEPTHCAM_FMT_GRAY8:
      return "8-bit";
    case DEPTHCAM_FMT_GRAY16:
      return "16-bit";
    case DEPTHCAM_FMT_GRAYF32:
      return "float";
    case DEPTHCAM_FMT_YUYV422:
    case DEPTHCAM_FMT_UYVY422:
      return "YUV 4:2:2";
    case DEPTHCAM_FMT_NV12:
    case DEPTHCAM_FMT_YUV420P:
      return "YUV 4:2:0";
    case DEPTHCAM_FMT_MJPEG:
      return "MJPEG";
    case DEPTHCAM_FMT_H264:
      return "H.264";
    case DEPTHCAM_FMT_H265:
      return "H.265";
    default:
      return "";
  }
}
}

void InputSettingsWidget::refreshProfiles()
{
  const QString uri = currentAddress().trimmed();
  if(uri == m_profilesFor)
    return;
  m_profilesFor = uri;

  const auto* backend = BackendRegistry::instance().find(BackendRegistry::backendOf(uri));
  const QByteArray u = uri.toUtf8();

  const auto fill = [&](QComboBox* box, uint32_t stream, QSpinBox* w, QSpinBox* h,
                        QSpinBox* fps) {
    box->clear();
    if(!backend || !backend->list_profiles || uri.isEmpty())
    {
      box->setVisible(false);
      return;
    }
    struct Collect
    {
      std::vector<depthcam_profile> list;
    } c;
    backend->list_profiles(
        u.constData(), stream,
        [](const depthcam_profile* p, void* user) {
      if(p && p->width > 0 && p->height > 0)
        static_cast<Collect*>(user)->list.push_back(*p);
        },
        &c);
    if(c.list.empty())
    {
      box->setVisible(false);
      return;
    }
    box->addItem(tr("Mode..."), QVariant{});
    for(const auto& p : c.list)
    {
      QString label = QStringLiteral("%1%2%3 @ %4 fps")
                          .arg(p.width)
                          .arg(QChar{0x00D7})
                          .arg(p.height)
                          .arg(p.fps);
      if(const char* f = formatName(p.format); *f)
        label += QStringLiteral(", ") + QString::fromLatin1(f);
      box->addItem(label, QVariantList{p.width, p.height, p.fps});
      if(p.width == w->value() && p.height == h->value() && p.fps == fps->value())
        box->setCurrentIndex(box->count() - 1);
    }
    box->setVisible(true);
  };

  fill(m_colorMode, DEPTHCAM_STREAM_COLOR, m_colorWidth, m_colorHeight, m_colorFps);
  fill(m_irMode, DEPTHCAM_STREAM_IR, m_irWidth, m_irHeight, m_irFps);
  fill(m_depthMode, DEPTHCAM_STREAM_DEPTH, m_depthWidth, m_depthHeight, m_depthFps);
}

QString InputSettingsWidget::currentAddress() const
{
  if(!m_network->isChecked())
    return m_address->text();

  const auto host = m_networkHost->text().trimmed();
  if(host.isEmpty())
    return {};

  // The backend takes an IPv6 literal in brackets, so a raw one has to be
  // wrapped before the port is appended -- otherwise the last colon of the
  // address reads as the port separator.
  const auto quoted
      = host.contains(':') && !host.startsWith('[') ? "[" + host + "]" : host;
  if(m_networkKind->currentData().toString() == QLatin1String("depthai"))
    return QStringLiteral("depthai:ip:%1").arg(host);

  return QStringLiteral("orbbec:net:%1:%2").arg(quoted).arg(m_networkPort->value());
}

void InputSettingsWidget::updateEnabledState()
{
  // One source of truth: with the network group on, the address is composed
  // from it, so showing the result read-only is clearer than leaving a second
  // editable copy that silently loses.
  const bool net = m_network->isChecked();
  m_networkKind->setEnabled(net);
  m_networkHost->setEnabled(net);
  // XLink has no configurable port; only the Orbbec grammar carries one.
  m_networkPort->setEnabled(
      net && m_networkKind->currentData().toString() == QLatin1String("orbbec"));
  m_address->setReadOnly(net);
  if(net)
    m_address->setText(currentAddress());

  const bool aligned
      = m_align->currentData().toInt() != int(DepthCameraSettings::AlignMode::None);
  // A coloured cloud needs depth and colour in one frame of reference; without
  // an alignment a backend would be asked to read mismatched buffers.
  const bool can_color = m_pointcloud->isChecked() && aligned;

  m_depthOutput->setEnabled(m_depth->isChecked());
  m_colorPointcloud->setEnabled(can_color);

  // Uncheck, not just disable: a disabled QCheckBox keeps its checked state and
  // getSettings() reads isChecked(), so greying it out alone would still let the
  // invalid combination through.
  if(!can_color && m_colorPointcloud->isChecked())
    m_colorPointcloud->setChecked(false);
}

Device::DeviceSettings InputSettingsWidget::getSettings() const
{
  Device::DeviceSettings s = m_settings;
  s.name = m_deviceNameEdit->text();
  s.protocol = InputFactory::static_concreteKey();

  // Start from what was loaded: the control values a device restores are not
  // edited here, and a fresh struct would silently drop them.
  DepthCameraSettings set = m_settings.deviceSpecificSettings.value<DepthCameraSettings>();
  set.device = currentAddress();
  set.rgb = m_rgb->isChecked();
  set.ir = m_ir->isChecked();
  set.depth = m_depth->isChecked();
  set.pointcloud = m_pointcloud->isChecked();
  set.colorPointcloud = m_colorPointcloud->isChecked();
  set.imu = m_imu->isChecked();
  set.align
      = static_cast<DepthCameraSettings::AlignMode>(m_align->currentData().toInt());
  set.colorWidth = m_colorWidth->value();
  set.colorHeight = m_colorHeight->value();
  set.colorFps = m_colorFps->value();
  set.depthWidth = m_depthWidth->value();
  set.depthHeight = m_depthHeight->value();
  set.depthFps = m_depthFps->value();
  set.irWidth = m_irWidth->value();
  set.irHeight = m_irHeight->value();
  set.irFps = m_irFps->value();
  set.depthOutput = static_cast<DepthCameraSettings::DepthOutput>(
      m_depthOutput->currentData().toInt());
  set.positions = m_positions->isChecked();
  set.normals = m_normals->isChecked();
  set.mesh = m_mesh->isChecked();

  s.deviceSpecificSettings = QVariant::fromValue(set);
  return s;
}

void InputSettingsWidget::setSettings(const Device::DeviceSettings& settings)
{
  m_settings = settings;
  m_deviceNameEdit->setText(settings.name);

  const auto set = settings.deviceSpecificSettings.value<DepthCameraSettings>();
  m_address->setText(set.device);

  // Split a network address back into the fields it came from, so an existing
  // device opens the dialog looking the way it was set up rather than as a URI.
  m_network->setChecked(false);
  if(static const QString prefix{"orbbec:net:"}; set.device.startsWith(prefix))
  {
    auto rest = set.device.mid(prefix.size());
    QString host = rest;
    int port = 8090;

    if(rest.startsWith('['))
    {
      // "[::1]:8090"
      if(const auto close = rest.indexOf(']'); close > 0)
      {
        host = rest.mid(1, close - 1);
        if(const auto colon = rest.indexOf(':', close); colon > 0)
          port = rest.mid(colon + 1).toInt();
      }
    }
    else if(const auto colon = rest.lastIndexOf(':'); colon > 0)
    {
      bool ok = false;
      if(const int p = rest.mid(colon + 1).toInt(&ok); ok && p > 0 && p <= 65535)
      {
        host = rest.left(colon);
        port = p;
      }
    }

    if(!host.isEmpty())
    {
      m_network->setChecked(true);
      if(const int i = m_networkKind->findData(QStringLiteral("orbbec")); i >= 0)
        m_networkKind->setCurrentIndex(i);
      m_networkHost->setText(host);
      m_networkPort->setValue(port);
    }
  }
  else if(static const QString ip{"depthai:ip:"}; set.device.startsWith(ip))
  {
    // No port to recover: XLink's is fixed.
    if(const auto host = set.device.mid(ip.size()); !host.isEmpty())
    {
      m_network->setChecked(true);
      if(const int i = m_networkKind->findData(QStringLiteral("depthai")); i >= 0)
        m_networkKind->setCurrentIndex(i);
      m_networkHost->setText(host);
    }
  }
  m_rgb->setChecked(set.rgb);
  m_ir->setChecked(set.ir);
  m_depth->setChecked(set.depth);
  m_pointcloud->setChecked(set.pointcloud);
  m_colorPointcloud->setChecked(set.colorPointcloud);
  m_imu->setChecked(set.imu);
  if(const int idx = m_align->findData(int(set.align)); idx >= 0)
    m_align->setCurrentIndex(idx);
  m_colorWidth->setValue(set.colorWidth);
  m_colorHeight->setValue(set.colorHeight);
  m_colorFps->setValue(set.colorFps);
  m_depthWidth->setValue(set.depthWidth);
  m_depthHeight->setValue(set.depthHeight);
  m_depthFps->setValue(set.depthFps);
  m_irWidth->setValue(set.irWidth);
  m_irHeight->setValue(set.irHeight);
  m_irFps->setValue(set.irFps);
  if(const int idx = m_depthOutput->findData(int(set.depthOutput)); idx >= 0)
    m_depthOutput->setCurrentIndex(idx);
  m_positions->setChecked(set.positions);
  m_normals->setChecked(set.normals);
  m_mesh->setChecked(set.mesh);

  updateEnabledState();
  refreshProfiles();
}

}
