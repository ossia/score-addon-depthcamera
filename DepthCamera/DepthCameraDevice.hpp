#pragma once

#include <Gfx/GfxInputDevice.hpp>
#include <Gfx/SharedInputSettings.hpp>

#include <DepthCamera/DepthCameraSettings.hpp>

class QCheckBox;
class QComboBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QSpinBox;

namespace Gfx::DepthCamera
{
class InputFactory final : public SharedInputProtocolFactory
{
  // Unchanged from when this protocol was Orbbec-only: the key is written into
  // every document that uses the device, so it outlives the rename.
  SCORE_CONCRETE("a3bd48ba-f5db-43b7-aa92-2b1a37a88a79")
public:
  QString prettyName() const noexcept override;
  QString category() const noexcept override;
  QUrl manual() const noexcept override;
  Device::DeviceEnumerators
  getEnumerators(const score::DocumentContext& ctx) const override;

  Device::DeviceInterface* makeDevice(
      const Device::DeviceSettings& settings,
      const Explorer::DeviceDocumentPlugin& plugin,
      const score::DocumentContext& ctx) override;
  const Device::DeviceSettings& defaultSettings() const noexcept override;

  Device::ProtocolSettingsWidget* makeSettingsWidget() override;

  QVariant makeProtocolSpecificSettings(const VisitorVariant& visitor) const override;
  void serializeProtocolSpecificSettings(
      const QVariant& data, const VisitorVariant& visitor) const override;
};

class InputSettingsWidget final : public Device::ProtocolSettingsWidget
{
public:
  explicit InputSettingsWidget(QWidget* parent = nullptr);

  Device::DeviceSettings getSettings() const override;
  void setSettings(const Device::DeviceSettings& settings) override;

private:
  void updateEnabledState();

  /// The address the dialog will hand back: either what was typed, or the one
  /// composed from the network fields.
  QString currentAddress() const;

  Device::DeviceSettings m_settings;

  QLineEdit* m_deviceNameEdit{};
  QLineEdit* m_address{};

  /// Orbbec cameras with an Ethernet port answer a broadcast, but a host
  /// firewall that drops broadcast input hides every one of them, and a camera
  /// on another subnet is never going to be discovered at all. Typing the
  /// address has to be a first-class way in, not a URI the user has to know the
  /// syntax of.
  QCheckBox* m_network{};
  QComboBox* m_networkKind{};
  QLineEdit* m_networkHost{};
  QSpinBox* m_networkPort{};

  QCheckBox* m_rgb{};
  QCheckBox* m_ir{};
  QCheckBox* m_depth{};
  QCheckBox* m_pointcloud{};
  QCheckBox* m_colorPointcloud{};
  QCheckBox* m_imu{};
  QComboBox* m_align{};
  QComboBox* m_depthOutput{};
  QCheckBox* m_positions{};
  QCheckBox* m_normals{};
  QCheckBox* m_mesh{};

  QSpinBox* m_colorWidth{};
  QSpinBox* m_colorHeight{};
  QSpinBox* m_colorFps{};
  QSpinBox* m_depthWidth{};
  QSpinBox* m_depthHeight{};
  QSpinBox* m_depthFps{};
  QSpinBox* m_irWidth{};
  QSpinBox* m_irHeight{};
  QSpinBox* m_irFps{};
};

}
