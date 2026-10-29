#pragma once

#include <Device/Protocol/DeviceInterface.hpp>
#include <Device/Protocol/DeviceSettings.hpp>

#include <Gfx/GfxApplicationPlugin.hpp>
#include <Gfx/GfxExecContext.hpp>
#include <Gfx/GfxInputDevice.hpp>
#include <Gfx/Graph/NodeRenderer.hpp>
#include <Gfx/Graph/VideoNode.hpp>

#include <ossia/gfx/texture_parameter.hpp>
#include <ossia/network/base/protocol.hpp>
#include <ossia/network/generic/generic_device.hpp>
#include <ossia/network/generic/generic_node.hpp>

#include <DepthCamera/DepthCameraControls.hpp>
#include <DepthCamera/DepthCameraGpu.hpp>
#include <DepthCamera/DepthCameraImu.hpp>
#include <DepthCamera/DepthCameraInfo.hpp>
#include <DepthCamera/DepthCameraStream.hpp>
#include <DepthCamera/DepthCameraSettings.hpp>

namespace Gfx::DepthCamera
{

/**
 * @brief A graphics node owned by the device rather than by a parameter.
 *
 * One node can feed several parameters -- depth, positions, normals and mesh
 * are four outlets of the same node, which uploads and processes each frame
 * once for all of them -- so it is registered once and unregistered when the
 * last parameter pointing at it goes.
 */
struct registered_node
{
  registered_node(GfxExecutionAction& ctx, std::unique_ptr<score::gfx::Node> n);
  ~registered_node();
  registered_node(const registered_node&) = delete;
  registered_node& operator=(const registered_node&) = delete;

  GfxExecutionAction* context{};
  score::gfx::Node* node{};
  int32_t id{};
};

class depthcam_parameter : public ossia::gfx::texture_parameter
{
public:
  depthcam_parameter(
      std::shared_ptr<registered_node> node, int outlet, ossia::net::node_base& n);
  ~depthcam_parameter() override;

  void pull_texture(port_index idx) override;

  const std::shared_ptr<registered_node> node;
  const int outlet{};
};

class depthcam_geometry_parameter : public ossia::gfx::geometry_parameter
{
public:
  depthcam_geometry_parameter(
      std::shared_ptr<registered_node> node, int outlet, ossia::net::node_base& n);
  ~depthcam_geometry_parameter() override;

  void pull_geometry(port_index idx) override;

  const std::shared_ptr<registered_node> node;
  const int outlet{};
};

class depthcam_node : public ossia::net::node_base
{
public:
  using parameter_factory
      = std::function<std::unique_ptr<ossia::net::parameter_base>(ossia::net::node_base&)>;

  depthcam_node(
      ossia::net::device_base& dev, std::string name, const parameter_factory& make);

  ossia::net::parameter_base* get_parameter() const override
  {
    return m_parameter.get();
  }

private:
  ossia::net::device_base& get_device() const override { return m_device; }

  // node_base::add_child does not set the parent -- reporting it is the node's
  // own job. Left unset, every stream parameter came out with an empty address
  // path ("cam:" instead of "cam:/depth"), which breaks anything addressed by
  // name: OSC exposure, remote control, and Score.iterateDevice in a script.
  ossia::net::node_base* get_parent() const override
  {
    return &m_device.get_root_node();
  }
  ossia::net::node_base& set_name(std::string) override { return *this; }
  ossia::net::parameter_base* create_parameter(ossia::val_type) override
  {
    return m_parameter.get();
  }
  bool remove_parameter() override { return false; }
  std::unique_ptr<ossia::net::node_base> make_child(const std::string&) override
  {
    return {};
  }
  void removing_child(ossia::net::node_base&) override { }

  ossia::net::device_base& m_device;
  std::unique_ptr<ossia::net::parameter_base> m_parameter;
};

class depthcam_protocol : public ossia::net::protocol_base
{
public:
  std::shared_ptr<InputStream> stream;

  depthcam_protocol(
      const depthcam_backend_v1* backend, const QString& uri,
      const DepthCameraSettings& stgs);
  ~depthcam_protocol();

  bool pull(ossia::net::parameter_base&) override;
  bool push(const ossia::net::parameter_base&, const ossia::value& v) override;
  bool push_raw(const ossia::net::full_parameter_data&) override;
  bool observe(ossia::net::parameter_base&, bool) override;
  bool update(ossia::net::node_base& node_base) override;

  void start_execution() override;
  void stop_execution() override;

  /// Kept so start_execution can republish the negotiated formats. Deliberately
  /// shared_ptr and deliberately *not* the gfx nodes: those are owned by the
  /// GfxContext, which destroys them on its own schedule (a 100ms QTimer after
  /// unregister_node) and may itself be gone by the time a document switch
  /// reaches stop_execution.
  void registerExtractor(std::shared_ptr<InputStreamExtractor> dec);

  /// Non-owning. The tree belongs to the device, which destroys it before
  /// ossia clears the nodes its parameters live in, and clears this first so a
  /// write arriving in between cannot reach a dying object.
  void setControls(ControlTree* c) noexcept { m_controls = c; }

private:
  std::vector<std::shared_ptr<InputStreamExtractor>> m_extractors;
  ControlTree* m_controls{};
};

class depthcam_device_impl : public ossia::net::generic_device
{
public:
  depthcam_device_impl(
      const DepthCameraSettings& settings, GfxExecutionAction& ctx,
      std::unique_ptr<depthcam_protocol> proto, std::string name,
      ControlTree::StoreCallback onStore = {});
  ~depthcam_device_impl();

  /// Drops the settings and IMU trees.
  ///
  /// Has to happen before anything clears the node tree, because both point at
  /// parameters those nodes own -- Device::DeviceInterface::disconnect() calls
  /// root.clear_children(), so the device object outlives its own parameters by
  /// a good margin on a reconnect or a device removal.
  void releaseControls() noexcept;

  const std::shared_ptr<RenderParams>& renderParams() const noexcept { return m_params; }
  const std::shared_ptr<Calibration>& calibration() const noexcept { return m_calib; }
  ControlTree* controls() const noexcept { return m_controls.get(); }
  InfoTree* info() const noexcept { return m_info.get(); }

private:
  std::shared_ptr<RenderParams> m_params;
  std::shared_ptr<Calibration> m_calib;
  std::unique_ptr<ControlTree> m_controls;
  std::unique_ptr<ImuTree> m_imu;
  std::unique_ptr<InfoTree> m_info;
};

}
