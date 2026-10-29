#pragma once

/**
 * @file DepthCameraInfo.hpp
 * @brief Read-only metadata: `<cam>/info`, `<cam>/calibration`, `<cam>/stats`.
 *
 *   info/{backend,model,serial,firmware,hardware_rev,transport,uri}  string
 *   info/depth_unit_mm          float, from the latest depth frame
 *   info/depth_range            vec2f, metres
 *   info/calibration_estimated  bool: true when the camera reported nothing
 *                               and a 60-degree pinhole stands in
 *   calibration/<s>/{width,height}       int, s in depth, color, ir
 *   calibration/<s>/intrinsics           vec4f fx fy cx cy, pixels of the
 *                                        stream as delivered
 *   calibration/<s>/distortion/model     string
 *   calibration/<s>/distortion/coeffs    list of 8: k1..k6 p1 p2
 *   calibration/<s>/projection           list of 16: OpenGL clip space from
 *                                        that camera's frame, see below
 *   calibration/depth_to_color           list of 16, column-major, metres
 *   calibration/depth_to_color/row0..3   vec4f, the same matrix by rows
 *   calibration/depth_to_imu             likewise
 *   stats/<s>/{frames,dropped}           int
 *   stats/<s>/{fps,device_time_ms,host_time_ms,interval_ms}  float
 *
 * The rows exist because an ISF shader has vector uniforms but no matrix ones.
 * The projection maps a point of the camera frame (+Y down, +Z forward) to
 * OpenGL clip space (+Y up, z in [-1, 1]) for the near and far planes of
 * render/near and render/far, or the depth range when those are unset.
 *
 * stats/ is written from the camera thread like the IMU: device_time_ms at
 * every frame, so something can trigger on arrival, the rest ten times a
 * second.
 */

#include <ossia/network/base/device.hpp>
#include <ossia/network/base/node.hpp>
#include <ossia/network/base/parameter.hpp>

#include <DepthCamera/DepthCameraCalibration.hpp>

#include <QString>

#include <array>
#include <chrono>
#include <memory>

namespace Gfx::DepthCamera
{
class InputStream;
struct StreamOutput;
struct StreamStats;

class InfoTree
{
public:
  InfoTree(
      const std::shared_ptr<InputStream>& stream, std::shared_ptr<Calibration> calib,
      ossia::net::node_base& root, QString uri, QString backendName,
      QString transport);
  ~InfoTree();

  InfoTree(const InfoTree&) = delete;
  InfoTree& operator=(const InfoTree&) = delete;

  /// Re-reads everything from the camera and republishes it. Qt thread.
  void refresh();

  /// The planes calibration/<s>/projection is built for. Qt thread.
  void setClipPlanes(float near_m, float far_m);

private:
  struct StreamNodes
  {
    uint32_t stream{};
    ossia::net::parameter_base* width{};
    ossia::net::parameter_base* height{};
    ossia::net::parameter_base* intrinsics{};
    ossia::net::parameter_base* model{};
    ossia::net::parameter_base* coeffs{};
    ossia::net::parameter_base* projection{};
  };
  struct StatsNodes
  {
    ossia::net::parameter_base* frames{};
    ossia::net::parameter_base* dropped{};
    ossia::net::parameter_base* fps{};
    ossia::net::parameter_base* device_time{};
    ossia::net::parameter_base* host_time{};
    ossia::net::parameter_base* interval{};
    std::chrono::steady_clock::time_point last_publish{};
  };

  void publish(const CalibrationData& d);
  void publishProjections(const CalibrationData& d);
  StreamNodes& streamNodes(uint32_t stream);
  void onStats(const StreamOutput& out, const StreamStats& st);

  std::shared_ptr<InputStream> m_stream;
  std::shared_ptr<Calibration> m_calib;
  ossia::net::node_base& m_root;

  ossia::net::node_base* m_info{};
  ossia::net::node_base* m_calibration{};
  ossia::net::parameter_base* m_unit{};
  float m_lastUnit{};

  std::vector<StreamNodes> m_streams;
  ossia::net::parameter_base* m_depthToColor{};
  std::array<ossia::net::parameter_base*, 4> m_depthToColorRows{};
  ossia::net::parameter_base* m_depthToImu{};
  std::array<ossia::net::parameter_base*, 4> m_depthToImuRows{};

  // Indexed by stream bit position: color, ir, depth, pointcloud.
  std::array<StatsNodes, 4> m_stats{};

  float m_near{}, m_far{};
};

/// The projection calibration/<s>/projection publishes; column-major.
Matrix4 clipProjection(const depthcam_intrinsics& in, float near_m, float far_m) noexcept;

}
