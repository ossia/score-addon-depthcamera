#pragma once

/**
 * @file DepthCameraImu.hpp
 * @brief Publish a camera's inertial samples as `<cam>/imu/accel` and `gyro`.
 *
 * These sit at the device root beside `rgb` and `depth`, not under `controls`:
 * they are what the camera measures, whereas `controls/imu/gyro_odr` and its
 * neighbours are how it is configured. Same word, opposite direction.
 *
 * Published straight from the camera's thread, which is the point -- an IMU at
 * 200Hz is only useful while it is current, and routing it through the Qt event
 * loop would add a frame of latency to the one thing here that has none. That
 * is also how every push-based ossia device behaves; an OSC device runs its
 * parameters' callbacks on the network thread in exactly the same way.
 */

#include <ossia/network/base/device.hpp>
#include <ossia/network/base/node.hpp>
#include <ossia/network/base/parameter.hpp>

#include <DepthCamera/Backend/depthcam_abi.h>

#include <memory>

namespace Gfx::DepthCamera
{
class InputStream;

class ImuTree
{
public:
  /// Builds `imu/accel`, `imu/gyro` and `imu/temperature` and starts listening.
  ImuTree(
      const std::shared_ptr<InputStream>& stream, ossia::net::device_base& dev,
      ossia::net::node_base& parent);

  /// Stops listening first, and does not return until any callback in flight
  /// has finished: the parameters below are about to be freed.
  ~ImuTree();

  ImuTree(const ImuTree&) = delete;
  ImuTree& operator=(const ImuTree&) = delete;

private:
  void onSample(const depthcam_imu_sample& s);

  std::shared_ptr<InputStream> m_stream;

  ossia::net::parameter_base* m_accel{};
  ossia::net::parameter_base* m_gyro{};
  ossia::net::parameter_base* m_temperature{};
};

}
