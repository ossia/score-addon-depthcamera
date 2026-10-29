#pragma once

/**
 * @file SyntheticBackend.hpp
 * @brief A depth camera made of arithmetic, for tests and for rehearsals.
 *
 * Films the scene of SyntheticScene.hpp with known calibration, deterministic
 * timestamps and one control of every kind. Nothing about it is faked from the
 * host's point of view: it is a backend like the others, speaking
 * depthcam_abi.h, built as score_depthcam_synthetic and loaded with dlopen.
 * The same implementation can also be linked straight into a test, which is
 * what this header is for: the hooks below only make sense in-process.
 *
 * The device is chosen by its URI, "synthetic:" followed by comma-separated
 * options, all optional:
 *
 *   unit=1mm | 0.1mm | f32mm | f32m   depth sample: 16-bit millimetres (the
 *                                    default), 16-bit tenths of a millimetre
 *                                    (RealSense D405), float millimetres
 *                                    (Kinect v2), float metres (Oak)
 *   ir=u16 | f32                     infrared sample type
 *   distortion=none | bc             Brown-Conrady on the depth camera
 *   raytable=1                       offer get_ray_table
 *   motion=1                         the sphere sways
 *   clock=manual                     frames only on synthetic::step()
 *   fps=N                            default 30
 *
 * Resolutions come from the open config, defaulting to 640x576 depth and IR
 * and 1280x720 colour; depth/mode overrides the depth one at the next start.
 */

#include <DepthCamera/Backend/depthcam_abi.h>

#include <atomic>
#include <cstdint>

namespace depthcam_synthetic
{

/// The table, at the ABI version this was built for.
const depthcam_backend_v1* backend();

/// What the backend has done, across all devices, since load.
struct Counters
{
  std::atomic<int64_t> frames_delivered{};
  std::atomic<int64_t> frames_released{};
  std::atomic<int64_t> pointclouds_computed{};
  std::atomic<int64_t> set_control_calls{};
  std::atomic<int64_t> starts{};
  std::atomic<int64_t> stops{};

  /// CPU time of the camera thread, in nanoseconds: inside the host's frame
  /// callback (what the host costs per frame on that thread), and computing
  /// the CPU point cloud (what an SDK spends on one).
  std::atomic<int64_t> host_callback_ns{};
  std::atomic<int64_t> pointcloud_ns{};
};

/// CPU time consumed by the calling thread, in nanoseconds.
int64_t threadCpuNs() noexcept;
Counters& counters();

/// With clock=manual: produce @p n frames on the device's thread and return
/// once they have been delivered.
void step(depthcam_device* dev, int n = 1);

/// The frame number of the next frame.
int64_t nextFrame(depthcam_device* dev);

}
