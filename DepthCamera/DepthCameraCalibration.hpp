#pragma once

/**
 * @file DepthCameraCalibration.hpp
 * @brief What the camera says about its own geometry, and the ray table built
 * from it.
 *
 * Read once after every open (ABI 4: get_intrinsics / get_extrinsics /
 * get_details) and shared between the device tree, which publishes it under
 * `info/` and `calibration/`, and the GPU renderer, which turns depth into
 * positions with it. No Qt and no ossia here, so the maths can be tested on
 * its own.
 */

#include <DepthCamera/Backend/depthcam_abi.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace Gfx::DepthCamera
{

/// A 4x4 rigid transform, column-major, metres: the layout GLSL's mat4 and
/// QMatrix4x4::data() use.
using Matrix4 = std::array<float, 16>;

Matrix4 toMatrix(const depthcam_extrinsics& e) noexcept;
Matrix4 identityMatrix() noexcept;

/// The model name the `distortion/model` parameter publishes.
const char* distortionName(int model) noexcept;

/**
 * @brief (x/z, y/z) for the centre of every pixel of a w x h image.
 *
 * If the intrinsics were measured at another resolution (an SDK reporting the
 * sensor's native mode for a binned stream) they are scaled to w x h first:
 * focal lengths by the size ratio, the principal point so that the image
 * corners stay where they were.
 *
 * Undistortion is iterative for the forward models, closed-form for the
 * inverse one. NaN where the model has no ray (outside a fisheye's domain).
 */
void computeRayTable(const depthcam_intrinsics& in, int w, int h, float* xy);

/// The ray through one pixel, as computeRayTable would give it.
std::array<double, 2> pixelRay(const depthcam_intrinsics& in, double u, double v);

/// A pinhole guess for a camera that does not report its calibration: 60
/// degrees of horizontal field of view, square pixels, centred.
depthcam_intrinsics estimatedIntrinsics(int w, int h) noexcept;

/// Everything the device knows, as last read.
struct CalibrationData
{
  struct Stream
  {
    uint32_t stream{};
    depthcam_intrinsics intrinsics{};
  };

  std::vector<Stream> streams;
  std::optional<Matrix4> depthToColor;
  std::optional<Matrix4> depthToImu;

  std::string model, serial, firmware, hardwareRev;
  float depthMin{}, depthMax{};

  /// Whether `streams` came from the camera or was guessed for an ABI 3 one.
  bool estimated{};

  const depthcam_intrinsics* intrinsics(uint32_t stream) const noexcept;
};

/**
 * @brief The calibration, shared between threads.
 *
 * Written on the Qt thread after an open or a restart, read by the render
 * thread once per frame through version(), which is cheap.
 */
class Calibration
{
public:
  /// Reads everything an ABI 4 backend offers about an open device. Falls back
  /// to estimatedIntrinsics() for streams it says nothing about.
  static CalibrationData query(
      const depthcam_backend_v1& backend, depthcam_device& device, uint32_t streams,
      int depthW, int depthH);

  void set(CalibrationData d);
  CalibrationData get() const;

  /// Bumped by every set().
  int64_t version() const noexcept { return m_version.load(std::memory_order_acquire); }

  /**
   * @brief The depth stream's ray table at w x h.
   *
   * The backend's own if it offers one at that size, computed from the
   * intrinsics otherwise; cached until the calibration or the size changes.
   */
  std::shared_ptr<const std::vector<float>> depthRays(int w, int h);

  /// Where to get a backend table from; set by the device, cleared on close.
  void setRaySource(const depthcam_backend_v1* backend, depthcam_device* device);

private:
  mutable std::mutex m_lock;
  CalibrationData m_data;
  std::atomic<int64_t> m_version{0};

  const depthcam_backend_v1* m_backend{};
  depthcam_device* m_device{};

  std::shared_ptr<const std::vector<float>> m_rays;
  int m_raysW{}, m_raysH{};
  int64_t m_raysVersion{-1};
};

}
