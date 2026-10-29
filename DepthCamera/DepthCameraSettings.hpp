#pragma once

#include <QMetaType>
#include <QString>

#include <vector>

#include <verdigris>

namespace Gfx::DepthCamera
{

/**
 * @brief Which camera to talk to, and what to ask it for.
 *
 * The camera is addressed by a text URI whose first component names the backend
 * and whose remainder is opaque to everything on this side:
 *
 *   orbbec:sn:CL8L1234ABC        orbbec:net:192.168.0.12:8090
 *   freenect2:sn:012345678901    freenect:index:0    k4a:index:0
 *
 * One field, one widget, and the same code path whether the camera was picked
 * from the device browser or typed in by hand -- which is how a networked
 * camera, or one that is not currently broadcasting, gets connected.
 */
struct DepthCameraSettings
{
  QString device;

  bool rgb{true};
  bool ir{false};
  bool depth{true};
  bool pointcloud{true};

  /// Colour the point cloud from the RGB sensor. Needs rgb + depth and an
  /// alignment other than None.
  bool colorPointcloud{false};

  /**
   * @brief Accelerometer and gyroscope, published as `imu/accel` and `imu/gyro`.
   *
   * Off by default, and not because it is expensive -- it is a few hundred
   * samples a second and no image data at all. It is off because asking for it
   * changes what the camera streams: an Orbbec has to have the IMU sensors
   * enabled in the pipeline configuration, and an Azure Kinect runs a second
   * capture loop for it. A device that did not ask should not pay.
   */
  bool imu{false};

  /**
   * @brief How depth and colour are brought into a common frame of reference.
   *
   * This also decides the point cloud's resolution, which dominates its cost.
   * Measured on a Femto Mega:
   *   DepthToColor  1920x1080 -> 2 073 600 points, 47.5 MB/frame
   *   ColorToDepth   640x576  ->   368 640 points,  8.4 MB/frame
   * DepthToColor upsamples depth to the colour resolution, inventing roughly
   * five points for every one the sensor actually measured.
   */
  enum class AlignMode
  {
    None,
    DepthToColor,
    ColorToDepth,
  };
  AlignMode align{AlignMode::ColorToDepth};

  /// 0 means "let the backend choose".
  int colorWidth{0}, colorHeight{0}, colorFps{0};
  int depthWidth{0}, depthHeight{0}, depthFps{0};
  int irWidth{0}, irHeight{0}, irFps{0};

  /**
   * @brief What the `depth` texture holds.
   *
   * Metres: an R32F texture of distances along +Z in metres, 0 where the
   * sensor has no measurement. Preview: grey, 0 to 4.5 m mapped to black to
   * white, which is what documents saved before the setting existed render.
   * A device created now gets Metres; a document that does not mention the
   * setting gets Preview.
   */
  enum class DepthOutput
  {
    Preview,
    Metres,
  };
  DepthOutput depthOutput{DepthOutput::Metres};

  /// GPU outputs computed from the depth stream, see DepthCameraGpu.hpp.
  /// Each is a node of its own and costs nothing while nothing reads it.
  bool positions{false};
  bool normals{false};
  bool mesh{false};

  /**
   * @brief Control values to restore after opening the camera.
   *
   * Only what was written to the device, not every value it reports: a
   * camera's defaults change with its firmware, and replaying a value nobody
   * chose would pin it. In the order they were last written, which is the order
   * they are replayed in -- turning an `auto` off has to come before the manual
   * value it unlocks.
   */
  struct ControlValue
  {
    QString id;                ///< "color/exposure", "render/near"
    std::vector<double> value; ///< one number, or three for a vec3
    bool operator==(const ControlValue&) const = default;
  };
  std::vector<ControlValue> controls;

  /// Records a write, moving the control to the end of the replay order.
  void storeControl(const QString& id, std::vector<double> value);

  /// The backend a device URI names, e.g. "orbbec". Empty if none.
  QString backend() const;
};

}

Q_DECLARE_METATYPE(Gfx::DepthCamera::DepthCameraSettings)
W_REGISTER_ARGTYPE(Gfx::DepthCamera::DepthCameraSettings)
