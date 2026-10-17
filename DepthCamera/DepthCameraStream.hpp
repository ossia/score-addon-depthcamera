#pragma once

#include <Video/CameraInput.hpp>

#include <DepthCamera/Backend/depthcam_abi.h>

#include <QString>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

namespace Gfx::DepthCamera
{
struct DepthCameraSettings;

/// One output of the camera: a frame queue plus the format it turned out to
/// carry. The format is only known once frames arrive, hence the atomics.
struct StreamOutput
{
  ::Video::FrameQueue queue;

  std::atomic<int> width{0};
  std::atomic<int> height{0};
  std::atomic<int> format{-1}; // AVPixelFormat
  std::atomic<double> fps{0.};
};

/**
 * @brief Owns one open camera and turns its frames into AVFrames.
 *
 * Everything camera-specific lives behind depthcam_abi.h; this class knows only
 * about the ABI. Adding a device family means writing a backend, not touching
 * anything here.
 */
class InputStream final
{
public:
  InputStream(
      const depthcam_backend_v1* backend, const QString& uri,
      const DepthCameraSettings& settings) noexcept;
  ~InputStream() noexcept;

  bool valid() const noexcept { return m_device != nullptr; }

  /// The open camera, for the settings tree. Null until a successful open, and
  /// only valid for as long as this object lives.
  depthcam_device* device() const noexcept { return m_device; }
  const depthcam_backend_v1* backend() const noexcept { return m_backend; }

  /// Bitmask of depthcam_stream: what the camera actually gave us, which is not
  /// always what was asked for.
  uint32_t activeStreams() const noexcept;

  StreamOutput m_rgb;
  StreamOutput m_depth;
  StreamOutput m_ir;
  StreamOutput m_pcl;

  bool start() noexcept;
  void stop() noexcept;

  /// Stops and hands the camera back, leaving this object alive but inert.
  ///
  /// Exists because the last owner of an InputStream is not the device: the
  /// graphics nodes hold the extractors, and score::gfx::GfxContext destroys a
  /// node on a 100ms timer after it is unregistered, so waiting for the
  /// shared_ptr to run out keeps the camera claimed for a moment after the
  /// device is gone. That moment is enough to break the next open on any SDK
  /// that claims the USB interface exclusively -- libk4a and libfreenect both
  /// do -- which is what made an Azure Kinect come up with no picture until the
  /// user hit Reconnect.
  void close() noexcept;

  /// Each extractor registers on construction and votes once it is done: all
  /// four share one camera, so the first to go idle must not stop it.
  void registerExtractor() noexcept { m_extractors.fetch_add(1); }
  void requestStop() noexcept;

  /// Millimetres per depth unit, as reported by the backend. 0 if unknown.
  std::atomic<float> depth_unit_mm{0.f};

  using imu_callback = std::function<void(const depthcam_imu_sample&)>;

  /**
   * @brief Where inertial samples go.
   *
   * Called from the camera's own thread, hundreds of times a second, with the
   * lock held -- which is what makes clearing it safe: setImuCallback({})
   * returns only once no call is in flight, so the node tree it writes into
   * cannot be freed underneath it. That matters because the callback outlives
   * nothing: the stream is closed after the node tree is cleared, not before.
   */
  void setImuCallback(imu_callback cb) noexcept;

private:
  static void onFrameThunk(const depthcam_frame* f, void* user);
  void onFrame(const depthcam_frame& f);
  void handleImage(StreamOutput& out, const depthcam_frame& f);
  void handlePointCloud(const depthcam_frame& f);
  void handleImu(const depthcam_frame& f);

  AVFrame* decodeCompressed(const depthcam_frame& f);
  AVCodecContext* codecContext(AVCodecID id);
  void releaseCodecs();

  const depthcam_backend_v1* m_backend{};
  depthcam_device* m_device{};

  std::atomic_bool m_running{};
  std::atomic<int> m_extractors{0};
  std::atomic<int> m_stopVotes{0};

  // Only for backends that hand us a compressed colour stream.
  AVCodecContext* m_codecMJPEG{};
  AVCodecContext* m_codec264{};
  AVCodecContext* m_codec265{};
  AVPacket* m_packet{};

  std::mutex m_imu_mutex;
  imu_callback m_imu_cb;
};

class InputStreamExtractor final : public ::Video::ExternalInput
{
public:
  InputStreamExtractor(
      std::shared_ptr<InputStream> s, StreamOutput& out, QString filter) noexcept;
  ~InputStreamExtractor();

  bool start() noexcept override;
  void stop() noexcept override;

  AVFrame* dequeue_frame() noexcept override;
  void release_frame(AVFrame* frame) noexcept override;

  void refreshMetadata() noexcept;

  const QString filter;

private:
  std::shared_ptr<InputStream> m_stream;
  StreamOutput& m_out;
};
}
