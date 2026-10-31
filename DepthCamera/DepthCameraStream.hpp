#pragma once

#include <Video/CameraInput.hpp>

#include <DepthCamera/Backend/depthcam_abi.h>

#include <QString>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace Gfx::DepthCamera
{
struct DepthCameraSettings;

/**
 * @brief What is known about a stream's timing, as of its last frame.
 *
 * The device clock is the backend's timestamp_ns, the host clock
 * std::chrono::steady_clock at the moment the frame reached the plug-in. Both
 * in nanoseconds; times are relative to the first frame since start.
 */
struct StreamStats
{
  int64_t frames{};  ///< delivered since start
  int64_t dropped{}; ///< discarded because nothing consumed them in time
  double fps{};      ///< from the device clock, smoothed
  double device_time_ms{};
  double host_time_ms{};
  double interval_ms{}; ///< device time between the last two frames
};

/// One output of the camera: a frame queue plus the format it turned out to
/// carry. The format is only known once frames arrive, hence the atomics.
struct StreamOutput
{
  explicit StreamOutput(uint32_t bit) noexcept
      : stream{bit}
  {
  }

  ::Video::FrameQueue queue;

  /// The depthcam_stream bit this output carries.
  const uint32_t stream;

  std::atomic<int> width{0};
  std::atomic<int> height{0};
  std::atomic<int> format{-1}; // AVPixelFormat
  std::atomic<double> fps{0.};

  /// Depth only: millimetres per sample of the latest frame, 0 until one came.
  /// Read by the renderer for every frame it uploads, so a camera that changes
  /// its unit mid-stream (a RealSense switching preset) is followed.
  std::atomic<float> unit_mm{0.f};

  /// The depthcam_format of the latest frame, -1 until one came.
  std::atomic<int> source_format{-1};

  /// Graphics nodes currently drawing this stream. While there are none a
  /// frame is handed straight back to the backend instead of being queued,
  /// and the backend is told it need not produce the stream at all.
  std::atomic<int> consumers{0};

  /// Steady-clock milliseconds at which consumers last went above zero, 0
  /// while there are none.
  std::atomic<int64_t> demanded_since_ms{0};

  /// Camera thread only.
  StreamStats stats;
  uint64_t first_device_ns{};
  int64_t first_host_ns{};
  uint64_t last_device_ns{};
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

  StreamOutput m_rgb{DEPTHCAM_STREAM_COLOR};
  StreamOutput m_depth{DEPTHCAM_STREAM_DEPTH};
  StreamOutput m_ir{DEPTHCAM_STREAM_IR};
  StreamOutput m_pcl{DEPTHCAM_STREAM_POINTCLOUD};

  /// A graphics node started or stopped drawing @p out. Any thread.
  void addConsumer(StreamOutput& out, int delta) noexcept;

  /// What the backend is told to produce: the streams with a consumer, plus
  /// the IMU, which has no graphics node and is wanted whenever it is open.
  uint32_t demand() const noexcept;

  /**
   * @brief One attempt, on the calling thread.
   *
   * A camera that refuses to start, or only starts on its default
   * configuration (the RealSense backend falls back and says so), is retried
   * on this object's worker thread with a backoff; false is returned
   * meanwhile. stop() and close() cancel the retries.
   */
  bool start() noexcept;
  void stop() noexcept;

  /**
   * @brief Restarts a camera that stopped delivering a stream it was asked for.
   *
   * A consumed stream that has had no frame for 3 s of demand while the others
   * do restarts the camera on the worker thread, at most three times per
   * start(). Called by the depth and infrared nodes once per frame.
   */
  static void checkHealth(const std::shared_ptr<InputStream>& self) noexcept;
  bool running() const noexcept { return m_running.load(std::memory_order_acquire); }

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

  /**
   * @brief Holds start() back until @p gate is ready.
   *
   * The control tree replays the values a document saved on its own thread,
   * right after open; a depth mode written after the stream started would need
   * a restart, so starting waits for the replay, up to a bound.
   */
  void setStartGate(std::shared_future<void> gate) noexcept;

  using stats_callback = std::function<void(const StreamOutput&, const StreamStats&)>;

  /// Per-frame timing, from the camera thread. Same locking contract as the
  /// IMU callback: clearing it returns only once no call is in flight.
  void setStatsCallback(stats_callback cb) noexcept;

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

  /// After the worker thread started the camera. Called on the worker thread;
  /// clearing it returns only once no call is in flight.
  void setRestartedCallback(std::function<void()> cb) noexcept;

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

  std::mutex m_stats_mutex;
  stats_callback m_stats_cb;

  std::shared_future<void> m_startGate;

  /// start() and stop() come from the execution engine, the GUI, the control
  /// worker (a restart for a depth mode) and this object's worker.
  std::recursive_mutex m_lifecycle;
  bool m_closed{};

  void pushDemand() noexcept;
  void noteFirstFrame(const depthcam_frame& f) noexcept;

  enum class Attempt
  {
    Started,
    Fallback,
    Failed,
  };
  Attempt attemptStart(int attempt, bool acceptFallback) noexcept;
  void stopLocked() noexcept;

  struct Job
  {
    uint64_t intent{};
    int firstAttempt{};
    bool restart{};
  };
  bool post(Job job) noexcept;
  void runWorker() noexcept;
  void runJob(const Job& job) noexcept;
  bool sleepFor(std::chrono::milliseconds d, uint64_t intent) noexcept;
  void notifyRestarted() noexcept;

  /// Bumped by stop() and close(): a pending retry or restart that was
  /// scheduled under another value is dropped.
  std::atomic<uint64_t> m_intent{0};
  std::mutex m_worker_mutex;
  std::condition_variable m_worker_cv;
  std::optional<Job> m_job;
  bool m_shutdown{};
  std::thread m_worker;

  std::mutex m_restarted_mutex;
  std::function<void()> m_restarted_cb;

  uint32_t m_requested{};
  std::atomic<uint32_t> m_active{0};
  std::atomic<uint32_t> m_seen{0};
  std::atomic_bool m_started{false};
  std::atomic<int> m_restarts{0};
  std::atomic<int64_t> m_startedAtMs{0};
  void stamp(StreamOutput& out, const depthcam_frame& f, AVFrame* av) noexcept;
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
