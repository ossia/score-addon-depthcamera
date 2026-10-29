#pragma once

/**
 * @file DepthCameraControls.hpp
 * @brief Publish a camera's settings as `<cam>/controls/<group>/<name>`.
 *
 * The backend side of this is depthcam_abi.h's list_controls(2)/get_control/
 * set_control. Nothing here names a specific camera control: an Orbbec
 * publishes its properties and filters, a RealSense its options, an Azure
 * Kinect its colour controls and a Kinect v1 its tilt motor, all through the
 * same calls, and the grouping comes from the identifier the backend chose.
 *
 * Beside them sit the host's own: `render/...`, the processing knobs of the GPU
 * outputs, which every camera gets whatever its SDK offers, and
 * `advanced/store_current_settings` / `advanced/reset_to_defaults`.
 *
 * What shapes the implementation:
 *
 *  - **Nothing touches the camera on a caller's thread.** Every get_control and
 *    set_control is a synchronous transfer -- a USB vendor request, or a TCP
 *    round trip for a camera on the network -- and the callers here are the
 *    GUI, the execution engine and the network layer. A worker thread owns all
 *    of it; the callers queue and return.
 *
 *  - **Most SDKs do not report a control changing.** Only librealsense does
 *    (ABI 4 set_control_changed_callback), so elsewhere a read-only sensor --
 *    a temperature, an exposure under auto -- can only be tracked by asking
 *    repeatedly. Only those are polled, and only while something is listening:
 *    ossia calls protocol_base::observe() when a parameter gains its first
 *    callback and again when it loses its last.
 *
 *  - **Never push into a parameter from inside that parameter's callback.**
 *    callback_container::send holds a non-recursive mutex while it runs
 *    callbacks, so a corrective push made inline deadlocks the writing thread.
 *    Everything the worker learns is published from the Qt thread instead.
 *
 *  - **What the user writes is remembered.** Each write is recorded and handed
 *    to the device, which keeps it in its settings; after the next open the
 *    worker replays the record, in order, before the stream starts.
 *
 *  - **Some controls need a restart** (ABI 4 DEPTHCAM_CONTROL_REQUIRES_RESTART:
 *    a depth mode). The worker stops the stream, writes, and starts it again,
 *    then asks for the calibration to be read again: it has changed.
 */

#include <ossia/network/base/device.hpp>
#include <ossia/network/base/node.hpp>
#include <ossia/network/base/parameter.hpp>
#include <ossia/network/value/value.hpp>

#include <QObject>

#include <DepthCamera/Backend/depthcam_abi.h>
#include <DepthCamera/DepthCameraSettings.hpp>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace Gfx::DepthCamera
{
class InputStream;
struct RenderParams;

class ControlTree
{
public:
  using ControlValues = std::vector<DepthCameraSettings::ControlValue>;

  /// Called on the Qt thread with the whole record, after writes (coalesced).
  using StoreCallback = std::function<void(const ControlValues&)>;

  /**
   * @brief Enumerates the camera's controls and mirrors them into the tree.
   *
   * Only the descriptors are read here. The values are seeded from the
   * backend's defaults -- or from @p saved -- and refreshed by the worker a
   * moment later: reading forty controls from the camera would otherwise
   * happen inline in reconnect(), on the GUI thread, and on a networked camera
   * that is seconds.
   *
   * @param stream for the restarts, and to hold its start until @p saved has
   *        been replayed. May be null.
   * @param params where the render/ controls write. May be null: no render/.
   * @param onRestarted Qt thread, after a restart: re-read the calibration.
   */
  ControlTree(
      const depthcam_backend_v1& backend, depthcam_device& device,
      std::shared_ptr<InputStream> stream, ossia::net::device_base& dev,
      ossia::net::node_base& parent, std::shared_ptr<RenderParams> params,
      ControlValues saved = {}, StoreCallback onStore = {},
      std::function<void()> onRestarted = {});
  ~ControlTree();

  ControlTree(const ControlTree&) = delete;
  ControlTree& operator=(const ControlTree&) = delete;

  bool empty() const noexcept { return m_entries.empty(); }

  /// Queues a write. Returns false only if @p param is not one of ours.
  bool write(const ossia::net::parameter_base& param, const ossia::value& v);

  /// Queues a re-read. The tree already holds the last known value, so there is
  /// nothing to wait for.
  bool read(ossia::net::parameter_base& param);

  /// ossia's observation hook: starts and stops the worker's interest in one
  /// read-only control.
  bool observe(const ossia::net::parameter_base& param, bool enable);

  /// Queues a re-read of every readable control.
  void refresh();

  /// What has been recorded so far. Any thread.
  ControlValues stored() const;

  /// Ready once the saved values have been written to the camera.
  std::shared_future<void> replayed() const { return m_replayed; }

  /// Blocks until the worker has nothing queued. For tests.
  void waitIdle();

  /// The parameter of a control, by id. For tests.
  ossia::net::parameter_base* parameter(std::string_view id) const noexcept;

private:
  enum class Origin
  {
    Backend,
    Render,
    Action,
  };

  struct Entry
  {
    std::string id; ///< as the backend spelled it, e.g. "color/exposure"
    ossia::net::parameter_base* param{};
    Origin origin{Origin::Backend};
    int kind{};
    int access{};
    uint32_t flags{};
    /// Index into m_descs, for the range and the enum labels.
    int desc{-1};

    /// The auto toggle that makes this one inactive, and those it rules.
    int autoEntry{-1};
    std::vector<int> ruled;

    /// Read-only or volatile, so the only way to track it is to ask. The
    /// worker reads it.
    bool pollable{};

    /// Flipped to read-only by its auto toggle, or by the backend's flag.
    std::atomic_bool inactive{false};

    /// Number of things listening. Touched from the Qt thread, read by the
    /// worker.
    std::atomic_int observers{0};

    /// Last value published. Qt thread only.
    double last{};
    bool has_last{};
  };

  struct Request
  {
    enum Kind
    {
      Read,
      Write,
      Gate,     ///< fulfil m_replayedPromise
      StoreAll, ///< record every current value
      Reset,    ///< every writable control back to its default
      Wait,     ///< for waitIdle()
    } kind{Read};
    int entry{-1};
    double value{};
  };

  struct Desc
  {
    depthcam_control2 c{};
    std::string unit;
    std::string autoControl;
  };

  Entry* find(const ossia::net::parameter_base& param) noexcept;
  int indexOf(const ossia::net::parameter_base& param) noexcept;
  int indexOf(std::string_view id) const noexcept;

  void buildBackendControls(ossia::net::node_base& root);
  void buildRenderControls(ossia::net::node_base& root);
  void buildActions(ossia::net::node_base& root);
  void applySaved(const ControlValues& saved);

  bool writeRender(Entry& e, const ossia::value& v);
  void record(const std::string& id, std::vector<double> v);
  void scheduleStore();

  /// Qt thread only.
  void publish(Entry& e, double v);
  void setInactive(Entry& e, bool inactive);

  void run();
  void post(Request r);
  void wake();
  void writeOne(Entry& e, double value, std::vector<std::pair<int, double>>& results);

  static void onChangedThunk(const char* id, void* user);

  const depthcam_backend_v1& m_backend;
  depthcam_device& m_device;
  std::shared_ptr<InputStream> m_stream;
  std::shared_ptr<RenderParams> m_params;
  StoreCallback m_onStore;
  std::function<void()> m_onRestarted;

  std::vector<std::unique_ptr<Entry>> m_entries;

  /// The descriptors as the backend gave them, plus copies of the strings ABI 4
  /// added. The ABI 2 strings belong to the backend and stay valid until
  /// close, which is why this object has to be destroyed before the device is
  /// closed.
  std::vector<Desc> m_descs;

  std::thread m_worker;
  std::mutex m_queue_lock;
  std::condition_variable m_queue_cv;
  std::vector<Request> m_pending;
  std::atomic_bool m_running{false};

  std::promise<void> m_replayedPromise;
  std::shared_future<void> m_replayed;

  mutable std::mutex m_store_lock;
  ControlValues m_store;
  std::atomic_bool m_storePending{false};

  std::mutex m_idle_lock;
  std::condition_variable m_idle_cv;
  int64_t m_idleAsked{0}, m_idleDone{0};

  /// How many pollable controls have a listener. Zero means the worker sleeps.
  std::atomic_int m_polled{0};

  QObject m_context;
};

}
