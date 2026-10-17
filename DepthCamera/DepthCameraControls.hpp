#pragma once

/**
 * @file DepthCameraControls.hpp
 * @brief Publish a camera's settings as `<cam>/controls/<group>/<name>`.
 *
 * The backend side of this is depthcam_abi.h's list_controls/get_control/
 * set_control. Nothing here names a specific control or a specific camera: an
 * Orbbec publishes its ~50 properties, a RealSense its options, an Azure Kinect
 * its colour controls and a Kinect v1 its tilt motor, all through the same
 * three calls, and the grouping comes from the identifier the backend chose.
 *
 * Three things shape the implementation:
 *
 *  - **Nothing touches the camera on a caller's thread.** Every get_control and
 *    set_control is a synchronous transfer -- a USB vendor request, or a TCP
 *    round trip for a camera on the network -- and the callers here are the
 *    GUI, the execution engine and the network layer. A worker thread owns all
 *    of it; the callers queue and return. Unfolding a node in the explorer makes
 *    score listen to every child at once, which is how a directly-polling
 *    version made the whole application stutter twice a second.
 *
 *  - **No SDK here reports a control changing.** Orbbec, librealsense, k4a,
 *    libfreenect and libfreenect2 all only offer a getter, so a read-only
 *    sensor -- a temperature, an accelerometer -- can only be tracked by asking
 *    repeatedly. Only read-only controls are polled, and only while something is
 *    listening: ossia calls protocol_base::observe() when a parameter gains its
 *    first callback and again when it loses its last. A setting we can write is
 *    one we already know the value of; it is re-read on an explicit refresh
 *    rather than continuously.
 *
 *  - **Never push into a parameter from inside that parameter's callback.**
 *    callback_container::send holds a non-recursive mutex while it runs
 *    callbacks, so a corrective push made inline deadlocks the writing thread.
 *    Everything the worker learns is published from the Qt thread instead.
 */

#include <ossia/network/base/device.hpp>
#include <ossia/network/base/node.hpp>
#include <ossia/network/base/parameter.hpp>
#include <ossia/network/value/value.hpp>

#include <QObject>

#include <DepthCamera/Backend/depthcam_abi.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace Gfx::DepthCamera
{

class ControlTree
{
public:
  /**
   * @brief Enumerates the camera's controls and mirrors them into the tree.
   *
   * Does nothing at all if the backend predates ABI 2 or has no controls to
   * offer, so a device whose backend implements none of this simply gets no
   * `controls` node rather than an empty one.
   *
   * Only the descriptors are read here. The values are seeded from the
   * backend's defaults and refreshed by the worker a moment later: reading
   * forty controls from the camera would otherwise happen inline in
   * reconnect(), on the GUI thread, and on a networked camera that is seconds.
   */
  ControlTree(
      const depthcam_backend_v1& backend, depthcam_device& device,
      ossia::net::device_base& dev, ossia::net::node_base& parent);
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

private:
  struct Entry
  {
    std::string id; ///< as the backend spelled it, e.g. "color/exposure"
    ossia::net::parameter_base* param{};
    int kind{};
    int access{};
    /// Index into m_descs, for the range and the enum labels.
    int desc{-1};

    /// Read-only, so the only way to track it is to ask. Set once at build
    /// time; the worker reads it.
    bool pollable{};

    /// Number of things listening. Touched from the Qt thread, read by the
    /// worker.
    std::atomic_int observers{0};

    /// Last value published. Qt thread only.
    double last{};
    bool has_last{};
  };

  /// One thing for the worker to do. A write carries a value; a read does not.
  struct Request
  {
    int entry{};
    double value{};
    bool is_write{};
  };

  Entry* find(const ossia::net::parameter_base& param) noexcept;
  int indexOf(const ossia::net::parameter_base& param) noexcept;

  /// Qt thread only.
  void publish(Entry& e, double v);

  void run();
  void post(Request r);
  void wake();

  const depthcam_backend_v1& m_backend;
  depthcam_device& m_device;

  std::vector<std::unique_ptr<Entry>> m_entries;

  /// The descriptors as the backend gave them, kept for the ranges and the enum
  /// labels. Their strings belong to the backend and stay valid until close,
  /// which is why this object has to be destroyed before the device is closed.
  std::vector<depthcam_control> m_descs;

  std::thread m_worker;
  std::mutex m_queue_lock;
  std::condition_variable m_queue_cv;
  std::vector<Request> m_pending;
  std::atomic_bool m_running{false};

  /// How many read-only controls have a listener. Zero means the worker sleeps.
  std::atomic_int m_polled{0};

  QObject m_context;
};

}
