#pragma once

#include <DepthCamera/Backend/depthcam_abi.h>

#include <QString>

#include <functional>
#include <memory>
#include <vector>

namespace ossia
{
class dylib_loader;
}

namespace Gfx::DepthCamera
{

/**
 * @brief Finds, loads and owns the camera backend libraries.
 *
 * Backends are plain shared objects discovered on a search path and opened with
 * dlopen/RTLD_LOCAL. The plug-in deliberately has no link-time dependency on any
 * of them: it is meant to be statically built into ossia score, so a user with
 * no depth camera downloads nothing, and a missing or broken backend costs one
 * device family rather than the whole addon.
 */
class BackendRegistry
{
public:
  struct Backend
  {
    const depthcam_backend_v1* api{};
    QString name;         ///< api->name, cached as a QString for comparisons
    QString display_name;
    QString path;         ///< the library it came from
    QString resource_dir; ///< passed to init(); where the SDK's blobs live
    bool initialized{};
    /// The backend reports hot-plug through set_changed_callback.
    bool hotplug{};
  };

  static BackendRegistry& instance();

  /// Discover and initialise everything on the search path. Idempotent.
  void load();

  /**
   * @brief Shut the SDKs down, keeping the libraries mapped. Idempotent.
   *
   * Must run while the process is still healthy -- see the note in
   * BackendRegistry::shutdown().
   */
  void shutdown();

  void unload();

  const std::vector<Backend>& backends() const noexcept { return m_backends; }

  /// Null if no backend of that name is loaded.
  const depthcam_backend_v1* find(const QString& backend) const noexcept;

  /// The backend named by a device URI ("orbbec:sn:..." -> "orbbec").
  static QString backendOf(const QString& uri);

  /// Ask every backend for its devices.
  void enumerate(const std::function<void(const depthcam_device_info&)>& f) const;

  /// Some loaded backend has no hot-plug: new devices need a new enumeration.
  bool needsRescanOnDemand() const noexcept;

  /// Invoked (on an arbitrary thread) when any backend reports a hot-plug.
  void setChangedCallback(std::function<void()> f);

private:
  BackendRegistry();
  ~BackendRegistry();
  BackendRegistry(const BackendRegistry&) = delete;
  BackendRegistry& operator=(const BackendRegistry&) = delete;

  static void onChanged(void* user);

  std::vector<QString> searchPaths() const;

  std::vector<Backend> m_backends;
  std::vector<std::unique_ptr<ossia::dylib_loader>> m_libs;
  std::function<void()> m_changed;
  bool m_loaded{};
};

}
