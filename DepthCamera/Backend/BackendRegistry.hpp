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
    /// The host's own copy of the backend's table, at this build's ABI. A
    /// backend built against an older header has a shorter struct; the copy has
    /// every member it lacks set to null, so the rest of the plug-in tests a
    /// pointer instead of an ABI version. Never the backend's own pointer.
    const depthcam_backend_v1* api{};
    std::shared_ptr<const depthcam_backend_v1> table;
    uint32_t abi_version{};
    QString name;         ///< api->name, cached as a QString for comparisons
    QString display_name;
    QString path;         ///< the library it came from
    QString resource_dir; ///< passed to init(); where the SDK's blobs live
    bool initialized{};
    /// The backend reports hot-plug through set_changed_callback.
    bool hotplug{};
  };

  static BackendRegistry& instance();

  /**
   * @brief Copies what a backend of @p api->abi_version provides into a full
   * table of this build's ABI, nulling everything that version does not have.
   *
   * Reads exactly as many bytes as that version's struct has, never more.
   * Null if the version is outside [DEPTHCAM_ABI_VERSION_MIN,
   * DEPTHCAM_ABI_VERSION].
   */
  static std::shared_ptr<const depthcam_backend_v1>
  adoptTable(const depthcam_backend_v1* api) noexcept;

  /// Bytes in the depthcam_backend_v1 of a given ABI version; 0 if unknown.
  static std::size_t tableSize(uint32_t abi_version) noexcept;

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
