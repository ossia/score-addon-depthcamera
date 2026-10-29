#include "BackendRegistry.hpp"

#include <ossia/detail/dylib_loader.hpp>

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QSettings>

#include <algorithm>
#include <cstddef>
#include <cstring>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace Gfx::DepthCamera
{
namespace
{

/// Folder containing this plug-in binary -- not the executable folder. Backends
/// installed as packages sit beside their own SDK copies, far from score.
QString thisBinaryFolder()
{
#if defined(_WIN32)
  HMODULE mod{};
  if(!GetModuleHandleExW(
         GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
             | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
         reinterpret_cast<LPCWSTR>(&thisBinaryFolder), &mod))
    return {};
  wchar_t path[MAX_PATH]{};
  if(GetModuleFileNameW(mod, path, MAX_PATH) == 0)
    return {};
  return QFileInfo{QString::fromWCharArray(path)}.absolutePath();
#else
  Dl_info info{};
  if(dladdr(reinterpret_cast<const void*>(&thisBinaryFolder), &info) == 0
     || !info.dli_fname)
    return {};
  return QFileInfo{QString::fromUtf8(info.dli_fname)}.absolutePath();
#endif
}

#if defined(_WIN32)
constexpr auto backend_glob = "score_depthcam_*.dll";
#elif defined(__APPLE__)
constexpr auto backend_glob = "score_depthcam_*.dylib";
#else
constexpr auto backend_glob = "score_depthcam_*.so";
#endif

/// The one package all the backends ship in.
///
/// Deliberately a single directory rather than one per SDK: the backends are a
/// few megabytes each, several of them need auxiliary files next to them (the
/// OrbbecSDK's extensions/ tree, k4a's depth engine), and a user who plugs in a
/// camera wants it to work rather than to work out which of five packages they
/// need.
constexpr auto package_name = "depth-camera";

}

std::size_t BackendRegistry::tableSize(uint32_t abi_version) noexcept
{
  switch(abi_version)
  {
    case 3:
      return offsetof(depthcam_backend_v1, get_intrinsics);
    case 4:
      return sizeof(depthcam_backend_v1);
    default:
      return 0;
  }
}

std::shared_ptr<const depthcam_backend_v1>
BackendRegistry::adoptTable(const depthcam_backend_v1* api) noexcept
{
  if(!api)
    return {};
  const auto version = api->abi_version;
  if(version < DEPTHCAM_ABI_VERSION_MIN || version > DEPTHCAM_ABI_VERSION)
    return {};
  const std::size_t n = tableSize(version);
  if(n == 0)
    return {};

  auto table = std::make_shared<depthcam_backend_v1>();
  std::memset(table.get(), 0, sizeof(depthcam_backend_v1));
  std::memcpy(table.get(), api, n);
  return table;
}

BackendRegistry::BackendRegistry() = default;
BackendRegistry::~BackendRegistry() = default;

BackendRegistry& BackendRegistry::instance()
{
  static BackendRegistry reg;
  return reg;
}

QString BackendRegistry::backendOf(const QString& uri)
{
  const int colon = uri.indexOf(':');
  return colon > 0 ? uri.left(colon) : QString{};
}

std::vector<QString> BackendRegistry::searchPaths() const
{
  std::vector<QString> paths;

  // 1. Explicit override, for development and for unusual installs.
  if(const auto env = qEnvironmentVariable("SCORE_DEPTHCAM_BACKEND_PATH");
     !env.isEmpty())
  {
#if defined(_WIN32)
    const auto sep = QLatin1Char(';');
#else
    const auto sep = QLatin1Char(':');
#endif
    for(const auto& p : env.split(sep, Qt::SkipEmptyParts))
      paths.push_back(p);
  }

  // 2. The installed package: <Library root>/packages/depth-camera/.
  //    Read straight from QSettings rather than through Library::Settings so the
  //    registry stays usable outside a document context -- the enumerators run
  //    before any document exists.
  if(const auto lib = QSettings{}.value("Library/RootPath").toString();
     !lib.isEmpty())
  {
    paths.push_back(lib + "/packages/" + package_name);

    // And support/, which is where score puts a package whose manifest says
    // kind: "support". Nothing forces the choice today, so both are searched.
    paths.push_back(lib + "/support/" + package_name);
  }

  // 3. Beside the plug-in, for a self-contained or bundled build.
  if(const auto folder = thisBinaryFolder(); !folder.isEmpty())
  {
    paths.push_back(folder);
    paths.push_back(folder + "/" + package_name);
  }

#if defined(SCORE_DEPTHCAM_BUILD_BACKEND_DIR)
  // 4. The build tree.
  paths.push_back(QStringLiteral(SCORE_DEPTHCAM_BUILD_BACKEND_DIR));
#endif

  return paths;
}

void BackendRegistry::onChanged(void* user)
{
  // Called from a backend thread. Just forward; the caller marshals.
  if(auto* self = static_cast<BackendRegistry*>(user); self && self->m_changed)
    self->m_changed();
}

void BackendRegistry::load()
{
  if(m_loaded)
    return;
  m_loaded = true;

  QStringList seen;
  for(const auto& dir_path : searchPaths())
  {
    QDir dir{dir_path};
    if(!dir.exists())
      continue;

    for(const auto& file : dir.entryList({backend_glob}, QDir::Files))
    {
      const auto full = dir.absoluteFilePath(file);

      // The same backend may be reachable through several search paths; the
      // first one wins.
      if(seen.contains(file))
        continue;
      seen << file;

      std::unique_ptr<ossia::dylib_loader> lib;
      try
      {
        lib = std::make_unique<ossia::dylib_loader>(full.toUtf8().constData());
      }
      catch(const std::exception& e)
      {
        qDebug() << "[depthcam] could not load" << full << ":" << e.what();
        continue;
      }

      auto entry = lib->symbol<score_depthcam_backend_v1_fn>(
          "score_depthcam_backend_v1");
      if(!entry)
      {
        qDebug() << "[depthcam]" << full << "has no score_depthcam_backend_v1";
        continue;
      }

      const depthcam_backend_v1* api = entry();
      if(!api)
        continue;

      // Older backends are accepted: the struct only ever grows, and the copy
      // fills in what they predate. A newer one is not -- it may rely on the
      // host reading members this build does not know about.
      auto table = adoptTable(api);
      if(!table)
      {
        qDebug() << "[depthcam]" << full << "speaks ABI" << api->abi_version
                 << "but this build accepts" << DEPTHCAM_ABI_VERSION_MIN << "to"
                 << DEPTHCAM_ABI_VERSION << "- skipping";
        continue;
      }
      api = table.get();

      Backend b;
      b.api = api;
      b.table = std::move(table);
      b.abi_version = api->abi_version;
      b.name = QString::fromUtf8(api->name ? api->name : "");
      b.display_name
          = QString::fromUtf8(api->display_name ? api->display_name : api->name);
      b.path = full;
      // SDKs that load auxiliary blobs at runtime (the OrbbecSDK's extensions/
      // tree, k4a's depth engine) get told where their own package lives.
      b.resource_dir = dir.absolutePath();

      if(api->init)
      {
        const auto res = b.resource_dir.toUtf8();
        b.initialized = api->init(res.constData()) != 0;
        if(!b.initialized)
        {
          const char* err = api->last_error ? api->last_error() : nullptr;
          qDebug() << "[depthcam] backend" << b.name << "failed to initialise:"
                   << (err ? err : "no reason given");
          continue;
        }
      }

      if(api->set_changed_callback)
      {
        api->set_changed_callback(&BackendRegistry::onChanged, this);
        b.hotplug = true;
      }

      qDebug() << "[depthcam] loaded backend" << b.name << "(ABI" << b.abi_version
               << ") from" << full;

      // A backend can initialise successfully and still be unusable -- the k4a
      // one stays registered when libk4a is absent so the browser can explain
      // itself. Without this the only symptom is a camera silently missing from
      // the device list.
      if(api->last_error)
        if(const char* err = api->last_error(); err && *err)
          qDebug() << "[depthcam]  " << b.name << "reports:" << err;

      m_libs.push_back(std::move(lib));
      m_backends.push_back(std::move(b));
    }
  }

  if(m_backends.empty())
  {
    qDebug() << "[depthcam] no camera backends found. Install one from the "
                "package manager, or set SCORE_DEPTHCAM_BACKEND_PATH.";
  }
}

void BackendRegistry::shutdown()
{
  // Every SDK here owns background threads that outlive the last device: a
  // hot-plug watcher polling libusb, the OrbbecSDK's GVCP network discovery,
  // spdlog's flusher... They only stop when the SDK context is destroyed, and
  // this registry is a function-local static, so without this the contexts were
  // destroyed by __cxa_finalize during exit(). By then the backend library's
  // *other* globals may already be gone -- easylogging++'s registered-logger
  // map in librealsense's case -- and the watcher thread, still running one
  // last poll, faults on them:
  //
  //   thread: el::base::RegisteredLoggers::get   <- statics already destroyed
  //           librealsense::platform::usb_context::usb_context
  //           librealsense::polling_device_watcher::polling
  //   main:   active_object<...>::stop
  //           ~rs2_context  <- from exit()
  //
  // So tear the contexts down explicitly, while the process is still healthy
  // and the threads have everything they expect to find.
  for(auto& b : m_backends)
  {
    if(!b.api)
      continue;
    if(b.api->set_changed_callback)
      b.api->set_changed_callback(nullptr, nullptr);
    if(b.initialized && b.api->shutdown)
    {
      b.api->shutdown();
      b.initialized = false;
    }
  }
  m_changed = {};
}

void BackendRegistry::unload()
{
  shutdown();

  m_backends.clear();
  m_libs.clear();
  m_loaded = false;
}

const depthcam_backend_v1* BackendRegistry::find(const QString& backend) const noexcept
{
  for(const auto& b : m_backends)
    if(b.name == backend)
      return b.api;
  return nullptr;
}

void BackendRegistry::enumerate(
    const std::function<void(const depthcam_device_info&)>& f) const
{
  if(!f)
    return;

  for(const auto& b : m_backends)
  {
    if(!b.initialized || !b.api || !b.api->enumerate)
      continue;

    // The callback is a plain function pointer across the C boundary, so the
    // std::function travels as the user pointer.
    auto trampoline = [](const depthcam_device_info* info, void* user) {
      if(info)
        (*static_cast<const std::function<void(const depthcam_device_info&)>*>(user))(
            *info);
    };
    b.api->enumerate(trampoline, const_cast<void*>(static_cast<const void*>(&f)));
  }
}

bool BackendRegistry::needsRescanOnDemand() const noexcept
{
  return std::any_of(m_backends.begin(), m_backends.end(), [](const Backend& b) {
    return b.initialized && !b.hotplug;
  });
}

void BackendRegistry::setChangedCallback(std::function<void()> f)
{
  m_changed = std::move(f);
}

}
