// Loads one backend module the way BackendRegistry does and runs it through
// init / enumerate / shutdown. No camera needed: zero devices is a pass; a
// module that does not load, has no entry point, fails or crashes is not.
//
//   depthcam-probe <path/to/score_depthcam_X.{so,dylib,dll}>
//
// Built with the backends (see Backends/CMakeLists.txt) and run by the
// Package workflow on the DepthAI backend, whose SDK is linked in statically:
// this is what catches a dependency missing from the link or from the package.
#include <depthcam_abi.h>

#include <cstdio>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

int main(int argc, char** argv)
{
  if(argc < 2)
  {
    std::fprintf(stderr, "usage: %s <backend module>\n", argv[0]);
    return 2;
  }
  const std::string path = argv[1];
  std::string dir = path;
  if(auto pos = dir.find_last_of("/\\"); pos != std::string::npos)
    dir.resize(pos);
  else
    dir = ".";

#if defined(_WIN32)
  HMODULE lib = LoadLibraryExA(path.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
  if(!lib)
  {
    std::fprintf(stderr, "LoadLibrary(%s) failed: %lu\n", path.c_str(), GetLastError());
    return 1;
  }
  auto entry = reinterpret_cast<score_depthcam_backend_v1_fn>(
      GetProcAddress(lib, "score_depthcam_backend_v1"));
#else
  void* lib = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if(!lib)
  {
    std::fprintf(stderr, "dlopen failed: %s\n", dlerror());
    return 1;
  }
  auto entry = reinterpret_cast<score_depthcam_backend_v1_fn>(
      dlsym(lib, "score_depthcam_backend_v1"));
#endif
  if(!entry)
  {
    std::fprintf(stderr, "no score_depthcam_backend_v1 in %s\n", path.c_str());
    return 1;
  }

  const depthcam_backend_v1* b = entry();
  if(!b || !b->init || !b->enumerate || !b->shutdown)
  {
    std::fprintf(stderr, "incomplete backend table\n");
    return 1;
  }
  std::printf("%s (%s): ABI %u\n", b->name, b->display_name, b->abi_version);
  if(b->abi_version < DEPTHCAM_ABI_VERSION_MIN)
  {
    std::fprintf(stderr, "ABI %u is older than %d\n", b->abi_version, DEPTHCAM_ABI_VERSION_MIN);
    return 1;
  }

  if(!b->init(dir.c_str()))
  {
    const char* e = b->last_error ? b->last_error() : nullptr;
    std::fprintf(stderr, "init failed: %s\n", e ? e : "(no error message)");
    return 1;
  }

  // enumerate() returns success, not a count: the devices arrive through the
  // callback.
  int devices = 0;
  const int ok = b->enumerate(
      [](const depthcam_device_info* info, void* user) {
        ++*static_cast<int*>(user);
        std::printf("  device: %s\n", info && info->uri ? info->uri : "?");
      },
      &devices);
  std::printf("enumerate: %s, %d device(s)\n", ok ? "ok" : "failed", devices);
  if(!ok)
  {
    const char* e = b->last_error ? b->last_error() : nullptr;
    std::fprintf(stderr, "enumerate failed: %s\n", e ? e : "(no error message)");
    b->shutdown();
    return 1;
  }
  b->shutdown();
  return 0;
}
