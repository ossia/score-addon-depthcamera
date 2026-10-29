// The one exported symbol. Built twice: as score_depthcam_synthetic at this
// header's ABI, and with SYNTHETIC_ABI_VERSION=3 as a stand-in for a backend
// package built before ABI 4, whose struct ends at active_streams.
#include "SyntheticBackend.hpp"

#include <cstddef>
#include <cstring>

#if SYNTHETIC_ABI_VERSION == 3 && !defined(_WIN32)
#include <sys/mman.h>
#include <unistd.h>
#endif

extern "C" DEPTHCAM_EXPORT const depthcam_backend_v1* score_depthcam_backend_v1(void)
{
#if SYNTHETIC_ABI_VERSION == 3
  // An ABI 3 struct is only this long. Hand out exactly that many bytes, and
  // on POSIX put them right before a page that cannot be read, so a host that
  // reads one byte too far faults instead of reading the next variable.
  static const depthcam_backend_v1* table = [] {
    const std::size_t n = offsetof(depthcam_backend_v1, get_intrinsics);
#if !defined(_WIN32)
    const std::size_t page = std::size_t(sysconf(_SC_PAGESIZE));
    auto* base = static_cast<unsigned char*>(
        mmap(nullptr, 2 * page, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
    if(base == MAP_FAILED)
      return static_cast<const depthcam_backend_v1*>(nullptr);
    mprotect(base + page, page, PROT_NONE);
    unsigned char* at = base + page - n;
#else
    static unsigned char storage[sizeof(depthcam_backend_v1)];
    unsigned char* at = storage;
#endif
    std::memcpy(at, depthcam_synthetic::backend(), n);
    auto* t = reinterpret_cast<depthcam_backend_v1*>(at);
    // Its own name, so a test can load both and tell them apart.
    t->name = "synthetic_abi3";
    t->display_name = "Synthetic camera (ABI 3)";
    return static_cast<const depthcam_backend_v1*>(t);
  }();
  return table;
#else
  return depthcam_synthetic::backend();
#endif
}
