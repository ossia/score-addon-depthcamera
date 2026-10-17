### Depth camera support for ossia score.
###
### Two halves that build independently:
###
###  - the **plug-in**, which links no camera SDK at all and is meant to be
###    statically built into score. It reaches cameras only through
###    DepthCamera/Backend/depthcam_abi.h, a pure-C ABI, and is a few hundred
###    kilobytes.
###  - the **backends**, one shared object per camera SDK, built from the
###    vendored submodules and shipped as a downloadable package.
###
### score's own CI wants only the first: cloning this repository recursively
### pulls the OrbbecSDK, libfreenect, libfreenect2, librealsense, the
### Oak3DVision SDK and depthai-core, and building them takes longer than the
### rest of score put together -- depthai alone bootstraps a vcpkg and builds a
### dependency tree before it builds itself. So a checkout without submodules builds the plug-in and nothing
### else -- which is a complete, working device that reports "no camera backend
### is installed" until a package is.
###
### 3rdparty/oaksdk is vendored *binaries*: Oak3DVision publishes no sources for
### libPointCloud, so nothing there is compiled. See Backends/Oak/README.md.
###
### 3rdparty/depthai-core is built, but as a nested CMake project rather than
### with add_subdirectory, and is the only SDK here that needs the network at
### build time. See Backends/DepthAI/README.md.
cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

if(NOT (WIN32 OR APPLE OR LINUX))
  return()
endif()

### Whether to build the camera SDKs.
###
### Defaults to whether the submodules are actually there, so neither case
### needs a flag: a recursive clone builds everything, score's CI clones
### without submodules and builds the plug-in. Set it explicitly to override.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/OrbbecSDK_v2/CMakeLists.txt")
  set(SCORE_DEPTHCAM_BACKENDS_DEFAULT ON)
else()
  set(SCORE_DEPTHCAM_BACKENDS_DEFAULT OFF)
endif()

option(SCORE_DEPTHCAM_BUILD_BACKENDS
       "Build the camera SDK backends alongside the plug-in"
       ${SCORE_DEPTHCAM_BACKENDS_DEFAULT})

### Whether to build *only* the backends, with no score at all.
###
### This is what CI packages. The backends share no code with the plug-in --
### they talk to it through the pure-C depthcam_abi.h and are dlopen'd at
### runtime -- so building them needs a C++ compiler and the vendored SDKs and
### nothing else: no score checkout, no Qt, no ossia SDK. Keeping the package
### job free of all that is not just a speed-up, it is what makes it possible to
### release a package without a working score build on that platform.
option(SCORE_DEPTHCAM_BACKENDS_ONLY
       "Build the camera backends and their package, without the score plug-in"
       OFF)

if(SCORE_DEPTHCAM_BACKENDS_ONLY)
  set(SCORE_DEPTHCAM_BUILD_BACKENDS ON CACHE BOOL "" FORCE)

  ### Static CRT on MSVC.
  ###
  ### A backend built /MD imports MSVCP140.dll and VCRUNTIME140.dll, so it will
  ### not load on a machine without the Visual C++ redistributable -- and score
  ### on Windows is built with llvm-mingw, so it does not bring one. A package
  ### the user downloads has to stand on its own.
  ###
  ### Per-module CRTs are what the ABI already assumes: depthcam_abi.h says in
  ### as many words that no C++ type, container or allocator crosses the
  ### boundary, and every frame is handed back through the backend's own
  ### release() hook. It also settles a mismatch that was already there, since
  ### the OrbbecSDK forces /MT on its vendored libusb and libjpeg-turbo.
  ###
  ### Only in this build. Inside score the plug-in has to match whatever score
  ### was built with.
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

if(NOT SCORE_DEPTHCAM_BACKENDS_ONLY AND NOT TARGET score_lib_base)
  include(ScoreExternalAddon)
endif()

project(score_addon_orbbec LANGUAGES C CXX)

if(SCORE_DEPTHCAM_BUILD_BACKENDS
   AND NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/OrbbecSDK_v2/CMakeLists.txt")
  message(FATAL_ERROR
    "score-addon-orbbec: SCORE_DEPTHCAM_BUILD_BACKENDS is on but "
    "3rdparty/OrbbecSDK_v2 is not checked out. Run "
    "`git submodule update --init --recursive`, or configure with "
    "-DSCORE_DEPTHCAM_BUILD_BACKENDS=OFF to build the plug-in alone.")
endif()

if(SCORE_DEPTHCAM_BUILD_BACKENDS)

# Where the SDK's extensions/ tree lands in this build. The addon needs to hand
# this to ob::Context::setExtensionsDirectory() before any context is created,
# because the SDK resolves it against the process working directory otherwise
# (see EnvConfig::getExtensionsDirectory, which caches on first use).
set(SCORE_ORBBEC_RUNTIME_DIR "${CMAKE_BINARY_DIR}/orbbec" CACHE INTERNAL "")

### --- Vendored OrbbecSDK -----------------------------------------------------
###
### Built here rather than consumed prebuilt on purpose: building it ourselves is
### what lets us keep its bundled libusb / libuvc / libjpeg / spdlog copies
### hidden (the SDK's own cmake/linux_config.cmake sets -fvisibility=hidden and
### -Wl,-Bsymbolic), so they cannot be interposed by the other depth-camera SDKs
### loaded into the same process.
if(NOT TARGET ob::OrbbecSDK)
  # These are options() in the SDK, so they must be cache entries to take effect.
  set(OB_BUILD_EXAMPLES OFF CACHE INTERNAL "")
  set(OB_BUILD_TESTS    OFF CACHE INTERNAL "")
  set(OB_BUILD_DOCS     OFF CACHE INTERNAL "")
  set(OB_BUILD_TOOLS    OFF CACHE INTERNAL "")
  set(OB_INSTALL_EXAMPLES_SOURCE OFF CACHE INTERNAL "")
  set(OB_BUILD_MAIN_PROJECT OFF CACHE INTERNAL "")

  # The SDK is pinned third-party code compiled by whatever toolchain score is
  # built with, which is regularly newer than the one its authors used. Every
  # new diagnostic would otherwise be a build failure.
  set(OB_WARNINGS_AS_ERRORS OFF CACHE INTERNAL "")

  block()
    # The SDK's own CMakeLists forces C++11 and a pile of -Wno-* at its directory
    # scope; SYSTEM keeps its headers from spamming our warning output.
    set(CMAKE_UNITY_BUILD 0)

    # NOTE: no -march=native here, ever. This tree is shipped as a binary package
    # and a -march=native build faults on any machine other than the builder's.

    # The SDK copies its runtime blobs (filters, frameprocessor, depthengine,
    # firmwareupdater) to OB_RESOURCES_OUTPUT_DIRECTORY, which it derives from
    # CMAKE_LIBRARY_OUTPUT_DIRECTORY. score does not necessarily set that, and an
    # empty value would make it copy to "/extensions", so pin it explicitly.
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${SCORE_ORBBEC_RUNTIME_DIR}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${SCORE_ORBBEC_RUNTIME_DIR}")

    add_subdirectory(3rdparty/OrbbecSDK_v2 SYSTEM)
  endblock()

  # The SDK sets -fvisibility=hidden, but that does not cover dependencies whose
  # own headers annotate their API with an explicit default visibility. libusb
  # (DEFAULT_VISIBILITY) and lz4 (LZ4LIB_API) both do, so without this they end
  # up as 140 globally-visible symbols on libOrbbecSDK.so -- and the moment
  # libfreenect/libfreenect2 (which link the *system* libusb-1.0) are loaded in
  # the same process, ELF interposition silently binds both SDKs to whichever
  # libusb was loaded first. Force everything coming from a static archive to be
  # local to the library.
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    include(CheckLinkerFlag)
    check_linker_flag(CXX "LINKER:--exclude-libs,ALL" OB_HAS_EXCLUDE_LIBS)
    if(OB_HAS_EXCLUDE_LIBS)
      target_link_options(OrbbecSDK PRIVATE "LINKER:--exclude-libs,ALL")
    endif()

    # libOrbbecSDK otherwise comes out with an RWE PT_GNU_STACK, inherited from
    # the surrounding build rather than from anything in the SDK (no assembly is
    # compiled here -- only jsimd_none.c). A hardened loader refuses to dlopen a
    # module that pulls in such a library, which breaks the backend.
    check_linker_flag(CXX "LINKER:-z,noexecstack" OB_HAS_NOEXECSTACK)
    if(OB_HAS_NOEXECSTACK)
      target_link_options(OrbbecSDK PRIVATE "LINKER:-z,noexecstack")
    endif()
  endif()

  # Windows on ARM: the alignment and undistortion filters are written against
  # the SSE intrinsics and reach NEON through the SSE2NEON.h the SDK bundles.
  # They pick it on `__ARM_NEON__ || __aarch64__ || __arm__` -- all of them gcc
  # and clang spellings, none of which MSVC defines -- so an ARM64 MSVC build
  # instead takes the x86 branch and dies in the toolchain's own header:
  # "xmmintrin.h(27): fatal error C1189: This header is specific to X86, X64,
  # ARM64, and ARM64EC targets". The SDK's windows_config.cmake defining
  # -D__SSSE3__ for every MSVC target is what gets it that far.
  #
  # Both macros are needed, and for different reasons. __ARM_NEON__ is what the
  # two filter headers test to decide *which* header to include; __NEON__ is
  # Orbbec's own, and the whole of their vendored SSE2NEON.h sits behind
  # `#ifdef __NEON__` on its second line -- so with only the first the file is
  # included and contributes nothing, and every _mm_* call becomes "identifier
  # not found" instead. It is the same pair the SDK's macos_config.cmake sets
  # for arm64.
  #
  # SSE2NEON.h does support _M_ARM64; it just insists on the conforming
  # preprocessor, which MSVC still does not use by default.
  if(MSVC AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(ARM64|arm64|aarch64)$" AND TARGET filter)
    target_compile_definitions(filter PRIVATE __ARM_NEON__ __NEON__)
    target_compile_options(filter PRIVATE /Zc:preprocessor)
  endif()
endif()

### --- The depth-camera package ----------------------------------------------
###
### Everything the backends need lands in one directory, laid out exactly as the
### shipped package is, so that whatever works in the build tree works
### installed. At runtime BackendRegistry looks for it in
### <library root>/packages/depth-camera, beside the plug-in binary, and here.
###
### One package rather than one per SDK. The backends are a few megabytes each,
### three of them need auxiliary files sitting next to them, and a user who
### plugged in a camera should not have to work out which of five downloads
### theirs is in.
set(SCORE_DEPTHCAM_PACKAGE_NAME "depth-camera")
set(SCORE_DEPTHCAM_BACKEND_DIR
    "${CMAKE_BINARY_DIR}/${SCORE_DEPTHCAM_PACKAGE_NAME}" CACHE INTERNAL "")

### Normalised to the keys score publishes under before anything is decided.
### x86_64h is a flavour of x86_64, not an architecture of its own, so a fat
### x86_64 + x86_64h binary is two slices but one package key; only a genuinely
### mixed Intel/Apple Silicon build has no key to publish under.
set(_depthcam_cpu "${CMAKE_SYSTEM_PROCESSOR}")
if(APPLE AND CMAKE_OSX_ARCHITECTURES)
  set(_depthcam_keys "")
  foreach(_a ${CMAKE_OSX_ARCHITECTURES})
    if(_a MATCHES "^(x86_64|x86_64h|amd64)$")
      list(APPEND _depthcam_keys "x86_64")
    elseif(_a MATCHES "^(arm64|arm64e|aarch64)$")
      list(APPEND _depthcam_keys "aarch64")
    else()
      list(APPEND _depthcam_keys "${_a}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _depthcam_keys)
  list(LENGTH _depthcam_keys _n_arch)
  set(CMAKE_OSX_ARCHITECTURES_KEYS "${_depthcam_keys}")
endif()

add_subdirectory(Backends)

### The manifest score writes after installing a package, generated here so the
### build tree is a valid package too.
###
### The architecture string has to match score::addonArchitecture() exactly:
### that is the key the package manager looks up in the remote manifest to
### decide which zip to download.
if(WIN32)
  set(SCORE_DEPTHCAM_OS "windows")
elseif(APPLE)
  set(SCORE_DEPTHCAM_OS "darwin")
else()
  set(SCORE_DEPTHCAM_OS "linux")
endif()

# CMAKE_OSX_ARCHITECTURES first, because on macOS it is the only one that tells
# the truth about a cross build: CMAKE_SYSTEM_PROCESSOR stays the *host's*, so
# an x86_64 package built on an Apple Silicon machine would otherwise be named
# darwin-aarch64 and hand an Intel score a set of arm64 binaries.
if(APPLE AND CMAKE_OSX_ARCHITECTURES)
  if(_n_arch GREATER 1)
    ### The *package* cannot be named, but the build is legitimate and score is
    ### built that way. Fatal only when a package was what was asked for.
    if(SCORE_DEPTHCAM_BACKENDS_ONLY)
      message(FATAL_ERROR
        "score-addon-orbbec: a universal binary has no single architecture key "
        "to publish under. Build one package per architecture.")
    endif()
    list(GET _depthcam_keys 0 _depthcam_cpu)
    message(STATUS
      "score-addon-orbbec: universal build (${CMAKE_OSX_ARCHITECTURES}); the "
      "package manifest is not publishable and says ${_depthcam_cpu}")
  else()
    set(_depthcam_cpu "${_depthcam_keys}")
  endif()
endif()

if(_depthcam_cpu MATCHES "^(aarch64|arm64|ARM64)$")
  set(SCORE_DEPTHCAM_CPU "aarch64")
elseif(_depthcam_cpu MATCHES "^(arm|armv7)")
  set(SCORE_DEPTHCAM_CPU "arm")
else()
  set(SCORE_DEPTHCAM_CPU "x86_64")
endif()

set(SCORE_DEPTHCAM_ARCH "${SCORE_DEPTHCAM_OS}-${SCORE_DEPTHCAM_CPU}")

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Deployment/package.json.in"
  "${SCORE_DEPTHCAM_BACKEND_DIR}/package.json"
  @ONLY)

### `cmake --build . --target depthcam-package` produces the zip that goes in
### the score-packages manifest. Named after the architecture because the
### contents are native code: one entry per platform in the manifest, keyed the
### way score::addonArchitecture() spells it.
###
### Staged and stripped rather than zipped in place. A release build of the
### Orbbec backend is 139MB unstripped and 12MB stripped -- the SDK is compiled
### into it -- and stripping the build tree itself would make the thing
### undebuggable for whoever built it.
set(SCORE_DEPTHCAM_STAGE "${CMAKE_BINARY_DIR}/depth-camera-stage")

add_custom_target(depthcam-package
  COMMAND ${CMAKE_COMMAND} -E rm -rf "${SCORE_DEPTHCAM_STAGE}"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${SCORE_DEPTHCAM_BACKEND_DIR}"
          "${SCORE_DEPTHCAM_STAGE}/${SCORE_DEPTHCAM_PACKAGE_NAME}"
  COMMAND ${CMAKE_COMMAND}
          -DDIR=${SCORE_DEPTHCAM_STAGE}/${SCORE_DEPTHCAM_PACKAGE_NAME}
          -DSTRIP=${CMAKE_STRIP}
          -P "${CMAKE_CURRENT_SOURCE_DIR}/Deployment/StripPackage.cmake"
  COMMAND ${CMAKE_COMMAND} -E rm -f
          "${CMAKE_BINARY_DIR}/depth-camera-${SCORE_DEPTHCAM_ARCH}.zip"
  COMMAND ${CMAKE_COMMAND} -E chdir "${SCORE_DEPTHCAM_STAGE}"
          ${CMAKE_COMMAND} -E tar cf
          "${CMAKE_BINARY_DIR}/depth-camera-${SCORE_DEPTHCAM_ARCH}.zip" --format=zip
          "${SCORE_DEPTHCAM_PACKAGE_NAME}"
  COMMENT "Packaging depth-camera-${SCORE_DEPTHCAM_ARCH}.zip"
  VERBATIM)

### So that `cmake --build . --target depthcam-package` is the whole recipe.
add_dependencies(depthcam-package depthcam-backends)

endif() # SCORE_DEPTHCAM_BUILD_BACKENDS

### Everything below needs score. Stop here when only the package was asked for.
if(SCORE_DEPTHCAM_BACKENDS_ONLY)
  return()
endif()

# General initialization
score_common_setup()

# Creation of the library.
#
# Note what is *not* here: any camera SDK. This target is meant to be statically
# linked into ossia score, so the protocol, settings widget and device browser
# are always present, while the SDKs arrive as downloadable backend packages
# that are dlopen'd at runtime. See DepthCamera/Backend/depthcam_abi.h.
add_library(score_addon_orbbec
  DepthCamera/Backend/depthcam_abi.h
  DepthCamera/Backend/BackendRegistry.hpp
  DepthCamera/Backend/BackendRegistry.cpp
  DepthCamera/ApplicationPlugin.hpp
  DepthCamera/ApplicationPlugin.cpp
  DepthCamera/DepthCameraSettings.hpp
  DepthCamera/DepthCameraSettings.cpp
  DepthCamera/DepthCameraStream.hpp
  DepthCamera/DepthCameraStream.cpp
  DepthCamera/DepthCameraDevice.hpp
  DepthCamera/DepthCameraDevice.cpp
  DepthCamera/DepthCameraProtocol.hpp
  DepthCamera/DepthCameraProtocol.cpp
  DepthCamera/DepthCameraControls.hpp
  DepthCamera/DepthCameraControls.cpp
  DepthCamera/DepthCameraImu.hpp
  DepthCamera/DepthCameraImu.cpp

  score_addon_orbbec.hpp
  score_addon_orbbec.cpp
)

target_include_directories(score_addon_orbbec
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/DepthCamera/Backend")

if(FFMPEG_INCLUDE_DIRS)
  target_include_directories(score_addon_orbbec SYSTEM PRIVATE ${FFMPEG_INCLUDE_DIRS})
endif()

# Only when there is a build tree to point at. Without it BackendRegistry looks
# in the installed package directory and beside the plug-in binary, which is
# exactly right for a plug-in built without backends.
if(SCORE_DEPTHCAM_BUILD_BACKENDS)
  target_compile_definitions(score_addon_orbbec
    PRIVATE
      SCORE_DEPTHCAM_BUILD_BACKEND_DIR="${SCORE_DEPTHCAM_BACKEND_DIR}"
  )
endif()

# Link
target_link_libraries(score_addon_orbbec PUBLIC
    score_plugin_engine
    score_plugin_media
    score_plugin_gfx
    score_plugin_avnd
)

set_target_properties(score_addon_orbbec PROPERTIES SCORE_CUSTOM_PCH 1)

# Target-specific options
setup_score_plugin(score_addon_orbbec)
