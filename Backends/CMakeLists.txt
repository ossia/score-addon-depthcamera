### Depth-camera backends.
###
### Each backend is a standalone shared object exporting exactly one symbol,
### score_depthcam_backend_v1, loaded with dlopen/RTLD_LOCAL by the score
### plug-in. None of this is linked into score: that is what lets the plug-in be
### statically built into the application while the SDKs ship as downloadable
### packages.
###
### The SDKs are git submodules under ../3rdparty, pinned to a tag, so a build is
### reproducible and needs no network at configure time.

set(SCORE_DEPTHCAM_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../3rdparty")

### Everything under here ends up inside a shared module, static archives
### included, so all of it has to be position-independent. score sets this for
### its whole build and the requirement went unnoticed; a standalone
### backends-only configuration fails at link time instead ("relocation
### R_X86_64_PC32 against symbol `_ZSt4cerr' can not be used when making a
### shared object").
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

### libfreenect and librealsense each declare a global `uninstall` target, and
### score already has one (3rdparty/sh4lt, 3rdparty/shmdata). CMake makes a
### duplicate custom target a hard error, and these are pinned upstream
### submodules we would rather not patch. Overriding the command here makes the
### duplicate a no-op; the override is directory-scoped, so it applies to the
### SDK subtrees added below and to nothing else in score.
function(add_custom_target)
  if("${ARGV0}" STREQUAL "uninstall" AND TARGET uninstall)
    return()
  endif()

  # librealsense's Windows 7 UVC driver target lists its DEPENDS as bare
  # filenames relative to the *source* drivers directory. ninja resolves them
  # against the build directory, finds nothing, and refuses to generate anything
  # at all: "'IntelRealSense_D400_series_win7.inf', needed by
  # 'realsense-driver', missing and no known rule to make it". The files
  # themselves are already copied by configure_file just above; only the DEPENDS
  # is wrong, and we ship no drivers, so drop it and keep the target so
  # librealsense's add_dependencies still resolves.
  if("${ARGV0}" STREQUAL "realsense-driver")
    _add_custom_target(realsense-driver)
    return()
  endif()

  _add_custom_target(${ARGV})
endfunction()

### Nothing in this tree is ever installed. The SDKs below all carry install()
### and install(EXPORT) rules for their own packaging, and those rules are a
### steady source of configure-time failures that have nothing to do with us:
### librealsense exports realsense2, realsense2 links the `usb` target we hand
### it, and CMake then refuses the export set because the vendored libusb behind
### it is in no export set either -- a chain with no end that only exists to
### produce files no one asked for.
###
### Gated on a directory-scoped variable rather than by overriding install()
### outright: command overrides are global and permanent, since a second override
### makes _install resolve to the first rather than to the builtin. A variable is
### inherited by subdirectories, invisible to sibling trees, and resolved against
### the caller's scope inside a function body, so this reaches the SDK subtrees
### added below and nothing else.
set(SCORE_DEPTHCAM_SUPPRESS_INSTALL 1)
function(install)
  if(NOT SCORE_DEPTHCAM_SUPPRESS_INSTALL)
    _install(${ARGV})
  endif()
endfunction()

### Settings every backend needs, whatever SDK it wraps.
###
### The visibility flags are not cosmetic. libfreenect, libfreenect2, k4a and the
### OrbbecSDK each bundle or link libusb / libuvc / libjpeg / a logging library,
### and those dependencies annotate their own API with explicit default
### visibility, which overrides -fvisibility=hidden: measured on libOrbbecSDK.so,
### 99 libusb_* and 41 LZ4_* symbols stayed globally visible until
### --exclude-libs was added. Two backends in one process would then bind to
### whichever copy the loader saw first.
function(score_depthcam_setup_backend Target)
  # The authoritative list of what this configuration produces. `depthcam-package`
  # depends on it, and CI compares it against what the platform is meant to
  # ship: a backend silently skipped because a dependency was missing would
  # otherwise just come out as a smaller zip.
  set_property(GLOBAL APPEND PROPERTY SCORE_DEPTHCAM_BACKENDS ${Target})

  set_target_properties(${Target} PROPERTIES
    # Stated, not inherited. Inside score every target is C++20 already, so the
    # backends compiled as C++20 by accident; standalone they get whatever the
    # compiler defaults to. Apple clang's default is old enough that
    # `std::lock_guard lock{m}` and std::size are both errors, and every backend
    # fills in its ABI vtable with designated initializers, which MSVC accepts
    # only at C++20 -- gcc and clang take them as an extension at any standard,
    # which is exactly why nobody noticed.
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_VISIBILITY_PRESET hidden
    C_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${SCORE_DEPTHCAM_BACKEND_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${SCORE_DEPTHCAM_BACKEND_DIR}"
  )

  target_include_directories(${Target} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../DepthCamera/Backend")

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    include(CheckLinkerFlag)

    check_linker_flag(CXX "LINKER:--exclude-libs,ALL" _has_exclude_libs)
    if(_has_exclude_libs)
      target_link_options(${Target} PRIVATE "LINKER:--exclude-libs,ALL")
    endif()

    # And state the one-symbol guarantee outright. --exclude-libs only covers
    # static archives; a version script also covers objects compiled straight
    # into the module, which matters because score dlopens everything in its
    # support/ directory with RTLD_GLOBAL.
    target_link_options(${Target} PRIVATE
      "LINKER:--version-script=${CMAKE_CURRENT_LIST_DIR}/backend.version")
    set_property(TARGET ${Target} APPEND PROPERTY
      LINK_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/backend.version")

    # Bind internal calls inside the library so a sibling backend can never
    # interpose them.
    check_linker_flag(CXX "LINKER:-Bsymbolic-functions" _has_bsymbolic)
    if(_has_bsymbolic)
      target_link_options(${Target} PRIVATE "LINKER:-Bsymbolic-functions")
    endif()

    # score links with -Wl,-zexecstack for its JIT plug-in, and everything built
    # in this tree inherits it. A hardened loader then refuses to dlopen the
    # module outright ("cannot enable executable stack as shared object
    # requires"). Nothing in a camera SDK needs an executable stack.
    check_linker_flag(CXX "LINKER:-z,noexecstack" _has_noexecstack)
    if(_has_noexecstack)
      target_link_options(${Target} PRIVATE "LINKER:-z,noexecstack")
    endif()

    # An explicit link flag rather than the RPATH target properties: score sets
    # CMAKE_SKIP_BUILD_RPATH, which silently drops BUILD_RPATH, so a backend in
    # the build tree ended up with no RPATH at all and dlopen could not find the
    # SDK sitting beside it ("libOrbbecSDK.so.2: cannot open shared object
    # file"). $ORIGIN is also exactly what a shipped package needs, so the build
    # tree and the package now resolve libraries the same way.
    target_link_options(${Target} PRIVATE "LINKER:-rpath,$ORIGIN")
    set_target_properties(${Target} PROPERTIES INSTALL_RPATH "$ORIGIN")
  elseif(APPLE)
    target_link_options(${Target} PRIVATE "LINKER:-rpath,@loader_path")
    set_target_properties(${Target} PROPERTIES INSTALL_RPATH "@loader_path")

    # CMake gives a MODULE library the .so suffix on macOS. BackendRegistry
    # globs score_depthcam_*.dylib there, and a plug-in loader that finds
    # nothing is indistinguishable from a package that failed to download.
    set_target_properties(${Target} PROPERTIES SUFFIX ".dylib")

    # The Mach-O counterpart of the version script below. Two-level namespaces
    # make macOS far less prone to the interposition this guards against, but
    # the guarantee is the same one the ABI documents, so state it on both.
    target_link_options(${Target} PRIVATE
      "LINKER:-exported_symbols_list,${CMAKE_CURRENT_LIST_DIR}/backend.symbols")
    set_property(TARGET ${Target} APPEND PROPERTY
      LINK_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/backend.symbols")
  endif()
endfunction()

### Copies a backend's udev rules next to it so the shipped package carries them.
### Every one of these SDKs talks to the camera over raw USB; without the rules
### the device nodes stay root-owned and the SDK reports no cameras, which is
### indistinguishable from "nothing is plugged in".
###
### configure_file rather than a POST_BUILD command: POST_BUILD only runs when
### the target relinks, so editing a .rules file otherwise leaves a stale copy in
### the output tree.
function(score_depthcam_install_udev_rules Dir)
  if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    return()
  endif()
  file(GLOB _rules "${Dir}/*.rules")
  foreach(r ${_rules})
    get_filename_component(_name "${r}" NAME)
    configure_file("${r}" "${SCORE_DEPTHCAM_BACKEND_DIR}/udev/${_name}" COPYONLY)
  endforeach()
endfunction()

### Several of these SDKs predate CMake 3.5 and declare a minimum that CMake 4
### refuses outright. This is the documented escape hatch.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE INTERNAL "")

### Never unity-build the vendored SDKs.
###
### They are pinned third-party trees that were not written for it, and a
### score build configured with -DCMAKE_UNITY_BUILD=1 otherwise drags them in:
### librealsense fails with ~28 redefinitions and ambiguous references once its
### translation units are concatenated (`redefinition of depth_sensor_snapshot`,
### `unexpected namespace name 'time'`), and libfreenect's C sources collide on
### a static/non-static `get_reply`.
###
### Our own backends are one source file each, so they lose nothing here; the
### score plug-in itself is defined in the parent directory and still honours
### whatever the build was configured with.
set(CMAKE_UNITY_BUILD OFF)

### --- libusb ----------------------------------------------------------------
###
### On Linux libusb is a system library. Every distribution ships it, it is what
### the udev rules in this tree are written against, and linking it is ordinary.
###
### Nowhere else. macOS has no libusb at all: linking Homebrew's leaves the
### backends referencing /opt/homebrew/opt/libusb/lib/libusb-1.0.0.dylib, an
### absolute path that exists only on the machine that built the package.
### Windows has none either, and shipping the DLL beside the backend does not
### help -- Windows resolves a module's dependencies through the standard search
### order, which does not include the directory the module itself came from, so
### a libusb-1.0.dll sitting right next to it is simply not found.
###
### So off Linux, link the copy the OrbbecSDK already vendors, statically. It is
### 1.0.26, it builds against winusb and darwin_usb, and it is already here.
### Each of the three consumers has to be told a different way.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" AND TARGET libusb_static)
  set(SCORE_DEPTHCAM_LIBUSB_INCLUDE
      "${SCORE_DEPTHCAM_SDK_DIR}/OrbbecSDK_v2/3rdparty/libusb/src/libusb"
      CACHE INTERNAL "")

  # libfreenect: its Findlibusb-1.0.cmake short-circuits when both of these are
  # already set ("in cache already").
  set(LIBUSB_1_INCLUDE_DIRS "${SCORE_DEPTHCAM_LIBUSB_INCLUDE}" CACHE PATH "" FORCE)
  set(LIBUSB_1_LIBRARIES libusb_static CACHE STRING "" FORCE)

  # libfreenect2: only reachable by shadowing its find module. See cmake/FindLibUSB.cmake.
  list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

  # librealsense is handled in its own section below: the target it looks for
  # has to join an export set that only exists once it has been added.

  set(SCORE_DEPTHCAM_VENDORED_LIBUSB ON)
  message(STATUS "score-addon-orbbec: using the vendored static libusb")
endif()

### --- Orbbec (Femto, Gemini, Astra) ----------------------------------------
option(SCORE_DEPTHCAM_BUILD_ORBBEC "Build the Orbbec backend" ON)

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

if(SCORE_DEPTHCAM_BUILD_ORBBEC AND TARGET ob::OrbbecSDK)
  add_library(score_depthcam_orbbec MODULE Orbbec/OrbbecBackend.cpp)

  # One file per backend where possible: the SDK's CMake only ever builds
  # libOrbbecSDK as a shared library ("only support shared library for now"),
  # but that library is just its public C API compiled on top of seven static
  # component libs. Building those sources into the backend instead removes the
  # separate .so, so the package is one object plus the vendor blobs the SDK
  # dlopens at runtime, which cannot be merged by anyone.
  option(SCORE_DEPTHCAM_ORBBEC_STATIC
         "Build the OrbbecSDK into the backend instead of alongside it" ON)

  if(SCORE_DEPTHCAM_ORBBEC_STATIC)
    # An object library rather than more sources on the backend, because the two
    # halves cannot agree on a C++ standard. Ours uses designated initializers
    # to fill in the ABI vtable, which MSVC accepts only at C++20 (gcc and clang
    # take them as an extension, which is why this went unnoticed). The SDK's
    # were written for C++11, and at C++20 spdlog's format strings turn consteval
    # and its logging calls stop compiling against Apple's libc++. Compile each
    # at what it needs and link the objects together.
    file(GLOB _ob_api
      "${SCORE_DEPTHCAM_SDK_DIR}/OrbbecSDK_v2/src/impl/*.cpp"
      "${SCORE_DEPTHCAM_SDK_DIR}/OrbbecSDK_v2/src/context/*.cpp")

    add_library(score_depthcam_orbbec_sdk OBJECT ${_ob_api})
    set_target_properties(score_depthcam_orbbec_sdk PROPERTIES
      CXX_STANDARD 17
      CXX_STANDARD_REQUIRED ON
      CXX_VISIBILITY_PRESET hidden
      C_VISIBILITY_PRESET hidden
      VISIBILITY_INLINES_HIDDEN ON
      POSITION_INDEPENDENT_CODE ON)

    target_include_directories(score_depthcam_orbbec_sdk SYSTEM PRIVATE
      "${SCORE_DEPTHCAM_SDK_DIR}/OrbbecSDK_v2/src"
      "${SCORE_DEPTHCAM_SDK_DIR}/OrbbecSDK_v2/include")

    # These come from the SDK's add_definitions(), which is directory-scoped and
    # so does not reach a target defined here. They have to match what the SDK's
    # own cmake/<platform>_config.cmake would have set, because these are the
    # same sources the SDK compiles: getting OS_LINUX onto a macOS build is not
    # a warning, it selects a different ObPal.
    #
    # OB_STATIC_DEFINE is ours. The SDK's generated Export.h marks its public C
    # API __declspec(dllimport) for anyone who is not building the DLL, and
    # these translation units *define* those functions -- MSVC rejects that
    # outright ("definition of dllimport function not allowed"). We are not
    # consuming a DLL on any platform here, so say so everywhere.
    target_compile_definitions(score_depthcam_orbbec_sdk PRIVATE
      OB_STATIC_DEFINE BUILD_USB_PAL BUILD_NET_PAL
      $<$<CONFIG:Debug>:OB_DEBUG> $<$<NOT:$<CONFIG:Debug>>:OB_RELEASE>)

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
      # GMSL is the coaxial/serialiser transport on Jetson-class boards. The
      # SDK only ever compiles its sources under OB_BUILD_LINUX.
      target_compile_definitions(score_depthcam_orbbec_sdk PRIVATE OS_LINUX BUILD_GMSL_PAL)
    elseif(APPLE)
      target_compile_definitions(score_depthcam_orbbec_sdk PRIVATE OS_MACOS)
      if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
        target_compile_definitions(score_depthcam_orbbec_sdk PRIVATE OS_MACOS_ARM64 __NEON__)
      else()
        target_compile_definitions(score_depthcam_orbbec_sdk PRIVATE OS_MACOS_X64)
      endif()
    elseif(WIN32)
      target_compile_definitions(score_depthcam_orbbec_sdk PRIVATE
        _UNICODE UNICODE _CRT_SECURE_NO_WARNINGS)
    endif()

    # Linked for their headers; the archives themselves go on the backend below.
    target_link_libraries(score_depthcam_orbbec_sdk PRIVATE
      ob::shared ob::core ob::filter ob::platform ob::device ob::pipeline ob::media)

    target_sources(score_depthcam_orbbec
      PRIVATE $<TARGET_OBJECTS:score_depthcam_orbbec_sdk>)
    target_compile_definitions(score_depthcam_orbbec PRIVATE OB_STATIC_DEFINE)
    target_include_directories(score_depthcam_orbbec SYSTEM PRIVATE
      "${SCORE_DEPTHCAM_SDK_DIR}/OrbbecSDK_v2/src"
      "${SCORE_DEPTHCAM_SDK_DIR}/OrbbecSDK_v2/include")
    target_link_libraries(score_depthcam_orbbec PRIVATE
      ob::shared ob::core ob::filter ob::platform ob::device ob::pipeline ob::media)
  else()
    target_link_libraries(score_depthcam_orbbec PRIVATE ob::OrbbecSDK)
  endif()
  score_depthcam_setup_backend(score_depthcam_orbbec)

  # The OrbbecSDK's own rules are well-formed, unlike Microsoft's; ship those.
  score_depthcam_install_udev_rules(
    "${SCORE_DEPTHCAM_SDK_DIR}/OrbbecSDK_v2/scripts/env_setup")

  # The blobs under extensions/ -- filters, frameprocessor, firmwareupdater --
  # are dlopen'd by path at runtime and cannot be linked in by anyone, so they
  # have to sit beside the backend, in the build tree exactly as in a shipped
  # package. That is what makes the backend directory *be* the package layout:
  # anything that works here works installed.
  add_custom_command(TARGET score_depthcam_orbbec POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SCORE_ORBBEC_RUNTIME_DIR}/extensions" "${SCORE_DEPTHCAM_BACKEND_DIR}/extensions"
    VERBATIM)

  # libOrbbecSDK itself only when it was not built into the backend. 72MB of
  # shared library that nothing loads is not something to ship.
  if(NOT SCORE_DEPTHCAM_ORBBEC_STATIC)
    add_custom_command(TARGET score_depthcam_orbbec POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              $<TARGET_FILE:OrbbecSDK> "${SCORE_DEPTHCAM_BACKEND_DIR}/"
      COMMAND ${CMAKE_COMMAND} -E create_symlink
              $<TARGET_FILE_NAME:OrbbecSDK> "${SCORE_DEPTHCAM_BACKEND_DIR}/$<TARGET_SONAME_FILE_NAME:OrbbecSDK>"
      VERBATIM)
  endif()

  if(TARGET extensions)
    add_dependencies(score_depthcam_orbbec extensions)
  endif()
endif()

### --- Kinect v2 (libfreenect2) ---------------------------------------------
option(SCORE_DEPTHCAM_BUILD_FREENECT2 "Build the Kinect v2 backend" ON)

if(SCORE_DEPTHCAM_BUILD_FREENECT2
   AND EXISTS "${SCORE_DEPTHCAM_SDK_DIR}/libfreenect2/CMakeLists.txt")
  # OpenGL stays off: OpenGLPacketPipeline creates its own GLFW context and
  # window, which has no business inside a Qt RHI process. OpenCL and CUDA are
  # worth having -- the CPU depth pipeline is by far the slowest part of a
  # Kinect v2 -- and the backend picks the best available at runtime.
  set(BUILD_EXAMPLES OFF CACHE INTERNAL "")
  set(BUILD_OPENNI2_DRIVER OFF CACHE INTERNAL "")
  set(ENABLE_OPENGL OFF CACHE INTERNAL "")
  set(ENABLE_VAAPI OFF CACHE INTERNAL "")
  set(ENABLE_TEGRAJPEG OFF CACHE INTERNAL "")
  set(ENABLE_CXX11 ON CACHE INTERNAL "")

  option(SCORE_DEPTHCAM_FREENECT2_OPENCL "Kinect v2 OpenCL depth pipeline" ON)
  option(SCORE_DEPTHCAM_FREENECT2_CUDA "Kinect v2 CUDA depth pipeline" OFF)

  set(ENABLE_OPENCL ${SCORE_DEPTHCAM_FREENECT2_OPENCL} CACHE INTERNAL "")

  # libfreenect2's CUDA sources include helper_math.h, which ships with the CUDA
  # *samples* rather than the toolkit. Off unless someone points at a copy.
  set(ENABLE_CUDA ${SCORE_DEPTHCAM_FREENECT2_CUDA} CACHE INTERNAL "")
  if(SCORE_DEPTHCAM_FREENECT2_CUDA AND SCORE_DEPTHCAM_CUDA_SAMPLES_DIR)
    include_directories("${SCORE_DEPTHCAM_CUDA_SAMPLES_DIR}")
  endif()

  # libjpeg-turbo for the colour stream, which arrives as MJPEG. libfreenect2
  # requires it everywhere but macOS, where VideoToolbox is the fallback.
  #
  # The copy the OrbbecSDK vendors, built static, whenever there is one: it
  # keeps the shipped backend from depending on a libturbojpeg.so the user may
  # not have, and the ossia SDK builds its own libjpeg without the TurboJPEG API,
  # so an SDK build has no other copy to find. A system one is only used when
  # the Orbbec SDK is not part of the build. With neither, the Kinect v2
  # backend is skipped rather than failing the whole configuration.
  set(_f2_jpeg_ok ON)
  if(TARGET turbojpeg_static)
    set(SCORE_DEPTHCAM_TURBOJPEG_INCLUDE
        "${SCORE_DEPTHCAM_SDK_DIR}/OrbbecSDK_v2/3rdparty/libjpeg/src"
        CACHE INTERNAL "")
    list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/turbojpeg")
    message(STATUS "score-addon-orbbec: Kinect v2 uses the vendored libjpeg-turbo")
  elseif(NOT APPLE)
    find_path(SCORE_DEPTHCAM_SYSTEM_TURBOJPEG_INCLUDE turbojpeg.h)
    find_library(SCORE_DEPTHCAM_SYSTEM_TURBOJPEG_LIB NAMES turbojpeg libturbojpeg)
    if(NOT SCORE_DEPTHCAM_SYSTEM_TURBOJPEG_INCLUDE OR NOT SCORE_DEPTHCAM_SYSTEM_TURBOJPEG_LIB)
      set(_f2_jpeg_ok OFF)
      message(STATUS
        "score-addon-orbbec: skipping the Kinect v2 backend: libfreenect2 needs "
        "libjpeg-turbo's TurboJPEG API, and neither the OrbbecSDK's copy nor a "
        "system one (turbojpeg.h + libturbojpeg) is available")
    endif()
  endif()

  if(_f2_jpeg_ok)
    add_subdirectory("${SCORE_DEPTHCAM_SDK_DIR}/libfreenect2" libfreenect2 EXCLUDE_FROM_ALL)
  endif()

  # See compat/freenect2_cl_compat.h: modern CL headers turned CL_ICDL_VERSION
  # into a macro, which breaks a local declaration of the same name in
  # libfreenect2's OpenCL processors.
  if(TARGET freenect2 AND SCORE_DEPTHCAM_FREENECT2_OPENCL)
    if(MSVC)
      # cl.exe spells -include as /FI, and it needs the shim just as much:
      # without it both OpenCL processors fail with "error C2059: syntax error:
      # 'constant'" on the same line.
      target_compile_options(freenect2 PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:SHELL:/FI${CMAKE_CURRENT_LIST_DIR}/Freenect2/compat/freenect2_cl_compat.h>")
    else()
      target_compile_options(freenect2 PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:SHELL:-include ${CMAKE_CURRENT_LIST_DIR}/Freenect2/compat/freenect2_cl_compat.h>")
    endif()
  endif()

  if(TARGET freenect2)
    add_library(score_depthcam_freenect2 MODULE Freenect2/Freenect2Backend.cpp)
    target_link_libraries(score_depthcam_freenect2 PRIVATE freenect2)

    # libfreenect2 predates usage requirements: its target carries no INTERFACE
    # include directories. The build dir is needed too -- config.h and export.h
    # are generated there.
    target_include_directories(score_depthcam_freenect2 SYSTEM PRIVATE
      "${SCORE_DEPTHCAM_SDK_DIR}/libfreenect2/include"
      "${CMAKE_CURRENT_BINARY_DIR}/libfreenect2"
    )
    score_depthcam_setup_backend(score_depthcam_freenect2)
    score_depthcam_install_udev_rules("${CMAKE_CURRENT_LIST_DIR}/Freenect2/udev")
  endif()
endif()

### --- Kinect v1 (libfreenect) ----------------------------------------------
option(SCORE_DEPTHCAM_BUILD_FREENECT "Build the Kinect v1 backend" ON)

if(SCORE_DEPTHCAM_BUILD_FREENECT
   AND EXISTS "${SCORE_DEPTHCAM_SDK_DIR}/libfreenect/CMakeLists.txt")
  # Only the core library: the examples want glut/OpenGL and the wrappers pull
  # in Python and OpenCV. BUILD_REDIST_PACKAGE avoids the audio-firmware
  # download, which needs Python at build time.
  set(BUILD_EXAMPLES OFF CACHE INTERNAL "")
  set(BUILD_FAKENECT OFF CACHE INTERNAL "")
  set(BUILD_C_SYNC OFF CACHE INTERNAL "")
  set(BUILD_CPP OFF CACHE INTERNAL "")
  set(BUILD_CV OFF CACHE INTERNAL "")
  set(BUILD_PYTHON OFF CACHE INTERNAL "")
  set(BUILD_OPENNI2_DRIVER OFF CACHE INTERNAL "")
  set(BUILD_AUDIO OFF CACHE INTERNAL "")
  set(BUILD_REDIST_PACKAGE ON CACHE INTERNAL "")

  add_subdirectory("${SCORE_DEPTHCAM_SDK_DIR}/libfreenect" libfreenect EXCLUDE_FROM_ALL)

  # libfreenect always defines both a shared `freenect` and a static
  # `freenectstatic`, and on Windows they collide: the import library of the DLL
  # and the static library are both freenect.lib, which ninja rejects outright
  # ("multiple rules generate lib/freenect.lib"). We link the static one.
  if(WIN32 AND TARGET freenectstatic)
    set_target_properties(freenectstatic PROPERTIES OUTPUT_NAME freenect_static)
  endif()

  set(_freenect_target "")
  if(TARGET freenectstatic)
    set(_freenect_target freenectstatic)
  elseif(TARGET freenect)
    set(_freenect_target freenect)
  endif()

  if(_freenect_target)
    add_library(score_depthcam_freenect MODULE Freenect/FreenectBackend.cpp)
    target_link_libraries(score_depthcam_freenect PRIVATE ${_freenect_target})
    target_include_directories(score_depthcam_freenect SYSTEM PRIVATE
      "${SCORE_DEPTHCAM_SDK_DIR}/libfreenect/include"
      "${CMAKE_CURRENT_BINARY_DIR}/libfreenect"
    )
    score_depthcam_setup_backend(score_depthcam_freenect)
    score_depthcam_install_udev_rules("${CMAKE_CURRENT_LIST_DIR}/Freenect/udev")
  endif()
endif()

### --- Azure Kinect DK (libk4a) ---------------------------------------------
###
### No link dependency on the SDK: the backend resolves libk4a with dlopen at
### runtime against vendored headers (see K4A/README.md). That keeps this target
### buildable in CI without the SDK, lets the user install it through their own
### package manager, and avoids redistributing libdepthengine, a closed
### Microsoft binary that any depth output requires.
### Off by default on macOS: Microsoft never released an Azure Kinect SDK for
### it, so the backend would build (it only needs the vendored headers) and then
### never find anything to dlopen.
if(APPLE)
  set(SCORE_DEPTHCAM_K4A_DEFAULT OFF)
else()
  set(SCORE_DEPTHCAM_K4A_DEFAULT ON)
endif()
option(SCORE_DEPTHCAM_BUILD_K4A "Build the Azure Kinect backend"
       ${SCORE_DEPTHCAM_K4A_DEFAULT})

if(SCORE_DEPTHCAM_BUILD_K4A)
  add_library(score_depthcam_k4a MODULE K4A/K4ABackend.cpp)
  target_include_directories(score_depthcam_k4a SYSTEM PRIVATE "${CMAKE_CURRENT_LIST_DIR}/K4A")
  score_depthcam_setup_backend(score_depthcam_k4a)
  score_depthcam_install_udev_rules("${CMAKE_CURRENT_LIST_DIR}/K4A/udev")
  if(UNIX AND NOT APPLE)
    target_link_libraries(score_depthcam_k4a PRIVATE ${CMAKE_DL_LIBS})
  endif()
endif()

### --- Intel RealSense (librealsense) ---------------------------------------
option(SCORE_DEPTHCAM_BUILD_REALSENSE "Build the RealSense backend" ON)

if(SCORE_DEPTHCAM_BUILD_REALSENSE
   AND EXISTS "${SCORE_DEPTHCAM_SDK_DIR}/librealsense/CMakeLists.txt")
  set(BUILD_EXAMPLES OFF CACHE INTERNAL "")
  set(BUILD_GRAPHICAL_EXAMPLES OFF CACHE INTERNAL "")
  set(BUILD_UNIT_TESTS OFF CACHE INTERNAL "")
  set(BUILD_TOOLS OFF CACHE INTERNAL "")
  set(BUILD_PYTHON_BINDINGS OFF CACHE INTERNAL "")
  set(BUILD_WITH_TM2 OFF CACHE INTERNAL "")
  set(BUILD_NETWORK_DEVICE OFF CACHE INTERNAL "")
  set(IMPORT_DEPTH_CAM_FW OFF CACHE INTERNAL "")
  set(CHECK_FOR_UPDATES OFF CACHE INTERNAL "")
  # The libusb backend rather than V4L2: V4L2 needs patched kernel modules for
  # full functionality, and the udev rules we ship grant raw USB access anyway.
  set(FORCE_RSUSB_BACKEND ON CACHE INTERNAL "")

  # librealsense's install_config.cmake runs configure_package_config_file,
  # which rejects a relative CMAKE_INSTALL_PREFIX outright ("INSTALL_PREFIX
  # must be an absolute path"). CMake accepts a relative one, and score's
  # sanitizer build tree uses "install", so a perfectly ordinary configuration
  # otherwise fails here. Nothing in this tree is ever installed; the value only
  # has to be well-formed.
  # librealsense: CMake/libusb_config.cmake does nothing at all if a target
  # called `usb` already exists, which is the cleanest of the three ways in.
  if(SCORE_DEPTHCAM_VENDORED_LIBUSB AND NOT TARGET usb)
    add_library(usb INTERFACE)
    target_link_libraries(usb INTERFACE libusb_static)
  endif()

  block()
    if(NOT IS_ABSOLUTE "${CMAKE_INSTALL_PREFIX}")
      set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_PREFIX}")
    endif()
    add_subdirectory("${SCORE_DEPTHCAM_SDK_DIR}/librealsense" librealsense EXCLUDE_FROM_ALL)
  endblock()

  # librealsense bundles an old copy of rosbag2 whose headers use uint8_t/
  # uint64_t without including <cstdint>. That compiled while libstdc++ still
  # pulled it in transitively; it does not any more. Force it in rather than
  # patching a pinned submodule.
  # librealsense's vendored zstd calls qsort_r, which glibc only declares under
  # _GNU_SOURCE. score defines that for its whole build, so this only surfaces
  # in a standalone backends-only configuration -- which is exactly what CI
  # packages.
  if(TARGET realsense-file AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_definitions(realsense-file PRIVATE _GNU_SOURCE)
  endif()

  foreach(_rs_target realsense-file realsense2)
    if(TARGET ${_rs_target} AND NOT MSVC)
      # C++ only: realsense-file mixes C and C++ sources, and <cstdint> does
      # not exist for the C ones.
      target_compile_options(${_rs_target} PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:SHELL:-include cstdint>")
    endif()
  endforeach()

  if(TARGET realsense2)
    add_library(score_depthcam_realsense MODULE RealSense/RealSenseBackend.cpp)
    target_link_libraries(score_depthcam_realsense PRIVATE realsense2)
    target_include_directories(score_depthcam_realsense SYSTEM PRIVATE
      "${SCORE_DEPTHCAM_SDK_DIR}/librealsense/include")
    score_depthcam_setup_backend(score_depthcam_realsense)
    score_depthcam_install_udev_rules("${CMAKE_CURRENT_LIST_DIR}/RealSense/udev")
  endif()
endif()

### --- Oak3DVision Oak CDK / QCDK (libPointCloud) ---------------------------
###
### Nothing to compile: Oak3DVision publishes no sources, so 3rdparty/oaksdk is
### a submodule of vendored binaries. That fixes the platform list at whatever
### they built in 2020 -- Linux x86_64, Linux aarch64, macOS x86_64, Windows x86
### -- and the backend is skipped elsewhere. See Oak/README.md.
set(_oak_platform "")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(_oak_platform "ubuntu_arm")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(_oak_platform "ubuntu")
  endif()
elseif(APPLE)
  # CMAKE_OSX_ARCHITECTURES first: on a cross build CMAKE_SYSTEM_PROCESSOR stays
  # the host's. It is a list, and the vendor's dylib is x86_64 only, so it can
  # only be linked when every requested slice is x86_64 -- x86_64h included,
  # which a generic x86_64 dylib links into fine.
  set(_oak_cpus "${CMAKE_SYSTEM_PROCESSOR}")
  if(CMAKE_OSX_ARCHITECTURES)
    set(_oak_cpus "${CMAKE_OSX_ARCHITECTURES}")
  endif()
  set(_oak_all_x86 ON)
  foreach(_cpu ${_oak_cpus})
    if(NOT _cpu MATCHES "^(x86_64|x86_64h|amd64)$")
      set(_oak_all_x86 OFF)
    endif()
  endforeach()
  if(_oak_cpus AND _oak_all_x86)
    set(_oak_platform "macos")
  endif()
elseif(WIN32)
  # The Windows binary is 32-bit -- pointcloud.dll and .lib both carry machine
  # type 0x014c -- so in practice there is no Oak backend on Windows, since
  # score has not shipped an i386 build for years.
  #
  # MSVC only: the DLL exports mangled C++ methods and passes std::string and
  # std::vector across the boundary, which a mingw build cannot match.
  if(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 4
     AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86|X86|i[36]86)$")
    set(_oak_platform "windows")
  endif()
endif()

option(SCORE_DEPTHCAM_BUILD_OAK "Build the Oak3DVision backend" ON)

### Its Linux binaries pass std::string across their API and were built
### against libstdc++. Built with libc++ -- the ossia SDK's toolchain -- the
### backend links (libpointcloud.so is resolved lazily) and then aborts the
### process at its first call into the SDK. Skip it there.
if(_oak_platform AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
  include(CheckCXXSourceCompiles)
  check_cxx_source_compiles(
    "#include <cstddef>\n#if !defined(_LIBCPP_VERSION)\n#error libstdc++\n#endif\nint main() { return 0; }"
    SCORE_DEPTHCAM_USES_LIBCXX)
  if(SCORE_DEPTHCAM_USES_LIBCXX)
    message(STATUS
      "score-addon-orbbec: skipping the Oak3DVision backend: its SDK binaries "
      "need libstdc++ and this build uses libc++")
    set(_oak_platform "")
  endif()
endif()

if(SCORE_DEPTHCAM_BUILD_OAK AND _oak_platform
   AND EXISTS "${SCORE_DEPTHCAM_SDK_DIR}/oaksdk/libs/${_oak_platform}/lib")
  set(_oak_root "${SCORE_DEPTHCAM_SDK_DIR}/oaksdk/libs/${_oak_platform}")

  ### The platform's own headers first -- PointCloudExports.h in particular must
  ### be the local one -- then Linux's to fill the gaps: the macOS and Windows
  ### trees ship 29 headers against Linux's 52, and the SDK as shipped does not
  ### compile there (Configuration.h includes a HardwareSerializer.h that is in
  ### neither). The missing 23 are the portable half of the library.
  set(_oak_include
    "${_oak_root}/include/pointcloud-1.0.0"
    "${SCORE_DEPTHCAM_SDK_DIR}/oaksdk/libs/ubuntu/include/pointcloud-1.0.0")

  if(WIN32)
    set(_oak_link "${_oak_root}/lib/pointcloud.lib")
    set(_oak_runtime "${_oak_root}/lib/pointcloud.dll")
  elseif(APPLE)
    set(_oak_link "${_oak_root}/lib/libpointcloud.dylib")
    set(_oak_runtime "${_oak_link}")
  else()
    set(_oak_link "${_oak_root}/lib/libpointcloud.so")
    set(_oak_runtime "${_oak_link}")
  endif()

  if(NOT EXISTS "${_oak_link}")
    message(STATUS
      "score-addon-orbbec: no Oak SDK binary at ${_oak_link}, skipping the Oak backend")
  else()
    add_library(score_depthcam_oak MODULE Oak/OakBackend.cpp)
    target_link_libraries(score_depthcam_oak PRIVATE "${_oak_link}")
    target_include_directories(score_depthcam_oak SYSTEM PRIVATE ${_oak_include})

    # See Oak/compat/oak_cxx17_compat.h.
    if(MSVC)
      target_compile_options(score_depthcam_oak PRIVATE
        "/FI${CMAKE_CURRENT_LIST_DIR}/Oak/compat/oak_cxx17_compat.h")
    else()
      target_compile_options(score_depthcam_oak PRIVATE
        "SHELL:-include ${CMAKE_CURRENT_LIST_DIR}/Oak/compat/oak_cxx17_compat.h")
    endif()

    if(MSVC)
      # MSVC, not WIN32: cl/link-only flags, and this tree is also built under
      # msys2.
      #
      # The dynamic CRT, alone in this package: SCORE_DEPTHCAM_BACKENDS_ONLY
      # otherwise forces /MT, and pointcloud.dll imports MSVCP140 while
      # CameraSystem::scan() hands us a std::vector we destroy -- two CRTs would
      # free it against the wrong heap on the first enumeration. The
      # redistributable is therefore a requirement of this backend.
      set_property(TARGET score_depthcam_oak PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")

      # Delay-load so backend_init() can load the DLL by full path first: the
      # process search order does not include the directory a module came from.
      target_link_options(score_depthcam_oak PRIVATE "/DELAYLOAD:pointcloud.dll")
      target_link_libraries(score_depthcam_oak PRIVATE delayimp)
    endif()

    score_depthcam_setup_backend(score_depthcam_oak)
    score_depthcam_install_udev_rules("${CMAKE_CURRENT_LIST_DIR}/Oak/udev")

    # The SDK itself, beside the backend, where $ORIGIN / @loader_path find it.
    add_custom_command(TARGET score_depthcam_oak POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${_oak_runtime}" "${SCORE_DEPTHCAM_BACKEND_DIR}/"
      VERBATIM)

    if(APPLE)
      # libpointcloud.dylib asks for @rpath/libusb-1.0.0.dylib, so ship the copy
      # the SDK bundles. No install_name_tool -add_rpath: the vendor's dylib
      # already carries LC_RPATH @loader_path, and adding it again is a hard
      # error ("would duplicate path"). Checked below so an SDK drop without it
      # is noticed rather than silently unloadable.
      add_custom_command(TARGET score_depthcam_oak POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_oak_root}/lib/libusb-1.0.0.dylib" "${SCORE_DEPTHCAM_BACKEND_DIR}/"
        VERBATIM)

      execute_process(
        COMMAND otool -l "${_oak_root}/lib/libpointcloud.dylib"
        OUTPUT_VARIABLE _oak_otool ERROR_QUIET)
      if(NOT _oak_otool MATCHES "@loader_path")
        message(WARNING
          "score-addon-orbbec: the Oak SDK's libpointcloud.dylib has no "
          "@loader_path rpath, so it will not find the libusb shipped beside "
          "it. The backend will load and then fail to open a camera.")
      endif()
    endif()

    # The two trees the SDK loads by path: its camera plug-ins and the
    # .conf/.dml model descriptions, without which a camera enumerates and then
    # fails to connect. Under oak/ so they cannot collide with another SDK's
    # runtime files; backend_init() points POINTCLOUD_{PLUGIN,CONF}_PATH here.
    add_custom_command(TARGET score_depthcam_oak POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
              "${_oak_root}/lib/plugin" "${SCORE_DEPTHCAM_BACKEND_DIR}/oak/plugin"
      COMMAND ${CMAKE_COMMAND} -E copy_directory
              "${_oak_root}/share/pointcloud-1.0.0/conf"
              "${SCORE_DEPTHCAM_BACKEND_DIR}/oak/conf"
      VERBATIM)
  endif()
elseif(SCORE_DEPTHCAM_BUILD_OAK AND NOT _oak_platform AND NOT SCORE_DEPTHCAM_USES_LIBCXX)
  message(STATUS
    "score-addon-orbbec: Oak3DVision ships no libPointCloud binary for this "
    "platform, skipping the Oak backend")
endif()

### --- Luxonis OAK (depthai-core) -------------------------------------------
###
### A nested project rather than add_subdirectory, which depthai's own
### documentation says is unsupported: it resolves ~20 dependencies through a
### vcpkg it bootstraps itself, and that has to be the toolchain file of the
### top-level configure. So ExternalProject_Add gives it its own.
###
### It is also the only SDK here that needs the network at build time (vcpkg,
### plus the MyriadX and RVC4 firmware it embeds), leaves about a gigabyte in
### CMAKE_BINARY_DIR, and adds ~30 MB to the package. Hence the default below:
### on only when the package is what was asked for, so that a developer who
### cloned this repository recursively does not turn every score build into a
### vcpkg bootstrap. Work on this backend with -DSCORE_DEPTHCAM_BUILD_DEPTHAI=ON.
### See DepthAI/README.md.
if(SCORE_DEPTHCAM_BACKENDS_ONLY)
  set(SCORE_DEPTHCAM_DEPTHAI_DEFAULT ON)
else()
  set(SCORE_DEPTHCAM_DEPTHAI_DEFAULT OFF)
endif()
option(SCORE_DEPTHCAM_BUILD_DEPTHAI "Build the Luxonis OAK backend"
       ${SCORE_DEPTHCAM_DEPTHAI_DEFAULT})

### A universal macOS build cannot include depthai: a vcpkg triplet names one
### architecture, so a nested configure handed `x86_64;arm64` gives up with
### "Unable to determine target architecture ... Continuing without vcpkg" and
### then fails much later on a missing dependency. Skipped rather than fatal --
### score itself is built universal.
###
### Distinct architectures, not slices: CMAKE_OSX_ARCHITECTURES_KEYS is the list
### collapsed by the top-level CMakeLists, so a fat x86_64 + x86_64h build is one
### architecture and a generic x86_64 dylib links into both slices.
set(_dai_multiarch OFF)
set(_dai_arch "")
if(APPLE AND CMAKE_OSX_ARCHITECTURES)
  list(LENGTH CMAKE_OSX_ARCHITECTURES_KEYS _dai_narch)
  if(_dai_narch GREATER 1)
    set(_dai_multiarch ON)
  else()
    # vcpkg knows x86_64 and arm64 and nothing about sub-architectures.
    list(GET CMAKE_OSX_ARCHITECTURES_KEYS 0 _dai_key)
    if(_dai_key STREQUAL "aarch64")
      set(_dai_arch "arm64")
    else()
      set(_dai_arch "x86_64")
    endif()
  endif()
endif()

### Nor msys2: vcpkg selects its triplet from the host platform rather than the
### compiler, so on Windows it picks x64-windows whatever CMAKE_CXX_COMPILER
### says and produces MSVC-ABI libraries that will not link into a mingw module.
### depthai ships no mingw triplet.
if(SCORE_DEPTHCAM_BUILD_DEPTHAI AND _dai_multiarch)
  message(STATUS
    "score-addon-orbbec: skipping the Luxonis OAK backend, which vcpkg cannot "
    "build for more than one architecture at a time "
    "(CMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES})")
elseif(SCORE_DEPTHCAM_BUILD_DEPTHAI AND WIN32 AND NOT MSVC)
  message(STATUS
    "score-addon-orbbec: skipping the Luxonis OAK backend: depthai resolves its "
    "dependencies through vcpkg, which selects an MSVC triplet on Windows and "
    "has none for mingw. Build it with MSVC.")
elseif(SCORE_DEPTHCAM_BUILD_DEPTHAI
   AND EXISTS "${SCORE_DEPTHCAM_SDK_DIR}/depthai-core/CMakeLists.txt")
  include(ExternalProject)

  set(_dai_prefix "${CMAKE_BINARY_DIR}/depthai")
  set(_dai_install "${_dai_prefix}/install")

  # depthai-core is built static and linked into the backend; see
  # DepthAI/LinkInterface.cmake for why, and for how its dependency closure is
  # resolved. The runtime directories are still swept for what stays shared:
  # libusb, which every depthai triplet keeps dynamic. On Windows the DLL is
  # installed to bin/.
  set(_dai_link_dir "${_dai_prefix}/link")
  set(_dai_link_rsp "${_dai_link_dir}/depthai-link.rsp")
  set(_dai_link_cpp "${_dai_link_dir}/depthai-link.cpp")
  if(MSVC)
    set(_dai_archive "${_dai_prefix}/build/depthai-core.lib")
  else()
    set(_dai_archive "${_dai_prefix}/build/libdepthai-core.a")
  endif()
  set(_dai_runtime_dirs "${_dai_install}/lib")
  if(WIN32)
    list(APPEND _dai_runtime_dirs "${_dai_install}/bin")
  endif()

  # The backend's include path before the ExternalProject has run; some
  # generators dislike a target whose include directory does not exist.
  # add_dependencies gets the ordering right.
  file(MAKE_DIRECTORY "${_dai_install}/include")

  ### Forwarded only when they have a value: `-DCMAKE_OSX_ARCHITECTURES=` with
  ### nothing after it defines the cache entry as the empty string, and vcpkg's
  ### toolchain then cannot determine the target architecture, warns "Continuing
  ### without vcpkg" and builds no dependencies at all -- surfacing much later
  ### as `Could not find a package configuration file provided by "lz4"`.
  ###
  ### CMAKE_MAKE_PROGRAM matters only on Windows: ExternalProject forwards the
  ### generator but not the program driving it, and CMake finds ninja through
  ### the Visual Studio installation rather than the PATH there.
  set(_dai_cmake_args
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DCMAKE_BUILD_TYPE=$<IF:$<CONFIG:Debug>,RelWithDebInfo,$<CONFIG>>
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON)
  foreach(_var CMAKE_C_COMPILER CMAKE_CXX_COMPILER CMAKE_MAKE_PROGRAM
               CMAKE_OSX_DEPLOYMENT_TARGET CMAKE_OSX_SYSROOT)
    if(${_var})
      list(APPEND _dai_cmake_args "-D${_var}=${${_var}}")
    endif()
  endforeach()
  # Not CMAKE_OSX_ARCHITECTURES verbatim: see _dai_arch above.
  if(_dai_arch)
    list(APPEND _dai_cmake_args "-DCMAKE_OSX_ARCHITECTURES=${_dai_arch}")
  endif()
  if(MSVC)
    list(APPEND _dai_cmake_args "-DCMAKE_CXX_FLAGS_INIT=/D_USE_MATH_DEFINES")
  endif()

  ExternalProject_Add(depthai_core_ext
    SOURCE_DIR "${SCORE_DEPTHCAM_SDK_DIR}/depthai-core"
    PREFIX "${_dai_prefix}"
    BINARY_DIR "${_dai_prefix}/build"
    INSTALL_DIR "${_dai_install}"
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
    CMAKE_ARGS
      ${_dai_cmake_args}
      -DBUILD_SHARED_LIBS=OFF
      "-DCMAKE_PROJECT_depthai_INCLUDE=${CMAKE_CURRENT_LIST_DIR}/DepthAI/LinkInterface.cmake"
      "-DSCORE_DAI_LINK_DIR=${_dai_link_dir}"
      -DDEPTHAI_INSTALL=ON
      # Everything optional off: none of OpenCV, protobuf, curl, mp4v2,
      # AprilTag, PCL or the Foxglove bridge is reachable through
      # depthcam_abi.h. The device firmware is not optional -- without it there
      # is no camera, and its RVC4 half is what makes the OAK-4 work.
      -DDEPTHAI_BUILD_EXAMPLES=OFF
      -DDEPTHAI_BUILD_TESTS=OFF
      -DDEPTHAI_BUILD_PYTHON=OFF
      -DDEPTHAI_BUILD_DOCS=OFF
      -DDEPTHAI_CLANG_FORMAT=OFF
      -DDEPTHAI_CLANG_TIDY=OFF
      -DDEPTHAI_OPENCV_SUPPORT=OFF
      -DDEPTHAI_PCL_SUPPORT=OFF
      -DDEPTHAI_ENABLE_CURL=OFF
      -DDEPTHAI_ENABLE_PROTOBUF=OFF
      -DDEPTHAI_ENABLE_MP4V2=OFF
      -DDEPTHAI_ENABLE_APRIL_TAG=OFF
      -DDEPTHAI_ENABLE_KOMPUTE=OFF
      -DDEPTHAI_ENABLE_BACKWARD=OFF
      -DDEPTHAI_DYNAMIC_CALIBRATION_SUPPORT=OFF
    BUILD_BYPRODUCTS "${_dai_archive}" "${_dai_link_rsp}" "${_dai_link_cpp}"
    USES_TERMINAL_CONFIGURE ON
    USES_TERMINAL_BUILD ON
    USES_TERMINAL_INSTALL ON
  )
  # The link list is written by depthai's configure; rerun it when the script
  # that writes it changes.
  ExternalProject_Add_StepDependencies(depthai_core_ext configure
    "${CMAKE_CURRENT_LIST_DIR}/DepthAI/LinkInterface.cmake")

  add_library(score_depthcam_depthai MODULE DepthAI/DepthAiBackend.cpp)
  add_dependencies(score_depthcam_depthai depthai_core_ext)
  target_include_directories(score_depthcam_depthai SYSTEM PRIVATE
    "${_dai_install}/include")
  # What depthai-core's static link interface says consumers define.
  target_compile_definitions(score_depthcam_depthai PRIVATE MCAP_STATIC)

  find_package(Threads REQUIRED)
  target_link_libraries(score_depthcam_depthai PRIVATE Threads::Threads)
  set_property(TARGET score_depthcam_depthai APPEND PROPERTY
    LINK_DEPENDS "${_dai_archive}" "${_dai_link_rsp}")
  if(MSVC)
    # The libraries as #pragma comment(lib): link.exe takes them in any order,
    # and a response file of our own would have to nest inside Ninja's.
    set_source_files_properties("${_dai_link_cpp}" PROPERTIES GENERATED ON)
    target_sources(score_depthcam_depthai PRIVATE "${_dai_link_cpp}")
  elseif(APPLE)
    # Driver-level; ld64 does not care where archives sit on the command line.
    target_link_options(score_depthcam_depthai PRIVATE "@${_dai_link_rsp}")
  else()
    # ld-level and after the objects, where an ELF linker needs archives.
    target_link_libraries(score_depthcam_depthai PRIVATE "-Wl,@${_dai_link_rsp}")
  endif()

  # MSVC, not WIN32: cl/link-only flags, and under msys2 /DELAYLOAD would be
  # handed to ld as a source file and `delayimp` does not exist. A mingw build
  # relies on the package directory being on the search path instead.
  if(MSVC)
    # The dynamic CRT: depthai's Windows triplet sets VCPKG_CRT_LINKAGE
    # dynamic, so everything linked in here was compiled /MD, and a static CRT
    # would not link against it. The redistributable is a requirement of this
    # backend.
    set_property(TARGET score_depthcam_depthai PROPERTY
      MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")

    # Delay-load so backend_init() can load libusb by full path first: the
    # process search order does not include the directory this module came
    # from, and the triplet keeps libusb a DLL.
    target_link_options(score_depthcam_depthai PRIVATE "/DELAYLOAD:libusb-1.0.dll")
    target_link_libraries(score_depthcam_depthai PRIVATE delayimp)
  endif()

  score_depthcam_setup_backend(score_depthcam_depthai)
  score_depthcam_install_udev_rules("${CMAKE_CURRENT_LIST_DIR}/DepthAI/udev")

  # The libusb depthai links, beside the backend where $ORIGIN and
  # @loader_path find it. See Deployment/CopyDepthaiRuntime.cmake.
  add_custom_command(TARGET score_depthcam_depthai POST_BUILD
    COMMAND ${CMAKE_COMMAND}
            "-DIN=${_dai_runtime_dirs}"
            -DOUT=${SCORE_DEPTHCAM_BACKEND_DIR}
            -P "${CMAKE_CURRENT_LIST_DIR}/../Deployment/CopyDepthaiRuntime.cmake"
    VERBATIM)
elseif(SCORE_DEPTHCAM_BUILD_DEPTHAI)
  message(STATUS
    "score-addon-orbbec: 3rdparty/depthai-core is not checked out, skipping "
    "the Luxonis OAK backend")
endif()

unset(CMAKE_POLICY_VERSION_MINIMUM CACHE)

### One target that builds whatever backends this configuration enabled, so
### neither CI nor a developer has to keep a list of target names in step with
### this file.
get_property(_backends GLOBAL PROPERTY SCORE_DEPTHCAM_BACKENDS)
add_custom_target(depthcam-backends)
if(_backends)
  add_dependencies(depthcam-backends ${_backends})
  message(STATUS "score-addon-orbbec: backends -> ${_backends}")
else()
  message(WARNING "score-addon-orbbec: no camera backend will be built")
endif()

### Loads one backend module and runs init / enumerate / shutdown on it; the
### Package workflow runs it on what it is about to ship. Beside the build tree
### rather than in the backend directory, so it never ends up in the package.
add_executable(depthcam-probe "${CMAKE_CURRENT_LIST_DIR}/../Tests/abi/probe.cpp")
target_include_directories(depthcam-probe PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/../DepthCamera/Backend")
target_link_libraries(depthcam-probe PRIVATE ${CMAKE_DL_LIBS})
set_target_properties(depthcam-probe PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
if(MSVC)
  set_property(TARGET depthcam-probe PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
endif()

### --- Synthetic camera -------------------------------------------------------
### No SDK: a known scene with known calibration, for the tests. Built beside the
### package, never into it; see Synthetic/Synthetic.cmake.
include("${CMAKE_CURRENT_LIST_DIR}/Synthetic/Synthetic.cmake")
