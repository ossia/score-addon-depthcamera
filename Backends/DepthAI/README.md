# Luxonis OAK backend

Wraps [`depthai-core`](https://github.com/luxonis/depthai-core) (MIT), pinned to
**v3.10.0**, for the whole Luxonis OAK range.

> Not to be confused with the `oak` backend next door, which is Oak3DVision's
> `libPointCloud` for their Oak CDK / QCDK time-of-flight modules. The names
> collide; the cameras and the SDKs have nothing to do with each other.

## What it produces

| Stream | Source | Format |
|---|---|---|
| `COLOR` | `Camera` node on the first colour socket | `RGB24` |
| `IR` | `Camera` node on the stereo pair's left socket | `GRAY8` |
| `DEPTH` | `Depth` node | `GRAY16`, millimetres (`depth_unit_mm = 1`) |
| `POINTCLOUD` | `PointCloud` node | `XYZ`, or `XYZRGB` when a tinted cloud is asked for |
| `IMU` | `IMU` node, calibrated accelerometer + gyroscope | m/s², rad/s |

Which of them a given camera can actually deliver is reported through
`active_streams()` rather than assumed: an OAK-1 has no depth, most OAKs have no
IMU, and only some have an illumination projector.

## Why `dai::node::Depth` and not `StereoDepth`

This is the decision the whole backend turns on. `Depth` is v3's unified node:
with `Algorithm::AUTO` it picks stereo matching, on-sensor ToF, or on-device
neural depth from what the camera in front of it actually has. Wiring
`StereoDepth` by hand — which is what most depthai examples and every v2 tutorial
do — works on an OAK-D and produces **nothing at all** on an OAK-D-SR-PoE,
whose depth comes from a ToF sensor rather than a stereo pair. Supporting the
range means letting the SDK choose.

It is also why v3 is pinned rather than the `v2_stable` branch: v2 has no RVC4
support, so the entire OAK-4 generation is invisible to it.

**The infrared camera is created before the depth node, and that ordering is
load-bearing.** `Depth` wires its own stereo pair, but `ensureStereoOutputs()`
looks for `Camera` nodes already on those sockets and reuses them rather than
creating a second one — which would be an error. Claiming the left socket first
therefore gets an IR stream out of the very sensor the depth pipeline is about
to use, at no extra bandwidth. Doing it the other way round gets a conflict.

## How it is built

depthai-core is the only SDK in this tree built as a **nested CMake project**
(`ExternalProject_Add`) rather than with `add_subdirectory`. That is not a
preference: it resolves a large dependency tree — libarchive, spdlog, Eigen,
yaml-cpp, xtensor, semver, httplib, its own libusb, and more — through a vcpkg
it bootstraps itself, and vcpkg has to be the toolchain file of the *top-level*
configure. From a subdirectory that is unreachable, and depthai's own
documentation says outright that `add_subdirectory` is not supported.

Everything optional is turned off: OpenCV, protobuf, curl, mp4v2, AprilTag, PCL,
the Foxglove websocket bridge, the embedded web frontend, dynamic calibration.
None of it is reachable through `depthcam_abi.h`, and each is dependencies to
build and megabytes to ship. The device firmware stays on, both halves of it —
that is what makes the camera work at all, and the RVC4 half is what makes the
OAK-4 generation work.

What that costs, and it is worth knowing:

- **Build time.** By far the longest thing in this repository: vcpkg bootstrap,
  its dependencies, then ~260 translation units.
- **Network at build time.** vcpkg, and separately the MyriadX and RVC4 firmware
  packages from `artifacts.luxonis.com`. Every other SDK here builds offline
  from a pinned submodule; this one cannot.
- **About 30 MB in the package**, most of it that embedded firmware.

depthai-core is built **static** and linked into the backend, which exports
`score_depthcam_backend_v1` and nothing else like every other one. As a shared
library it would have to export its whole API: it has no export macros, and on
Windows `CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS` on a library that size overflows the
PE export table (`LNK1189: library limit of 65535 objects exceeded`). The
archive's dependency closure — spdlog, fmt, yaml-cpp, libarchive, lz4, XLink,
the CMakeRC firmware library — is resolved inside depthai's own configure by
`LinkInterface.cmake` and handed to the backend's link as a generated list,
since the package config depthai installs does not find those again.

What is left beside the backend is the libusb every depthai triplet keeps
dynamic, found through `$ORIGIN` / `@loader_path` like everything else here.

## Telemetry is turned off

depthai-core reports anonymised usage to Luxonis unless `DEPTHAI_TELEMETRY` says
otherwise. It would be doing that from inside a media application the user never
pointed at the internet, so `init()` sets `DEPTHAI_TELEMETRY=0`.

It sets it only if it is unset: someone who has deliberately exported
`DEPTHAI_TELEMETRY=1` has opted in, and this must not quietly undo that. The
same rule applies to `DEPTHAI_LEVEL`, which is pinned to `warn` so the library
does not narrate itself to the host's stderr.

## Controls

depthai has no getters for camera settings — `dai::CameraControl` is a message
you send into a node's input, and nothing reads back. So, as in the Kinect v2
backend, what is published is what was last written, seeded with the SDK's
defaults. Writing a manual value implies manual mode: setting `exposure_time`
turns auto exposure off rather than being silently ignored, which is the one
behaviour a performer cannot debug.

Three controls are not mirrors:

- `depth/laser_power` and `ir/flood_light` are device calls, and are published
  only where `getIrDrivers()` reports the hardware. A plain OAK-D has neither; an
  OAK-D-Pro has both. The laser projector is off by default and is what makes
  depth work on a blank wall.
- `sensors/temperature` is read straight from `getChipTemperature()`.

## Platforms

Built the same way everywhere — Linux, macOS and Windows, x86\_64 and arm64 —
but **only in a backends-only build**: `SCORE_DEPTHCAM_BUILD_DEPTHAI` defaults to
the value of `SCORE_DEPTHCAM_BACKENDS_ONLY`. That is the one thing separating
this backend from the rest, and it is about build cost, not about platforms:
everything else here compiles alongside score for free, while this bootstraps a
vcpkg, needs the network, and leaves about a gigabyte of build tree in score's
own build directory. score never reaches this file anyway (`ci/common.deps.sh`
clones this repository with `NO_SUBMODULES=1`), but a developer who cloned it
recursively would, and that is who the default protects. Pass
`-DSCORE_DEPTHCAM_BUILD_DEPTHAI=ON` to work on it.

**msys2 is not supported, and cannot easily be.** vcpkg picks its triplet from
the host platform rather than from the compiler: on Windows it selects
`x64-windows` whatever `CMAKE_CXX_COMPILER` says, then looks for a Visual Studio
instance to build its ports with, and the MSVC-ABI libraries that come out
cannot be linked into a mingw module. depthai ships triplets for `x64-windows`,
`x64-windows-static-crt`, `x64-linux` and the two macOS ones, and none for
mingw. A mingw build therefore skips this backend with a message rather than
failing; making it work means carrying an `x64-mingw-dynamic` triplet and
finding out which of the twenty dependencies build under it.

**A universal macOS build cannot include it either**, for the same
one-triplet-one-architecture reason. `x86_64` + `x86_64h` is fine -- that is one
architecture in two flavours, and depthai is built once for `x86_64` -- but
Intel + Apple Silicon in one binary is not, and is skipped with a message.

Windows-with-MSVC needs three things the Unix builds do not, all of them the
same plumbing the Oak backend next door already uses:

- **the dynamic CRT.** depthai's Windows triplet sets `VCPKG_CRT_LINKAGE
  dynamic`, so everything vcpkg builds for it, and depthai-core itself, is
  compiled `/MD`. This backend, which links them in, is therefore built `/MD`
  too, unlike the rest of the package, and needs the Visual C++
  redistributable. The triplets are ours (`triplets/score-{x64,arm64}-windows`):
  depthai ships none for arm64 Windows, and vcpkg's stock one builds every
  dependency as a DLL that the backend would import and the package not ship.
- **a delay-load.** `libusb-1.0.dll` sits beside the backend, where Windows
  does not look for a module's imports. `/DELAYLOAD:libusb-1.0.dll` lets
  `init()` load it by full path with `LOAD_WITH_ALTERED_SEARCH_PATH` first.
- **two install directories.** depthai installs RUNTIME to `bin/` and
  ARCHIVE/LIBRARY to `lib/`, so libusb's DLL is not where its import library
  is. `Deployment/CopyDepthaiRuntime.cmake` sweeps both.

## What was and was not verified

There was no OAK camera available while this was written.

Verified on **macOS arm64**, against a staged package with no camera attached:
depthai-core builds and installs, the backend links it, exports exactly one
symbol, `CheckPackage.sh` passes, and it `dlopen`s and runs the whole ABI
(`init` / `enumerate` / failed `open` / `shutdown`) cleanly through
`@rpath`/`@loader_path`.

Verified on **Windows**: it is correctly skipped under all three msys2
environments. Under **MSVC it is still unproven** -- the machine available had
only Visual Studio 2026, and depthai's pinned vcpkg baseline (2025-06-02)
predates it and rejects the install as "Unable to find a valid Visual Studio
instance / Could not locate a complete Visual Studio instance", even though
`vcvarsall.bat` is exactly where it looked. That is a vcpkg-vintage problem
rather than anything in this tree; GitHub's `windows-2022` runner has VS2022,
which is what depthai's own CI builds against. Everything up to that point --
configure, the Oak/depthai target selection, the delay-load and `/MD` flags --
is correct there.

Verified, on Linux x86\_64, against a staged package with no camera attached:

- depthai-core v3.10.0 configures, builds and installs with the option set above;
- the backend compiles warning-clean against its headers, links the installed
  library, and exports exactly one symbol (`Deployment/CheckPackage.sh` passes);
- it `dlopen`s, reports ABI 3, `init()` succeeds and turns telemetry off;
- `enumerate()` runs an XLink discovery sweep and returns zero devices;
- `open()` fails with a readable message rather than crashing, and `shutdown()`
  is clean.

Not verified — it follows from the v3 headers and Luxonis' own examples, but has
not been seen to run:

- **any platform other than Linux x86\_64.** macOS and Windows are built the same
  way and are in Luxonis' own CI matrix, and the Windows plumbing above is the
  same the Oak backend uses, but none of it was exercised here. The `expect`
  lists in `.github/workflows/package.yml` are what will say so, loudly, on the
  first package run: Windows on ARM is the least-trodden of the six, since
  Luxonis ships no `arm64-windows` triplet and vcpkg's stock one would be doing
  the work;
- that a camera opens, that the pipeline starts, and that any frame arrives;
- the IR-before-Depth socket reuse described above. It follows from reading
  `Depth::ensureStereoOutputs`, which explicitly looks for pre-existing `Camera`
  nodes on the stereo sockets, but a camera would settle it in seconds;
- whether `Algorithm::AUTO` picks ToF on an OAK-D-SR-PoE and neural depth on an
  RVC4 device, which is the entire reason for using that node;
- the IMU pairing. depthai batches an accelerometer and a gyroscope report into
  one `IMUPacket`, and both fields are published per packet on that basis.
