# Tests

Two layers, because the bugs live in two places.

`abi/` talks to a backend `.so` directly through `depthcam_abi.h` and needs
nothing else — no score, no Qt. That is where "does this camera stream" gets
answered.

`score/` drives score itself through its JS scripting API, which is the only
way to cover the parts a user actually touches: the protocol factory, the
enumerators, settings travelling through `QVariant`, device creation, the node
tree, and teardown. Several of the nastiest bugs here were invisible at the ABI
layer and obvious at this one.

Both need real hardware. Neither is in CI for that reason; CI checks that things
build and that each backend still exports exactly one symbol.

## abi/

```sh
g++ -std=c++20 -O1 -o /tmp/nettest Tests/abi/nettest.cpp \
    -I DepthCamera/Backend -ldl
/tmp/nettest <build>/depth-camera/score_depthcam_orbbec.so \
             <build>/depth-camera [uri]
```

| Harness | What it answers |
|---|---|
| `nettest` | Does it enumerate, open and deliver frames? Takes an optional address, which is how Ethernet cameras get tested. |
| `gaptest` | Does a long gap between `open()` and `start()` break streaming? (It does not — which is how the Azure Kinect bug was traced to the score side.) |
| `reopen` | open / close / reopen, and open-while-open. Catches a backend that does not let go. |
| `ctltest` | Lists every control with its kind, access, range and current value. |
| `ctlwrite` | Writes every writable control and reads it back. Distinguishes "took", "refused" and "silently ignored". |
| `k4aenum` | Whether enumerating interferes with an open device, and vice versa. |
| `imutest` | Inertial samples: rate, which fields arrive, and whether a camera at rest really measures one gravity. |
| `streamsq` | What `open()` actually granted, against what was asked for. |

## score/

```sh
Tests/score/run.sh <build-dir> <suite> [timeout]
Tests/score/run.sh ~/ossia/score/build-release reconnect
```

Use the runner rather than invoking score directly. score does **not** exit
after evaluating a `--script`, so a plain `timeout N` burns the whole N every
time -- forty minutes per suite under a sanitizer, for work that is over in
two. The runner watches the report file for its last line and stops there. It
also splits any sanitizer findings into ours and everyone else's.

By hand, if you need to:

```sh
QT_ASSUME_STDERR_HAS_CONSOLE=1 \
  ossia-score --no-gui --script="$(cat Tests/score/reconnect_e2e.js)"
```

`--no-gui` matters: with a GUI the start screen defers document creation and the
script never evaluates. Results go to a file under `/tmp` because `console.log`
is not routed to stdout in this configuration. Each script runs against every
camera the enumerators report, so plugging in more hardware widens the test.

| Script | What it answers |
|---|---|
| `depthcam_e2e` | Enumerators, device creation, the stream nodes, a hand-typed address. |
| `controls_e2e` | The `controls` subtree: how many, grouped how, per backend. |
| `reconnect_e2e` | That a camera is released the instant its device is removed, and that a second device on the same camera cannot take the first one's streams away. |
| `latency_e2e` | That nothing blocks the caller: walking the tree, listening to every node, and re-walking while polling all have to stay under 100ms whatever the camera is. |
| `imu_e2e` | The inertial toggle: nodes on a camera that has an IMU, nothing on one that does not, nothing at all when the toggle is off, and no stall from listening to a 1.6kHz signal. |

`latency_e2e` is the regression test for a specific mistake — polling the camera
from the GUI thread — and it is the one to run after touching
`DepthCameraControls`.

## Sanitizers

`build-asan-ubsan` (clang, `-fsanitize=address,undefined,integer`) builds the
plug-in and all seven backends. The harnesses have to be built with the same
flags — ASan's runtime must come first in the initial library list, and a plain
executable that `dlopen`s an instrumented backend aborts with *"ASan runtime
does not come first"*. `Tests/abi/asan_run.sh` does both and reports per test.

Two things about the results are worth knowing in advance.

**An Azure Kinect cannot be tested under ASan beyond its colour stream.**
Anything involving depth initialises `libdepthengine`, Microsoft's closed
transform engine, which fails with `error code: 108` in an instrumented
process. `DEPTHCAM_TEST_STREAMS=1` on `nettest` asks for colour only, which is
the one configuration that runs.

**Most of what a sanitizer reports here belongs to the vendored SDKs.** Filter
by path before reading anything into it: `librealsense`, `libjpeg`, `libusb`,
`libfreenect`, `libfreenect2`, `libpointcloud`, depthai-core (inside `score_depthcam_depthai`), and
libstdc++ itself under
`-fsanitize=integer` (which flags the `0 - 1` in every `std::string` and
`std::vector` internal, where unsigned wraparound is defined and intended). A
run that reports nothing in `DepthCamera/` or `Backends/*/` is the outcome to
want.

`libpointcloud` is worse than the others here and cannot be improved: it is a
prebuilt binary, so it is neither instrumented nor rebuildable, and everything
ASan can say about it is a shadow-less report from a stripped frame. Findings
inside the Oak backend's own translation unit still count.

### Check the tests actually ran

A backend with no camera, or no SDK, reports nothing — and a sanitizer run that
did nothing looks exactly like a clean one. Every log has to be checked for
evidence of work before it counts: `frames:` for a stream test, a control count,
`open #4` for a reopen test. The first pass here scored the Azure Kinect clean
four times over while libk4a was not even present in the sanitizer build tree.

### Two traps in the scripting API

Both cost real time here, so they are worth stating.

**The settings an enumerator hands to JS are an opaque QVariant.** Passing
`cams[i].settings` straight to `Score.createDevice` works. Copying its
properties into a plain object -- `for (var k in d.settings) s[k] = d.settings[k]`
-- silently yields `{}`, which means an empty address, which every backend reads
as *the first camera it can see*. A test written that way opens the same camera
eight times while printing a different name each time and passing throughout.
`imu_e2e` addresses cameras by bare backend prefix (`"orbbec:"`, `"k4a:"`) for
exactly this reason.

**A device's inertial readings and its inertial settings are both spelled
`imu`.** `cam:/imu/accel` is a reading; `cam:/controls/imu/gyro_odr` is a
setting. Matching on `"/imu/"` finds both.

### A caveat worth knowing

`Score.deviceToJson()` aborts the process on any Gfx input device, this one
included: `ossia::presets::make_json_preset` throws `value_to_json_value: no
type` on the texture and geometry parameters, which hold no ossia value at all.
That is not specific to depth cameras and not something this add-on can fix, so
the scripts avoid it.
