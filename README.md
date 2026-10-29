# Depth cameras for ossia score

Streams colour, infrared, depth and point clouds from depth cameras into
[ossia score](https://ossia.io), as an ordinary device you add from the device
explorer.

| Camera | Backend | Linux | Windows | macOS |
|---|---|---|---|---|
| Orbbec Femto Mega / Mega i / Bolt | `orbbec` | yes | yes | colour + IR only¹ |
| Orbbec Gemini 330/335/336/345, Gemini 2, Astra 2, DaBai | `orbbec` | yes | yes | yes |
| Kinect v2 (Xbox One) | `freenect2` | yes | yes | yes |
| Kinect v1 (Xbox 360) | `freenect` | yes | yes | yes |
| Azure Kinect DK | `k4a` | yes | yes | no² |
| Intel RealSense D400/D500/L500/SR300 | `realsense` | yes | yes | yes |
| Oak3DVision Oak CDK / QCDK | `oak` | yes | no³ | x86\_64 only³ |
| Luxonis OAK-D / OAK-1 / OAK-4, all variants | `depthai` | yes | yes | yes |

¹ Femto Mega and Bolt compute depth with a proprietary depth engine that Orbbec
does not ship for macOS, so those two have no depth or point cloud there. The
Gemini and Astra lines compute depth on-device and are unaffected.

² Microsoft never released an Azure Kinect SDK for macOS.

³ Oak3DVision ships `libPointCloud` only as prebuilt binaries, and only ever
built four: Linux x86\_64, Linux aarch64, macOS x86\_64 and Windows **x86** —
the last of which is 32-bit, so it cannot be linked into the 64-bit module score
loads. There is nothing to recompile, so there is no Oak backend on Apple
Silicon, and none on Windows at all. It also has runtime prerequisites the others do not — see
[Oak3DVision cameras](#oak3dvision-cameras). Note that these are **not** the
Luxonis OAK-D: the names collide, and those are a different camera on a
different SDK. Luxonis cameras are the `depthai` backend, one row down.

⁴ The Luxonis backend is the one that is not built alongside score even with the
submodules checked out — see [Luxonis OAK cameras](#luxonis-oak-cameras).

Orbbec Femto Mega also works over Ethernet — see [Connecting by address](#connecting-by-address).

## Installing

The plug-in itself contains no camera SDK, so it is always present and costs
nothing if you have no camera. The SDKs arrive as one **Depth cameras** package
you install from score's package manager. It lands in:

```
~/Documents/ossia/score/packages/depth-camera/
```

and contains one self-contained shared object per camera family, plus the
auxiliary files two of the SDKs load at runtime:

```
depth-camera/
  package.json
  score_depthcam_orbbec.so        Femto, Gemini, Astra   (USB and Ethernet)
  score_depthcam_freenect.so      Kinect v1
  score_depthcam_freenect2.so     Kinect v2
  score_depthcam_k4a.so           Azure Kinect DK
  score_depthcam_realsense.so     Intel RealSense
  score_depthcam_oak.so           Oak3DVision Oak CDK / QCDK
  score_depthcam_depthai.so       Luxonis OAK
  extensions/                     OrbbecSDK runtime blobs, loaded by path
  libpointcloud.so                the Oak SDK, which ships as a binary
  oak/plugin/, oak/conf/          its camera plug-ins and model descriptions
  libdepthai-core.so              the Luxonis SDK, with the device firmware in it
  udev/                           the Linux rules, see below
```

The whole thing is about 65 MB installed, roughly half of which is the Luxonis
SDK with the MyriadX and RVC4 device firmware compiled into it. One package
rather than one per SDK: plugging in a camera should not require working out
which of seven downloads yours is in.

After installing, **install the udev rules** (Linux only, see below) and restart
score.

Nothing else has to be set up — no `LD_LIBRARY_PATH`, no rpath. Each backend
resolves whatever sits beside it through `$ORIGIN`, and is given its own
directory to find `extensions/` and the Oak SDK's `oak/` tree in.

The one backend that is not self-contained is `oak`: Oak3DVision publishes
`libPointCloud` as a binary rather than as sources, and it brings dependencies
of its own. See [Oak3DVision cameras](#oak3dvision-cameras).

### Linux: udev rules are not optional

Every one of these SDKs talks to the camera over raw USB. Without the matching
udev rules the device nodes stay root-owned, the SDK reports *no cameras*, and
the symptom is indistinguishable from nothing being plugged in.

The package ships the rules for all the supported hardware in its `udev/`
subdirectory:

```sh
sudo cp ~/Documents/ossia/score/packages/depth-camera/udev/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then **unplug and replug the camera**, or reboot.

The rules grant access through `TAG+="uaccess"` (systemd hands the logged-in
user an ACL) with `MODE="0666"` as a fallback for systems without logind.

> If you previously installed Microsoft's own `99-k4a.rules`, remove it. It does
> not work on a current udev: it opens with `BUS!="usb"` (a key udev removed years
> ago) and assigns `GROUP="plugdev"`, a group that does not exist on Arch, Fedora
> or openSUSE. When the group cannot be resolved udev discards the whole rule,
> `MODE` included, so the camera stays inaccessible.

To check that the rules took effect:

```sh
lsusb                       # find the bus/device numbers of your camera
ls -l /dev/bus/usb/<bus>/<dev>   # should be mode 666, not 664
```

### Azure Kinect: the SDK is not redistributable

The `k4a` backend does not bundle libk4a. The Azure Kinect requires
`libdepthengine`, a closed Microsoft binary we have no licence to ship, and
libk4a is useless without it. The backend therefore looks for the SDK at
runtime, in this order:

1. next to the backend, inside its own package directory;
2. the normal library search path (a system-installed SDK).

So you can either install the [Azure Kinect Sensor
SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) system-wide, or drop
it into the package folder yourself:

```
depth-camera/
  libk4a.so.1.4
  libk4a1.4/
    libdepthengine.so.2.0
```

The subdirectory is not a suggestion: libk4a's own `RUNPATH` is
`$ORIGIN/libk4a1.4`, so that is the only place it looks for the depth engine.
Put the engine flat next to libk4a and the camera opens but produces no depth.

If it is missing, score's log says so explicitly rather than leaving the camera
mysteriously absent:

```
[depthcam] "k4a" reports: libk4a not found. Install the Azure Kinect Sensor SDK,
           or place libk4a and libdepthengine next to this backend.
```

### Oak3DVision cameras

The **Oak CDK** (`14b4:9108`) and **Oak QCDK** (`14b4:9107`) are time-of-flight
modules with an illuminator and no colour sensor, so they offer `depth`, `ir`
and `pointcloud` and nothing else. The `ir` node carries the ToF amplitude
image, which is what the sensor measured of its own returning light; ask for a
coloured point cloud and you get that amplitude as grey.

> These are **not** the Luxonis OAK-D. The names collide. OAK-D, OAK-D-Lite and
> OAK-D-Pro are stereo-depth cameras driven by
> [`depthai-core`](https://github.com/luxonis/depthai-core), and none of them is
> reachable through this backend.

Unlike every other SDK here, `libPointCloud` is not built from source —
Oak3DVision publishes none. Their repository *is* the binary distribution, so
this backend links the prebuilt library for the platform being built and ships
it, its camera plug-ins and its `.conf`/`.dml` model descriptions in the
package. Two things follow from that.

**There is no Oak backend on Apple Silicon, and none on Windows.** Oak3DVision
built four binaries — Linux x86\_64, Linux aarch64, macOS x86\_64 and Windows
**x86**. That last one is 32-bit (`pointcloud.dll` and `pointcloud.lib` are both
machine type `0x014c`), so it cannot link into the 64-bit module score loads;
linking it against x64 gives "library machine type 'x86' conflicts with target
machine type 'x64'" and 23 unresolved externals. There is nothing to recompile.

**The vendor's binary brings its own dependencies**, which the rest of the
package deliberately does not have:

| | needs |
|---|---|
| Linux x86\_64 | `libglfw3`, `libgl1`, `ocl-icd-libopencl1` |
| Linux aarch64 | nothing beyond libusb |
| macOS | `OpenCL.framework` (part of the OS) |
| Windows | the **Visual C++ redistributable**, and `OpenCL.dll` from a graphics driver |

On Debian and Ubuntu:

```sh
sudo apt install libglfw3 libgl1 ocl-icd-libopencl1
```

If they are missing, only this backend is affected — the module fails to load,
score logs it, and every other camera keeps working:

```
[depthcam] could not load .../score_depthcam_oak.so:
           libglfw.so.3: cannot open shared object file
```

The Windows failures are reported through the backend instead, as a sentence in
the log, because `pointcloud.dll` is delay-loaded there on purpose.

Camera settings are a curated list rather than everything the SDK offers: the
MLX75027's register map alone is 307 raw fields, most of them meaningless
outside the vendor's bring-up tool, and several will stop the sensor if written.
Set `SCORE_DEPTHCAM_OAK_ALL_CONTROLS=1` to publish all of them under
`advanced/`. See `Backends/Oak/README.md`, which also lists what has and has not
been tried on hardware.

### Luxonis OAK cameras

The whole OAK range, RVC2 and RVC4 alike: OAK-D and its variants (Lite, S2, Pro,
W, SR, LR), the OAK-1 mono models, the PoE versions over Ethernet, and the OAK-4
generation. Colour, infrared, depth, point cloud and — on the models that have
one — the inertial sensor.

Which of those a given camera can actually deliver is not assumed. An OAK-1 has
no depth; most OAKs have no IMU; only some have an illumination projector. The
backend reports what it actually wired up, so score does not publish nodes that
nothing will ever fill in.

Depth comes from depthai's unified `Depth` node with `Algorithm::AUTO`, which
picks stereo matching, on-sensor time-of-flight, or on-device neural depth from
what the camera in front of it has. That is deliberate and is what makes one
backend cover the range: hand-wiring `StereoDepth`, which is what most depthai
examples do, works on an OAK-D and produces nothing at all on an OAK-D-SR-PoE.

Two settings are worth knowing about:

- **`depth/laser_power`** — the structured-light dot projector, on an OAK-D-Pro
  and friends. It is **off by default** and it is what makes depth work on a
  blank wall. If depth looks empty on a plain surface, this is the first thing
  to turn up.
- **`ir/flood_light`** — the infrared floodlight, on the same models.

Both are published only where `getIrDrivers()` reports the hardware, so they do
not appear on a camera that has none.

**Telemetry is turned off.** depthai-core reports anonymised usage to Luxonis
unless told otherwise, and it would be doing that from inside a media
application you never pointed at the internet. The backend sets
`DEPTHAI_TELEMETRY=0` — but only if you have not set it yourself, so an explicit
`DEPTHAI_TELEMETRY=1` is respected as the opt-in it is.

**It is the one backend not built alongside score.** Every other SDK here is
cheap enough that compiling it next to score costs nothing anyone notices; this
one bootstraps a vcpkg, builds twenty dependencies before it builds itself, is
the only SDK here that needs the network at build time, and leaves about a
gigabyte of build tree behind. So it is on by
default *only* in a backends-only build — the one that produces the package —
and a developer who cloned this repository recursively does not turn every score
build into a vcpkg bootstrap. Pass `-DSCORE_DEPTHCAM_BUILD_DEPTHAI=ON` to work
on it.

**Not under msys2.** depthai resolves its dependencies through vcpkg, which
picks its triplet from the host platform rather than from the compiler: on
Windows that is always an MSVC triplet, and the libraries it produces cannot be
linked into a mingw module. A mingw build skips this backend and says so; build
it with MSVC.

**Not in a universal macOS binary**, for the same reason — a vcpkg triplet names
one architecture. A fat `x86_64` + `x86_64h` build is fine, because that is one
architecture in two flavours; Intel and Apple Silicon in one binary is not, and
is skipped with a message rather than failing the build.

**On Windows with MSVC it needs the Visual C++ redistributable**, alone in this
package.
depthai's vcpkg triplet builds against the dynamic CRT, so the backend has to
match it — `std::shared_ptr`s allocated inside `depthai-core.dll` are released
here, and two CRTs would free them against the wrong heap. If it is missing, the
backend says so in the log instead of quietly not appearing.

### USB 3 is required for several cameras

The Azure Kinect DK, Kinect v2 and RealSense D400 series all need USB 3.

- On USB 2 the **Kinect v2 and Azure Kinect do not enumerate their cameras at
  all** — only the internal hub and microphone array appear, which looks exactly
  like a permissions problem.
- A **RealSense D435i on USB 2** *does* work, but drops to a handful of low-rate
  profiles: there is no 30fps colour mode at all, and you may see under 1fps on
  every stream. If a RealSense feels frozen, check this first.

To see what link a camera actually negotiated:

```sh
for d in /sys/bus/usb/devices/*/; do
  [ -f "$d/idVendor" ] && echo "$(basename $d) $(cat $d/idVendor):$(cat $d/idProduct) $(cat $d/speed) Mbps"
done
```

`480` is USB 2. Note that a USB-C *cable* does not imply a USB 3 *port* — check
the port, not the cable.

## Using it

Add a device, pick **Depth Camera Input** under *Video In*. Each installed
backend appears as its own category in the browser, listing the cameras it can
see. Selecting one fills in the address.

The device exposes one child node per enabled stream:

| Node | Type | Notes |
|---|---|---|
| `rgb` | texture | the Oak CDK/QCDK have no colour sensor and do not offer it |
| `ir` | texture | R32F, the sensor's 8- or 16-bit range mapped to 0–1 and scaled by `controls/render/ir_gain`. Not offered by Kinect v1, which can stream IR only *instead of* colour; on a ToF camera this is the amplitude image |
| `depth` | texture | R32F, metres along +Z, 0 where the sensor has no measurement. Or, with *Depth output: Preview*, the 0–4.5 m grey of earlier documents |
| `positions` | texture | RGBA32F: xyz in metres in the depth camera's frame, w = 1 where valid. One texel per depth pixel. Opt-in |
| `normals` | texture | RGBA16F, unit normals facing the camera, w = 1 where valid. Opt-in |
| `mesh` | geometry | the depth grid as GPU vertex and index buffers: `position`, `normal`, `texcoord0` into the colour image. Triangles across a depth edge or with an invalid corner are dropped. Opt-in |
| `pointcloud` | geometry | the SDK's CPU point cloud; connect to *Model Display* or any geometry input |
| `imu/accel` | vec3f | acceleration in m/s², including gravity; off by default |
| `imu/gyro` | vec3f | angular velocity in rad/s |
| `imu/temperature` | float | degrees Celsius, where the camera reports it |

Point-cloud and position coordinates are in **metres**, right-handed, +X right /
+Y down / +Z forward from the depth camera. Colours, where present, are
normalised to 0–1.

A stream nothing is connected to costs nothing: its frames go straight back to
the SDK, and the backend is told not to produce it — which matters most for the
CPU point cloud, the most expensive thing a camera does here.

### Metric depth and the GPU outputs

`depth`, `positions`, `normals` and `mesh` are four outlets of one node: each
depth frame is uploaded once, as the 16-bit or float samples the camera sent,
and turned into positions in one pass on the GPU, whichever outlets are
connected. The depth unit comes with every frame, so a camera in 0.1 mm mode, a
Kinect v2's float millimetres and an Oak's float metres all come out in metres.

A float texture keeps its precision only into a float input: a shader input
whose texture format is RGBA8 clamps metres to 1. Set the input's format to
R32F (or RGBA32F for positions), or declare the input `STATIC`, which samples
the device's own texture.

The mesh needs compute shaders, which every backend has except OpenGL below 4.3 and OpenGL ES below 3.1;
the textures do not.

Host-side processing applies to all four, on every camera, whatever its SDK
offers:

| Control | |
|---|---|
| `controls/render/near`, `far` | depth outside these, in metres, is invalid; 0 for no limit |
| `controls/render/stage_box/enabled`, `min`, `max` | only what lies inside the box, in the depth camera's frame, is kept |
| `controls/render/temporal` | weight of the previous frames in a running average, 0 for none; a change of more than 5 % restarts it |
| `controls/render/edge_cull` | a mesh triangle whose longest edge is more than this times its depth is dropped; 0 keeps every triangle with three valid corners |
| `controls/render/topology` | `triangles` or `points` for the mesh |
| `controls/render/ir_gain` | infrared brightness |

For depth from anything else — the Kinect v2 device, a capture card, a
monocular depth model — `Presets/Compute Shader/Depth to positions.cs` makes the
same position map from a depth image and its intrinsics.

### Calibration and metadata

Read-only, alongside the streams:

```
cam:/info/{backend,model,serial,firmware,hardware_rev,transport,uri}
cam:/info/depth_unit_mm            millimetres per raw depth sample, from the latest frame
cam:/info/depth_range              vec2f, metres
cam:/info/calibration_estimated    true when the camera reports nothing and a 60° pinhole stands in
cam:/calibration/<s>/{width,height}             s = depth, color, ir
cam:/calibration/<s>/intrinsics                 vec4f fx, fy, cx, cy
cam:/calibration/<s>/distortion/{model,coeffs}  coeffs: k1 k2 k3 k4 k5 k6 p1 p2
cam:/calibration/<s>/projection                 16 floats, column-major
cam:/calibration/depth_to_color                 16 floats, column-major, metres
cam:/calibration/depth_to_color/row0 ... row3   the same matrix, as vec4 rows
cam:/calibration/depth_to_imu
cam:/stats/<s>/{frames,dropped,fps,device_time_ms,host_time_ms,interval_ms}
```

Conventions: intrinsics are those of the stream **as delivered** (with depth
aligned to colour, the depth stream has the colour camera's), in its pixels,
with the centre of pixel (u, v) at (u, v) as in OpenCV. Matrices are
column-major, metres. `projection` maps that camera's frame to OpenGL clip
space for the near and far planes of `render/near` and `render/far` (the depth
range when they are 0). The rows exist because ISF shaders take vectors but no
matrices. The calibration is read again after anything that restarts the
stream.

`stats/<s>/device_time_ms` is updated at every frame, from the camera's clock,
so a process can trigger on arrival; the rest ten times a second. Each frame
also carries its capture time on the device clock and its arrival on the host
clock through the graph.

### Orbbec cameras over Ethernet

A Femto Mega with an Ethernet port shows up in the browser like any other
camera, alongside the USB ones, with `Ethernet` as its transport. There is
nothing to configure: the SDK broadcasts a GVCP discovery request on every
interface and lists whatever answers.

A Luxonis OAK PoE is the same, and the **Network** checkbox now covers both:
pick the family beside it, since the two are addressed differently — an Orbbec
takes a host and a port, an OAK takes a host alone (XLink's port is fixed), so
the port field is only enabled for Orbbec.

**If no network camera appears, suspect your firewall before the camera.** The
cameras reply to `255.255.255.255`, not to the host that asked — the firmware
ignores the GigE Vision "unicast acknowledge" flag — so the answer arrives as a
broadcast datagram, and a host firewall that drops broadcast input hides every
networked camera on the segment while they are all answering perfectly well.

`ufw` does exactly that out of the box; its `ufw-after-input` chain sends
anything with a broadcast destination straight to the default DROP policy. Since
the reply's *destination* port is ephemeral, the rule has to match the *source*
port:

```sh
sudo ufw allow proto udp from 192.168.0.0/24 port 3956 comment 'Orbbec GVCP discovery'
```

with `firewalld`:

```sh
sudo firewall-cmd --permanent --add-rich-rule='rule family=ipv4 source address=192.168.0.0/24 port port=3956 protocol=udp accept'
sudo firewall-cmd --reload
```

To check whether the cameras are answering at all, independently of score:

```sh
sudo tcpdump -ni any 'udp port 3956'
```

Each camera should send one 256-byte reply per discovery request. If you see the
replies here but no camera in score, it is the firewall.

Discovery is a broadcast, so it also never crosses a router: a camera on another
subnet has to be addressed directly. Tick **Connect over the network** in the
device dialog and type its host and port (8090 unless you changed it). That path
is a plain TCP connection and is unaffected by any of the above.

### Connecting by address

The **Camera** field is editable. Picking a camera from the browser fills it in,
but typing one by hand is the same code path — which is how you reach a camera
that is not currently being discovered:

```
orbbec:sn:CL8L1234ABC          match by serial number
orbbec:uid:2-1.3               match by SDK uid (survives identical serials)
orbbec:net:192.168.0.12        Femto Mega over Ethernet, default port 8090
orbbec:net:192.168.0.12:8090   explicit port
freenect2:sn:012345678901      Kinect v2
freenect:index:0               Kinect v1
freenect:index:0?motor=1       Kinect v1, claiming the tilt motor as well
k4a:sn:000123456789            Azure Kinect
realsense:sn:040322070203      RealSense
```

Leave it empty to take the first camera any backend reports.

If a named camera is not present the device fails to connect rather than falling
back to a different one: silently streaming from the wrong camera is worse than
not connecting.

### Accelerometer and gyroscope

Tick **Accelerometer / gyroscope** to publish the camera's inertial sensors at
the device root. Units are SI — metres per second squared and radians per second
— for the same reason point clouds are in metres: a patch should not have to
know which camera it is reading.

| Camera | Rate | Temperature |
|---|---|---|
| Azure Kinect DK | 1.6 kHz, both | yes |
| Intel RealSense D435i | 200 Hz gyro, 63 Hz accel | no |
| Orbbec Femto Mega | as configured, 50 Hz by default | yes |
| Kinect v1 | 2 Hz, accelerometer only, needs `?motor=1` | no |
| Kinect v2 | — no inertial sensor | — |

A camera that has none publishes no nodes at all rather than three that never
move, so the tree tells you what the hardware can actually do.

Accelerometer and gyroscope are sampled independently and rarely at the same
rate, so each arrives on its own: on a D435i the gyro node updates three times
per accelerometer update. Nothing is paired up or held back.

Off by default, and not because it is expensive — it is a few hundred samples a
second and no image data. It is off because asking changes what the camera
streams: an Orbbec runs a second pipeline for it, an Azure Kinect a second
capture loop.

Where the camera lets you choose a rate, it is a setting like any other:
`controls/imu/accel_odr` and `controls/imu/gyro_odr` on an Orbbec, with
`*_full_scale` for the ranges. Those are the IMU's *configuration*, at
`controls/imu/`; the readings are at `imu/`. Same word, opposite direction.

### Camera settings

Whatever the camera's SDK will let you change appears under a `controls` node,
grouped by what it affects:

```
cam:/controls/color/exposure
cam:/controls/color/auto_exposure
cam:/controls/depth/laser_power
cam:/controls/imu/gyro_sensitivity
cam:/controls/sensors/temperature
cam:/controls/advanced/reboot_device
```

These are ordinary ossia parameters: automate them, drive them from OSC, read
them back. Each carries its real type, range and access mode, so a slider knows
its bounds and a read-only sensor is not offered as writable.

Nothing about the list is hard-coded. Orbbec and librealsense enumerate their
settings at runtime, so a camera that did not exist when this was written still
comes out organised; k4a and the two libfreenect backends have fixed lists but
ask the device what it supports. Roughly what to expect:

| Camera | Controls |
|---|---|
| Orbbec Femto Mega | 39 — colour, depth, laser, IMU, fan, sync, sensors — plus its recommended filters, presets and depth work modes |
| Intel RealSense D435i | 43 — per sensor: stereo module, RGB, IMU — plus the post-processing filters |
| Azure Kinect DK | 12 — the colour controls, plus sensor temperature and the depth mode |
| Kinect v2 | 5 — the three exposure modes and their parameters — plus the depth processing range and filters |
| Kinect v1 | 6 — tilt, LED, accelerometer (see below) |

Every control's description states its unit where the SDK states one, and its
step; distances and times also carry an ossia unit. Tags mark what is
`advanced`, what is an SDK `filter`, what is `volatile` (changes on its own)
and what needs a `restart`.

A few groups are worth calling out:

- **`filters/<name>/...`** are the SDK's own depth post-processing, where it has
  any: the Orbbec recommended filter chain, librealsense's decimation,
  threshold, spatial, temporal and hole filling, DepthAI's stereo settings,
  libfreenect2's depth range and filters. They apply to the depth the device
  delivers, and to the SDK point cloud.
- **`device/preset`** and **`depth/mode`** are named lists: an Orbbec preset or
  work mode, a RealSense visual preset, the k4a depth modes.
- **`sensors/`** holds read-only values: temperatures, accelerometers. Most SDKs
  do not report a change, so these — and a control its `auto` toggle has made
  read-only, such as an exposure under auto-exposure — are polled, but only
  while something is listening to them. librealsense does report changes,
  and its controls are read again when it does. A control nobody is watching
  costs nothing.
- **`advanced/`** holds triggers with no readable value: rebooting the camera,
  entering recovery mode, and the two below. They are grouped away deliberately
  so they are not one click from a performer mid-performance.

Writes go to the camera from a thread of their own. A camera that clamps or
rounds a value, or refuses it outright, is reflected on the next read rather
than silently accepted. While an `auto` toggle is on, the controls it rules are
read-only. A control that only takes effect when the stream restarts (a depth
mode) restarts it, and the calibration is read again.

**What you write is saved with the document**, in the order you wrote it, and
written back to the camera the next time it opens, before it starts streaming.
`advanced/store_current_settings` saves every current setting instead,
including ones changed in another program; `advanced/reset_to_defaults` puts
every setting back to the camera's default and forgets the saved ones.

**Kinect v1 tilt and LED** are opt-in: append `?motor=1` to the address. The
motor is a separate USB interface, and on the original model 1414 claiming it
costs nothing — but on a model 1473 or a Kinect for Windows libfreenect drives
the motor through the *audio* interface, which needs a firmware upload this
build does not ship. Asking for it there fails the open outright and resets the
camera for several seconds. A camera that streams is worth more than a tilt
motor, so nothing is risked unless you ask.

### Alignment, and why it decides your frame rate

**Alignment** brings depth and colour into a common frame of reference. It also
decides the point cloud's resolution, which dominates everything else about its
cost. On a Femto Mega:

| Mode | Cloud resolution | Points | Per frame |
|---|---|---|---|
| Depth to color | colour (1920×1080) | 2 073 600 | 47.5 MB |
| Color to depth | depth (640×576) | 368 640 | 8.4 MB |
| None | depth | 368 640 | 8.4 MB |

*Depth to color* upsamples depth to the colour resolution, inventing roughly five
points for every one the sensor measured. *Color to depth* is the default and is
usually what you want for a point cloud; *depth to color* is right when you want
`rgb` and `depth` pixel-aligned for your own shader work.

**A coloured point cloud requires an alignment other than None** — the option is
disabled otherwise. Without one the SDK would read mismatched buffers, and at
least one of them walks off the end of the colour image.

### Resolution and frame rate

`Any` lets the backend choose. Where the backend can list them, each stream
also offers the modes the camera has, and picking one fills in the three
values. Two things worth knowing:

- Setting a **colour resolution** is the simplest way to cut the cost of a
  *depth to color* point cloud.
- With a coloured point cloud the Orbbec backend requests **raw RGB**, because
  the point-cloud filter cannot consume the MJPEG the camera negotiates by
  default. If you leave the resolution on `Any` it pins 1920×1080, since an
  explicit format with "any" resolution makes the SDK pick its *largest* profile
  — 3840×2160 on a Femto Mega, which is 8.3M points and ~199 MB per frame.

## Troubleshooting

Set `SCORE_DEPTHCAM_DEBUG=1` to have each point-cloud node report, once a
second, how many render passes, new frames and uploads it saw. That separates
"the camera is not delivering" from "the graph is not asking", which look
identical from the outside.

| Symptom | Likely cause |
|---|---|
| No cameras in the browser at all | No backend installed; check score's log for `[depthcam] loaded backend` |
| One camera family missing | That backend is missing, or its udev rules are not installed |
| Azure Kinect missing | libk4a not found — see above |
| Kinect v2 / Azure Kinect missing entirely | Plugged into USB 2 |
| Everything very slow, RealSense | Plugged into USB 2 |
| Coloured point cloud is white | Fixed; if you see it again, report the camera model |
| `Colored point cloud` greyed out | Alignment is set to None |
| No **Ethernet** Orbbec cameras, USB ones fine | A host firewall dropping broadcast input — see [Orbbec cameras over Ethernet](#orbbec-cameras-over-ethernet) |
| Kinect v1 opens once, then "is not connected" | A model 1473 reports a different serial each time it is listed; fixed, but an address saved by an older build may need re-picking from the browser |
| No `controls` node | That backend has nothing to offer for this camera, or (Kinect v1) the motor was not requested |

In a Release build score's own diagnostics go to stderr, which is not attached
to a console by default on any platform. To see them:

```sh
QT_ASSUME_STDERR_HAS_CONSOLE=1 ossia-score
```

## Building from source

Requires the submodules:

```sh
git clone --recursive https://github.com/jcelerier/score-addon-orbbec
```

The plug-in and the backends build independently. A checkout **without**
submodules builds the plug-in alone — which is a complete, working device that
says "no camera backend is installed" until a package is.

That is exactly what score does with this repository. `src/addons/*` is
gitignored there and the add-ons are cloned by `ci/common.deps.sh`, where this
one is the entry that carries `NO_SUBMODULES=1`:

```sh
NO_SUBMODULES=1 clone_addon https://github.com/ossia/score-addon-orbbec
```

so score gets the top level and none of the six vendored SDKs, which together
take longer to build than the rest of score — depthai alone bootstraps a vcpkg
and needs the network. That one word is what keeps the two halves apart; drop it
and every score build starts building camera SDKs.

```
-DSCORE_DEPTHCAM_BUILD_BACKENDS=OFF   # plug-in only; the default with no submodules
-DSCORE_DEPTHCAM_BUILD_BACKENDS=ON    # also build the SDKs; the default with them
```

Individual backends build only when their submodule is present, and each can be
turned off:

```
-DSCORE_DEPTHCAM_BUILD_ORBBEC=OFF
-DSCORE_DEPTHCAM_BUILD_FREENECT=OFF
-DSCORE_DEPTHCAM_BUILD_FREENECT2=OFF
-DSCORE_DEPTHCAM_BUILD_K4A=OFF          # off by default on macOS: no SDK exists there
-DSCORE_DEPTHCAM_BUILD_REALSENSE=OFF
-DSCORE_DEPTHCAM_BUILD_OAK=OFF          # off where Oak3DVision shipped no binary
-DSCORE_DEPTHCAM_BUILD_DEPTHAI=ON       # OFF unless SCORE_DEPTHCAM_BACKENDS_ONLY; see below
-DSCORE_DEPTHCAM_FREENECT2_OPENCL=ON    # Kinect v2 GPU depth pipeline, on by default
-DSCORE_DEPTHCAM_ORBBEC_STATIC=ON       # build the OrbbecSDK into the backend
```

`SCORE_DEPTHCAM_BUILD_DEPTHAI` is the odd one out: it defaults **off** unless
`SCORE_DEPTHCAM_BACKENDS_ONLY` is set, so it is built when the package is what
was asked for and not when you happen to have the submodules checked out. It is
a nested CMake project that bootstraps a vcpkg, it dominates the build time of
this repository, and its build tree is about a gigabyte in `CMAKE_BINARY_DIR` —
which, inside a score build, is score's own build directory. Pass `=ON` to work
on that backend.

### macOS architectures, and msys2

The plug-in half builds anywhere score does. For the backends:

| `CMAKE_OSX_ARCHITECTURES` | package key | Oak | Luxonis |
|---|---|---|---|
| `arm64` | `darwin-aarch64` | no¹ | yes |
| `x86_64` | `darwin-x86_64` | yes | yes |
| `x86_64h` | `darwin-x86_64` | yes | yes |
| `x86_64;x86_64h` | `darwin-x86_64` | yes | yes |
| `x86_64;arm64` | — | no¹ | no² |

¹ Oak3DVision shipped no arm64 macOS binary. ² vcpkg builds one architecture at
a time. `x86_64h` is a flavour of `x86_64`, not an architecture of its own, so a
fat binary of the two is one package key and one vcpkg build; only a genuine
Intel + Apple Silicon universal binary is unpublishable, and that configuration
is an error **only** when a package is what was asked for
(`SCORE_DEPTHCAM_BACKENDS_ONLY`) — an ordinary universal score build carries on.

On Windows the tree builds under MSVC and under msys2 (`mingw64`, `ucrt64` and
`clang64` all verified). Neither of the two newest backends is available under
msys2 — Oak's vendor binary is 32-bit MSVC, and depthai needs an MSVC vcpkg
triplet — so both skip themselves with a message and the rest builds normally.
Every MSVC-only flag in this tree is guarded on `MSVC`, not on `WIN32`.

### Building the package on its own

The reverse of the above: backends and no plug-in.

```sh
cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Release \
      -DSCORE_DEPTHCAM_BACKENDS_ONLY=1
cmake --build build --target depthcam-package
```

This needs a C++ compiler and the vendored SDKs and **nothing else** — no score
checkout, no Qt, no ossia SDK. The backends share no code with the plug-in; they
meet it at the pure-C `depthcam_abi.h` and are `dlopen`'d at runtime. That is
what lets the `Package` workflow publish a package for a platform without a
working score build on it, in minutes rather than hours.

### What each platform needs installed

| | Linux | macOS | Windows |
|---|---|---|---|
| libusb | system (`libusb-1.0-0-dev`) | vendored, static | vendored, static |
| libjpeg-turbo | system (`libturbojpeg0-dev`) | vendored, static | vendored, static |
| OpenCL (Kinect v2 depth) | system (`ocl-icd-opencl-dev`), optional | — | — |
| udev | system (`libudev-dev`) | — | — |
| libPointCloud (Oak) | vendored, prebuilt | vendored, prebuilt | vendored, prebuilt |
| depthai-core (Luxonis) | built from source¹ | built from source¹ | built from source¹ |

The Oak SDK is the odd one out: it is a submodule of *binaries* rather than
sources, so nothing needs installing to build it — but its own dependencies do
have to be present at **run** time, on the user's machine rather than the
builder's. See [Oak3DVision cameras](#oak3dvision-cameras).

¹ Only in a backends-only build; see `SCORE_DEPTHCAM_BUILD_DEPTHAI` above.

depthai-core is the odd one out in the other direction. It is built as a nested
CMake project, because it resolves its dependencies through a vcpkg it
bootstraps itself and that has to be the toolchain of the top-level configure —
so `add_subdirectory` cannot reach it, and its own documentation says as much.
Nothing has to be installed for it either, but two things follow:

- it is **by far the longest build here** (vcpkg, its dependencies, then ~260
  translation units of depthai), so `-DSCORE_DEPTHCAM_BUILD_DEPTHAI=OFF` is
  worth knowing about when you are iterating on something else;
- alone among the SDKs, it **needs the network at build time** — for vcpkg, and
  for the MyriadX and RVC4 device firmware it compiles into the library. Every
  other submodule here builds offline.

The vendored copies are the OrbbecSDK's, which already carries libusb 1.0.26
and libjpeg-turbo and builds both for winusb and darwin_usb.

Off Linux that is not a preference, it is the only thing that works. macOS has
no system libusb, and linking Homebrew's leaves the shipped backend referencing
`/opt/homebrew/opt/libusb/lib/libusb-1.0.0.dylib`, a path that exists only on
the machine that built it. Windows has none either, and shipping the DLL beside
the backend does not help: Windows resolves a module's dependencies through the
standard search order, which does not include the directory the module itself
came from. libfreenect2 also *requires* libjpeg-turbo on Windows, where it has
no VideoToolbox to fall back on, so without a vendored copy there is no Kinect
v2 backend there at all.

Each of the three consumers has to be told a different way — see the libusb
section of `Backends/CMakeLists.txt` and `Backends/cmake/`.

### Cutting a release

Tagging is the whole of it. `git tag -a vX.Y && git push origin vX.Y` runs the
`Package` workflow, which attaches one `depth-camera-<arch>.zip` per
architecture to the release.

`depth-camera.json` at the root is the *remote* manifest — the file score's
package manager fetches to find the download for `score::addonArchitecture()`.
It is pinned to a tag rather than to `releases/latest`, so that `version` keeps
meaning something to the update check, which means it has to be regenerated when
the tag changes:

```sh
python3 Deployment/make_manifest.py --tag vX.Y --repo ossia/score-addon-orbbec \
        --out depth-camera.json
```

`--only` restricts it to the architectures a given release actually carries. A
key pointing at an asset that does not exist is worse than a missing one: score
reports a failed download rather than "not available for your platform".

ossia/score-packages lists the raw URL of that file, so it does not need
touching again after the first time.

`Deployment/CheckPackage.sh <package-dir> "<backends>"` is what CI runs
afterwards: it fails if a backend the platform is meant to ship is missing, if
one is there that should not be, or if any of them exports more than its single
entry point. Every backend is optional at configure time, so a missing system
dependency otherwise drops one silently and the build still passes.

To produce the distributable package:

```sh
cmake --build . --target depthcam-package
```

which stages, strips and zips `<build>/depth-camera/` into
`depth-camera-<arch>.zip`, named the way `score::addonArchitecture()` spells it
(`linux-x86_64`, `darwin-aarch64`, `windows-x86_64`) — that is the key score's
package manager looks up in the remote manifest. Stripping is not cosmetic: with
the SDK built in, the Orbbec backend is 139 MB unstripped and 9 MB stripped.

`<build>/depth-camera/` *is* the package layout, so anything that works in the
build tree works installed.

The Kinect v2's depth decode is by far the slowest part of that camera. With the
CPU pipeline it delivers about 1.6fps; with OpenCL it reaches 30fps, the sensor's
own rate. The backend picks CUDA, then OpenCL, then CPU at runtime and reports
which it chose. `OpenGLPacketPipeline` is never used: it creates its own GLFW
context and window, which has no business inside score's renderer.

## How it is put together

The score plug-in links **no camera SDK**. Each SDK lives in its own shared
object behind a small C ABI (`DepthCamera/Backend/depthcam_abi.h`), loaded with
`dlopen`/`RTLD_LOCAL` and exporting exactly one symbol.

That is not only for packaging. libfreenect, libfreenect2, k4a, the OrbbecSDK and
librealsense each bundle or link their own libusb, libuvc, libjpeg and logging
library. With default ELF visibility the first definition loaded wins for every
caller, so two SDKs sharing a process silently bind to one another's copies —
and libjpeg in particular bakes struct sizes into its callers, so a mismatch
corrupts memory rather than failing to link. Each backend therefore hides
everything it links (`-fvisibility=hidden` plus `--exclude-libs,ALL` plus a
linker version script) and exports exactly one symbol. CI fails the build if any
backend exports more than one, because that guarantee is what the whole design
rests on: score `dlopen`s the contents of its package directories, on Linux with
`RTLD_GLOBAL`. macOS gets the same guarantee from an `-exported_symbols_list`.

Windows is the exception, and does not need to be one. A PE has no global symbol
namespace — imports bind per module, by DLL name — so a second backend exporting
`freenect_init` cannot be bound to by the first. It is also not achievable there:
libfreenect and librealsense both mark their public API `__declspec(dllexport)`
unconditionally, in headers we do not own, and neither a `.def` file nor a linker
flag removes an export the compiler put in. So on Windows the check is that the
entry point is present, and the total is reported rather than enforced.

The Windows package is built against the **static** CRT. A backend built `/MD`
imports `MSVCP140.dll` and will not load without the Visual C++ redistributable,
which score — built with llvm-mingw — does not bring. A CRT per module is what
the ABI already assumes anyway: nothing but plain C crosses it, and every frame
goes back through the backend's own `release()`. The Oak backend is the one
exception, and has to be: it links a prebuilt DLL that imports `MSVCP140.dll`
and hands `std::vector`s across, so a second CRT would free an allocation
against the wrong heap on the first enumeration. That is why the Oak backend
alone requires the redistributable.

One more consequence of the one-symbol rule is worth knowing before writing a
backend that links a **C++** SDK: a module that exports one symbol exports no
type information either, so its copy of a type's RTTI cannot be merged with the
SDK's. On Linux and Windows `dynamic_cast` survives that, because both compare
`type_info` by name. On x86\_64 macOS libc++ compares it by *address*, on the
assumption that the linker merged every copy — so every `dynamic_cast` across
the boundary silently returns null. The Oak backend is the only one that links a
C++ SDK, and it works around this by going through the SDK's own casts and by
keying frame handling on the frame type rather than on RTTI; see the comments
in `Backends/Oak/OakBackend.cpp`.

Not every SDK can be built the way the others are, and the two most recent
additions each break the rule differently. Oak3DVision publishes no sources at
all, so `3rdparty/oaksdk` is a submodule of *binaries* and the backend links the
one for the platform. depthai-core is built, but as a **nested CMake project**:
it resolves its dependencies through a vcpkg it bootstraps itself, and vcpkg has
to be the toolchain file of the top-level configure, which `add_subdirectory`
cannot reach — its documentation says as much. Both still meet the part that
matters: one shared object, one exported symbol, reached only through
`depthcam_abi.h`.

Writing a backend for another camera means implementing one header. Nothing on
the score side is camera-specific.

The ABI only grows. score accepts backends from ABI 3 (the version-1 package)
to its own, ABI 4 (the version-2 package): it copies a backend's table into one
of its own layout and leaves what an older backend lacks empty, so an ABI 3
package still streams, with an estimated calibration and its controls as they
were. A package newer than score is refused.

## Testing

`Tests/unit`, `Tests/gfx` and `Tests/perf` need no camera: they run against
`Backends/Synthetic`, a backend whose scene, calibration and timestamps are
known in closed form, built with score's tests (`SCORE_TESTING`) and never
packaged. They cover the units and sample types, timestamps, calibration, the
metadata tree, the controls and their persistence, settings from older
documents, ABI 3 backends, and the GPU outputs on every graphics backend
present. `depthcam_perf` reports what a frame costs, CPU path against GPU path.

`Tests/abi` and `Tests/score` drive a real camera, through the ABI directly or
through score's JS API; see [Tests/README.md](Tests/README.md).

Notes on the vendored OrbbecSDK and what upgrading it would involve are in
[Documentation/orbbec-sdk-upgrade.md](Documentation/orbbec-sdk-upgrade.md).

## Licences

This addon is under the same licence as ossia score. The SDKs it builds against
are not:

Two of them are forked to carry memory-safety fixes AddressSanitizer found
here; see `Documentation/sdk-forks.md`.

| Component | Licence | Redistributable |
|---|---|---|
| OrbbecSDK v2 | MIT | yes |
| OrbbecSDK `extensions/` blobs | Orbbec proprietary | **check before shipping** |
| `extensions/depthengine` (Femto Bolt) | Microsoft, redistributed by Orbbec | as the OrbbecSDK does |
| libfreenect / libfreenect2 | Apache-2.0 / GPLv2 dual | yes |
| librealsense | Apache-2.0 | yes |
| Azure Kinect Sensor SDK | MIT | yes |
| `libdepthengine` | Microsoft proprietary | **no — not shipped** |
| Oak3DVision `libPointCloud` | none stated | **check before shipping** |
| depthai-core | MIT | yes |
| MyriadX / RVC4 device firmware | Luxonis, see the LICENSE it ships | as depthai-core does |
