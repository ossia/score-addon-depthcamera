/*
 * Kinect v1 (Xbox 360) backend: libfreenect behind depthcam_abi.h.
 *
 * libfreenect owns its frame buffers and recycles them between callbacks, so
 * unlike the Orbbec and k4a backends this one has to copy. At 640x480 that is
 * ~900KB per colour frame; the alternative (freenect_set_video_buffer with our
 * own pool) would let the device write straight into a buffer the host might
 * still be rendering from.
 *
 * libfreenect is also an event-loop API -- freenect_process_events blocks --
 * so the backend runs its own thread.
 */
#include <depthcam_abi.h>

#include <libfreenect.h>
#include <libfreenect_registration.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{
std::string g_last_error;
std::mutex g_error_mutex;

void set_error(const std::string& e)
{
  std::lock_guard lock{g_error_mutex};
  g_last_error = e;
}

const char* backend_last_error()
{
  std::lock_guard lock{g_error_mutex};
  return g_last_error.empty() ? nullptr : g_last_error.c_str();
}

freenect_context* g_ctx{};

struct Address
{
  enum Kind
  {
    Any,
    Serial,
    Index
  } kind{Any};
  std::string serial;
  int index{0};

  /// "?motor=1": claim the motor subdevice as well. Off by default; see
  /// backend_open for why that is not simply an oversight.
  bool motor{};
};

Address parse_uri(const char* uri)
{
  Address a;
  if(!uri)
    return a;
  std::string s{uri};
  if(s.rfind("freenect:", 0) == 0)
    s = s.substr(9);

  // Options come after a '?', as in a URL: "freenect:sn:A00365...?motor=1".
  if(const auto q = s.find('?'); q != std::string::npos)
  {
    const auto opts = s.substr(q + 1);
    s.resize(q);
    a.motor = opts.find("motor=1") != std::string::npos
              || opts.find("motor=true") != std::string::npos;
  }

  if(s.empty())
    return a;

  if(s.rfind("sn:", 0) == 0)
  {
    a.kind = Address::Serial;
    a.serial = s.substr(3);
  }
  else if(s.rfind("index:", 0) == 0)
  {
    try
    {
      a.index = std::stoi(s.substr(6));
      a.kind = Address::Index;
    }
    catch(...)
    {
    }
  }
  else
  {
    a.kind = Address::Serial;
    a.serial = s;
  }
  if(a.kind == Address::Serial && a.serial.empty())
    a.kind = Address::Any;
  return a;
}

struct EnumEntry
{
  std::string uri, serial;
};

/// Owns a copy of a libfreenect buffer for as long as the host holds it.
struct VectorHolder
{
  std::vector<uint8_t> data;
};

void release_vector_holder(void* owner)
{
  delete static_cast<VectorHolder*>(owner);
}

struct FloatHolder
{
  std::vector<float> data;
};

void release_float_holder(void* owner)
{
  delete static_cast<FloatHolder*>(owner);
}
// --- controls ---------------------------------------------------------------

/**
 * The Kinect v1 has no image controls at all -- exposure, gain and white
 * balance are fixed in firmware -- but it does have the two things no other
 * camera here has: a tilt motor and a coloured LED, plus the accelerometer that
 * comes with the motor board.
 *
 * All four are on the *motor* subdevice, a separate USB interface, and every
 * one of them is a synchronous control transfer. libfreenect has no locking of
 * its own, so none of these may be issued from another thread while
 * freenect_process_events is running: writes are queued for the capture thread
 * and reads come from what that thread last cached. That is also why the
 * accelerometer costs nothing to publish -- the state arrives in one message
 * along with the tilt angle.
 */
const char* const led_labels[]
    = {"off", "green", "red", "yellow", "blink green", "5", "blink red/yellow"};

const char* const tilt_status_labels[] = {"stopped", "limit", "2", "3", "moving"};

struct ControlDef
{
  const char* id;
  const char* name;
  const char* description;
  int kind;
  int access;
  double min, max, step, def;
  const char* const* labels;
  int label_count;
  uint32_t flags;
};

constexpr int no_labels = 0;

const ControlDef freenect_controls[] = {
    // Volatile: what is read back is where the motor is, which lags a write
    // and moves on its own when the camera is knocked.
    {"motor/tilt", "tilt", "Tilt angle in degrees from the horizon",
     DEPTHCAM_CONTROL_INT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, -30, 30, 1,
     0, nullptr, no_labels, DEPTHCAM_CONTROL_VOLATILE},
    {"motor/led", "LED", nullptr, DEPTHCAM_CONTROL_ENUM,
     DEPTHCAM_ACCESS_WRITE, 0, 6, 1, 1, led_labels, 7, 0},
    {"sensors/tilt_status", "tilt status", nullptr, DEPTHCAM_CONTROL_ENUM,
     DEPTHCAM_ACCESS_READ, 0, 4, 1, 0, tilt_status_labels, 5,
     DEPTHCAM_CONTROL_VOLATILE},
    {"sensors/accel_x", "acceleration X", "In g", DEPTHCAM_CONTROL_FLOAT,
     DEPTHCAM_ACCESS_READ, -4, 4, 0, 0, nullptr, no_labels,
     DEPTHCAM_CONTROL_VOLATILE},
    {"sensors/accel_y", "acceleration Y", "In g", DEPTHCAM_CONTROL_FLOAT,
     DEPTHCAM_ACCESS_READ, -4, 4, 0, 0, nullptr, no_labels,
     DEPTHCAM_CONTROL_VOLATILE},
    {"sensors/accel_z", "acceleration Z", "In g", DEPTHCAM_CONTROL_FLOAT,
     DEPTHCAM_ACCESS_READ, -4, 4, 0, 0, nullptr, no_labels,
     DEPTHCAM_CONTROL_VOLATILE},
};

depthcam_control base_of(const ControlDef& d)
{
  depthcam_control c{};
  c.id = d.id;
  c.name = d.name;
  c.description = d.description;
  c.kind = d.kind;
  c.access = d.access;
  c.min = d.min;
  c.max = d.max;
  c.step = d.step;
  c.def = d.def;
  c.enum_labels = d.labels;
  c.enum_count = d.label_count;
  return c;
}

} // namespace

// ---------------------------------------------------------------------------

struct depthcam_device
{
  freenect_device* dev{};

  /// Whether the host asked for inertial data. Only the accelerometer is
  /// available, and only through the motor board.
  bool want_imu{};

  /// True when the motor subdevice came up. A Kinect for Windows, or a Kinect
  /// whose motor interface is claimed by something else, has cameras but no
  /// motor, and publishing controls that can never work is worse than none.
  bool has_motor{};

  // Read by get_control, written by the capture thread. See the note above
  // freenect_controls: libfreenect is not safe to touch from two threads.
  std::atomic<double> tilt_deg{0.};
  std::atomic<int> tilt_status{0};
  std::atomic<double> accel[3]{};

  /// Pending writes, applied by the capture thread. INT_MIN / -1 mean "nothing
  /// asked for".
  std::atomic<int> want_tilt{INT_MIN};
  std::atomic<int> want_led{-1};


  depthcam_open_config cfg{};
  bool color_pointcloud{};
  /// Depth delivered as FREENECT_DEPTH_REGISTERED, i.e. in the colour image.
  bool registered{};

  /// Which opened streams the host consumes; see backend_set_stream_demand.
  std::atomic<uint32_t> demand{~0u};

  /// The camera's zero-plane calibration, read once at open; see
  /// backend_get_intrinsics.
  freenect_zero_plane_info zero_plane{};
  bool have_zero_plane{};

  std::string serial;

  std::atomic_bool running{};
  std::thread thread;

  depthcam_frame_cb on_frame{};
  void* user{};

  int color_w{640}, color_h{480};
  int depth_w{640}, depth_h{480};

  /// The cloud needs colour and depth together; the callbacks are separate.
  std::mutex pair_mutex;
  std::vector<uint8_t> last_color;
  std::vector<uint16_t> last_depth;
  bool have_color{}, have_depth{};

  std::vector<float> cloud_buf;

  ~depthcam_device();

  void onVideo(void* data, uint32_t timestamp);
  void onDepth(void* data, uint32_t timestamp);
  void emitPointCloud();
  void run();
};

namespace
{
void video_cb(freenect_device* dev, void* data, uint32_t timestamp)
{
  if(auto* self = static_cast<depthcam_device*>(freenect_get_user(dev)))
    self->onVideo(data, timestamp);
}

void depth_cb(freenect_device* dev, void* data, uint32_t timestamp)
{
  if(auto* self = static_cast<depthcam_device*>(freenect_get_user(dev)))
    self->onDepth(data, timestamp);
}
} // namespace

depthcam_device::~depthcam_device()
{
  if(dev)
  {
    freenect_stop_video(dev);
    freenect_stop_depth(dev);
    freenect_close_device(dev);
  }
}

void depthcam_device::onVideo(void* data, uint32_t timestamp)
{
  if(!running.load(std::memory_order_acquire) || !data)
    return;

  const size_t bytes = size_t(color_w) * color_h * 3; // FREENECT_VIDEO_RGB
  const uint32_t wanted = demand.load(std::memory_order_relaxed);

  const bool want_cloud = (cfg.streams & DEPTHCAM_STREAM_POINTCLOUD)
                          && (wanted & DEPTHCAM_STREAM_POINTCLOUD);
  if(want_cloud && color_pointcloud)
  {
    std::lock_guard lock{pair_mutex};
    last_color.assign(
        static_cast<uint8_t*>(data), static_cast<uint8_t*>(data) + bytes);
    have_color = true;
  }

  if((cfg.streams & DEPTHCAM_STREAM_COLOR) && (wanted & DEPTHCAM_STREAM_COLOR)
     && on_frame)
  {
    auto* holder = new VectorHolder{};
    holder->data.assign(
        static_cast<uint8_t*>(data), static_cast<uint8_t*>(data) + bytes);

    depthcam_frame out{};
    out.stream = DEPTHCAM_STREAM_COLOR;
    out.format = DEPTHCAM_FMT_RGB24;
    out.width = color_w;
    out.height = color_h;
    out.stride = color_w * 3;
    out.timestamp_ns = uint64_t(timestamp) * 1000ull;
    out.data = holder->data.data();
    out.bytes = holder->data.size();
    out.owner = holder;
    out.release = &release_vector_holder;
    on_frame(&out, user);
  }
}

void depthcam_device::onDepth(void* data, uint32_t timestamp)
{
  if(!running.load(std::memory_order_acquire) || !data)
    return;

  const size_t count = size_t(depth_w) * depth_h;
  const auto* d = static_cast<const uint16_t*>(data);
  const uint32_t wanted = demand.load(std::memory_order_relaxed);

  // The per-pixel freenect_camera_to_world loop is the costly part of this
  // backend; a cloud nobody reads is neither copied for nor computed.
  const bool want_cloud = (cfg.streams & DEPTHCAM_STREAM_POINTCLOUD)
                          && (wanted & DEPTHCAM_STREAM_POINTCLOUD);
  if(want_cloud)
  {
    std::lock_guard lock{pair_mutex};
    last_depth.assign(d, d + count);
    have_depth = true;
  }

  if((cfg.streams & DEPTHCAM_STREAM_DEPTH) && (wanted & DEPTHCAM_STREAM_DEPTH)
     && on_frame)
  {
    auto* holder = new VectorHolder{};
    holder->data.resize(count * sizeof(uint16_t));
    std::memcpy(holder->data.data(), d, holder->data.size());

    depthcam_frame out{};
    out.stream = DEPTHCAM_STREAM_DEPTH;
    out.format = DEPTHCAM_FMT_GRAY16;
    out.width = depth_w;
    out.height = depth_h;
    out.stride = depth_w * int(sizeof(uint16_t));
    out.timestamp_ns = uint64_t(timestamp) * 1000ull;
    // FREENECT_DEPTH_MM / _REGISTERED are already millimetres.
    out.depth_unit_mm = 1.f;
    out.data = holder->data.data();
    out.bytes = holder->data.size();
    out.owner = holder;
    out.release = &release_vector_holder;
    on_frame(&out, user);
  }

  if(want_cloud)
    emitPointCloud();
}

void depthcam_device::emitPointCloud()
{
  if(!on_frame || !dev)
    return;

  std::lock_guard lock{pair_mutex};
  if(!have_depth)
    return;
  if(color_pointcloud && !have_color)
    return;

  const int stride = color_pointcloud ? 6 : 3;
  cloud_buf.clear();
  cloud_buf.reserve(size_t(depth_w) * depth_h * stride);

  for(int y = 0; y < depth_h; y++)
  {
    for(int x = 0; x < depth_w; x++)
    {
      const int i = y * depth_w + x;
      const uint16_t z = last_depth[i];
      // 0 and the 11-bit saturation value both mean "no reading".
      if(z == 0 || z == 2047 || z >= 10000)
        continue;

      double wx{}, wy{};
      freenect_camera_to_world(dev, x, y, z, &wx, &wy);

      // freenect_camera_to_world and DEPTH_MM are both millimetres; the ABI
      // wants metres.
      constexpr float mm_to_m = 0.001f;
      cloud_buf.push_back(float(wx) * mm_to_m);
      cloud_buf.push_back(float(wy) * mm_to_m);
      cloud_buf.push_back(float(z) * mm_to_m);

      if(color_pointcloud)
      {
        // Depth is registered to the colour camera, so the indices line up.
        const uint8_t* p = last_color.data() + size_t(i) * 3;
        cloud_buf.push_back(p[0] / 255.f);
        cloud_buf.push_back(p[1] / 255.f);
        cloud_buf.push_back(p[2] / 255.f);
      }
    }
  }

  if(cloud_buf.empty())
    return;

  auto* holder = new FloatHolder{cloud_buf};

  depthcam_frame out{};
  out.stream = DEPTHCAM_STREAM_POINTCLOUD;
  out.format = color_pointcloud ? DEPTHCAM_FMT_XYZRGB : DEPTHCAM_FMT_XYZ;
  out.point_count = int32_t(holder->data.size() / stride);
  out.data = holder->data.data();
  out.bytes = holder->data.size() * sizeof(float);
  out.owner = holder;
  out.release = &release_float_holder;

  on_frame(&out, user);
}

void depthcam_device::run()
{
  int since_tilt = 0;
  while(running.load(std::memory_order_acquire))
  {
    // Returns after handling pending USB events or the 100ms timeout, so the
    // running flag is checked regularly.
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    if(freenect_process_events_timeout(g_ctx, &tv) < 0)
      break;

    if(!has_motor)
      continue;

    // Everything below is a synchronous control transfer, and this is the only
    // thread allowed to issue one.
    if(const int t = want_tilt.exchange(INT_MIN); t != INT_MIN)
      freenect_set_tilt_degs(dev, double(t));
    if(const int l = want_led.exchange(-1); l >= 0)
      freenect_set_led(dev, freenect_led_options(l));

    // Twice a second: the state is a round trip over USB and nothing here
    // moves fast enough to want more.
    if(++since_tilt < 5)
      continue;
    since_tilt = 0;

    if(freenect_update_tilt_state(dev) < 0)
      continue;
    auto* state = freenect_get_tilt_state(dev);
    if(!state)
      continue;

    tilt_deg.store(freenect_get_tilt_degs(state), std::memory_order_relaxed);
    tilt_status.store(int(freenect_get_tilt_status(state)), std::memory_order_relaxed);

    double x{}, y{}, z{};
    freenect_get_mks_accel(state, &x, &y, &z);
    // "mks" is metre-kilogram-second: these are already m/s^2, which is what
    // the ABI wants. The sensors/accel_* controls publish g instead, because a
    // number a user reads off a tree is friendlier in gravities.
    accel[0].store(x / 9.80665, std::memory_order_relaxed);
    accel[1].store(y / 9.80665, std::memory_order_relaxed);
    accel[2].store(z / 9.80665, std::memory_order_relaxed);

    if(want_imu && on_frame)
    {
      // 2Hz, which is all this is: reading the state is a synchronous control
      // transfer on the motor board and there is no gyroscope to pair it with.
      // Enough to know which way up the camera is, not enough to track motion.
      depthcam_imu_sample sample{};
      sample.fields = DEPTHCAM_IMU_ACCEL;
      sample.accel[0] = float(x);
      sample.accel[1] = float(y);
      sample.accel[2] = float(z);

      depthcam_frame out{};
      out.stream = DEPTHCAM_STREAM_IMU;
      out.format = DEPTHCAM_FMT_IMU;
      out.data = &sample;
      out.bytes = sizeof(sample);
      on_frame(&out, user);
    }
  }
}

// ---------------------------------------------------------------------------

namespace
{

int backend_init(const char*)
{
  if(g_ctx)
    return 1;

  if(freenect_init(&g_ctx, nullptr) < 0)
  {
    set_error("freenect_init failed");
    g_ctx = nullptr;
    return 0;
  }

  // libfreenect logs at Notice on stdout by default.
  freenect_set_log_level(g_ctx, FREENECT_LOG_WARNING);

  // Which subdevices to claim is decided per-open, not here: see backend_open.
  // Audio is never asked for -- it makes open fail on a Kinect whose audio
  // firmware has not been uploaded, and nothing here uses it.
  freenect_select_subdevices(g_ctx, FREENECT_DEVICE_CAMERA);
  return 1;
}

void backend_shutdown()
{
  if(g_ctx)
  {
    freenect_shutdown(g_ctx);
    g_ctx = nullptr;
  }
}

/**
 * @brief Is this serial actually an identity?
 *
 * A model 1473 and a Kinect for Windows have no camera serial: the descriptor
 * reads "0000000000000000", and libfreenect substitutes the *audio* device's
 * serial when it can reach it. It cannot always -- listing the devices resets
 * the audio interface on those models (fnusb_keep_alive_led), so the very same
 * camera enumerates as "A70775V03010249A" once and as all zeros a moment
 * later. Anything stored in a document from the first listing then fails to
 * open, which is exactly what "the camera vanished after I closed it" looked
 * like.
 */
bool usable_serial(const std::string& s)
{
  return !s.empty()
         && s.find_first_not_of('0') != std::string::npos;
}

int backend_enumerate(depthcam_enumerate_cb cb, void* user)
{
  if(!g_ctx || !cb)
    return 0;

  freenect_device_attributes* attrs{};
  const int n = freenect_list_device_attributes(g_ctx, &attrs);
  if(n < 0)
    return 0;

  int i = 0;
  for(auto* a = attrs; a; a = a->next, i++)
  {
    EnumEntry e;
    e.serial = a->camera_serial ? a->camera_serial : "";
    e.uri = usable_serial(e.serial) ? ("freenect:sn:" + e.serial)
                                    : ("freenect:index:" + std::to_string(i));

    depthcam_device_info info{};
    info.backend = "freenect";
    info.uri = e.uri.c_str();
    info.name = "Kinect v1";
    info.serial = e.serial.c_str();
    info.transport = "usb2";
    // No infrared: libfreenect can stream IR, but only instead of colour, not
    // alongside it, and the ABI has no way to express that exclusivity.
    // IMU only with ?motor=1: the accelerometer is on the motor board.
    info.streams = DEPTHCAM_STREAM_COLOR | DEPTHCAM_STREAM_DEPTH
                   | DEPTHCAM_STREAM_POINTCLOUD | DEPTHCAM_STREAM_IMU;
    cb(&info, user);
  }

  if(attrs)
    freenect_free_device_attributes(attrs);
  return 1;
}

depthcam_device* backend_open(const char* uri, const depthcam_open_config* config)
{
  if(!g_ctx || !config)
    return nullptr;

  const auto addr = parse_uri(uri);

  auto dev = std::make_unique<depthcam_device>();
  dev->cfg = *config;
  dev->color_pointcloud = (config->streams & DEPTHCAM_STREAM_POINTCLOUD)
                          && config->color_pointcloud != 0;
  dev->want_imu = (config->streams & DEPTHCAM_STREAM_IMU) != 0;

  // Resolved here rather than left to freenect_open_device_by_camera_serial,
  // because on the models that have no camera serial that call is a coin toss;
  // see usable_serial.
  int index = addr.kind == Address::Index ? addr.index : 0;
  if(addr.kind == Address::Serial)
  {
    freenect_device_attributes* attrs{};
    const int n = freenect_list_device_attributes(g_ctx, &attrs);

    int found = -1, unidentified = -1, unidentified_count = 0, i = 0;
    for(auto* a = attrs; a; a = a->next, i++)
    {
      const std::string sn = a->camera_serial ? a->camera_serial : "";
      if(sn == addr.serial)
        found = i;
      else if(!usable_serial(sn))
      {
        unidentified = i;
        unidentified_count++;
      }
    }
    if(attrs)
      freenect_free_device_attributes(attrs);

    if(found >= 0)
    {
      index = found;
      dev->serial = addr.serial;
    }
    else if(unidentified_count == 1 && n == 1)
    {
      // The one camera present cannot say who it is, and there is nothing else
      // it could be confused with. Matching it is the only way an address saved
      // from an earlier session keeps working on a 1473.
      index = unidentified;
    }
    else if(n <= 0)
    {
      // Not "not connected": a Kinect whose camera interface is already claimed
      // -- by another device in this document, or by another process -- drops
      // out of the listing entirely.
      set_error("no Kinect v1 available (already in use?)");
      return nullptr;
    }
    else
    {
      set_error("Kinect v1 '" + addr.serial + "' is not connected");
      return nullptr;
    }
  }

  const auto open_with = [&](freenect_device_flags flags) {
    freenect_select_subdevices(g_ctx, flags);
    return freenect_open_device(g_ctx, &dev->dev, index);
  };

  // The motor is opt-in, and deliberately so.
  //
  // It carries the tilt, the LED and the accelerometer, and on the original
  // model 1414 it is a plain second USB interface that costs nothing to claim.
  // On a 1473 or a Kinect for Windows it is not: libfreenect drops the motor
  // flag for those and enables the *audio* interface instead, because that is
  // where the newer motor lives (usb_libusb10.c, fnusb_open_subdevices). Audio
  // needs a firmware upload we do not ship, so the open then fails outright --
  // and worse, the attempt resets the audio device, which takes the camera down
  // with it for several seconds. A camera that streams is worth more than a
  // tilt motor, so nothing is risked unless it was asked for.
  int rc = open_with(
      addr.motor ? freenect_device_flags(
                       FREENECT_DEVICE_CAMERA | FREENECT_DEVICE_MOTOR)
                 : FREENECT_DEVICE_CAMERA);

  if(rc < 0 || !dev->dev)
  {
    if(addr.kind == Address::Serial)
      set_error(
          "Kinect v1 '" + addr.serial + "' is not connected"
          + (addr.motor ? " (or its motor could not be claimed: a model 1473 "
                          "and a Kinect for Windows drive the motor through the "
                          "audio interface, which needs firmware this build does "
                          "not ship -- drop ?motor=1 from the address)"
                        : ""));
    else
      set_error("could not open the Kinect v1");
    return nullptr;
  }

  freenect_set_user(dev->dev, dev.get());

  // freenect_select_subdevices is a request; libfreenect drops what it could
  // not bring up, so this is the only way to know whether the motor is really
  // there.
  dev->has_motor
      = (freenect_enabled_subdevices(g_ctx) & FREENECT_DEVICE_MOTOR) != 0;

  const auto video = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM,
                                              FREENECT_VIDEO_RGB);
  if(freenect_set_video_mode(dev->dev, video) < 0)
  {
    set_error("could not set the Kinect v1 video mode");
    return nullptr;
  }
  dev->color_w = video.width;
  dev->color_h = video.height;

  // REGISTERED maps depth onto the colour camera, which is what makes a
  // coloured point cloud possible at all here -- libfreenect has no
  // colour-to-depth direction, so both alignment modes use it and only the
  // uncoloured/unaligned case takes raw millimetres.
  const bool want_registered
      = config->align != DEPTHCAM_ALIGN_NONE || dev->color_pointcloud;
  const auto depth = freenect_find_depth_mode(
      FREENECT_RESOLUTION_MEDIUM,
      want_registered ? FREENECT_DEPTH_REGISTERED : FREENECT_DEPTH_MM);
  if(freenect_set_depth_mode(dev->dev, depth) < 0)
  {
    set_error("could not set the Kinect v1 depth mode");
    return nullptr;
  }
  dev->depth_w = depth.width;
  dev->depth_h = depth.height;
  dev->registered = want_registered;

  // Fetched from the camera by freenect_camera_init during open
  // (src/cameras.c:1260), so it is valid here. The copy also builds three
  // lookup tables that are of no use to us, hence freed straight away.
  {
    freenect_registration reg = freenect_copy_registration(dev->dev);
    dev->zero_plane = reg.zero_plane_info;
    freenect_destroy_registration(&reg);
    dev->have_zero_plane = dev->zero_plane.reference_distance > 0.f
                           && dev->zero_plane.reference_pixel_size > 0.f;
  }

  freenect_set_video_callback(dev->dev, &video_cb);
  freenect_set_depth_callback(dev->dev, &depth_cb);

  return dev.release();
}

void backend_stop(depthcam_device* dev);

void backend_close(depthcam_device* dev)
{
  if(!dev)
    return;
  backend_stop(dev);
  delete dev;
}

int backend_start(depthcam_device* dev, depthcam_frame_cb on_frame, void* user)
{
  if(!dev || !dev->dev)
    return 0;
  if(dev->running.load(std::memory_order_acquire))
    return 1;

  dev->on_frame = on_frame;
  dev->user = user;
  dev->running.store(true, std::memory_order_release);

  const bool want_color = (dev->cfg.streams & DEPTHCAM_STREAM_COLOR)
                          || dev->color_pointcloud;
  const bool want_depth = (dev->cfg.streams & DEPTHCAM_STREAM_DEPTH)
                          || (dev->cfg.streams & DEPTHCAM_STREAM_POINTCLOUD);

  if(want_color && freenect_start_video(dev->dev) < 0)
  {
    set_error("could not start the Kinect v1 video stream");
    dev->running.store(false, std::memory_order_release);
    return 0;
  }
  if(want_depth && freenect_start_depth(dev->dev) < 0)
  {
    set_error("could not start the Kinect v1 depth stream");
    dev->running.store(false, std::memory_order_release);
    return 0;
  }

  dev->thread = std::thread{[dev] { dev->run(); }};
  return 1;
}

void backend_stop(depthcam_device* dev)
{
  if(!dev)
    return;
  if(!dev->running.exchange(false, std::memory_order_acq_rel))
    return;
  if(dev->thread.joinable())
    dev->thread.join();
  if(dev->dev)
  {
    freenect_stop_video(dev->dev);
    freenect_stop_depth(dev->dev);
  }
}

int backend_list_controls(depthcam_device* dev, depthcam_control_cb cb, void* user)
{
  if(!dev || !cb || !dev->has_motor)
    return 0;

  for(const auto& d : freenect_controls)
  {
    const depthcam_control c = base_of(d);
    cb(&c, user);
  }
  return 1;
}

int backend_list_controls2(depthcam_device* dev, depthcam_control2_cb cb, void* user)
{
  if(!dev || !cb || !dev->has_motor)
    return 0;

  for(const auto& d : freenect_controls)
  {
    depthcam_control2 c{};
    c.struct_size = sizeof(c);
    c.base = base_of(d);
    // None of the ABI's units fit: these are degrees and g.
    c.unit = nullptr;
    c.flags = d.flags;
    cb(&c, user);
  }
  return 1;
}

int backend_get_control(depthcam_device* dev, const char* id, double* out)
{
  if(!dev || !id || !out || !dev->has_motor)
    return 0;

  const std::string s{id};
  if(s == "motor/tilt")
    *out = dev->tilt_deg.load(std::memory_order_relaxed);
  else if(s == "sensors/tilt_status")
    *out = dev->tilt_status.load(std::memory_order_relaxed);
  else if(s == "sensors/accel_x")
    *out = dev->accel[0].load(std::memory_order_relaxed);
  else if(s == "sensors/accel_y")
    *out = dev->accel[1].load(std::memory_order_relaxed);
  else if(s == "sensors/accel_z")
    *out = dev->accel[2].load(std::memory_order_relaxed);
  else
    return 0;
  return 1;
}

int backend_set_control(depthcam_device* dev, const char* id, double value)
{
  if(!dev || !id || !dev->has_motor)
    return 0;

  const std::string s{id};
  if(s == "motor/tilt")
  {
    const int v = int(value < -30 ? -30 : (value > 30 ? 30 : value));
    dev->want_tilt.store(v, std::memory_order_relaxed);
    return 1;
  }
  if(s == "motor/led")
  {
    const int v = int(value < 0 ? 0 : (value > 6 ? 6 : value));
    dev->want_led.store(v, std::memory_order_relaxed);
    return 1;
  }
  return 0;
}

uint32_t backend_active_streams(depthcam_device* dev)
{
  if(!dev)
    return 0;
  uint32_t streams = dev->cfg.streams;
  // The accelerometer is on the motor board, which is opt-in and which a 1473
  // will not give us at all -- see backend_open.
  if(!dev->has_motor)
    streams &= ~uint32_t(DEPTHCAM_STREAM_IMU);
  return streams;
}

// --- ABI 4 -------------------------------------------------------------------

/**
 * Nominal colour-camera field of view, from the OpenNI2 driver that ships with
 * libfreenect (OpenNI2-FreenectDriver/src/ColorStream.hpp:17-19, "from NUI
 * library"). libfreenect itself has no colour intrinsics at all: neither a
 * calibration read from the camera nor a constant.
 */
constexpr double color_hfov_deg = 62.0;
constexpr double color_vfov_deg = 48.6;

void fill_color_nominal(const depthcam_device& d, depthcam_intrinsics& r)
{
  constexpr double deg = 3.14159265358979323846 / 180.;
  r.width = d.color_w;
  r.height = d.color_h;
  r.fx = float((d.color_w / 2.) / std::tan(color_hfov_deg * deg / 2.));
  r.fy = float((d.color_h / 2.) / std::tan(color_vfov_deg * deg / 2.));
  // The optical axis on the image centre, in the ABI's pixel-centre
  // convention.
  r.cx = float((d.color_w - 1) / 2.);
  r.cy = float((d.color_h - 1) / 2.);
  r.model = DEPTHCAM_DISTORTION_NONE;
}

int backend_get_intrinsics(depthcam_device* dev, uint32_t stream, depthcam_intrinsics* out)
{
  if(!dev || !out)
    return 0;

  depthcam_intrinsics r{};
  r.struct_size = out->struct_size;

  switch(stream)
  {
    case DEPTHCAM_STREAM_DEPTH:
    case DEPTHCAM_STREAM_POINTCLOUD:
      if(dev->registered)
      {
        // FREENECT_DEPTH_REGISTERED is depth resampled into the colour image
        // (libfreenect_registration.h:123-124 calls it the "depth -> RGB"
        // mapping), so it has the colour camera's geometry.
        fill_color_nominal(*dev, r);
      }
      else
      {
        if(!dev->have_zero_plane)
          return 0;
        // The model libfreenect itself deprojects with,
        // freenect_camera_to_world (src/registration.c:341-353):
        //   x = (u - 320) * 2 * pixel_size * z / reference_distance
        // with reference_distance "the focal length of the IR camera, in mm"
        // and pixel_size "the size of a single pixel on the zero plane, in mm"
        // (libfreenect_registration.h:94-95), and the factor 2 because those
        // are for the 1280x1024 sensor of which 640x480 is a 2:1 downscale.
        // So f = reference_distance / (2 pixel_size), the same on both axes,
        // with the principal point at the image centre and integer pixel
        // indices as centres -- the ABI's convention as is. No distortion
        // model exists in libfreenect.
        const auto& z = dev->zero_plane;
        const float f = z.reference_distance / (2.f * z.reference_pixel_size);
        r.width = dev->depth_w;
        r.height = dev->depth_h;
        r.fx = f;
        r.fy = f;
        r.cx = float(dev->depth_w / 2);
        r.cy = float(dev->depth_h / 2);
        r.model = DEPTHCAM_DISTORTION_NONE;
      }
      break;
    case DEPTHCAM_STREAM_COLOR:
      fill_color_nominal(*dev, r);
      break;
    default:
      return 0;
  }

  std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
  return 1;
}

int backend_get_extrinsics(
    depthcam_device* dev, uint32_t from, uint32_t to, depthcam_extrinsics* out)
{
  if(!dev || !out)
    return 0;

  const auto is_depth = [](uint32_t s) {
    return s == DEPTHCAM_STREAM_DEPTH || s == DEPTHCAM_STREAM_POINTCLOUD;
  };

  float tx = 0.f;
  if(from == to || (is_depth(from) && is_depth(to)))
    ;
  else if(dev->registered
          && ((is_depth(from) && to == DEPTHCAM_STREAM_COLOR)
              || (from == DEPTHCAM_STREAM_COLOR && is_depth(to))))
    ; // registered depth is already in the colour camera's frame
  else if(is_depth(from) || is_depth(to))
  {
    if(!dev->have_zero_plane || !(from == DEPTHCAM_STREAM_COLOR || to == DEPTHCAM_STREAM_COLOR))
      return 0;
    // The two cameras sit side by side on one bar, and libfreenect's own
    // registration treats them as a pure horizontal translation of
    // dcmos_rcmos_dist, "distance between IR camera and RGB camera, in cm"
    // (libfreenect_registration.h:93). The sign is the one
    // freenect_init_depth_to_rgb uses (src/registration.c:59-73): the RGB
    // column of a point is its IR column plus B (1 - ref / z), whose
    // depth-dependent part -B ref / z is the parallax f tx / z of a camera
    // displaced by tx = -dcmos_rcmos_dist. So p_color = p_depth + (tx, 0, 0).
    // Rotation is taken as identity, as libfreenect does.
    const float d = -dev->zero_plane.dcmos_rcmos_dist * 0.01f;
    tx = is_depth(from) ? d : -d;
  }
  else
    return 0;

  depthcam_extrinsics r{};
  r.struct_size = out->struct_size;
  r.rotation[0] = r.rotation[4] = r.rotation[8] = 1.f;
  r.translation[0] = tx;
  std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
  return 1;
}

int backend_get_details(depthcam_device* dev, depthcam_device_details* out)
{
  if(!dev || !out)
    return 0;

  depthcam_device_details r{};
  r.struct_size = out->struct_size;
  r.model = "Kinect v1";
  // Known only when the camera was opened by a serial that the listing
  // confirmed: listing again here would reset the audio interface on a 1473
  // (see usable_serial).
  r.serial = dev->serial.c_str();
  // libfreenect reads no version or revision from the camera.
  r.firmware = "";
  r.hardware_rev = "";
  // 0 to FREENECT_DEPTH_MM_MAX_VALUE (libfreenect.h:45) is only the encoding;
  // the measuring range is documented nowhere in libfreenect.
  r.depth_min_m = 0.f;
  r.depth_max_m = 0.f;
  std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
  return 1;
}

int backend_list_profiles(const char*, uint32_t stream, depthcam_profile_cb cb, void* user)
{
  if(!cb)
    return 0;

  depthcam_profile p{};
  p.struct_size = sizeof(p);

  // What backend_open sets, which is the medium-resolution RGB and depth mode
  // of libfreenect's own mode table (src/cameras.c:45,71-72): 640x480 at 30
  // fps. The 1280x1024 colour mode exists too, but only at 10 fps and not
  // alongside depth on every model, and this backend does not offer it.
  switch(stream)
  {
    case DEPTHCAM_STREAM_COLOR:
      p.width = 640;
      p.height = 480;
      p.fps = 30;
      p.format = DEPTHCAM_FMT_RGB24;
      cb(&p, user);
      return 1;
    case DEPTHCAM_STREAM_DEPTH:
      p.width = 640;
      p.height = 480;
      p.fps = 30;
      p.format = DEPTHCAM_FMT_GRAY16;
      cb(&p, user);
      return 1;
    default:
      return 0;
  }
}

void backend_set_stream_demand(depthcam_device* dev, uint32_t streams)
{
  if(dev)
    dev->demand.store(streams, std::memory_order_relaxed);
}

const depthcam_backend_v1 g_backend{
    .abi_version = DEPTHCAM_ABI_VERSION,
    .name = "freenect",
    .display_name = "Kinect v1",
    .last_error = &backend_last_error,
    .init = &backend_init,
    .shutdown = &backend_shutdown,
    .enumerate = &backend_enumerate,
    // The SDK has no hot-plug notification.
    .set_changed_callback = nullptr,
    .open = &backend_open,
    .close = &backend_close,
    .start = &backend_start,
    .stop = &backend_stop,
    .list_controls = &backend_list_controls,
    .get_control = &backend_get_control,
    .set_control = &backend_set_control,
    .active_streams = &backend_active_streams,
    .get_intrinsics = &backend_get_intrinsics,
    .get_extrinsics = &backend_get_extrinsics,
    .get_details = &backend_get_details,
    .get_ray_table = nullptr,
    .list_controls2 = &backend_list_controls2,
    .set_control_changed_callback = nullptr,
    .list_profiles = &backend_list_profiles,
    .set_stream_demand = &backend_set_stream_demand,
};

} // namespace

extern "C" DEPTHCAM_EXPORT const depthcam_backend_v1* score_depthcam_backend_v1(void)
{
  return &g_backend;
}
