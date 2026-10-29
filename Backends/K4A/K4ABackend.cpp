/*
 * Azure Kinect DK backend: libk4a behind depthcam_abi.h.
 *
 * Unlike the other backends this one does not link its SDK. libk4a exposes a
 * flat C API, so it is resolved with dlopen at runtime, which means:
 *   - this target builds anywhere, CI included, with no SDK present;
 *   - the user installs the SDK through their own package manager;
 *   - we never redistribute libdepthengine, a closed Microsoft binary that any
 *     depth output requires and that we have no licence to ship.
 * A missing SDK is reported as "no cameras", not as a load failure.
 *
 * k4a is also a *pull* API -- k4a_device_get_capture blocks -- so unlike Orbbec
 * and libfreenect2 this backend runs its own capture thread.
 */
#include <depthcam_abi.h>

#include <k4a/k4a.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

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

/// The subset of libk4a this backend uses, resolved at runtime.
struct K4AApi
{
  void* lib{};

  uint32_t (*device_get_installed_count)();
  k4a_result_t (*device_open)(uint32_t, k4a_device_t*);
  void (*device_close)(k4a_device_t);
  k4a_buffer_result_t (*device_get_serialnum)(k4a_device_t, char*, size_t*);
  k4a_result_t (*device_start_cameras)(k4a_device_t, const k4a_device_configuration_t*);
  void (*device_stop_cameras)(k4a_device_t);
  k4a_wait_result_t (*device_get_capture)(k4a_device_t, k4a_capture_t*, int32_t);
  k4a_result_t (*device_start_imu)(k4a_device_t);
  void (*device_stop_imu)(k4a_device_t);
  k4a_wait_result_t (*device_get_imu_sample)(k4a_device_t, k4a_imu_sample_t*, int32_t);
  k4a_result_t (*device_get_calibration)(
      k4a_device_t, k4a_depth_mode_t, k4a_color_resolution_t, k4a_calibration_t*);
  k4a_result_t (*device_get_version)(k4a_device_t, k4a_hardware_version_t*);
  k4a_result_t (*calibration_2d_to_3d)(
      const k4a_calibration_t*, const k4a_float2_t*, const float,
      const k4a_calibration_type_t, const k4a_calibration_type_t, k4a_float3_t*, int*);

  k4a_result_t (*device_get_color_control_capabilities)(
      k4a_device_t, k4a_color_control_command_t, bool*, int32_t*, int32_t*, int32_t*,
      int32_t*, k4a_color_control_mode_t*);
  k4a_result_t (*device_get_color_control)(
      k4a_device_t, k4a_color_control_command_t, k4a_color_control_mode_t*, int32_t*);
  k4a_result_t (*device_set_color_control)(
      k4a_device_t, k4a_color_control_command_t, k4a_color_control_mode_t, int32_t);

  void (*capture_release)(k4a_capture_t);
  float (*capture_get_temperature_c)(k4a_capture_t);
  k4a_image_t (*capture_get_color_image)(k4a_capture_t);
  k4a_image_t (*capture_get_depth_image)(k4a_capture_t);
  k4a_image_t (*capture_get_ir_image)(k4a_capture_t);

  void (*image_release)(k4a_image_t);
  void (*image_reference)(k4a_image_t);
  uint8_t* (*image_get_buffer)(k4a_image_t);
  size_t (*image_get_size)(k4a_image_t);
  int (*image_get_width_pixels)(k4a_image_t);
  int (*image_get_height_pixels)(k4a_image_t);
  int (*image_get_stride_bytes)(k4a_image_t);
  k4a_image_format_t (*image_get_format)(k4a_image_t);
  uint64_t (*image_get_device_timestamp_usec)(k4a_image_t);
  k4a_result_t (*image_create)(k4a_image_format_t, int, int, int, k4a_image_t*);

  k4a_transformation_t (*transformation_create)(const k4a_calibration_t*);
  void (*transformation_destroy)(k4a_transformation_t);
  k4a_result_t (*transformation_depth_image_to_point_cloud)(
      k4a_transformation_t, const k4a_image_t, const k4a_calibration_type_t,
      k4a_image_t);
  k4a_result_t (*transformation_color_image_to_depth_camera)(
      k4a_transformation_t, const k4a_image_t, const k4a_image_t, k4a_image_t);
  k4a_result_t (*transformation_depth_image_to_color_camera)(
      k4a_transformation_t, const k4a_image_t, k4a_image_t);

  explicit operator bool() const noexcept { return lib != nullptr; }
};

K4AApi g_k4a;

void* load_symbol(void* lib, const char* name)
{
#if defined(_WIN32)
  return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(lib), name));
#else
  return dlsym(lib, name);
#endif
}

bool load_k4a(K4AApi& api, const char* resource_dir)
{
  // SONAME first, then the unversioned development symlink.
#if defined(_WIN32)
  const char* names[] = {"k4a.dll"};
#elif defined(__APPLE__)
  // Microsoft has never shipped an Azure Kinect SDK for macOS; kept only so the
  // backend still compiles there.
  const char* names[] = {"libk4a.dylib"};
#else
  const char* names[] = {"libk4a.so.1.4", "libk4a.so.1", "libk4a.so"};
#endif

  std::vector<std::string> candidates;

  // The backend's own package directory first: a shipped package carries libk4a
  // and libdepthengine beside the backend, and must not depend on the user
  // having installed the SDK system-wide.
  if(resource_dir && *resource_dir)
    for(const char* n : names)
      candidates.push_back(std::string{resource_dir} + "/" + n);

  // Then the ordinary loader search path, for a system-installed SDK.
  for(const char* n : names)
    candidates.push_back(n);

  for(const auto& c : candidates)
  {
#if defined(_WIN32)
    api.lib = LoadLibraryA(c.c_str());
#else
    api.lib = dlopen(c.c_str(), RTLD_LAZY | RTLD_LOCAL);
#endif
    if(api.lib)
      break;
  }

  if(!api.lib)
  {
    set_error(
        "libk4a not found. Install the Azure Kinect Sensor SDK, or place libk4a "
        "and libdepthengine next to this backend. Without it no Azure Kinect "
        "can be listed.");
    return false;
  }

  bool ok = true;
  const auto sym = [&](auto& fn, const char* name) {
    auto* p = load_symbol(api.lib, name);
    if(!p)
    {
      set_error(std::string{"libk4a is missing "} + name);
      ok = false;
      return;
    }
    fn = reinterpret_cast<std::decay_t<decltype(fn)>>(p);
  };

  sym(api.device_get_installed_count, "k4a_device_get_installed_count");
  sym(api.device_open, "k4a_device_open");
  sym(api.device_close, "k4a_device_close");
  sym(api.device_get_serialnum, "k4a_device_get_serialnum");
  sym(api.device_start_cameras, "k4a_device_start_cameras");
  sym(api.device_stop_cameras, "k4a_device_stop_cameras");
  sym(api.device_get_capture, "k4a_device_get_capture");
  sym(api.device_start_imu, "k4a_device_start_imu");
  sym(api.device_stop_imu, "k4a_device_stop_imu");
  sym(api.device_get_imu_sample, "k4a_device_get_imu_sample");
  sym(api.device_get_calibration, "k4a_device_get_calibration");
  sym(api.device_get_version, "k4a_device_get_version");        // k4a.h:1406
  sym(api.calibration_2d_to_3d, "k4a_calibration_2d_to_3d"); // k4a.h:1809
  sym(api.device_get_color_control_capabilities,
      "k4a_device_get_color_control_capabilities");
  sym(api.device_get_color_control, "k4a_device_get_color_control");
  sym(api.device_set_color_control, "k4a_device_set_color_control");
  sym(api.capture_release, "k4a_capture_release");
  sym(api.capture_get_temperature_c, "k4a_capture_get_temperature_c");
  sym(api.capture_get_color_image, "k4a_capture_get_color_image");
  sym(api.capture_get_depth_image, "k4a_capture_get_depth_image");
  sym(api.capture_get_ir_image, "k4a_capture_get_ir_image");
  sym(api.image_release, "k4a_image_release");
  sym(api.image_reference, "k4a_image_reference");
  sym(api.image_get_buffer, "k4a_image_get_buffer");
  sym(api.image_get_size, "k4a_image_get_size");
  sym(api.image_get_width_pixels, "k4a_image_get_width_pixels");
  sym(api.image_get_height_pixels, "k4a_image_get_height_pixels");
  sym(api.image_get_stride_bytes, "k4a_image_get_stride_bytes");
  sym(api.image_get_format, "k4a_image_get_format");
  sym(api.image_get_device_timestamp_usec, "k4a_image_get_device_timestamp_usec");
  sym(api.image_create, "k4a_image_create");
  sym(api.transformation_create, "k4a_transformation_create");
  sym(api.transformation_destroy, "k4a_transformation_destroy");
  sym(api.transformation_depth_image_to_point_cloud,
      "k4a_transformation_depth_image_to_point_cloud");
  sym(api.transformation_color_image_to_depth_camera,
      "k4a_transformation_color_image_to_depth_camera");
  sym(api.transformation_depth_image_to_color_camera,
      "k4a_transformation_depth_image_to_color_camera");

  if(!ok)
  {
#if !defined(_WIN32)
    dlclose(api.lib);
#endif
    api.lib = nullptr;
  }
  return ok;
}

int to_depthcam_format(k4a_image_format_t f)
{
  switch(f)
  {
    case K4A_IMAGE_FORMAT_COLOR_MJPG:
      return DEPTHCAM_FMT_MJPEG;
    case K4A_IMAGE_FORMAT_COLOR_NV12:
      return DEPTHCAM_FMT_NV12;
    case K4A_IMAGE_FORMAT_COLOR_YUY2:
      return DEPTHCAM_FMT_YUYV422;
    case K4A_IMAGE_FORMAT_COLOR_BGRA32:
      return DEPTHCAM_FMT_BGRA;
    case K4A_IMAGE_FORMAT_DEPTH16:
    case K4A_IMAGE_FORMAT_IR16:
      return DEPTHCAM_FMT_GRAY16;
    default:
      return DEPTHCAM_FMT_NONE;
  }
}

struct Address
{
  enum Kind
  {
    Any,
    Serial,
    Index
  } kind{Any};
  std::string serial;
  uint32_t index{0};
};

Address parse_uri(const char* uri)
{
  Address a;
  if(!uri)
    return a;
  std::string s{uri};
  if(s.rfind("k4a:", 0) == 0)
    s = s.substr(4);
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
      a.index = uint32_t(std::stoul(s.substr(6)));
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

std::string serial_of(k4a_device_t dev)
{
  size_t len = 0;
  if(g_k4a.device_get_serialnum(dev, nullptr, &len) != K4A_BUFFER_RESULT_TOO_SMALL)
    return {};
  std::string s(len, '\0');
  if(g_k4a.device_get_serialnum(dev, s.data(), &len) != K4A_BUFFER_RESULT_SUCCEEDED)
    return {};
  if(!s.empty() && s.back() == '\0')
    s.pop_back();
  return s;
}

struct EnumEntry
{
  std::string uri, serial;
};

/// Serials of the cameras this process holds open, by index: libk4a cannot
/// open a camera twice, so enumeration reads them from here.
std::mutex g_open_mutex;
std::vector<std::pair<uint32_t, std::string>> g_open_serials;

std::optional<std::string> open_serial(uint32_t index)
{
  std::lock_guard lock{g_open_mutex};
  for(const auto& [i, serial] : g_open_serials)
    if(i == index)
      return serial;
  return std::nullopt;
}

void remember_open(uint32_t index, std::string serial)
{
  std::lock_guard lock{g_open_mutex};
  g_open_serials.emplace_back(index, std::move(serial));
}

void forget_open(uint32_t index)
{
  std::lock_guard lock{g_open_mutex};
  for(auto it = g_open_serials.begin(); it != g_open_serials.end(); ++it)
    if(it->first == index)
    {
      g_open_serials.erase(it);
      return;
    }
}

// --- controls ---------------------------------------------------------------

/**
 * libk4a has no enumeration: the commands are a fixed list in k4atypes.h, and
 * the only thing that varies per camera is the range, which
 * k4a_device_get_color_control_capabilities reports. So unlike Orbbec and
 * librealsense the list is written out here, and the capabilities call decides
 * what actually makes it into the tree.
 *
 * Every colour control has a mode as well as a value. A camera left in AUTO
 * ignores writes to the value, which is confusing enough on its own; the
 * `_auto` switch beside each one that supports it makes the pairing visible,
 * and writing a value pushes the control to MANUAL because that is invariably
 * what the writer meant.
 */
struct ColorControlDef
{
  k4a_color_control_command_t command;
  const char* slug;
  const char* description;
};

constexpr ColorControlDef color_controls[] = {
    {K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, "exposure_time",
     "Exposure time in microseconds"},
    {K4A_COLOR_CONTROL_BRIGHTNESS, "brightness", nullptr},
    {K4A_COLOR_CONTROL_CONTRAST, "contrast", nullptr},
    {K4A_COLOR_CONTROL_SATURATION, "saturation", nullptr},
    {K4A_COLOR_CONTROL_SHARPNESS, "sharpness", nullptr},
    {K4A_COLOR_CONTROL_WHITEBALANCE, "white_balance",
     "Colour temperature in degrees Kelvin"},
    {K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, "backlight_compensation", nullptr},
    {K4A_COLOR_CONTROL_GAIN, "gain", nullptr},
    {K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, "powerline_frequency",
     "1 = 50 Hz, 2 = 60 Hz"},
};

struct ControlEntry
{
  /// The command this addresses, or -1 for the ones that are not colour
  /// controls: the temperature readout and the depth mode. The `_auto`
  /// switches carry their value's command.
  int command{-1};
  bool is_mode{};      ///< the `_auto` switch rather than the value
  bool is_temperature{};
  bool is_depth_mode{};
  std::string path;
  std::string name;
  std::string auto_control;
  std::vector<const char*> labels;
  depthcam_control2 desc{};
};

/// "50 Hz" for 1 and "60 Hz" for 2 (k4atypes.h, K4A_COLOR_CONTROL_POWERLINE_FREQUENCY).
constexpr const char* powerline_labels[] = {"50 Hz", "60 Hz"};

/// Indexed by k4a_depth_mode_t from K4A_DEPTH_MODE_NFOV_2X2BINNED (k4atypes.h:292-297).
constexpr const char* depth_mode_labels[] = {
    "NFOV 2x2 binned (320x288)", "NFOV unbinned (640x576)", "WFOV 2x2 binned (512x512)",
    "WFOV unbinned (1024x1024)", "Passive IR (1024x1024)",
};

/**
 * @brief The operating range of a depth mode, in metres.
 *
 * Not in the SDK headers: these are the Azure Kinect DK hardware
 * specification's figures ("Depth camera supported operating modes"), which
 * k4atypes.h:274 defers to. Passive IR measures no depth.
 */
std::pair<float, float> depth_range_of(k4a_depth_mode_t m)
{
  switch(m)
  {
    case K4A_DEPTH_MODE_NFOV_2X2BINNED:
      return {0.5f, 5.46f};
    case K4A_DEPTH_MODE_NFOV_UNBINNED:
      return {0.5f, 3.86f};
    case K4A_DEPTH_MODE_WFOV_2X2BINNED:
      return {0.25f, 2.88f};
    case K4A_DEPTH_MODE_WFOV_UNBINNED:
      return {0.25f, 2.21f};
    default:
      return {0.f, 0.f};
  }
}

/**
 * @brief 30 fps is not offered in WFOV unbinned nor at 3072p; 15 is the
 * fastest those run at.
 *
 * Likewise from the hardware specification, not the headers (k4atypes.h:274).
 * k4a_device_start_cameras refuses the combination outright, so asking for it
 * is better turned into the nearest one that works.
 */
void clamp_fps(k4a_device_configuration_t& c)
{
  if(c.camera_fps == K4A_FRAMES_PER_SECOND_30
     && (c.depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED
         || c.color_resolution == K4A_COLOR_RESOLUTION_3072P))
    c.camera_fps = K4A_FRAMES_PER_SECOND_15;
}

} // namespace

// ---------------------------------------------------------------------------

struct depthcam_device
{
  k4a_device_t dev{};
  uint32_t index{};
  /// Written only while the capture thread is stopped (open, and start() on a
  /// depth mode change); the lock is for the ABI 4 readers on other threads.
  std::mutex calib_lock;
  k4a_calibration_t calibration{};
  k4a_device_configuration_t k4acfg{};
  k4a_transformation_t transformation{};

  /// depth/mode as last written: REQUIRES_RESTART, so start() applies it.
  int depth_mode_request{};
  bool applyDepthMode();

  /// Which opened streams the host consumes; see set_stream_demand.
  std::atomic<uint32_t> demand{~0u};

  std::string serial, firmware;

  /// get_ray_table's answer for `rays_mode`; see backend_get_ray_table.
  std::vector<float> rays;
  int rays_mode{-1};

  depthcam_open_config cfg{};
  bool color_pointcloud{};
  bool cloud_at_color{}; ///< DEPTH_TO_COLOR: cloud at the colour resolution

  std::atomic_bool running{};
  std::thread thread;

  /// A thread of its own, because libk4a is a pull API twice over: the capture
  /// loop is blocked in k4a_device_get_capture and cannot also be waiting on
  /// k4a_device_get_imu_sample. The IMU runs at 1.6kHz against the cameras' 30,
  /// so interleaving them on one thread would not have worked either.
  bool want_imu{};
  std::atomic_bool imu_running{};
  std::thread imu_thread;
  void runImu();

  depthcam_frame_cb on_frame{};
  void* user{};

  // Reused across frames; k4a images are refcounted so these are cheap to keep.
  k4a_image_t xyz_image{};
  k4a_image_t transformed_color{};
  k4a_image_t transformed_depth{};
  std::vector<float> cloud_buf;

  std::vector<ControlEntry> controls;
  bool controls_scanned{};
  std::mutex control_lock;
  /// Reported per capture rather than polled: k4a_capture_get_temperature_c is
  /// the only way to read it, so the capture thread caches the last one.
  std::atomic<float> temperature{0.f};

  void scanControls();
  ControlEntry* findControl(const char* id);

  ~depthcam_device();

  void run();
  void handleCapture(k4a_capture_t capture);
  void emitImage(uint32_t stream, k4a_image_t img, float depth_unit);
  void emitPointCloud(k4a_image_t depth, k4a_image_t color);
};

namespace
{
/// Holds a reference on a k4a image for as long as the host holds its buffer.
struct ImageHolder
{
  k4a_image_t image{};
};

void release_image_holder(void* owner)
{
  auto* h = static_cast<ImageHolder*>(owner);
  if(h->image)
    g_k4a.image_release(h->image);
  delete h;
}

struct VectorHolder
{
  std::vector<float> data;
};

void release_vector_holder(void* owner)
{
  delete static_cast<VectorHolder*>(owner);
}
} // namespace

depthcam_device::~depthcam_device()
{
  if(transformation)
    g_k4a.transformation_destroy(transformation);
  for(auto* img : {&xyz_image, &transformed_color, &transformed_depth})
    if(*img)
      g_k4a.image_release(*img);
  if(dev)
    g_k4a.device_close(dev);
}

void depthcam_device::emitImage(uint32_t stream, k4a_image_t img, float depth_unit)
{
  if(!img || !on_frame)
    return;

  const int format = to_depthcam_format(g_k4a.image_get_format(img));
  if(format == DEPTHCAM_FMT_NONE)
    return;

  // Take our own reference; the capture is released as soon as we return.
  g_k4a.image_reference(img);
  auto* holder = new ImageHolder{img};

  depthcam_frame out{};
  out.stream = stream;
  out.format = format;
  out.width = g_k4a.image_get_width_pixels(img);
  out.height = g_k4a.image_get_height_pixels(img);
  out.stride = g_k4a.image_get_stride_bytes(img);
  out.timestamp_ns = g_k4a.image_get_device_timestamp_usec(img) * 1000ull;
  out.data = g_k4a.image_get_buffer(img);
  out.bytes = g_k4a.image_get_size(img);
  out.depth_unit_mm = depth_unit;
  out.owner = holder;
  out.release = &release_image_holder;

  on_frame(&out, user);
}

void depthcam_device::emitPointCloud(k4a_image_t depth, k4a_image_t color)
{
  if(!transformation || !on_frame || !depth)
    return;
  if(color_pointcloud && !color)
    return;

  // Which camera the cloud is expressed in decides its resolution, and that
  // dominates the cost -- exactly as with the other backends.
  const k4a_calibration_type_t target
      = cloud_at_color ? K4A_CALIBRATION_TYPE_COLOR : K4A_CALIBRATION_TYPE_DEPTH;

  k4a_image_t depth_for_cloud = depth;

  if(cloud_at_color)
  {
    const int w = calibration.color_camera_calibration.resolution_width;
    const int h = calibration.color_camera_calibration.resolution_height;
    if(!transformed_depth
       && g_k4a.image_create(
              K4A_IMAGE_FORMAT_DEPTH16, w, h, w * int(sizeof(uint16_t)),
              &transformed_depth)
              != K4A_RESULT_SUCCEEDED)
      return;
    if(g_k4a.transformation_depth_image_to_color_camera(
           transformation, depth, transformed_depth)
       != K4A_RESULT_SUCCEEDED)
      return;
    depth_for_cloud = transformed_depth;
  }

  const int w = g_k4a.image_get_width_pixels(depth_for_cloud);
  const int h = g_k4a.image_get_height_pixels(depth_for_cloud);

  if(!xyz_image
     && g_k4a.image_create(
            K4A_IMAGE_FORMAT_CUSTOM, w, h, w * 3 * int(sizeof(int16_t)), &xyz_image)
            != K4A_RESULT_SUCCEEDED)
    return;

  if(g_k4a.transformation_depth_image_to_point_cloud(
         transformation, depth_for_cloud, target, xyz_image)
     != K4A_RESULT_SUCCEEDED)
    return;

  const auto* xyz = reinterpret_cast<const int16_t*>(g_k4a.image_get_buffer(xyz_image));
  if(!xyz)
    return;

  // Colour, resampled into the same camera as the cloud.
  const uint8_t* rgba = nullptr;
  if(color_pointcloud)
  {
    if(cloud_at_color)
    {
      rgba = g_k4a.image_get_buffer(color);
    }
    else
    {
      if(!transformed_color
         && g_k4a.image_create(
                K4A_IMAGE_FORMAT_COLOR_BGRA32, w, h, w * 4, &transformed_color)
                != K4A_RESULT_SUCCEEDED)
        return;
      if(g_k4a.transformation_color_image_to_depth_camera(
             transformation, depth, color, transformed_color)
         != K4A_RESULT_SUCCEEDED)
        return;
      rgba = g_k4a.image_get_buffer(transformed_color);
    }
    if(!rgba)
      return;
  }

  // k4a delivers XYZ as int16 millimetres, so a conversion pass is unavoidable
  // here; the ABI promises float.
  const int stride = color_pointcloud ? 6 : 3;
  cloud_buf.clear();
  cloud_buf.reserve(size_t(w) * h * stride);

  for(int i = 0; i < w * h; i++)
  {
    const int16_t z = xyz[i * 3 + 2];
    if(z == 0) // no depth at this pixel
      continue;

    // k4a gives int16 millimetres; the ABI wants float metres.
    constexpr float mm_to_m = 0.001f;
    cloud_buf.push_back(float(xyz[i * 3 + 0]) * mm_to_m);
    cloud_buf.push_back(float(xyz[i * 3 + 1]) * mm_to_m);
    cloud_buf.push_back(float(z) * mm_to_m);

    if(color_pointcloud)
    {
      // BGRA32.
      cloud_buf.push_back(rgba[i * 4 + 2] / 255.f);
      cloud_buf.push_back(rgba[i * 4 + 1] / 255.f);
      cloud_buf.push_back(rgba[i * 4 + 0] / 255.f);
    }
  }

  if(cloud_buf.empty())
    return;

  auto* holder = new VectorHolder{cloud_buf};

  depthcam_frame out{};
  out.stream = DEPTHCAM_STREAM_POINTCLOUD;
  out.format = color_pointcloud ? DEPTHCAM_FMT_XYZRGB : DEPTHCAM_FMT_XYZ;
  out.point_count = int32_t(holder->data.size() / stride);
  out.timestamp_ns = g_k4a.image_get_device_timestamp_usec(depth) * 1000ull;
  out.data = holder->data.data();
  out.bytes = holder->data.size() * sizeof(float);
  out.owner = holder;
  out.release = &release_vector_holder;

  on_frame(&out, user);
}

void depthcam_device::handleCapture(k4a_capture_t capture)
{
  if(g_k4a.capture_get_temperature_c)
    temperature.store(
        g_k4a.capture_get_temperature_c(capture), std::memory_order_relaxed);

  k4a_image_t color = g_k4a.capture_get_color_image(capture);
  k4a_image_t depth = g_k4a.capture_get_depth_image(capture);
  k4a_image_t ir = g_k4a.capture_get_ir_image(capture);

  // Each stream independently: a capture missing colour must still yield depth,
  // IR and the point cloud. What nobody consumes is not delivered -- above all
  // the cloud, whose transformation and conversion pass cost more than the
  // rest of the capture.
  const uint32_t want = cfg.streams & demand.load(std::memory_order_acquire);
  if(want & DEPTHCAM_STREAM_COLOR)
    emitImage(DEPTHCAM_STREAM_COLOR, color, 0.f);
  if(want & DEPTHCAM_STREAM_IR)
    emitImage(DEPTHCAM_STREAM_IR, ir, 0.f);
  if(want & DEPTHCAM_STREAM_DEPTH)
    emitImage(DEPTHCAM_STREAM_DEPTH, depth, 1.f); // DEPTH16 is millimetres
  if(want & DEPTHCAM_STREAM_POINTCLOUD)
    emitPointCloud(depth, color);

  for(auto img : {color, depth, ir})
    if(img)
      g_k4a.image_release(img);
}

void depthcam_device::runImu()
{
  while(imu_running.load(std::memory_order_acquire))
  {
    k4a_imu_sample_t s{};
    const auto r = g_k4a.device_get_imu_sample(dev, &s, 1000);
    if(r == K4A_WAIT_RESULT_TIMEOUT)
      continue;
    if(r != K4A_WAIT_RESULT_SUCCEEDED)
      break;

    if(!on_frame)
      continue;

    depthcam_imu_sample out_sample{};
    // Metres per second squared and radians per second, which is what the ABI
    // asks for; nothing to convert.
    out_sample.accel[0] = s.acc_sample.xyz.x;
    out_sample.accel[1] = s.acc_sample.xyz.y;
    out_sample.accel[2] = s.acc_sample.xyz.z;
    out_sample.gyro[0] = s.gyro_sample.xyz.x;
    out_sample.gyro[1] = s.gyro_sample.xyz.y;
    out_sample.gyro[2] = s.gyro_sample.xyz.z;
    out_sample.temperature_c = s.temperature;
    out_sample.fields
        = DEPTHCAM_IMU_ACCEL | DEPTHCAM_IMU_GYRO | DEPTHCAM_IMU_TEMPERATURE;

    depthcam_frame out{};
    out.stream = DEPTHCAM_STREAM_IMU;
    out.format = DEPTHCAM_FMT_IMU;
    out.timestamp_ns = uint64_t(s.acc_timestamp_usec) * 1000ull;
    out.data = &out_sample;
    out.bytes = sizeof(out_sample);

    try
    {
      on_frame(&out, user);
    }
    catch(...)
    {
      // Never let anything escape into libk4a's caller.
    }
  }
}

void depthcam_device::run()
{
  while(running.load(std::memory_order_acquire))
  {
    k4a_capture_t capture{};
    const auto r = g_k4a.device_get_capture(dev, &capture, 1000);

    if(r == K4A_WAIT_RESULT_TIMEOUT)
      continue;
    if(r != K4A_WAIT_RESULT_SUCCEEDED)
    {
      set_error("capture failed");
      break;
    }

    try
    {
      handleCapture(capture);
    }
    catch(...)
    {
      // Never let anything escape into libk4a's caller.
    }
    g_k4a.capture_release(capture);
  }
}

void depthcam_device::scanControls()
{
  if(controls_scanned)
    return;
  controls_scanned = true;

  if(!dev || !g_k4a.device_get_color_control_capabilities)
    return;

  for(const auto& def : color_controls)
  {
    bool supports_auto = false;
    int32_t min = 0, max = 0, step = 0, def_value = 0;
    k4a_color_control_mode_t def_mode = K4A_COLOR_CONTROL_MODE_MANUAL;

    if(g_k4a.device_get_color_control_capabilities(
           dev, def.command, &supports_auto, &min, &max, &step, &def_value, &def_mode)
       != K4A_RESULT_SUCCEEDED)
      continue;

    // A camera that reports an empty range does not really have the control.
    if(max <= min)
      continue;

    ControlEntry e;
    e.command = int(def.command);
    e.path = std::string{"color/"} + def.slug;
    e.name = def.slug;
    auto& d = e.desc.base;
    d.kind = DEPTHCAM_CONTROL_INT;
    d.access = DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE;
    d.min = min;
    d.max = max;
    d.step = step > 0 ? step : 1;
    d.def = def_value;
    d.description = def.description;

    // Units as k4atypes.h states them for each command.
    switch(def.command)
    {
      case K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE:
        e.desc.unit = "us"; // "Exposure time is measured in microseconds."
        break;
      case K4A_COLOR_CONTROL_WHITEBALANCE:
        e.desc.unit = "K"; // "The unit is degrees Kelvin."
        break;
      case K4A_COLOR_CONTROL_POWERLINE_FREQUENCY:
        if(min == 1 && max == 2 && d.step == 1)
        {
          d.kind = DEPTHCAM_CONTROL_ENUM;
          e.labels.assign(std::begin(powerline_labels), std::end(powerline_labels));
        }
        e.desc.flags |= DEPTHCAM_CONTROL_ADVANCED;
        break;
      case K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION:
        e.desc.flags |= DEPTHCAM_CONTROL_ADVANCED;
        break;
      default:
        break;
    }

    if(supports_auto)
    {
      // While the switch is on the camera picks the value itself.
      e.auto_control = std::string{"color/"} + def.slug + "_auto";
      e.desc.flags |= DEPTHCAM_CONTROL_VOLATILE;
    }
    controls.push_back(std::move(e));

    if(supports_auto)
    {
      ControlEntry m;
      m.command = int(def.command);
      m.is_mode = true;
      m.path = std::string{"color/"} + def.slug + "_auto";
      m.name = std::string{def.slug} + " auto";
      auto& md = m.desc.base;
      md.kind = DEPTHCAM_CONTROL_BOOL;
      md.access = DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE;
      md.min = 0;
      md.max = 1;
      md.step = 1;
      md.def = def_mode == K4A_COLOR_CONTROL_MODE_AUTO ? 1 : 0;
      controls.push_back(std::move(m));
    }
  }

  k4a_depth_mode_t mode{};
  {
    std::lock_guard l{calib_lock};
    mode = k4acfg.depth_mode;
  }
  // Only with the depth camera on: a colour-only open has no mode to pick.
  if(mode != K4A_DEPTH_MODE_OFF)
  {
    ControlEntry m;
    m.is_depth_mode = true;
    m.path = "depth/mode";
    m.name = "depth mode";
    auto& d = m.desc.base;
    d.kind = DEPTHCAM_CONTROL_ENUM;
    d.access = DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE;
    d.min = K4A_DEPTH_MODE_NFOV_2X2BINNED;
    d.max = K4A_DEPTH_MODE_PASSIVE_IR;
    d.step = 1;
    d.def = mode;
    d.description
        = "Field of view and binning of the depth camera. Passive IR delivers "
          "infrared only, without depth.";
    m.labels.assign(std::begin(depth_mode_labels), std::end(depth_mode_labels));
    // libk4a takes the mode in k4a_device_start_cameras only (k4atypes.h,
    // k4a_device_configuration_t::depth_mode).
    m.desc.flags = DEPTHCAM_CONTROL_REQUIRES_RESTART;
    controls.push_back(std::move(m));
  }

  if(g_k4a.capture_get_temperature_c)
  {
    ControlEntry t;
    t.is_temperature = true;
    t.path = "sensors/temperature";
    t.name = "temperature";
    auto& d = t.desc.base;
    d.kind = DEPTHCAM_CONTROL_FLOAT;
    d.access = DEPTHCAM_ACCESS_READ;
    d.min = -40;
    d.max = 125;
    d.step = 0;
    d.def = 0;
    d.description = "Colour sensor temperature in degrees Celsius, as of the last capture";
    t.desc.flags = DEPTHCAM_CONTROL_VOLATILE;
    controls.push_back(std::move(t));
  }

  // After the vector has stopped reallocating.
  for(auto& e : controls)
  {
    e.desc.struct_size = sizeof(depthcam_control2);
    e.desc.base.id = e.path.c_str();
    e.desc.base.name = e.name.c_str();
    e.desc.base.enum_labels = e.labels.empty() ? nullptr : e.labels.data();
    e.desc.base.enum_count = int32_t(e.labels.size());
    e.desc.auto_control = e.auto_control.empty() ? nullptr : e.auto_control.c_str();
  }
}

ControlEntry* depthcam_device::findControl(const char* id)
{
  if(!id)
    return nullptr;
  for(auto& e : controls)
    if(e.path == id)
      return &e;
  return nullptr;
}

bool depthcam_device::applyDepthMode()
{
  int requested{};
  {
    std::lock_guard l{control_lock};
    requested = depth_mode_request;
  }
  if(k4acfg.depth_mode == K4A_DEPTH_MODE_OFF || requested == int(k4acfg.depth_mode))
    return true;

  k4a_device_configuration_t next = k4acfg;
  next.depth_mode = k4a_depth_mode_t(requested);
  clamp_fps(next);

  k4a_calibration_t cal{};
  if(g_k4a.device_get_calibration(dev, next.depth_mode, next.color_resolution, &cal)
     != K4A_RESULT_SUCCEEDED)
  {
    set_error("could not read the Azure Kinect calibration for the new depth mode");
    std::lock_guard l{control_lock};
    depth_mode_request = k4acfg.depth_mode;
    return false;
  }

  // The mode is part of k4a_device_start_cameras' configuration, so the
  // cameras go down and up again. stop() has already stopped the IMU, which
  // libk4a only runs while the cameras do.
  g_k4a.device_stop_cameras(dev);
  if(g_k4a.device_start_cameras(dev, &next) != K4A_RESULT_SUCCEEDED)
  {
    set_error("the Azure Kinect refused the new depth mode; keeping the previous one");
    g_k4a.device_start_cameras(dev, &k4acfg);
    std::lock_guard l{control_lock};
    depth_mode_request = k4acfg.depth_mode;
    return false;
  }

  // Everything sized or computed for the previous mode.
  for(auto* img : {&xyz_image, &transformed_color, &transformed_depth})
    if(*img)
    {
      g_k4a.image_release(*img);
      *img = nullptr;
    }

  std::lock_guard l{calib_lock};
  calibration = cal;
  k4acfg = next;
  rays.clear();
  rays_mode = -1;
  if(transformation)
  {
    g_k4a.transformation_destroy(transformation);
    transformation = g_k4a.transformation_create(&calibration);
    if(!transformation)
      set_error("could not create the Azure Kinect transformation");
  }
  return true;
}

// ---------------------------------------------------------------------------

namespace
{

int backend_list_controls(depthcam_device* dev, depthcam_control_cb cb, void* user)
{
  if(!dev || !cb)
    return 0;
  std::lock_guard lock{dev->control_lock};
  dev->scanControls();
  for(const auto& e : dev->controls)
    cb(&e.desc.base, user);
  return 1;
}

int backend_list_controls2(depthcam_device* dev, depthcam_control2_cb cb, void* user)
{
  if(!dev || !cb)
    return 0;
  std::lock_guard lock{dev->control_lock};
  dev->scanControls();
  for(const auto& e : dev->controls)
    cb(&e.desc, user);
  return 1;
}

int backend_get_control(depthcam_device* dev, const char* id, double* out)
{
  if(!dev || !out)
    return 0;
  std::lock_guard lock{dev->control_lock};
  auto* e = dev->findControl(id);
  if(!e)
    return 0;

  if(e->is_temperature)
  {
    *out = dev->temperature.load(std::memory_order_relaxed);
    return 1;
  }
  if(e->is_depth_mode)
  {
    *out = dev->depth_mode_request;
    return 1;
  }

  if(!g_k4a.device_get_color_control)
    return 0;

  k4a_color_control_mode_t mode{};
  int32_t value{};
  if(g_k4a.device_get_color_control(
         dev->dev, k4a_color_control_command_t(e->command), &mode, &value)
     != K4A_RESULT_SUCCEEDED)
    return 0;

  *out = e->is_mode ? (mode == K4A_COLOR_CONTROL_MODE_AUTO ? 1. : 0.) : double(value);
  return 1;
}

int backend_set_control(depthcam_device* dev, const char* id, double value)
{
  if(!dev)
    return 0;
  std::lock_guard lock{dev->control_lock};
  auto* e = dev->findControl(id);
  if(!e || e->is_temperature)
    return 0;

  if(e->is_depth_mode)
  {
    // Applied by the next start(): the host stops the stream around this.
    const double v = std::clamp(std::round(value), e->desc.base.min, e->desc.base.max);
    dev->depth_mode_request = int(v);
    return 1;
  }

  if(!g_k4a.device_set_color_control || !g_k4a.device_get_color_control)
    return 0;

  const auto command = k4a_color_control_command_t(e->command);

  if(e->is_mode)
  {
    // Switching to manual has to carry a value, and the only sensible one is
    // whatever the camera settled on while it was automatic -- otherwise the
    // picture jumps the moment the switch is flipped.
    k4a_color_control_mode_t current{};
    int32_t held{};
    g_k4a.device_get_color_control(dev->dev, command, &current, &held);

    return g_k4a.device_set_color_control(
               dev->dev, command,
               value != 0. ? K4A_COLOR_CONTROL_MODE_AUTO
                           : K4A_COLOR_CONTROL_MODE_MANUAL,
               held)
           == K4A_RESULT_SUCCEEDED;
  }

  // Writing a value means the writer wants that value, so leave auto behind.
  return g_k4a.device_set_color_control(
             dev->dev, command, K4A_COLOR_CONTROL_MODE_MANUAL, int32_t(value))
         == K4A_RESULT_SUCCEEDED;
}

int backend_init(const char* resource_dir)
{
  // A missing SDK is not a load failure: the backend stays registered and simply
  // reports no cameras, so the device browser can say so.
  if(!g_k4a)
    load_k4a(g_k4a, resource_dir);
  return 1;
}

void backend_shutdown() { }

int backend_enumerate(depthcam_enumerate_cb cb, void* user)
{
  if(!g_k4a || !cb)
    return 0;

  const uint32_t n = g_k4a.device_get_installed_count();
  for(uint32_t i = 0; i < n; i++)
  {
    // The serial is only readable from an open handle, so each camera is opened
    // briefly and closed again. Unlike the Orbbec SDK there is no metadata-only
    // path, but this happens once per browser refresh and the handle is not
    // retained.
    EnumEntry e;
    if(auto serial = open_serial(i))
    {
      e.serial = *serial;
    }
    else
    {
      k4a_device_t dev{};
      if(g_k4a.device_open(i, &dev) != K4A_RESULT_SUCCEEDED)
        continue;
      e.serial = serial_of(dev);
      g_k4a.device_close(dev);
    }

    e.uri = e.serial.empty() ? ("k4a:index:" + std::to_string(i))
                             : ("k4a:sn:" + e.serial);

    depthcam_device_info info{};
    info.backend = "k4a";
    info.uri = e.uri.c_str();
    info.name = "Azure Kinect DK";
    info.serial = e.serial.c_str();
    info.transport = "usb3";
    info.streams = DEPTHCAM_STREAM_COLOR | DEPTHCAM_STREAM_IR
                   | DEPTHCAM_STREAM_DEPTH | DEPTHCAM_STREAM_POINTCLOUD
                   | DEPTHCAM_STREAM_IMU;
    cb(&info, user);
  }
  return 1;
}

/// Nearest supported colour mode for a requested height.
k4a_color_resolution_t color_resolution_for(int height)
{
  if(height <= 0)
    return K4A_COLOR_RESOLUTION_1080P;
  if(height <= 720)
    return K4A_COLOR_RESOLUTION_720P;
  if(height <= 1080)
    return K4A_COLOR_RESOLUTION_1080P;
  if(height <= 1440)
    return K4A_COLOR_RESOLUTION_1440P;
  if(height <= 1536)
    return K4A_COLOR_RESOLUTION_1536P;
  if(height <= 2160)
    return K4A_COLOR_RESOLUTION_2160P;
  return K4A_COLOR_RESOLUTION_3072P;
}

/**
 * @brief The depth mode an open starts in.
 *
 * Only the starting point: depth/mode picks any of the five explicitly, and
 * the host replays it before the first start. A size that is exactly one
 * mode's (k4atypes.h:292-297) selects it; otherwise NFOV unbinned (640x576)
 * unless a smaller depth was asked for.
 */
k4a_depth_mode_t depth_mode_for(int width, int height)
{
  if(width <= 0 && height <= 0)
    return K4A_DEPTH_MODE_NFOV_UNBINNED;
  if(width == 320 && height == 288)
    return K4A_DEPTH_MODE_NFOV_2X2BINNED;
  if(width == 640 && height == 576)
    return K4A_DEPTH_MODE_NFOV_UNBINNED;
  if(width == 512 && height == 512)
    return K4A_DEPTH_MODE_WFOV_2X2BINNED;
  if(width == 1024 && height == 1024)
    return K4A_DEPTH_MODE_WFOV_UNBINNED;
  if(width <= 320 || height <= 288)
    return K4A_DEPTH_MODE_NFOV_2X2BINNED;
  if(width >= 1024 || height >= 1024)
    return K4A_DEPTH_MODE_WFOV_UNBINNED;
  return K4A_DEPTH_MODE_NFOV_UNBINNED;
}

k4a_fps_t fps_for(int fps)
{
  if(fps <= 0)
    return K4A_FRAMES_PER_SECOND_30;
  if(fps <= 5)
    return K4A_FRAMES_PER_SECOND_5;
  if(fps <= 15)
    return K4A_FRAMES_PER_SECOND_15;
  return K4A_FRAMES_PER_SECOND_30;
}

depthcam_device* backend_open(const char* uri, const depthcam_open_config* config)
{
  if(!g_k4a || !config)
  {
    set_error("libk4a is not available");
    return nullptr;
  }

  const auto addr = parse_uri(uri);
  const uint32_t n = g_k4a.device_get_installed_count();
  if(n == 0)
  {
    set_error("no Azure Kinect connected");
    return nullptr;
  }

  auto dev = std::make_unique<depthcam_device>();
  dev->cfg = *config;

  uint32_t index = 0;
  switch(addr.kind)
  {
    case Address::Index:
      if(addr.index >= n)
      {
        set_error("no Azure Kinect at index " + std::to_string(addr.index));
        return nullptr;
      }
      index = addr.index;
      break;
    case Address::Serial: {
      bool found = false;
      for(uint32_t i = 0; i < n && !found; i++)
      {
        k4a_device_t probe{};
        if(g_k4a.device_open(i, &probe) != K4A_RESULT_SUCCEEDED)
          continue;
        if(serial_of(probe) == addr.serial)
        {
          index = i;
          found = true;
        }
        g_k4a.device_close(probe);
      }
      if(!found)
      {
        // No silent fallback: streaming from a different camera than the
        // document names is worse than failing to connect.
        set_error("Azure Kinect '" + addr.serial + "' is not connected");
        return nullptr;
      }
      break;
    }
    case Address::Any:
      index = 0;
      break;
  }

  if(g_k4a.device_open(index, &dev->dev) != K4A_RESULT_SUCCEEDED)
  {
    set_error("could not open the Azure Kinect (is it on a USB3 port?)");
    return nullptr;
  }
  dev->index = index;
  dev->serial = serial_of(dev->dev);
  remember_open(index, dev->serial);

  k4a_hardware_version_t v{};
  if(g_k4a.device_get_version(dev->dev, &v) == K4A_RESULT_SUCCEEDED)
  {
    const auto ver = [](const k4a_version_t& x) {
      return std::to_string(x.major) + "." + std::to_string(x.minor) + "."
             + std::to_string(x.iteration);
    };
    // k4a_hardware_version_t, k4atypes.h: one version per firmware component.
    dev->firmware = "rgb " + ver(v.rgb) + ", depth " + ver(v.depth) + ", depth sensor "
                    + ver(v.depth_sensor) + ", audio " + ver(v.audio);
  }

  dev->want_imu = (config->streams & DEPTHCAM_STREAM_IMU) != 0;

  const bool want_cloud = (config->streams & DEPTHCAM_STREAM_POINTCLOUD) != 0;
  dev->color_pointcloud = want_cloud && config->color_pointcloud != 0
                          && config->align != DEPTHCAM_ALIGN_NONE;
  dev->cloud_at_color = config->align == DEPTHCAM_ALIGN_DEPTH_TO_COLOR;

  k4a_device_configuration_t k4acfg = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

  const bool need_color
      = (config->streams & DEPTHCAM_STREAM_COLOR) || dev->color_pointcloud;
  const bool need_depth = (config->streams & DEPTHCAM_STREAM_DEPTH)
                          || (config->streams & DEPTHCAM_STREAM_IR) || want_cloud;

  if(need_color)
  {
    // BGRA32 rather than the default MJPG: the transformation functions need
    // uncompressed colour, and the host would otherwise decode it only to have
    // the cloud need it again. The SDK converts on the host either way.
    k4acfg.color_format = dev->color_pointcloud ? K4A_IMAGE_FORMAT_COLOR_BGRA32
                                                : K4A_IMAGE_FORMAT_COLOR_MJPG;
    k4acfg.color_resolution = color_resolution_for(config->color_height);
  }

  if(need_depth)
    k4acfg.depth_mode = depth_mode_for(config->depth_width, config->depth_height);

  k4acfg.camera_fps = fps_for(config->color_fps > 0 ? config->color_fps
                                                    : config->depth_fps);
  // Without this the colour and depth images of one capture can be milliseconds
  // apart, which shows up as fringing on a coloured point cloud.
  k4acfg.synchronized_images_only = need_color && need_depth;
  clamp_fps(k4acfg);
  dev->k4acfg = k4acfg;
  dev->depth_mode_request = k4acfg.depth_mode;

  if(g_k4a.device_get_calibration(
         dev->dev, k4acfg.depth_mode, k4acfg.color_resolution, &dev->calibration)
     != K4A_RESULT_SUCCEEDED)
  {
    set_error("could not read the Azure Kinect calibration");
    return nullptr;
  }

  if(want_cloud)
  {
    dev->transformation = g_k4a.transformation_create(&dev->calibration);
    if(!dev->transformation)
    {
      set_error("could not create the Azure Kinect transformation");
      return nullptr;
    }
  }

  if(g_k4a.device_start_cameras(dev->dev, &k4acfg) != K4A_RESULT_SUCCEEDED)
  {
    // Almost always either a USB2 port or a missing libdepthengine.
    set_error("could not start the Azure Kinect cameras; check the USB3 "
              "connection and that libdepthengine is installed");
    return nullptr;
  }

  return dev.release();
}

void backend_stop(depthcam_device* dev);

void backend_close(depthcam_device* dev)
{
  if(!dev)
    return;
  backend_stop(dev);
  if(dev->dev)
  {
    g_k4a.device_stop_cameras(dev->dev);
    forget_open(dev->index);
  }
  delete dev;
}

int backend_start(depthcam_device* dev, depthcam_frame_cb on_frame, void* user)
{
  if(!dev || !dev->dev)
    return 0;
  if(dev->running.load(std::memory_order_acquire))
    return 1;

  // Before the thread exists, so the calibration and the cached images can be
  // replaced without racing it.
  dev->applyDepthMode();

  dev->on_frame = on_frame;
  dev->user = user;
  dev->running.store(true, std::memory_order_release);
  // k4a is a pull API, so unlike the other backends this one owns a thread.
  dev->thread = std::thread{[dev] { dev->run(); }};

  if(dev->want_imu && g_k4a.device_start_imu)
  {
    // A camera whose IMU refuses to start still has video, which matters more.
    if(g_k4a.device_start_imu(dev->dev) == K4A_RESULT_SUCCEEDED)
    {
      dev->imu_running.store(true, std::memory_order_release);
      dev->imu_thread = std::thread{[dev] { dev->runImu(); }};
    }
    else
    {
      set_error("could not start the Azure Kinect IMU");
    }
  }
  return 1;
}

void backend_stop(depthcam_device* dev)
{
  if(!dev)
    return;

  if(dev->imu_running.exchange(false, std::memory_order_acq_rel))
  {
    // stop_imu first: it unblocks a thread sitting in get_imu_sample, which is
    // documented to be safe from another thread and is the only way not to wait
    // out the timeout.
    if(g_k4a.device_stop_imu)
      g_k4a.device_stop_imu(dev->dev);
    if(dev->imu_thread.joinable())
      dev->imu_thread.join();
  }

  if(!dev->running.exchange(false, std::memory_order_acq_rel))
    return;
  if(dev->thread.joinable())
    dev->thread.join();
}

/// Map a delivered stream to the k4a camera whose frame of reference it is in;
/// called with calib_lock held.
/// Depth and IR are the same camera; the cloud moves to colour under
/// DEPTH_TO_COLOR (see emitPointCloud), the depth and IR images never do.
std::optional<k4a_calibration_type_t> camera_of(const depthcam_device& dev, uint32_t stream)
{
  switch(stream)
  {
    case DEPTHCAM_STREAM_DEPTH:
    case DEPTHCAM_STREAM_IR:
      if(dev.k4acfg.depth_mode == K4A_DEPTH_MODE_OFF)
        return std::nullopt;
      return K4A_CALIBRATION_TYPE_DEPTH;
    case DEPTHCAM_STREAM_POINTCLOUD:
      if(dev.k4acfg.depth_mode == K4A_DEPTH_MODE_OFF)
        return std::nullopt;
      return dev.cloud_at_color ? K4A_CALIBRATION_TYPE_COLOR : K4A_CALIBRATION_TYPE_DEPTH;
    case DEPTHCAM_STREAM_COLOR:
      if(dev.k4acfg.color_resolution == K4A_COLOR_RESOLUTION_OFF)
        return std::nullopt;
      return K4A_CALIBRATION_TYPE_COLOR;
    case DEPTHCAM_STREAM_IMU:
      return K4A_CALIBRATION_TYPE_ACCEL;
    default:
      return std::nullopt;
  }
}

int backend_get_intrinsics(depthcam_device* dev, uint32_t stream, depthcam_intrinsics* out)
{
  if(!dev || !out || !(dev->cfg.streams & stream))
    return 0;
  std::lock_guard l{dev->calib_lock};
  const auto cam = camera_of(*dev, stream);
  if(!cam || (*cam != K4A_CALIBRATION_TYPE_DEPTH && *cam != K4A_CALIBRATION_TYPE_COLOR))
    return 0;

  const k4a_calibration_camera_t& c = *cam == K4A_CALIBRATION_TYPE_COLOR
                                          ? dev->calibration.color_camera_calibration
                                          : dev->calibration.depth_camera_calibration;
  if(c.resolution_width <= 0 || c.resolution_height <= 0)
    return 0;

  // The calibration is read for the mode in use ("may be different for each
  // operating depth_mode and color_resolution", k4a.h:1598-1600) and carries
  // the resolution it describes; should that not be the delivered size, the
  // host scales it, and get_ray_table declines.
  const auto& p = c.intrinsics.parameters.param;
  depthcam_intrinsics r{};
  r.struct_size = out->struct_size;
  r.width = c.resolution_width;
  r.height = c.resolution_height;
  r.fx = p.fx;
  r.fy = p.fy;
  // The headers do not say where k4a puts a pixel's centre; they call the model
  // "compatible with OpenCV" (k4atypes.h:1013-1014), whose centres are at
  // integer coordinates, so cx, cy are passed through unshifted. get_ray_table
  // asks the SDK itself with the same convention.
  r.cx = p.cx;
  r.cy = p.cy;

  // k4a_calibration_intrinsic_parameters_t (k4atypes.h:1024-1046) names every
  // coefficient, in the order cx cy fx fy k1..k6 codx cody p2 p1: note p2
  // before p1. k4..k6 are OpenCV's rational denominator, which is what
  // DEPTHCAM_DISTORTION_BROWN_CONRADY's k4..k6 are.
  switch(c.intrinsics.type)
  {
    case K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY:
    // Early internal devices only (k4atypes.h:688); the same polynomial about a
    // centre of distortion (codx, cody) that is dropped here. The ray table
    // still comes from the SDK, so the positions stay exact.
    case K4A_CALIBRATION_LENS_DISTORTION_MODEL_RATIONAL_6KT:
      r.model = DEPTHCAM_DISTORTION_BROWN_CONRADY;
      r.coeffs[0] = p.k1;
      r.coeffs[1] = p.k2;
      r.coeffs[2] = p.k3;
      r.coeffs[3] = p.k4;
      r.coeffs[4] = p.k5;
      r.coeffs[5] = p.k6;
      r.coeffs[6] = p.p1;
      r.coeffs[7] = p.p2;
      break;
    default:
      // THETA and POLYNOMIAL_3K are "Deprecated (not supported)" (k4atypes.h:684-687).
      r.model = DEPTHCAM_DISTORTION_NONE;
      break;
  }
  std::memcpy(out, &r, std::min<size_t>(out->struct_size, sizeof(r)));
  return 1;
}

int backend_get_extrinsics(
    depthcam_device* dev, uint32_t from, uint32_t to, depthcam_extrinsics* out)
{
  if(!dev || !out)
    return 0;
  std::lock_guard l{dev->calib_lock};
  const auto a = camera_of(*dev, from);
  const auto b = camera_of(*dev, to);
  if(!a || !b)
    return 0;

  depthcam_extrinsics r{};
  r.struct_size = out->struct_size;
  r.rotation[0] = r.rotation[4] = r.rotation[8] = 1.f;
  if(*a != *b)
  {
    // "use the parameters stored under extrinsics[source][target]"
    // (k4atypes.h:1105-1111): row-major rotation, translation in millimetres
    // (k4atypes.h:1003-1004).
    const auto& x = dev->calibration.extrinsics[*a][*b];
    std::memcpy(r.rotation, x.rotation, sizeof(r.rotation));
    for(int i = 0; i < 3; i++)
      r.translation[i] = x.translation[i] * 0.001f;
  }
  std::memcpy(out, &r, std::min<size_t>(out->struct_size, sizeof(r)));
  return 1;
}

int backend_get_details(depthcam_device* dev, depthcam_device_details* out)
{
  if(!dev || !out)
    return 0;
  depthcam_device_details r{};
  r.struct_size = out->struct_size;
  r.model = "Azure Kinect DK";
  r.serial = dev->serial.c_str();
  r.firmware = dev->firmware.c_str();
  // libk4a reports firmware versions only, no board revision.
  r.hardware_rev = "";
  {
    std::lock_guard l{dev->calib_lock};
    const auto [lo, hi] = depth_range_of(dev->k4acfg.depth_mode);
    r.depth_min_m = lo;
    r.depth_max_m = hi;
  }
  std::memcpy(out, &r, std::min<size_t>(out->struct_size, sizeof(r)));
  return 1;
}

int backend_get_ray_table(
    depthcam_device* dev, uint32_t stream, float* xy, int32_t w, int32_t h)
{
  // The delivered depth image is always the depth camera's: alignment here
  // only moves the point cloud.
  if(!dev || !xy || stream != DEPTHCAM_STREAM_DEPTH || !g_k4a.calibration_2d_to_3d)
    return 0;

  std::lock_guard l{dev->calib_lock};
  if(dev->k4acfg.depth_mode == K4A_DEPTH_MODE_OFF)
    return 0;
  const auto& c = dev->calibration.depth_camera_calibration;
  if(w != c.resolution_width || h != c.resolution_height || w <= 0 || h <= 0)
    return 0;

  // One per depth mode: the calibration only changes with it.
  const size_t n = size_t(w) * size_t(h) * 2;
  if(dev->rays_mode != int(dev->k4acfg.depth_mode) || dev->rays.size() != n)
  {
    try
    {
      dev->rays.resize(n);
    }
    catch(...)
    {
      return 0;
    }
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    for(int v = 0; v < h; v++)
      for(int u = 0; u < w; u++)
      {
        // At 1000 mm the point is the ray scaled by z; "valid" is 0 outside
        // the calibration's domain (k4a.h:1777-1785).
        const k4a_float2_t px{{float(u), float(v)}};
        k4a_float3_t p{};
        int valid = 0;
        const bool ok
            = g_k4a.calibration_2d_to_3d(
                  &dev->calibration, &px, 1000.f, K4A_CALIBRATION_TYPE_DEPTH,
                  K4A_CALIBRATION_TYPE_DEPTH, &p, &valid)
                  == K4A_RESULT_SUCCEEDED
              && valid && p.xyz.z > 0.f;
        float* o = dev->rays.data() + 2 * (size_t(v) * w + u);
        o[0] = ok ? p.xyz.x / p.xyz.z : nan;
        o[1] = ok ? p.xyz.y / p.xyz.z : nan;
      }
    dev->rays_mode = int(dev->k4acfg.depth_mode);
  }
  std::memcpy(xy, dev->rays.data(), n * sizeof(float));
  return 1;
}

int backend_list_profiles(const char*, uint32_t stream, depthcam_profile_cb cb, void* user)
{
  if(!cb)
    return 0;
  const auto emit = [&](int w, int h, int fps, int fmt) {
    depthcam_profile p{};
    p.struct_size = sizeof(p);
    p.width = w;
    p.height = h;
    p.fps = fps;
    p.format = fmt;
    cb(&p, user);
  };

  // libk4a fixes the modes: the resolutions are those of k4atypes.h:292-297
  // and :311-317, the rates k4a_fps_t's (:480-482), limited to 15 fps where
  // the hardware specification says so (see clamp_fps).
  struct Mode
  {
    int w, h, max_fps;
  };
  switch(stream)
  {
    case DEPTHCAM_STREAM_DEPTH:
    case DEPTHCAM_STREAM_POINTCLOUD:
    case DEPTHCAM_STREAM_IR: {
      const bool ir = stream == DEPTHCAM_STREAM_IR;
      std::vector<Mode> modes{{320, 288, 30}, {640, 576, 30}, {512, 512, 30}, {1024, 1024, 15}};
      // Passive IR has no depth, and runs at the full rate.
      if(ir)
        modes.push_back({1024, 1024, 30});
      std::vector<std::array<int, 3>> seen;
      for(const auto& m : modes)
        for(int fps : {5, 15, 30})
        {
          const std::array<int, 3> key{m.w, m.h, fps};
          if(fps > m.max_fps || std::find(seen.begin(), seen.end(), key) != seen.end())
            continue;
          seen.push_back(key);
          // DEPTH16 and IR16 (k4atypes.h:398,413).
          emit(m.w, m.h, fps, DEPTHCAM_FMT_GRAY16);
        }
      return 1;
    }
    case DEPTHCAM_STREAM_COLOR: {
      const Mode modes[]{{1280, 720, 30},  {1920, 1080, 30}, {2560, 1440, 30},
                         {2048, 1536, 30}, {3840, 2160, 30}, {4096, 3072, 15}};
      for(const auto& m : modes)
        for(int fps : {5, 15, 30})
          if(fps <= m.max_fps)
            // MJPEG as open() asks for it; BGRA32 at the same modes when a
            // coloured cloud is wanted.
            emit(m.w, m.h, fps, DEPTHCAM_FMT_MJPEG);
      return 1;
    }
    default:
      return 0;
  }
}

void backend_set_stream_demand(depthcam_device* dev, uint32_t streams)
{
  if(dev)
    dev->demand.store(streams, std::memory_order_release);
}

uint32_t backend_active_streams(depthcam_device* dev)
{
  if(!dev)
    return 0;
  uint32_t streams = dev->cfg.streams;
  // Every Azure Kinect has an IMU; what it may not have is a libk4a new enough
  // to expose it.
  if(!dev->want_imu || !g_k4a.device_start_imu)
    streams &= ~uint32_t(DEPTHCAM_STREAM_IMU);
  return streams;
}

const depthcam_backend_v1 g_backend{
    .abi_version = DEPTHCAM_ABI_VERSION,
    .name = "k4a",
    .display_name = "Azure Kinect",
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
    .get_ray_table = &backend_get_ray_table,
    .list_controls2 = &backend_list_controls2,
    // libk4a reports no setting changing.
    .set_control_changed_callback = nullptr,
    .list_profiles = &backend_list_profiles,
    .set_stream_demand = &backend_set_stream_demand,
};

} // namespace

extern "C" DEPTHCAM_EXPORT const depthcam_backend_v1* score_depthcam_backend_v1(void)
{
  return &g_backend;
}
