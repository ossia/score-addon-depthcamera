/*
 * Intel RealSense backend: librealsense2 behind depthcam_abi.h.
 *
 * The closest fit of any SDK here: frames are refcounted (so zero-copy), the
 * pipeline is push-based (so no thread of our own), rs2::align maps one-to-one
 * onto our alignment modes, and vertices are already float metres.
 */
#include <depthcam_abi.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cctype>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{

/// The device browser groups by vendor already, so the model alone is enough
/// and keeps the name short. Case-insensitive: the SDKs are not consistent.
std::string strip_vendor(std::string name, std::string_view prefix)
{
  if(name.size() > prefix.size()
     && std::equal(
         prefix.begin(), prefix.end(), name.begin(), [](char a, char b) {
    return std::tolower(static_cast<unsigned char>(a))
           == std::tolower(static_cast<unsigned char>(b));
  }))
    name.erase(0, prefix.size());
  return name;
}
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

std::unique_ptr<rs2::context> g_context;
depthcam_changed_cb g_changed_cb = nullptr;
void* g_changed_user = nullptr;

int to_depthcam_format(rs2_format f)
{
  switch(f)
  {
    case RS2_FORMAT_RGB8:
      return DEPTHCAM_FMT_RGB24;
    case RS2_FORMAT_BGR8:
      return DEPTHCAM_FMT_BGR24;
    case RS2_FORMAT_RGBA8:
      return DEPTHCAM_FMT_RGBA;
    case RS2_FORMAT_BGRA8:
      return DEPTHCAM_FMT_BGRA;
    case RS2_FORMAT_YUYV:
      return DEPTHCAM_FMT_YUYV422;
    case RS2_FORMAT_UYVY:
      return DEPTHCAM_FMT_UYVY422;
    case RS2_FORMAT_Y8:
      return DEPTHCAM_FMT_GRAY8;
    case RS2_FORMAT_Y16:
    case RS2_FORMAT_Z16:
      return DEPTHCAM_FMT_GRAY16;
    case RS2_FORMAT_MJPEG:
      return DEPTHCAM_FMT_MJPEG;
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
  int index{0};
};

Address parse_uri(const char* uri)
{
  Address a;
  if(!uri)
    return a;
  std::string s{uri};
  if(s.rfind("realsense:", 0) == 0)
    s = s.substr(10);
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

std::string info_of(const rs2::device& d, rs2_camera_info i)
{
  try
  {
    return d.supports(i) ? d.get_info(i) : std::string{};
  }
  catch(...)
  {
    return {};
  }
}

struct EnumEntry
{
  std::string uri, name, serial, transport;
};
// --- controls ---------------------------------------------------------------

/**
 * @brief Turn RS2_OPTION_ENABLE_AUTO_EXPOSURE into "auto_exposure".
 *
 * rs2_option_to_string gives "Enable Auto Exposure": human-readable, but not an
 * address. The enum spelling is the stable one, so that is what is used.
 */
std::string option_slug(rs2_option opt)
{
  std::string n = rs2_option_to_string(opt);
  for(auto& c : n)
    c = (c == ' ' || c == '-' || c == '/') ? '_'
                                           : char(std::tolower((unsigned char)c));
  return n;
}

/**
 * @brief Which group a sensor's options land in.
 *
 * librealsense names its sensors after the hardware block -- "Stereo Module",
 * "RGB Camera", "Motion Module" -- and every option belongs to exactly one, so
 * the sensor is the natural category. Anything unrecognised keeps its own name
 * rather than being lumped together, because a camera we have never seen is
 * exactly the case where guessing is wrong.
 */
std::string sensor_group(const std::string& name)
{
  std::string n = name;
  for(auto& c : n)
    c = (c == ' ' || c == '-') ? '_' : char(std::tolower((unsigned char)c));

  if(n.find("stereo") != std::string::npos || n.find("depth") != std::string::npos)
    return "depth";
  if(n.find("rgb") != std::string::npos || n.find("color") != std::string::npos)
    return "color";
  if(n.find("motion") != std::string::npos)
    return "imu";
  if(n.find("safety") != std::string::npos)
    return "safety";
  return n.empty() ? "device" : n;
}

/// One option, plus the sensor or processing block it has to be addressed
/// through.
struct ControlEntry
{
  /// The owner of a sensor option. Empty for a filter's, whose block lives in
  /// the device (see `filter`).
  rs2::sensor sensor;
  /// Index into depthcam_device::filters, or -1 for a sensor option.
  int filter{-1};
  /// filters/<name>/enabled: ours, not an rs2_option.
  bool is_enable{};
  rs2_option option{};
  std::string path;
  std::string name;
  std::string description;
  std::string unit;
  std::string auto_control;
  uint32_t flags{};
  std::vector<std::string> labels;
  std::vector<const char*> label_ptrs;
  depthcam_control2 desc{};
};

/**
 * @brief Where set_control_changed_callback's callback lives.
 *
 * Shared with the lambdas handed to librealsense rather than owned by the
 * device: those run on librealsense's options-watcher thread, and nothing in
 * rs_options.hpp says unsubscribing waits for a call already under way. The
 * mutex is held across the call so that clearing returns only once no call is
 * in flight, which is what the ABI promises the host.
 */
struct ChangeNotifier
{
  std::mutex lock;
  depthcam_control_changed_cb cb{};
  void* user{};

  void emit(const char* id)
  {
    std::lock_guard l{lock};
    if(cb)
    {
      try
      {
        cb(id, user);
      }
      catch(...)
      {
      }
    }
  }
};

/// One of librealsense's post-processing blocks, as filters/<name>/...
struct FilterSlot
{
  const char* name;
  rs2::filter* block;
  std::atomic_bool enabled{false};
};

/**
 * @brief Fill a ControlEntry's range, kind and labels from what the sensor or
 * block says about one option.
 *
 * @param path_for builds the address once it is known whether the option is
 *        read-only.
 * @return false for an option that cannot be used.
 */
template<typename PathFor>
bool describeOption(
    ControlEntry& e, const rs2::options& opts, rs2_option opt, PathFor&& path_for)
{
  e.name = rs2_option_to_string(opt);

  rs2::option_range range{};
  bool read_only = false;
  try
  {
    if(!opts.supports(opt))
      return false;
    range = opts.get_option_range(opt);
    read_only = opts.is_option_read_only(opt);
    if(const char* desc = opts.get_option_description(opt))
      e.description = desc;
  }
  catch(...)
  {
    // An option the sensor lists but refuses to describe is not usable.
    return false;
  }

  e.path = path_for(read_only);
  if(read_only)
    e.flags |= DEPTHCAM_CONTROL_VOLATILE;

  auto& d = e.desc.base;
  d.access = DEPTHCAM_ACCESS_READ | (read_only ? 0 : DEPTHCAM_ACCESS_WRITE);
  d.min = range.min;
  d.max = range.max;
  d.step = range.step;
  d.def = range.def;

  // Every librealsense option is a float over the wire; the range is what
  // says whether it is really a switch, a menu or a number.
  const bool integral = range.step >= 1.f && range.min == std::floor(range.min)
                        && range.max == std::floor(range.max);

  // The visual preset is a menu whatever its span; it is the one that matters.
  const bool menu_candidate
      = integral
        && ((range.max - range.min) <= 32.f
            || (opt == RS2_OPTION_VISUAL_PRESET && (range.max - range.min) <= 256.f));

  if(integral && range.min == 0.f && range.max == 1.f && range.step == 1.f)
  {
    d.kind = DEPTHCAM_CONTROL_BOOL;
  }
  else if(menu_candidate)
  {
    // Values with names are a menu. rs2_get_option_value_description returns
    // null for anything that is just a number (rs_options.hpp: "or null if no
    // special meaning"), so this only fires for the handful that really are
    // enumerations (visual preset, sequence id).
    bool named = false;
    for(float v = range.min; v <= range.max; v += range.step)
    {
      const char* label = nullptr;
      try
      {
        label = opts.get_option_value_description(opt, v);
      }
      catch(...)
      {
      }
      e.labels.push_back(label ? label : std::to_string(int(v)));
      named = named || (label != nullptr);
    }
    if(named)
    {
      d.kind = DEPTHCAM_CONTROL_ENUM;
    }
    else
    {
      e.labels.clear();
      d.kind = DEPTHCAM_CONTROL_INT;
    }
  }
  else
  {
    d.kind = integral ? DEPTHCAM_CONTROL_INT : DEPTHCAM_CONTROL_FLOAT;
  }
  return true;
}

/**
 * @brief Units, flags and auto toggles of a sensor option, where the SDK says
 * what they are.
 *
 * Only units librealsense states are given: rs_option.h documents the laser in
 * mW, depth units in metres, the baseline and depth offset in millimetres and
 * the auto-exposure limit in microseconds; the D400 depth exposure is
 * registered as "Depth Exposure (usec)" (src/ds/d400/d400-device.cpp:779).
 * The colour exposure and the temperatures are documented in no unit, so they
 * get none.
 */
void describeSensorOption(ControlEntry& e, const std::string& group)
{
  switch(e.option)
  {
    case RS2_OPTION_LASER_POWER:
      e.unit = "mW";
      break;
    case RS2_OPTION_DEPTH_UNITS:
      e.unit = "m";
      e.flags |= DEPTHCAM_CONTROL_ADVANCED;
      break;
    case RS2_OPTION_STEREO_BASELINE:
    case RS2_OPTION_DEPTH_OFFSET:
      e.unit = "mm";
      break;
    case RS2_OPTION_AUTO_EXPOSURE_LIMIT:
      e.unit = "us";
      break;
    case RS2_OPTION_EXPOSURE:
      if(group == "depth")
        e.unit = "us";
      [[fallthrough]];
    case RS2_OPTION_GAIN:
      // Both are driven by the sensor's auto exposure while it is on.
      e.auto_control = group + "/" + option_slug(RS2_OPTION_ENABLE_AUTO_EXPOSURE);
      e.flags |= DEPTHCAM_CONTROL_VOLATILE;
      break;
    case RS2_OPTION_WHITE_BALANCE:
      e.auto_control = group + "/" + option_slug(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
      e.flags |= DEPTHCAM_CONTROL_VOLATILE;
      break;
    case RS2_OPTION_FRAMES_QUEUE_SIZE:
    case RS2_OPTION_ERROR_POLLING_ENABLED:
    case RS2_OPTION_OUTPUT_TRIGGER_ENABLED:
    case RS2_OPTION_AUTO_EXPOSURE_CONVERGE_STEP:
    case RS2_OPTION_GLOBAL_TIME_ENABLED:
    case RS2_OPTION_HOST_PERFORMANCE:
    case RS2_OPTION_STREAM_FILTER:
    case RS2_OPTION_STREAM_FORMAT_FILTER:
    case RS2_OPTION_STREAM_INDEX_FILTER:
    case RS2_OPTION_SEQUENCE_NAME:
    case RS2_OPTION_SEQUENCE_SIZE:
    case RS2_OPTION_SEQUENCE_ID:
    case RS2_OPTION_EMITTER_ON_OFF:
    case RS2_OPTION_THERMAL_COMPENSATION:
    case RS2_OPTION_ENABLE_MOTION_CORRECTION:
      e.flags |= DEPTHCAM_CONTROL_ADVANCED;
      break;
    default:
      break;
  }
}

} // namespace

// ---------------------------------------------------------------------------

struct depthcam_device
{
  rs2::pipeline pipeline;
  rs2::config config;
  std::unique_ptr<rs2::align> align;
  rs2::pointcloud pointcloud;

  /// Kept only for the controls: an rs2::device is a handle, so holding it
  /// alongside the pipeline costs nothing and does not claim the camera twice.
  rs2::device rsdev;

  depthcam_open_config cfg{};
  std::string serial;
  bool color_pointcloud{};
  bool want_imu{};
  std::atomic_bool running{};
  float depth_scale_mm{1.f};

  depthcam_frame_cb on_frame{};
  void* user{};

  std::vector<float> cloud_buf;

  std::vector<ControlEntry> controls;
  bool controls_scanned{};
  std::mutex control_lock;

  void scanControls();
  ControlEntry* findControl(const char* id);
  rs2::options& optionsOf(ControlEntry& e);

  /// Post-processing, applied to depth in this order: the one
  /// doc/post-processing-filters.md:106 recommends, which wraps spatial and
  /// temporal in a depth -> disparity -> depth round trip on stereo cameras
  /// ("Applicable for stereo-based depth cameras (D4XX)", :107).
  rs2::decimation_filter decimation;
  rs2::threshold_filter threshold;
  rs2::spatial_filter spatial;
  rs2::temporal_filter temporal;
  rs2::hole_filling_filter hole_filling;
  rs2::disparity_transform to_disparity{true};
  rs2::disparity_transform from_disparity{false};
  bool stereo{};
  enum
  {
    FilterDecimation,
    FilterThreshold,
    FilterSpatial,
    FilterTemporal,
    FilterHoleFilling,
  };
  std::array<FilterSlot, 5> filters{{
      {"decimation", &decimation},
      {"threshold", &threshold},
      {"spatial", &spatial},
      {"temporal", &temporal},
      {"hole_filling", &hole_filling},
  }};
  rs2::frameset filterDepth(const rs2::frameset& fs);

  /// Which opened streams the host consumes; see set_stream_demand.
  std::atomic<uint32_t> demand{~0u};

  std::shared_ptr<ChangeNotifier> notifier{std::make_shared<ChangeNotifier>()};
  bool subscribed{};
  void subscribeOptions();

  /// What the pipeline streams: resolved from the config at open, so that the
  /// calibration can be read before start(), and replaced by the profile
  /// start() actually got (which differs after a fallback).
  std::mutex profile_lock;
  rs2::pipeline_profile profile;
  /// The intrinsics of the frames last delivered, which is what the ABI asks
  /// for: rs2::align gives an aligned frame the target's intrinsics
  /// (src/proc/align.cpp:197-200) and decimation shrinks depth. The profile
  /// pointer is only compared, never dereferenced, so a profile that has since
  /// gone away is harmless.
  struct Delivered
  {
    const void* key{};
    std::optional<rs2_intrinsics> intrinsics;
  };
  Delivered delivered_depth, delivered_color, delivered_ir;
  void remember(Delivered& d, const rs2::frame& f);

  std::optional<rs2_intrinsics> profileIntrinsics(rs2_stream s, int index);
  std::optional<rs2_intrinsics> intrinsicsOf(uint32_t stream);
  rs2::stream_profile frameOf(uint32_t stream);
  bool alignedTo(rs2_stream target);

  std::mutex ray_lock;
  std::vector<float> rays;
  rs2_intrinsics rays_for{};

  std::string model, firmware;

  void handle(const rs2::frameset& fs);
  void handleMotion(const rs2::motion_frame& f);
  void emitImage(uint32_t stream, const rs2::video_frame& f, float depth_unit);
  void emitPointCloud(const rs2::frameset& fs);
};

namespace
{
/// Keeps a librealsense frame alive for exactly as long as the host holds its
/// buffer. rs2::frame is refcounted, so this is a reference, not a copy.
struct FrameHolder
{
  rs2::frame frame;
};

void release_frame_holder(void* owner)
{
  delete static_cast<FrameHolder*>(owner);
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

void depthcam_device::emitImage(
    uint32_t stream, const rs2::video_frame& f, float depth_unit)
{
  if(!f || !on_frame)
    return;

  const int format = to_depthcam_format(f.get_profile().format());
  if(format == DEPTHCAM_FMT_NONE)
    return;

  auto* holder = new FrameHolder{f};

  depthcam_frame out{};
  out.stream = stream;
  out.format = format;
  out.width = f.get_width();
  out.height = f.get_height();
  out.stride = f.get_stride_in_bytes();
  out.timestamp_ns = uint64_t(f.get_timestamp() * 1e6); // ms -> ns
  out.data = f.get_data();
  out.bytes = size_t(f.get_data_size());
  out.depth_unit_mm = depth_unit;
  out.owner = holder;
  out.release = &release_frame_holder;

  on_frame(&out, user);
}

void depthcam_device::emitPointCloud(const rs2::frameset& fs)
{
  if(!on_frame)
    return;

  auto depth = fs.get_depth_frame();
  if(!depth)
    return;

  rs2::video_frame color = fs.get_color_frame();
  if(color_pointcloud && !color)
    return;

  try
  {
    if(color_pointcloud)
      pointcloud.map_to(color);

    rs2::points pts = pointcloud.calculate(depth);
    if(!pts)
      return;

    const size_t n = pts.size();
    if(n == 0)
      return;

    const auto* verts = pts.get_vertices();
    if(!verts)
      return;

    // Uncoloured: hand librealsense's own buffer over untouched. rs2::vertex is
    // three floats in metres, which is exactly DEPTHCAM_FMT_XYZ.
    if(!color_pointcloud)
    {
      static_assert(sizeof(rs2::vertex) == 3 * sizeof(float));

      auto* holder = new FrameHolder{pts};

      depthcam_frame out{};
      out.stream = DEPTHCAM_STREAM_POINTCLOUD;
      out.format = DEPTHCAM_FMT_XYZ;
      out.point_count = int32_t(n);
      out.timestamp_ns = uint64_t(depth.get_timestamp() * 1e6);
      out.data = verts;
      out.bytes = n * sizeof(rs2::vertex);
      out.owner = holder;
      out.release = &release_frame_holder;

      on_frame(&out, user);
      return;
    }

    // Coloured: librealsense gives texture coordinates rather than per-point
    // colour, so the interleave has to be built here.
    const auto* uvs = pts.get_texture_coordinates();
    if(!uvs)
      return;

    const int cw = color.get_width();
    const int ch = color.get_height();
    const int cstride = color.get_stride_in_bytes();
    const int cbpp = color.get_bytes_per_pixel();
    const auto* cdata = static_cast<const uint8_t*>(color.get_data());
    if(!cdata || cbpp < 3)
      return;

    const bool bgr = color.get_profile().format() == RS2_FORMAT_BGR8
                     || color.get_profile().format() == RS2_FORMAT_BGRA8;

    cloud_buf.clear();
    cloud_buf.reserve(n * 6);

    for(size_t i = 0; i < n; i++)
    {
      // z == 0 means no reading at that pixel.
      if(verts[i].z == 0.f)
        continue;

      const int u = int(uvs[i].u * cw + 0.5f);
      const int v = int(uvs[i].v * ch + 0.5f);
      if(u < 0 || v < 0 || u >= cw || v >= ch)
        continue;

      cloud_buf.push_back(verts[i].x);
      cloud_buf.push_back(verts[i].y);
      cloud_buf.push_back(verts[i].z);

      const uint8_t* p = cdata + size_t(v) * cstride + size_t(u) * cbpp;
      if(bgr)
      {
        cloud_buf.push_back(p[2] / 255.f);
        cloud_buf.push_back(p[1] / 255.f);
        cloud_buf.push_back(p[0] / 255.f);
      }
      else
      {
        cloud_buf.push_back(p[0] / 255.f);
        cloud_buf.push_back(p[1] / 255.f);
        cloud_buf.push_back(p[2] / 255.f);
      }
    }

    if(cloud_buf.empty())
      return;

    auto* holder = new VectorHolder{cloud_buf};

    depthcam_frame out{};
    out.stream = DEPTHCAM_STREAM_POINTCLOUD;
    out.format = DEPTHCAM_FMT_XYZRGB;
    out.point_count = int32_t(holder->data.size() / 6);
    out.timestamp_ns = uint64_t(depth.get_timestamp() * 1e6);
    out.data = holder->data.data();
    out.bytes = holder->data.size() * sizeof(float);
    out.owner = holder;
    out.release = &release_vector_holder;

    on_frame(&out, user);
  }
  catch(const rs2::error& e)
  {
    set_error(std::string{"point cloud: "} + e.what());
  }
  catch(...)
  {
  }
}

void depthcam_device::scanControls()
{
  if(controls_scanned)
    return;
  controls_scanned = true;

  std::vector<rs2::sensor> sensors;
  try
  {
    sensors = rsdev.query_sensors();
  }
  catch(...)
  {
    return;
  }

  for(auto& sensor : sensors)
  {
    std::string sname;
    try
    {
      if(sensor.supports(RS2_CAMERA_INFO_NAME))
        sname = sensor.get_info(RS2_CAMERA_INFO_NAME);
    }
    catch(...)
    {
    }
    const auto group = sensor_group(sname);

    std::vector<rs2_option> options;
    try
    {
      options = sensor.get_supported_options();
    }
    catch(...)
    {
      continue;
    }

    for(auto opt : options)
    {
      ControlEntry e;
      e.sensor = sensor;
      e.option = opt;
      // Read-only options are temperatures, frame counters and the settings an
      // `auto` mode chose for itself: worth watching, never worth a slider next
      // to the ones that do something. The sensor name stays in the leaf there,
      // because two sensors each report their own temperature.
      if(describeOption(e, sensor, opt, [&](bool read_only) {
           return read_only ? "sensors/" + group + "_" + option_slug(opt)
                            : group + "/" + option_slug(opt);
         }))
      {
        if(std::any_of(controls.begin(), controls.end(), [&](const ControlEntry& o) {
             return o.path == e.path;
           }))
          continue;
        describeSensorOption(e, group);
        controls.push_back(std::move(e));
      }
    }
  }

  // The post-processing blocks, after the sensors so that the camera's own
  // settings come first in the tree.
  for(int i = 0; i < int(filters.size()); i++)
  {
    const std::string base = std::string{"filters/"} + filters[i].name + "/";

    ControlEntry en;
    en.filter = i;
    en.is_enable = true;
    en.path = base + "enabled";
    en.name = std::string{filters[i].name} + " enabled";
    en.description = "Apply librealsense's " + std::string{filters[i].name}
                     + " filter to the depth stream and the point cloud";
    en.flags = DEPTHCAM_CONTROL_FILTER;
    en.desc.base.kind = DEPTHCAM_CONTROL_BOOL;
    en.desc.base.access = DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE;
    en.desc.base.min = 0;
    en.desc.base.max = 1;
    en.desc.base.step = 1;
    en.desc.base.def = 0;
    controls.push_back(std::move(en));

    rs2::filter& block = *filters[i].block;
    std::vector<rs2_option> options;
    try
    {
      options = block.get_supported_options();
    }
    catch(...)
    {
      continue;
    }
    for(auto opt : options)
    {
      // Which stream a block processes is fixed by how it is used here.
      if(opt == RS2_OPTION_STREAM_FILTER || opt == RS2_OPTION_STREAM_FORMAT_FILTER
         || opt == RS2_OPTION_STREAM_INDEX_FILTER || opt == RS2_OPTION_FRAMES_QUEUE_SIZE)
        continue;

      ControlEntry e;
      e.filter = i;
      e.option = opt;
      if(!describeOption(e, block, opt, [&](bool) { return base + option_slug(opt); }))
        continue;
      e.flags |= DEPTHCAM_CONTROL_FILTER;
      // "Min range in meters" / "Max range in meters", src/proc/threshold.cpp:22-24.
      if(i == FilterThreshold
         && (opt == RS2_OPTION_MIN_DISTANCE || opt == RS2_OPTION_MAX_DISTANCE))
        e.unit = "m";
      controls.push_back(std::move(e));
    }
  }

  // An auto toggle that this camera does not have cannot make anything inactive.
  for(auto& e : controls)
    if(!e.auto_control.empty() && !findControl(e.auto_control.c_str()))
    {
      e.auto_control.clear();
      e.flags &= ~uint32_t(DEPTHCAM_CONTROL_VOLATILE);
    }

  // Point the descriptors at their owner's stable storage, after the vector has
  // stopped reallocating.
  for(auto& e : controls)
  {
    e.label_ptrs.clear();
    e.label_ptrs.reserve(e.labels.size());
    for(const auto& l : e.labels)
      e.label_ptrs.push_back(l.c_str());

    auto& d = e.desc;
    d.struct_size = sizeof(depthcam_control2);
    d.base.id = e.path.c_str();
    d.base.name = e.name.c_str();
    d.base.description = e.description.empty() ? nullptr : e.description.c_str();
    d.base.enum_labels = e.label_ptrs.empty() ? nullptr : e.label_ptrs.data();
    d.base.enum_count = int32_t(e.label_ptrs.size());
    d.unit = e.unit.empty() ? nullptr : e.unit.c_str();
    d.flags = e.flags;
    d.auto_control = e.auto_control.empty() ? nullptr : e.auto_control.c_str();
  }
}

rs2::options& depthcam_device::optionsOf(ControlEntry& e)
{
  if(e.filter >= 0)
    return *filters[e.filter].block;
  return e.sensor;
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

void depthcam_device::handleMotion(const rs2::motion_frame& f)
{
  if(!running.load(std::memory_order_acquire) || !on_frame)
    return;

  const auto stream = f.get_profile().stream_type();
  const auto v = f.get_motion_data();

  depthcam_imu_sample sample{};
  if(stream == RS2_STREAM_ACCEL)
  {
    // librealsense reports metres per second squared, which the ABI wants too.
    sample.accel[0] = v.x;
    sample.accel[1] = v.y;
    sample.accel[2] = v.z;
    sample.fields = DEPTHCAM_IMU_ACCEL;
  }
  else if(stream == RS2_STREAM_GYRO)
  {
    // Radians per second, likewise.
    sample.gyro[0] = v.x;
    sample.gyro[1] = v.y;
    sample.gyro[2] = v.z;
    sample.fields = DEPTHCAM_IMU_GYRO;
  }
  else
  {
    return;
  }

  // One field per sample, deliberately: the D435i's accelerometer and gyroscope
  // run at different rates (63Hz and 200Hz by default) and pairing them would
  // mean either holding the fast one back or repeating the slow one.
  depthcam_frame out{};
  out.stream = DEPTHCAM_STREAM_IMU;
  out.format = DEPTHCAM_FMT_IMU;
  out.timestamp_ns = uint64_t(f.get_timestamp() * 1e6);
  out.data = &sample;
  out.bytes = sizeof(sample);
  on_frame(&out, user);
}

rs2::frameset depthcam_device::filterDepth(const rs2::frameset& fs)
{
  const auto on = [this](int i) {
    return filters[i].enabled.load(std::memory_order_relaxed);
  };
  if(!std::any_of(filters.begin(), filters.end(), [](const FilterSlot& f) {
       return f.enabled.load(std::memory_order_relaxed);
     }))
    return fs;

  try
  {
    // Each block takes the whole frameset and replaces only the depth frame in
    // it (generic_processing_block::prepare_output,
    // src/proc/synthetic-stream.cpp:113-170); decimation only touches Z16
    // depth (src/proc/decimation-filter.cpp:217-218), so IR keeps its size.
    rs2::frame f = fs;
    if(on(FilterDecimation))
      f = decimation.process(f);
    if(on(FilterThreshold))
      f = threshold.process(f);
    const bool disparity = stereo && (on(FilterSpatial) || on(FilterTemporal));
    if(disparity)
      f = to_disparity.process(f);
    if(on(FilterSpatial))
      f = spatial.process(f);
    if(on(FilterTemporal))
      f = temporal.process(f);
    if(disparity)
      f = from_disparity.process(f);
    if(on(FilterHoleFilling))
      f = hole_filling.process(f);
    if(auto out = f.as<rs2::frameset>())
      return out;
  }
  catch(const std::exception& e)
  {
    set_error(std::string{"depth filters: "} + e.what());
  }
  return fs;
}

void depthcam_device::remember(Delivered& d, const rs2::frame& f)
{
  if(!f)
    return;
  const auto p = f.get_profile();
  std::lock_guard l{profile_lock};
  if(p.get() == d.key)
    return;
  std::optional<rs2_intrinsics> in;
  if(auto vp = p.as<rs2::video_stream_profile>())
    in = vp.get_intrinsics();
  d.key = p.get();
  d.intrinsics = in;
}

void depthcam_device::handle(const rs2::frameset& frames)
{
  if(!running.load(std::memory_order_acquire))
    return;

  const uint32_t want = cfg.streams & demand.load(std::memory_order_acquire);
  const bool want_depth = want & (DEPTHCAM_STREAM_DEPTH | DEPTHCAM_STREAM_POINTCLOUD);

  rs2::frameset fs = frames;
  if(want_depth)
    fs = filterDepth(fs);

  // Only what alignment changes: depth (and so the cloud) when aligning to
  // colour, the other video streams when aligning to depth.
  if(align)
  {
    const uint32_t aligned
        = cfg.align == DEPTHCAM_ALIGN_DEPTH_TO_COLOR
              ? uint32_t(DEPTHCAM_STREAM_DEPTH | DEPTHCAM_STREAM_POINTCLOUD)
              : uint32_t(
                    DEPTHCAM_STREAM_COLOR | DEPTHCAM_STREAM_IR
                    | DEPTHCAM_STREAM_POINTCLOUD);
    if(want & aligned)
      fs = align->process(fs);
  }

  // Each stream independently: a frameset without colour must still yield
  // depth, IR and the point cloud.
  if(want & DEPTHCAM_STREAM_COLOR)
    if(auto c = fs.get_color_frame())
    {
      remember(delivered_color, c);
      emitImage(DEPTHCAM_STREAM_COLOR, c, 0.f);
    }

  if(want & DEPTHCAM_STREAM_IR)
    if(auto ir = fs.get_infrared_frame())
    {
      remember(delivered_ir, ir);
      emitImage(DEPTHCAM_STREAM_IR, ir, 0.f);
    }

  if(want_depth)
    if(auto d = fs.get_depth_frame())
    {
      remember(delivered_depth, d);
      if(want & DEPTHCAM_STREAM_DEPTH)
        emitImage(DEPTHCAM_STREAM_DEPTH, d, depth_scale_mm);
    }

  if(want & DEPTHCAM_STREAM_POINTCLOUD)
    emitPointCloud(fs);
}

bool depthcam_device::alignedTo(rs2_stream target)
{
  // An alignment whose target is not streamed does nothing.
  if(!align)
    return false;
  if(target == RS2_STREAM_COLOR && cfg.align != DEPTHCAM_ALIGN_DEPTH_TO_COLOR)
    return false;
  if(target == RS2_STREAM_DEPTH && cfg.align != DEPTHCAM_ALIGN_COLOR_TO_DEPTH)
    return false;
  std::lock_guard l{profile_lock};
  if(!profile)
    return false;
  try
  {
    return bool(profile.get_stream(target));
  }
  catch(...)
  {
    return false;
  }
}

std::optional<rs2_intrinsics> depthcam_device::profileIntrinsics(rs2_stream s, int index)
{
  std::lock_guard l{profile_lock};
  if(!profile)
    return std::nullopt;
  try
  {
    if(auto vp = profile.get_stream(s, index).as<rs2::video_stream_profile>())
      return vp.get_intrinsics();
  }
  catch(...)
  {
  }
  return std::nullopt;
}

std::optional<rs2_intrinsics> depthcam_device::intrinsicsOf(uint32_t stream)
{
  // What the frames said first; before any has arrived (the host reads the
  // calibration right after open), what the pipeline will deliver, with the
  // alignment applied the way rs2::align applies it. Decimation is not
  // anticipated there: the host scales intrinsics to the delivered size.
  const auto delivered = [this](const Delivered& d) {
    std::lock_guard l{profile_lock};
    return d.intrinsics;
  };
  switch(stream)
  {
    case DEPTHCAM_STREAM_DEPTH:
    case DEPTHCAM_STREAM_POINTCLOUD:
      if(auto in = delivered(delivered_depth))
        return in;
      return alignedTo(RS2_STREAM_COLOR) ? profileIntrinsics(RS2_STREAM_COLOR, -1)
                                         : profileIntrinsics(RS2_STREAM_DEPTH, -1);
    case DEPTHCAM_STREAM_COLOR:
      if(auto in = delivered(delivered_color))
        return in;
      return alignedTo(RS2_STREAM_DEPTH) ? intrinsicsOf(DEPTHCAM_STREAM_DEPTH)
                                         : profileIntrinsics(RS2_STREAM_COLOR, -1);
    case DEPTHCAM_STREAM_IR:
      if(auto in = delivered(delivered_ir))
        return in;
      return alignedTo(RS2_STREAM_DEPTH) ? intrinsicsOf(DEPTHCAM_STREAM_DEPTH)
                                         : profileIntrinsics(RS2_STREAM_INFRARED, 1);
    default:
      return std::nullopt;
  }
}

rs2::stream_profile depthcam_device::frameOf(uint32_t stream)
{
  // The physical sensor whose frame of reference a delivered stream is in: an
  // aligned stream has moved into its target's.
  rs2_stream type{};
  int index = -1;
  switch(stream)
  {
    case DEPTHCAM_STREAM_DEPTH:
    case DEPTHCAM_STREAM_POINTCLOUD:
      type = alignedTo(RS2_STREAM_COLOR) ? RS2_STREAM_COLOR : RS2_STREAM_DEPTH;
      break;
    case DEPTHCAM_STREAM_COLOR:
      type = alignedTo(RS2_STREAM_DEPTH) ? RS2_STREAM_DEPTH : RS2_STREAM_COLOR;
      break;
    case DEPTHCAM_STREAM_IR:
      type = alignedTo(RS2_STREAM_DEPTH) ? RS2_STREAM_DEPTH : RS2_STREAM_INFRARED;
      index = type == RS2_STREAM_INFRARED ? 1 : -1;
      break;
    case DEPTHCAM_STREAM_IMU:
      type = RS2_STREAM_ACCEL;
      break;
    default:
      return {};
  }
  std::lock_guard l{profile_lock};
  if(!profile)
    return {};
  try
  {
    return profile.get_stream(type, index);
  }
  catch(...)
  {
    return {};
  }
}

void depthcam_device::subscribeOptions()
{
  // Collected under the control lock, subscribed outside it: subscribing
  // starts librealsense's watcher thread, and a notification arriving on it
  // must be free to call back into get_control.
  //
  // Grouped by the underlying rs2_options, not by handle: every ControlEntry
  // holds its own rs2::sensor copy, and a second subscription on the same
  // rs2_options replaces the first (src/rs.cpp:1602 assigns it).
  struct Target
  {
    const void* key;
    rs2::options* opts;
    std::vector<std::pair<rs2_option, std::string>> ids;
  };
  std::vector<Target> targets;
  {
    std::lock_guard l{control_lock};
    if(subscribed)
      return;
    subscribed = true;
    scanControls();

    for(auto& e : controls)
    {
      if(e.is_enable)
        continue;
      const void* key = e.filter >= 0 ? static_cast<const void*>(filters[e.filter].block)
                                      : static_cast<const void*>(e.sensor.get().get());
      auto it = std::find_if(targets.begin(), targets.end(), [&](const Target& t) {
        return t.key == key;
      });
      if(it == targets.end())
      {
        targets.push_back({key, &optionsOf(e), {}});
        it = targets.end() - 1;
      }
      it->ids.emplace_back(e.option, e.path);
    }
  }

  for(auto& [key, opts, ids] : targets)
  {
    try
    {
      // rs_options.hpp:292. librealsense polls the options on a thread of its
      // own (src/core/options-watcher.cpp:96-130) and reports what changed,
      // whoever changed it. The subscription is held by the rs2_options it
      // was made on (src/rs.cpp:1602), so it ends when the sensor handles in
      // `controls` do. Processing blocks accept the call but never report:
      // their options_container returns an empty subscription
      // (src/core/options-container.h:72).
      opts->on_options_changed(
          [n = notifier, ids = std::move(ids)](const rs2::options_list& list) {
        try
        {
          for(auto&& v : list)
            for(const auto& [opt, path] : ids)
              if(opt == v->id)
                n->emit(path.c_str());
        }
        catch(...)
        {
        }
      });
    }
    catch(const std::exception& ex)
    {
      set_error(std::string{"option notifications: "} + ex.what());
    }
  }
}

// ---------------------------------------------------------------------------

namespace
{

int backend_init(const char*)
{
  try
  {
    g_context = std::make_unique<rs2::context>();

    g_context->set_devices_changed_callback(
        [](rs2::event_information&) {
      if(g_changed_cb)
        g_changed_cb(g_changed_user);
    });
    return 1;
  }
  catch(const rs2::error& e)
  {
    set_error(e.what());
    return 0;
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
    return 0;
  }
  catch(...)
  {
    return 0;
  }
}

void backend_shutdown()
{
  g_changed_cb = nullptr;
  g_changed_user = nullptr;
  g_context.reset();
}

int backend_enumerate(depthcam_enumerate_cb cb, void* user)
{
  if(!g_context || !cb)
    return 0;

  try
  {
    // Metadata only; no pipeline is started.
    for(auto&& d : g_context->query_devices())
    {
      EnumEntry e;
      e.serial = info_of(d, RS2_CAMERA_INFO_SERIAL_NUMBER);
      e.name = strip_vendor(info_of(d, RS2_CAMERA_INFO_NAME), "Intel RealSense ");
      e.transport = info_of(d, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
      if(e.name.empty())
        e.name = "RealSense";
      if(!e.transport.empty())
        e.transport = "usb" + e.transport;

      e.uri = e.serial.empty() ? std::string{"realsense:index:0"}
                               : ("realsense:sn:" + e.serial);

      depthcam_device_info info{};
      info.backend = "realsense";
      info.uri = e.uri.c_str();
      info.name = e.name.c_str();
      info.serial = e.serial.c_str();
      info.transport = e.transport.c_str();
      info.streams = DEPTHCAM_STREAM_COLOR | DEPTHCAM_STREAM_IR
                     | DEPTHCAM_STREAM_DEPTH | DEPTHCAM_STREAM_POINTCLOUD
                     | DEPTHCAM_STREAM_IMU;
      cb(&info, user);
    }
    return 1;
  }
  catch(const rs2::error& e)
  {
    set_error(e.what());
    return 0;
  }
  catch(...)
  {
    return 0;
  }
}

void backend_set_changed_callback(depthcam_changed_cb cb, void* user)
{
  g_changed_cb = cb;
  g_changed_user = user;
}

int backend_list_controls(
    depthcam_device* dev, depthcam_control_cb cb, void* user)
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
  if(!e || !(e->desc.base.access & DEPTHCAM_ACCESS_READ))
    return 0;
  if(e->is_enable)
  {
    *out = dev->filters[e->filter].enabled.load(std::memory_order_relaxed) ? 1. : 0.;
    return 1;
  }
  try
  {
    *out = double(dev->optionsOf(*e).get_option(e->option));
    return 1;
  }
  catch(const std::exception& ex)
  {
    set_error(ex.what());
    return 0;
  }
  catch(...)
  {
    return 0;
  }
}

int backend_set_control(depthcam_device* dev, const char* id, double value)
{
  if(!dev)
    return 0;
  std::lock_guard lock{dev->control_lock};
  auto* e = dev->findControl(id);
  if(!e || !(e->desc.base.access & DEPTHCAM_ACCESS_WRITE))
    return 0;
  if(e->is_enable)
  {
    dev->filters[e->filter].enabled.store(value != 0., std::memory_order_relaxed);
    return 1;
  }
  try
  {
    dev->optionsOf(*e).set_option(e->option, float(value));
    return 1;
  }
  catch(const std::exception& ex)
  {
    // Routine rather than exceptional: several options are only writable while
    // their `auto` counterpart is off, and librealsense reports that by
    // throwing.
    set_error(ex.what());
    return 0;
  }
  catch(...)
  {
    return 0;
  }
}

void backend_set_control_changed_callback(
    depthcam_device* dev, depthcam_control_changed_cb cb, void* user)
{
  if(!dev)
    return;
  {
    // Waits for a notification in flight: see ChangeNotifier.
    std::lock_guard l{dev->notifier->lock};
    dev->notifier->cb = cb;
    dev->notifier->user = user;
  }
  // Only once something listens: every subscription is a polling thread in
  // librealsense.
  if(cb)
  {
    try
    {
      dev->subscribeOptions();
    }
    catch(...)
    {
    }
  }
}

/// k1 k2 k3 k4 k5 k6 p1 p2, from rs2_intrinsics.
void fill_intrinsics(const rs2_intrinsics& in, depthcam_intrinsics* out)
{
  depthcam_intrinsics r{};
  r.struct_size = out->struct_size;
  r.width = in.width;
  r.height = in.height;
  r.fx = in.fx;
  r.fy = in.fy;
  // "pixel offset from the left edge" (h/rs_types.h:64-65), with the pixel
  // centre convention the ABI already takes from librealsense's own
  // deprojection: no shift.
  r.cx = in.ppx;
  r.cy = in.ppy;

  // rs2_intrinsics::coeffs: "Order for Brown-Conrady: [k1, k2, p1, p2, k3].
  // Order for F-Theta Fish-eye: [k1, k2, k3, k4, 0]" (h/rs_types.h:69);
  // Kannala-Brandt reads coeffs[0..3] as k1..k4 (rs2_deproject_pixel_to_point,
  // src/rs.cpp:4338-4350).
  switch(in.model)
  {
    case RS2_DISTORTION_NONE:
      r.model = DEPTHCAM_DISTORTION_NONE;
      break;
    case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
    case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
    case RS2_DISTORTION_BROWN_CONRADY:
      r.model = in.model == RS2_DISTORTION_BROWN_CONRADY
                    ? DEPTHCAM_DISTORTION_BROWN_CONRADY
                : in.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY
                    ? DEPTHCAM_DISTORTION_INVERSE_BROWN_CONRADY
                    : DEPTHCAM_DISTORTION_MODIFIED_BROWN_CONRADY;
      r.coeffs[0] = in.coeffs[0]; // k1
      r.coeffs[1] = in.coeffs[1]; // k2
      r.coeffs[2] = in.coeffs[4]; // k3
      r.coeffs[6] = in.coeffs[2]; // p1
      r.coeffs[7] = in.coeffs[3]; // p2
      break;
    case RS2_DISTORTION_KANNALA_BRANDT4:
      r.model = DEPTHCAM_DISTORTION_KANNALA_BRANDT4;
      for(int i = 0; i < 4; i++)
        r.coeffs[i] = in.coeffs[i];
      break;
    case RS2_DISTORTION_FTHETA:
      r.model = DEPTHCAM_DISTORTION_FTHETA;
      r.coeffs[0] = in.coeffs[0];
      break;
    default:
      r.model = DEPTHCAM_DISTORTION_NONE;
      break;
  }
  std::memcpy(out, &r, std::min<size_t>(out->struct_size, sizeof(r)));
}

int backend_get_intrinsics(depthcam_device* dev, uint32_t stream, depthcam_intrinsics* out)
{
  if(!dev || !out || !(dev->cfg.streams & stream))
    return 0;
  try
  {
    const auto in = dev->intrinsicsOf(stream);
    if(!in)
      return 0;
    fill_intrinsics(*in, out);
    return 1;
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
    return 0;
  }
  catch(...)
  {
    return 0;
  }
}

int backend_get_extrinsics(
    depthcam_device* dev, uint32_t from, uint32_t to, depthcam_extrinsics* out)
{
  if(!dev || !out)
    return 0;
  try
  {
    depthcam_extrinsics r{};
    r.struct_size = out->struct_size;
    r.rotation[0] = r.rotation[4] = r.rotation[8] = 1.f;

    const auto a = dev->frameOf(from);
    const auto b = dev->frameOf(to);
    if(!a || !b)
      return 0;
    // Same sensor -- also what an aligned stream and its target come to.
    if(a.unique_id() != b.unique_id())
    {
      const rs2_extrinsics x = a.get_extrinsics_to(b);
      // "Column-major 3x3 rotation matrix", "in meters" (h/rs_sensor.h:111-112).
      for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
          r.rotation[3 * i + j] = x.rotation[3 * j + i];
      for(int i = 0; i < 3; i++)
        r.translation[i] = x.translation[i];
    }
    std::memcpy(out, &r, std::min<size_t>(out->struct_size, sizeof(r)));
    return 1;
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
    return 0;
  }
  catch(...)
  {
    return 0;
  }
}

int backend_get_details(depthcam_device* dev, depthcam_device_details* out)
{
  if(!dev || !out)
    return 0;
  depthcam_device_details r{};
  r.struct_size = out->struct_size;
  r.model = dev->model.c_str();
  r.serial = dev->serial.c_str();
  r.firmware = dev->firmware.c_str();
  // rs2_camera_info has no hardware revision (h/rs_sensor.h:23-42), and no
  // query gives the range of the current mode, which on a D400 depends on the
  // resolution, the baseline and the scene.
  r.hardware_rev = "";
  r.depth_min_m = 0.f;
  r.depth_max_m = 0.f;
  std::memcpy(out, &r, std::min<size_t>(out->struct_size, sizeof(r)));
  return 1;
}

int backend_get_ray_table(
    depthcam_device* dev, uint32_t stream, float* xy, int32_t w, int32_t h)
{
  if(!dev || !xy || w <= 0 || h <= 0)
    return 0;
  if(stream != DEPTHCAM_STREAM_DEPTH && stream != DEPTHCAM_STREAM_POINTCLOUD)
    return 0;
  try
  {
    const auto in = dev->intrinsicsOf(DEPTHCAM_STREAM_DEPTH);
    if(!in || in->width != w || in->height != h)
      return 0;
    // rs2_deproject_pixel_to_point asserts that it is not handed a forward
    // model (src/rs.cpp:4296) and leaves it undistorted otherwise; the host's
    // own undistortion is the better answer there.
    if(in->model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY)
      return 0;

    std::lock_guard l{dev->ray_lock};
    const size_t n = size_t(w) * size_t(h) * 2;
    if(dev->rays.size() != n || std::memcmp(&dev->rays_for, &*in, sizeof(rs2_intrinsics)) != 0)
    {
      dev->rays.resize(n);
      constexpr float nan = std::numeric_limits<float>::quiet_NaN();
      for(int v = 0; v < h; v++)
        for(int u = 0; u < w; u++)
        {
          // rsutil.h:23. Pixel centres at integer coordinates, as the ABI has
          // them; at unit depth the point is the ray (x/z, y/z, 1).
          const float pixel[2]{float(u), float(v)};
          float p[3]{};
          rs2_deproject_pixel_to_point(p, &*in, pixel, 1.f);
          const bool ok = std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])
                          && p[2] > 0.f;
          float* o = dev->rays.data() + 2 * (size_t(v) * w + u);
          o[0] = ok ? p[0] / p[2] : nan;
          o[1] = ok ? p[1] / p[2] : nan;
        }
      dev->rays_for = *in;
    }
    std::memcpy(xy, dev->rays.data(), n * sizeof(float));
    return 1;
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
    return 0;
  }
  catch(...)
  {
    return 0;
  }
}

int backend_list_profiles(
    const char* uri, uint32_t stream, depthcam_profile_cb cb, void* user)
{
  if(!g_context || !cb)
    return 0;

  // The formats open() asks for, so a listed mode is one it can actually get.
  rs2_stream type{};
  int index = -1;
  rs2_format format{};
  switch(stream)
  {
    case DEPTHCAM_STREAM_DEPTH:
    case DEPTHCAM_STREAM_POINTCLOUD:
      type = RS2_STREAM_DEPTH;
      format = RS2_FORMAT_Z16;
      break;
    case DEPTHCAM_STREAM_COLOR:
      type = RS2_STREAM_COLOR;
      format = RS2_FORMAT_RGB8;
      break;
    case DEPTHCAM_STREAM_IR:
      type = RS2_STREAM_INFRARED;
      index = 1;
      format = RS2_FORMAT_Y8;
      break;
    default:
      return 0;
  }

  try
  {
    const auto addr = parse_uri(uri);
    auto devices = g_context->query_devices();
    rs2::device found;
    switch(addr.kind)
    {
      case Address::Serial:
        for(auto&& d : devices)
          if(info_of(d, RS2_CAMERA_INFO_SERIAL_NUMBER) == addr.serial)
            found = d;
        break;
      case Address::Index:
        if(addr.index >= 0 && addr.index < int(devices.size()))
          found = devices[addr.index];
        break;
      case Address::Any:
        if(devices.size() > 0)
          found = devices[0];
        break;
    }
    if(!found)
    {
      set_error("RealSense not connected");
      return 0;
    }

    // Metadata from the device list: nothing is opened.
    std::vector<std::array<int, 3>> seen;
    for(auto&& sensor : found.query_sensors())
      for(auto&& p : sensor.get_stream_profiles())
      {
        if(p.stream_type() != type || p.format() != format)
          continue;
        if(index >= 0 && p.stream_index() != index)
          continue;
        auto vp = p.as<rs2::video_stream_profile>();
        if(!vp)
          continue;
        const std::array<int, 3> key{vp.width(), vp.height(), vp.fps()};
        if(std::find(seen.begin(), seen.end(), key) != seen.end())
          continue;
        seen.push_back(key);

        depthcam_profile out{};
        out.struct_size = sizeof(out);
        out.width = key[0];
        out.height = key[1];
        out.fps = key[2];
        out.format = to_depthcam_format(format);
        cb(&out, user);
      }
    return 1;
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
    return 0;
  }
  catch(...)
  {
    return 0;
  }
}

void backend_set_stream_demand(depthcam_device* dev, uint32_t streams)
{
  // Read once per frameset in handle(); the point cloud, the filters and the
  // alignment are all skipped for what nobody consumes.
  if(dev)
    dev->demand.store(streams, std::memory_order_release);
}

depthcam_device* backend_open(const char* uri, const depthcam_open_config* config)
{
  if(!g_context || !config)
    return nullptr;

  try
  {
    const auto addr = parse_uri(uri);

    auto devices = g_context->query_devices();
    if(devices.size() == 0)
    {
      set_error("no RealSense device connected");
      return nullptr;
    }

    std::string serial;
    switch(addr.kind)
    {
      case Address::Serial: {
        for(auto&& d : devices)
          if(info_of(d, RS2_CAMERA_INFO_SERIAL_NUMBER) == addr.serial)
            serial = addr.serial;
        if(serial.empty())
        {
          // No silent fallback to another camera.
          set_error("RealSense '" + addr.serial + "' is not connected");
          return nullptr;
        }
        break;
      }
      case Address::Index:
        if(addr.index < 0 || addr.index >= int(devices.size()))
        {
          set_error("no RealSense at index " + std::to_string(addr.index));
          return nullptr;
        }
        serial = info_of(devices[addr.index], RS2_CAMERA_INFO_SERIAL_NUMBER);
        break;
      case Address::Any:
        serial = info_of(devices[0], RS2_CAMERA_INFO_SERIAL_NUMBER);
        break;
    }

    auto dev = std::make_unique<depthcam_device>();
    dev->cfg = *config;

    // Held for the controls; see depthcam_device::rsdev.
    for(auto&& d : devices)
      if(info_of(d, RS2_CAMERA_INFO_SERIAL_NUMBER) == serial)
        dev->rsdev = d;

    dev->model = info_of(dev->rsdev, RS2_CAMERA_INFO_NAME);
    dev->firmware = info_of(dev->rsdev, RS2_CAMERA_INFO_FIRMWARE_VERSION);
    try
    {
      for(auto&& s : dev->rsdev.query_sensors())
        dev->stereo = dev->stereo || s.is<rs2::depth_stereo_sensor>();
    }
    catch(...)
    {
    }

    const bool want_cloud = (config->streams & DEPTHCAM_STREAM_POINTCLOUD) != 0;
    dev->color_pointcloud = want_cloud && config->color_pointcloud != 0
                            && config->align != DEPTHCAM_ALIGN_NONE;

    dev->serial = serial;
    if(!serial.empty())
      dev->config.enable_device(serial);

    const bool need_color
        = (config->streams & DEPTHCAM_STREAM_COLOR) || dev->color_pointcloud;
    const bool need_depth = (config->streams & DEPTHCAM_STREAM_DEPTH) || want_cloud;

    // 0 means "let the backend choose"; librealsense reads 0 the same way.
    if(need_color)
      dev->config.enable_stream(
          RS2_STREAM_COLOR, config->color_width, config->color_height,
          RS2_FORMAT_RGB8, config->color_fps);

    if(need_depth)
      dev->config.enable_stream(
          RS2_STREAM_DEPTH, config->depth_width, config->depth_height,
          RS2_FORMAT_Z16, config->depth_fps);

    if(config->streams & DEPTHCAM_STREAM_IR)
    {
      // Index 1 is the left imager. 0 for the dimensions means "any", which is
      // how librealsense reads it too.
      dev->config.enable_stream(
          RS2_STREAM_INFRARED, 1, config->ir_width, config->ir_height,
          RS2_FORMAT_Y8, config->ir_fps);
    }

    if(config->streams & DEPTHCAM_STREAM_IMU)
    {
      // Same pipeline as the video, unlike the Orbbec backend: librealsense
      // delivers motion frames on their own rather than folding them into a
      // frameset, so the IMU keeps its own rate without a second pipeline.
      //
      // Tolerated rather than required -- a D435 has no IMU where a D435i does,
      // and the two are told apart only by trying.
      try
      {
        dev->config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        dev->config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        dev->want_imu = true;
      }
      catch(const std::exception& e)
      {
        set_error(std::string{"no IMU on this camera: "} + e.what());
      }
    }

    switch(config->align)
    {
      case DEPTHCAM_ALIGN_DEPTH_TO_COLOR:
        dev->align = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
        break;
      case DEPTHCAM_ALIGN_COLOR_TO_DEPTH:
        dev->align = std::make_unique<rs2::align>(RS2_STREAM_DEPTH);
        break;
      default:
        break;
    }

    // What start() will get, so that the calibration can be read now. Resolving
    // picks the profiles without streaming (rs_pipeline.hpp:314); a camera
    // that cannot satisfy the config is found out at start() as before.
    try
    {
      dev->profile = dev->config.resolve(dev->pipeline);
    }
    catch(const std::exception& e)
    {
      set_error(std::string{"stream configuration does not resolve yet: "} + e.what());
    }

    return dev.release();
  }
  catch(const rs2::error& e)
  {
    set_error(e.what());
    return nullptr;
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
    return nullptr;
  }
  catch(...)
  {
    set_error("unknown error while opening the device");
    return nullptr;
  }
}

void backend_stop(depthcam_device* dev);

void backend_close(depthcam_device* dev)
{
  if(!dev)
    return;
  backend_stop(dev);
  {
    // A notification racing the close finds nobody to call.
    std::lock_guard l{dev->notifier->lock};
    dev->notifier->cb = nullptr;
    dev->notifier->user = nullptr;
  }
  delete dev;
}

int backend_start(depthcam_device* dev, depthcam_frame_cb on_frame, void* user)
{
  if(!dev)
    return 0;
  if(dev->running.load(std::memory_order_acquire))
    return 1;

  dev->on_frame = on_frame;
  dev->user = user;
  {
    // A restart may change what is delivered.
    std::lock_guard l{dev->profile_lock};
    dev->delivered_depth = {};
    dev->delivered_color = {};
    dev->delivered_ir = {};
  }
  dev->running.store(true, std::memory_order_release);

  const auto callback = [dev](const rs2::frame& f) {
    try
    {
      if(auto fs = f.as<rs2::frameset>())
        dev->handle(fs);
      else if(auto mf = f.as<rs2::motion_frame>())
        dev->handleMotion(mf);
    }
    catch(...)
    {
      // Never let anything escape into librealsense's caller.
    }
  };

  try
  {
    rs2::pipeline_profile profile;
    try
    {
      profile = dev->pipeline.start(dev->config, callback);
    }
    catch(const rs2::error& e)
    {
      // The requested combination is unavailable. This is routine on USB2,
      // where a D400 drops to a handful of low-rate profiles (a D435i on
      // USB 2.1 offers colour only at 1080p@8 or 720p@15, and no 30fps colour
      // at all), so the defaults a USB3 camera would accept simply do not
      // exist. Let librealsense choose rather than failing outright.
      set_error(
          std::string{"requested stream configuration unavailable ("} + e.what()
          + "); falling back to the camera's own defaults");

      rs2::config fallback;
      if(!dev->serial.empty())
        fallback.enable_device(dev->serial);
      profile = dev->pipeline.start(fallback, callback);
    }

    {
      std::lock_guard l{dev->profile_lock};
      dev->profile = profile;
    }

    // get_depth_scale() is metres per unit; the ABI wants millimetres per unit.
    try
    {
      auto ds = profile.get_device().first<rs2::depth_sensor>();
      dev->depth_scale_mm = ds.get_depth_scale() * 1000.f;
    }
    catch(...)
    {
      dev->depth_scale_mm = 1.f;
    }

    return 1;
  }
  catch(const rs2::error& e)
  {
    set_error(e.what());
    dev->running.store(false, std::memory_order_release);
    return 0;
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
    dev->running.store(false, std::memory_order_release);
    return 0;
  }
}

void backend_stop(depthcam_device* dev)
{
  if(!dev)
    return;
  if(!dev->running.exchange(false, std::memory_order_acq_rel))
    return;
  try
  {
    dev->pipeline.stop();
  }
  catch(...)
  {
  }
}

uint32_t backend_active_streams(depthcam_device* dev)
{
  if(!dev)
    return 0;
  uint32_t streams = dev->cfg.streams;
  // A D435 has no IMU where a D435i does, and enable_stream is where that
  // shows up.
  if(!dev->want_imu)
    streams &= ~uint32_t(DEPTHCAM_STREAM_IMU);
  return streams;
}

const depthcam_backend_v1 g_backend{
    .abi_version = DEPTHCAM_ABI_VERSION,
    .name = "realsense",
    .display_name = "Intel RealSense",
    .last_error = &backend_last_error,
    .init = &backend_init,
    .shutdown = &backend_shutdown,
    .enumerate = &backend_enumerate,
    .set_changed_callback = &backend_set_changed_callback,
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
    .set_control_changed_callback = &backend_set_control_changed_callback,
    .list_profiles = &backend_list_profiles,
    .set_stream_demand = &backend_set_stream_demand,
};

} // namespace

extern "C" DEPTHCAM_EXPORT const depthcam_backend_v1* score_depthcam_backend_v1(void)
{
  return &g_backend;
}
