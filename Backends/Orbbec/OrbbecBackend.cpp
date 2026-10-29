/*
 * Orbbec backend: OrbbecSDK v2 behind depthcam_abi.h.
 *
 * Everything here was verified against a Femto Mega; the non-obvious parts are
 * commented where they cost time to find.
 */
#include <depthcam_abi.h>

#include <cstdlib>

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <tuple>
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

// ---------------------------------------------------------------------------

depthcam_format to_depthcam_format(OBFormat fmt)
{
  switch(fmt)
  {
    case OB_FORMAT_YUYV:
    case OB_FORMAT_YUY2:
    // ObTypes.h documents OB_FORMAT_GRAY as "the actual format is the same as
    // YUYV" despite the name.
    case OB_FORMAT_GRAY:
      return DEPTHCAM_FMT_YUYV422;
    case OB_FORMAT_UYVY:
      return DEPTHCAM_FMT_UYVY422;
    case OB_FORMAT_NV12:
      return DEPTHCAM_FMT_NV12;
    case OB_FORMAT_I420:
    // YV12 is planar YUV 4:2:0 with the chroma planes swapped, not a grey
    // format; the plane order is a layout detail the host handles.
    case OB_FORMAT_YV12:
      return DEPTHCAM_FMT_YUV420P;

    case OB_FORMAT_RGB:
      return DEPTHCAM_FMT_RGB24;
    case OB_FORMAT_BGR:
      return DEPTHCAM_FMT_BGR24;
    case OB_FORMAT_RGBA:
      return DEPTHCAM_FMT_RGBA;
    case OB_FORMAT_BGRA:
      return DEPTHCAM_FMT_BGRA;

    // BA81 is documented by Orbbec as "same as Y8, used for the right IR
    // stream", despite the name matching a Bayer fourcc.
    case OB_FORMAT_Y8:
    case OB_FORMAT_BA81:
      return DEPTHCAM_FMT_GRAY8;

    // The SDK unpacks Y10/Y11/Y12/Y14 into 16-bit containers.
    case OB_FORMAT_Y16:
    case OB_FORMAT_Y10:
    case OB_FORMAT_Y11:
    case OB_FORMAT_Y12:
    case OB_FORMAT_Y14:
    case OB_FORMAT_Z16:
    case OB_FORMAT_RLE:
    case OB_FORMAT_RVL:
    case OB_FORMAT_RW16:
      return DEPTHCAM_FMT_GRAY16;

    case OB_FORMAT_MJPG:
      return DEPTHCAM_FMT_MJPEG;
    case OB_FORMAT_H264:
      return DEPTHCAM_FMT_H264;
    case OB_FORMAT_H265:
    case OB_FORMAT_HEVC:
      return DEPTHCAM_FMT_H265;

    case OB_FORMAT_POINT:
      return DEPTHCAM_FMT_XYZ;
    case OB_FORMAT_RGB_POINT:
      return DEPTHCAM_FMT_XYZRGB;

    default:
      return DEPTHCAM_FMT_NONE;
  }
}

/**
 * Address forms accepted in the URI, mirroring what the settings widget offers:
 *   orbbec:sn:SERIAL   orbbec:uid:UID   orbbec:net:HOST[:PORT]   orbbec:
 */
struct Address
{
  enum Kind
  {
    Any,
    Serial,
    Uid,
    Network
  } kind{Any};
  std::string value;
  uint16_t port{8090};
};

Address parse_uri(const char* uri)
{
  Address a;
  if(!uri)
    return a;

  std::string s{uri};
  if(s.rfind("orbbec:", 0) == 0)
    s = s.substr(7);
  if(s.empty())
    return a;

  const auto starts = [&](const char* p) { return s.rfind(p, 0) == 0; };

  if(starts("sn:"))
  {
    a.kind = Address::Serial;
    a.value = s.substr(3);
  }
  else if(starts("uid:"))
  {
    a.kind = Address::Uid;
    a.value = s.substr(4);
  }
  else if(starts("net:"))
  {
    a.kind = Address::Network;
    a.value = s.substr(4);

    // Split on the last colon only, and only if a number follows, so a bare
    // IPv6 literal is not mangled.
    if(const auto colon = a.value.rfind(':'); colon != std::string::npos && colon > 0)
    {
      const auto tail = a.value.substr(colon + 1);
      if(!tail.empty()
         && tail.find_first_not_of("0123456789") == std::string::npos)
      {
        const auto p = std::stoul(tail);
        if(p > 0 && p <= 65535)
        {
          a.port = static_cast<uint16_t>(p);
          a.value = a.value.substr(0, colon);
        }
      }
    }
  }
  else
  {
    a.kind = Address::Serial;
    a.value = s;
  }

  if(a.value.empty())
    a.kind = Address::Any;
  return a;
}

// ---------------------------------------------------------------------------

std::unique_ptr<ob::Context> g_context;
depthcam_changed_cb g_changed_cb = nullptr;
void* g_changed_user = nullptr;

/// Strings handed to the host must outlive the enumerate callback, so they are
/// parked here rather than in a temporary.
struct EnumEntry
{
  std::string uri, name, serial, transport;
};

/**
 * The colour and depth resolution a coloured point cloud must not be given.
 *
 * The SDK's transformationDepthToRGBDPointCloud indexes the colour buffer as
 * i * colorWidth * scale with no bounds check at all; when colour and depth
 * differ it walks straight off the end. Equal dimensions are the only case
 * where its arithmetic stays in range.
 */
bool color_matches_depth(ob::FrameSet& fs)
{
  auto c = fs.colorFrame();
  auto d = fs.depthFrame();
  if(!c || !d)
    return false;
  auto cv = c->as<ob::VideoFrame>();
  auto dv = d->as<ob::VideoFrame>();
  if(!cv || !dv)
    return false;
  return cv->getWidth() == dv->getWidth() && cv->getHeight() == dv->getHeight();
}

constexpr int default_raw_color_width = 1920;
constexpr int default_raw_color_height = 1080;

// --- controls ---------------------------------------------------------------

/**
 * @brief Turn OB_PROP_COLOR_AUTO_EXPOSURE_BOOL into "color/auto_exposure".
 *
 * The SDK has no category or "advanced" metadata -- OBPropertyItem carries only
 * id, name, type and permission -- but the names are systematic and the
 * permission is genuinely informative, so a useful tree falls out of the two:
 *
 *   permission -w  a trigger with no readable value: reboot, recovery mode,
 *                  forced capture. Grouped under advanced/ so they are not one
 *                  click away from a performer mid-performance.
 *   permission r-  a sensor: temperature, power state, the exposure the camera
 *                  chose for itself. Grouped under sensors/.
 *   otherwise      grouped by the first token of the name.
 */
std::string control_id_for(const std::string& raw_name, OBPermissionType perm)
{
  std::string n = raw_name;

  // Strip the type prefix and the type suffix the SDK encodes in the name.
  for(const char* p : {"OB_PROP_", "OB_STRUCT_", "OB_RAW_DATA_"})
    if(n.rfind(p, 0) == 0)
    {
      n = n.substr(std::strlen(p));
      break;
    }
  for(const char* suffix : {"_BOOL", "_INT", "_FLOAT"})
  {
    const auto len = std::strlen(suffix);
    if(n.size() > len && n.compare(n.size() - len, len, suffix) == 0)
    {
      n.resize(n.size() - len);
      break;
    }
  }

  for(auto& c : n)
    c = char(std::tolower((unsigned char)c));

  if(n.empty())
    return {};

  const auto starts = [&](const char* p) { return n.rfind(p, 0) == 0; };

  if(perm == OB_PERMISSION_WRITE)
    return "advanced/" + n;
  if(perm == OB_PERMISSION_READ)
    return "sensors/" + n;

  if(starts("color_"))
    return "color/" + n.substr(6);
  if(starts("depth_"))
    return "depth/" + n.substr(6);
  if(starts("ir_"))
    return "ir/" + n.substr(3);
  if(starts("tof_"))
    return "depth/" + n;
  if(starts("gyro_") || starts("accel_"))
    return "imu/" + n;
  if(starts("laser") || starts("ldp"))
    return "laser/" + n;

  return "device/" + n;
}

/**
 * @brief Names for the properties whose integer values are really an enum.
 *
 * The SDK reports these through getIntPropertyRange like any other number, so
 * without this an IMU's output data rate shows up as "7" rather than "100 Hz"
 * and there is no way to tell from the tree what the range even means. The
 * values are what ObTypes.h assigns; anything the table does not cover keeps
 * its number.
 */
struct NamedValue
{
  int value;
  const char* label;
};

struct NamedProperty
{
  /// The control path, not the OBPropertyID: the ids for these live in an
  /// internal header, while the name the SDK reports is public and is what the
  /// path is derived from anyway.
  const char* path;
  const NamedValue* values;
  int count;
};

constexpr NamedValue sample_rate_names[] = {
    {1, "1.5625 Hz"}, {2, "3.125 Hz"}, {3, "6.25 Hz"},  {4, "12.5 Hz"},
    {5, "25 Hz"},     {6, "50 Hz"},    {7, "100 Hz"},   {8, "200 Hz"},
    {9, "500 Hz"},    {10, "1 kHz"},   {11, "2 kHz"},   {12, "4 kHz"},
    {13, "8 kHz"},    {14, "16 kHz"},  {15, "32 kHz"},  {16, "400 Hz"},
    {17, "800 Hz"},
};

constexpr NamedValue gyro_fs_names[] = {
    {1, "16 dps"},   {2, "31 dps"},   {3, "62 dps"},  {4, "125 dps"},
    {5, "250 dps"},  {6, "500 dps"},  {7, "1000 dps"},{8, "2000 dps"},
    {9, "400 dps"},  {10, "800 dps"},
};

constexpr NamedValue accel_fs_names[] = {
    {1, "2 g"},  {2, "4 g"},  {3, "8 g"},  {4, "16 g"},
    {5, "3 g"},  {6, "6 g"},  {7, "12 g"}, {8, "24 g"},
};

constexpr NamedProperty named_properties[] = {
    {"imu/gyro_odr", sample_rate_names, int(std::size(sample_rate_names))},
    {"imu/accel_odr", sample_rate_names, int(std::size(sample_rate_names))},
    {"imu/gyro_full_scale", gyro_fs_names, int(std::size(gyro_fs_names))},
    {"imu/accel_full_scale", accel_fs_names, int(std::size(accel_fs_names))},
};

const NamedProperty* named_property_for(const std::string& path)
{
  for(const auto& n : named_properties)
    if(path == n.path)
      return &n;
  return nullptr;
}

/**
 * Units and flags for the properties whose unit the SDK documents. Exposures
 * and gains are not here: their unit differs between models (the SDK's own
 * StartOfExposureTimestampAdjuster.cpp:38 has colour in 100us steps and depth
 * in 1us on the same camera), and the ABI has no "100us".
 */
struct PropertyMeta
{
  OBPropertyID id;
  const char* unit;
  uint32_t flags;
};

constexpr PropertyMeta property_meta[] = {
    {OB_PROP_LDP_MEASURE_DISTANCE_INT, "mm", 0},                      // Property.h:259
    {OB_PROP_TIMER_RESET_DELAY_US_INT, "us", DEPTHCAM_CONTROL_ADVANCED}, // Property.h:274
    {OB_PROP_DEVICE_REBOOT_DELAY_INT, "ms", DEPTHCAM_CONTROL_ADVANCED},  // Property.h:391
    {OB_PROP_FRAME_INTERLEAVE_LASER_PATTERN_SYNC_DELAY_INT, "us",
     DEPTHCAM_CONTROL_ADVANCED}, // Property.h:498
};

const PropertyMeta* property_meta_for(OBPropertyID id)
{
  for(const auto& m : property_meta)
    if(m.id == id)
      return &m;
  return nullptr;
}

/// "SpatialAdvancedFilter" -> "spatial_advanced", "min depth" -> "min_depth".
std::string snake_case(std::string_view s)
{
  if(s.size() > 6 && s.substr(s.size() - 6) == "Filter")
    s.remove_suffix(6);
  std::string out;
  for(std::size_t i = 0; i < s.size(); i++)
  {
    const auto c = static_cast<unsigned char>(s[i]);
    if(std::isupper(c))
    {
      if(i > 0 && !out.empty() && out.back() != '_')
        out += '_';
      out += char(std::tolower(c));
    }
    else if(std::isalnum(c))
      out += char(c);
    else if(!out.empty() && out.back() != '_')
      out += '_';
  }
  while(!out.empty() && out.back() == '_')
    out.pop_back();
  return out;
}

/// Cached descriptor plus everything needed to talk to the control again.
struct ControlEntry
{
  enum Source
  {
    Property,
    Preset,
    DepthMode,
    FilterEnabled,
    FilterParam,
  } source{Property};

  OBPropertyID id{};
  OBPropertyType type{};

  /// FilterEnabled / FilterParam only.
  std::shared_ptr<ob::Filter> filter;
  std::string param;

  std::string path;
  std::string name;
  std::string description;
  std::string unit;
  std::string auto_control;
  /// Labels for an enum, one per value in [min, max].
  std::vector<std::string> labels;
  std::vector<const char*> label_ptrs;
  depthcam_control2 desc{};
};

} // namespace

// ---------------------------------------------------------------------------

struct depthcam_device
{
  std::shared_ptr<ob::Device> device;
  std::shared_ptr<ob::Config> config;
  std::unique_ptr<ob::Pipeline> pipeline;

  /// A second pipeline, for the IMU only.
  ///
  /// Not the accelerometer and gyroscope added to the video pipeline: a
  /// frameset is emitted at the slowest enabled stream's rate, so an IMU
  /// sampling at 200Hz would be delivered at 30 and the point of having it
  /// would be gone. Orbbec's own IMU example uses a pipeline of its own for
  /// the same reason.
  std::unique_ptr<ob::Pipeline> imu_pipeline;
  std::shared_ptr<ob::Config> imu_config;
  std::shared_ptr<ob::PointCloudFilter> point_cloud;
  std::shared_ptr<ob::Align> align; // only for colour-to-depth

  depthcam_open_config cfg{};
  bool color_pointcloud{};
  std::atomic_bool running{};

  /// What build_config() got the camera to agree to.
  bool color_on{}, depth_on{}, ir_on{};
  /// Depth is aligned to colour by the device (ALIGN_D2C_HW_MODE).
  bool d2c{};
  OBSensorType ir_sensor{OB_SENSOR_IR};

  /// Which streams the host consumes; see set_stream_demand.
  std::atomic<uint32_t> demand{~0u};

  depthcam_frame_cb on_frame{};
  void* user{};

  /// The SDK's calibration for the profiles the config resolves to. Written
  /// from the host's thread only (open, start); the mutex is for the ray
  /// table, which is computed outside it.
  std::mutex calib_mutex;
  std::unique_ptr<OBCalibrationParam> calib;
  /// The last ray table, and the calibration it was computed from.
  std::vector<float> ray_table;
  OBCameraIntrinsic ray_intrinsic{};
  OBCameraDistortion ray_distortion{};

  std::string model, serial, firmware, hardware_rev;

  /// The depth sensor's recommended post-processing chain, in SDK order.
  std::vector<std::shared_ptr<ob::Filter>> filters;

  std::vector<std::string> presets;
  std::vector<std::string> depth_modes;
  /// Index into depth_modes chosen while stopped, applied by the next start().
  int pending_depth_mode{-1};

  /// Held while the change callback runs, so clearing it waits for a call in
  /// flight.
  std::mutex changed_mutex;
  depthcam_control_changed_cb changed_cb{};
  void* changed_user{};

  std::vector<ControlEntry> controls;
  bool controls_scanned{};

  void scanControls();
  void scanExtraControls();
  ControlEntry* findControl(const char* id);
  void notifyChanged(const char* id);

  std::shared_ptr<ob::Frame> filterDepth(std::shared_ptr<ob::Frame> depth);
  void deliver_video(uint32_t stream, const std::shared_ptr<ob::Frame>& f);
  void deliver_pointcloud(
      const std::shared_ptr<ob::FrameSet>& fs, const std::shared_ptr<ob::Frame>& depth);
  void handle(const std::shared_ptr<ob::FrameSet>& fs);
  void handleImu(const std::shared_ptr<ob::FrameSet>& fs);
};

namespace
{
/// Keeps an SDK frame alive for exactly as long as the host holds the buffer.
struct FrameHolder
{
  std::shared_ptr<ob::Frame> frame;
};

void release_holder(void* owner)
{
  delete static_cast<FrameHolder*>(owner);
}
} // namespace

void depthcam_device::deliver_video(
    uint32_t stream, const std::shared_ptr<ob::Frame>& f)
{
  if(!f || !on_frame)
    return;
  if(!(cfg.streams & stream))
    return;

  auto vf = f->as<ob::VideoFrame>();
  if(!vf)
    return;

  const auto fmt = to_depthcam_format(f->getFormat());
  if(fmt == DEPTHCAM_FMT_NONE)
    return;

  auto* holder = new FrameHolder{f};

  depthcam_frame out{};
  out.stream = stream;
  out.format = fmt;
  out.width = int32_t(vf->getWidth());
  out.height = int32_t(vf->getHeight());
  out.stride = 0;
  out.timestamp_ns = uint64_t(f->getTimeStampUs()) * 1000ull;
  out.data = f->getData();
  out.bytes = f->getDataSize();
  out.owner = holder;
  out.release = &release_holder;

  if(stream == DEPTHCAM_STREAM_DEPTH)
  {
    if(auto df = f->as<ob::DepthFrame>())
      out.depth_unit_mm = df->getValueScale();
  }

  on_frame(&out, user);
}

std::shared_ptr<ob::Frame> depthcam_device::filterDepth(std::shared_ptr<ob::Frame> depth)
{
  // The SDK does not check the enabled flag itself (FilterDecorator.cpp:316
  // processes unconditionally), and its own post-processing example skips the
  // disabled ones the same way. Each filter locks its own config and process
  // (FilterDecorator.cpp:166,324), so set_control can run concurrently.
  for(const auto& f : filters)
  {
    try
    {
      if(!f->isEnabled())
        continue;
      if(auto out = f->process(depth))
        depth = std::move(out);
    }
    catch(const std::exception& e)
    {
      set_error("filter " + f->getName() + ": " + e.what());
    }
    catch(...)
    {
    }
  }
  return depth;
}

void depthcam_device::deliver_pointcloud(
    const std::shared_ptr<ob::FrameSet>& fs, const std::shared_ptr<ob::Frame>& depth_frame)
{
  if(!point_cloud || !on_frame)
    return;
  if(!(cfg.streams & DEPTHCAM_STREAM_POINTCLOUD))
    return;

  if(!depth_frame)
    return;
  if(color_pointcloud && !fs->colorFrame())
    return;

  try
  {
    if(!depth_frame->is<ob::DepthFrame>())
      return;
    auto depth = depth_frame->as<ob::DepthFrame>();
    auto source = fs;

    // Colour-to-depth is a filter pass; depth-to-colour is done by the device
    // through the Config and needs nothing here.
    if(align)
    {
      auto aligned = align->process(fs);
      if(!aligned)
        return;
      source = aligned->as<ob::FrameSet>();
      if(!source)
        return;
    }

    if(color_pointcloud && !color_matches_depth(*source))
      return;

    // getValueScale() is millimetres per depth unit; the extra 1/1000 makes the
    // filter emit metres, which is what DEPTHCAM_FMT_XYZ promises. Free: the
    // filter multiplies by this factor anyway.
    point_cloud->setPositionDataScaled(depth->getValueScale() * 0.001f);

    auto pcf = point_cloud->process(source);
    if(!pcf)
      return;

    const auto bytes = pcf->getDataSize();
    if(!pcf->getData() || bytes == 0)
      return;

    const auto fmt = to_depthcam_format(pcf->getFormat());
    const auto point_bytes
        = (fmt == DEPTHCAM_FMT_XYZRGB) ? sizeof(OBColorPoint) : sizeof(OBPoint);

    auto* holder = new FrameHolder{pcf};

    depthcam_frame out{};
    out.stream = DEPTHCAM_STREAM_POINTCLOUD;
    out.format = fmt;
    out.point_count = int32_t(bytes / point_bytes);
    out.timestamp_ns = uint64_t(depth->getTimeStampUs()) * 1000ull;
    out.data = pcf->getData();
    out.bytes = bytes;
    out.owner = holder;
    out.release = &release_holder;

    on_frame(&out, user);
  }
  catch(const std::exception& e)
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

  int count = 0;
  try
  {
    count = device->getSupportedPropertyCount();
  }
  catch(...)
  {
    return;
  }

  for(int i = 0; i < count; i++)
  {
    OBPropertyItem item{};
    try
    {
      item = device->getSupportedProperty(uint32_t(i));
    }
    catch(...)
    {
      continue;
    }

    // Struct and raw properties (serial number, calibration JSON, sync and IP
    // config, device time) have no scalar form; each would need its own
    // decoding, so they are left out rather than published as something they
    // are not.
    if(item.type != OB_BOOL_PROPERTY && item.type != OB_INT_PROPERTY
       && item.type != OB_FLOAT_PROPERTY)
      continue;
    if(item.permission == OB_PERMISSION_DENY)
      continue;

    ControlEntry e;
    e.name = item.name ? item.name : "";
    e.path = control_id_for(e.name, item.permission);
    if(e.path.empty())
      continue;

    // The device can list the same property twice; the first wins.
    if(std::any_of(
           controls.begin(), controls.end(),
           [&](const ControlEntry& o) { return o.path == e.path; }))
      continue;

    e.id = item.id;
    e.type = item.type;

    auto& d = e.desc.base;
    d.access = 0;
    if(item.permission == OB_PERMISSION_READ
       || item.permission == OB_PERMISSION_READ_WRITE)
      d.access |= DEPTHCAM_ACCESS_READ;
    if(item.permission == OB_PERMISSION_WRITE
       || item.permission == OB_PERMISSION_READ_WRITE)
      d.access |= DEPTHCAM_ACCESS_WRITE;

    try
    {
      switch(item.type)
      {
        case OB_BOOL_PROPERTY: {
          // A write-only bool is a trigger, not a setting.
          if(item.permission == OB_PERMISSION_WRITE)
          {
            d.kind = DEPTHCAM_CONTROL_ACTION;
            d.min = 0;
            d.max = 1;
            d.step = 1;
            d.def = 0;
          }
          else
          {
            auto r = device->getBoolPropertyRange(item.id);
            d.kind = DEPTHCAM_CONTROL_BOOL;
            d.min = 0;
            d.max = 1;
            d.step = 1;
            d.def = r.def ? 1 : 0;
          }
          break;
        }
        case OB_INT_PROPERTY: {
          auto r = device->getIntPropertyRange(item.id);
          d.kind = DEPTHCAM_CONTROL_INT;
          d.min = r.min;
          d.max = r.max;
          d.step = r.step > 0 ? r.step : 1;
          d.def = r.def;

          // An integer whose values have names is an enum, whatever the SDK
          // calls it. One label per value in [min, max], because that is how
          // the ABI indexes them.
          if(const auto* named = named_property_for(e.path);
             named && r.max >= r.min && (r.max - r.min) < 64)
          {
            for(int v = r.min; v <= r.max; v++)
            {
              const char* label = nullptr;
              for(int k = 0; k < named->count && !label; k++)
                if(named->values[k].value == v)
                  label = named->values[k].label;
              e.labels.push_back(label ? label : std::to_string(v));
            }
            d.kind = DEPTHCAM_CONTROL_ENUM;
          }
          break;
        }
        case OB_FLOAT_PROPERTY: {
          auto r = device->getFloatPropertyRange(item.id);
          d.kind = DEPTHCAM_CONTROL_FLOAT;
          d.min = r.min;
          d.max = r.max;
          d.step = r.step > 0 ? r.step : 0;
          d.def = r.def;
          break;
        }
        default:
          continue;
      }
    }
    catch(...)
    {
      // A property the device lists but refuses to describe is not usable.
      continue;
    }

    if(const auto* meta = property_meta_for(e.id))
    {
      e.unit = meta->unit;
      e.desc.flags |= meta->flags;
    }
    // A read-only property is a sensor: it changes by itself.
    if(d.access == DEPTHCAM_ACCESS_READ)
      e.desc.flags |= DEPTHCAM_CONTROL_VOLATILE;
    // The write-only triggers control_id_for put under advanced/.
    if(e.path.rfind("advanced/", 0) == 0)
      e.desc.flags |= DEPTHCAM_CONTROL_ADVANCED;

    controls.push_back(std::move(e));
  }

  // "<group>/exposure" is chosen by the camera while "<group>/auto_exposure" is
  // on, and likewise white balance. Only those two pairs: whether gain follows
  // auto-exposure differs by model and is not stated anywhere in the SDK.
  const auto has = [&](const std::string& p) {
    return std::any_of(controls.begin(), controls.end(), [&](const ControlEntry& o) {
      return o.path == p && o.type == OB_BOOL_PROPERTY;
    });
  };
  for(auto& e : controls)
  {
    const auto slash = e.path.find('/');
    if(slash == std::string::npos)
      continue;
    const auto group = e.path.substr(0, slash);
    const auto leaf = e.path.substr(slash + 1);
    std::string toggle;
    if(leaf == "exposure")
      toggle = group + "/auto_exposure";
    else if(leaf == "white_balance")
      toggle = group + "/auto_white_balance";
    if(!toggle.empty() && has(toggle))
    {
      e.auto_control = toggle;
      e.desc.flags |= DEPTHCAM_CONTROL_VOLATILE;
    }
  }

  scanExtraControls();

  // Point the descriptors at their owner's stable strings, after the vector has
  // stopped reallocating.
  for(auto& e : controls)
  {
    e.label_ptrs.clear();
    e.label_ptrs.reserve(e.labels.size());
    for(const auto& l : e.labels)
      e.label_ptrs.push_back(l.c_str());

    auto& b = e.desc.base;
    b.id = e.path.c_str();
    b.name = e.name.c_str();
    b.description = e.description.empty() ? nullptr : e.description.c_str();
    b.enum_labels = e.label_ptrs.empty() ? nullptr : e.label_ptrs.data();
    b.enum_count = int32_t(e.label_ptrs.size());

    e.desc.struct_size = sizeof(depthcam_control2);
    e.desc.unit = e.unit.empty() ? nullptr : e.unit.c_str();
    e.desc.auto_control = e.auto_control.empty() ? nullptr : e.auto_control.c_str();
  }
}

/**
 * The controls that are not properties: the device preset, the depth work
 * mode and the depth sensor's post-processing filters.
 */
void depthcam_device::scanExtraControls()
{
  constexpr int RW = DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE;

  const auto add_enum = [&](ControlEntry::Source src, const char* path, const char* name,
                            const std::vector<std::string>& labels, uint32_t flags) {
    if(labels.empty())
      return;
    ControlEntry e;
    e.source = src;
    e.path = path;
    e.name = name;
    e.labels = labels;
    auto& d = e.desc.base;
    d.kind = DEPTHCAM_CONTROL_ENUM;
    d.access = RW;
    d.min = 0;
    d.max = double(labels.size() - 1);
    d.step = 1;
    d.def = 0;
    e.desc.flags = flags;
    controls.push_back(std::move(e));
  };

  // Loading a preset rewrites other properties at once (Device.hpp:845-850),
  // so it is not a restart; set_control reports the change instead.
  add_enum(ControlEntry::Preset, "device/preset", "Preset", presets, 0);

  // A depth work mode changes the depth profiles the sensor offers, and with
  // them the calibration, so it is applied when the pipeline is rebuilt.
  add_enum(
      ControlEntry::DepthMode, "depth/mode", "Depth mode", depth_modes,
      DEPTHCAM_CONTROL_REQUIRES_RESTART);

  std::vector<std::string> used;
  for(const auto& f : filters)
  {
    std::string fname;
    try
    {
      fname = f->getName();
    }
    catch(...)
    {
      continue;
    }
    std::string group = snake_case(fname);
    if(group.empty())
      continue;
    // Two filters of the same kind in one chain would collide.
    for(int n = 2; std::find(used.begin(), used.end(), group) != used.end(); n++)
      group = snake_case(fname) + "_" + std::to_string(n);
    used.push_back(group);

    uint32_t flags = DEPTHCAM_CONTROL_FILTER;
    // Decimation changes the delivered resolution and intrinsics; the host
    // re-reads those after a restart, not after a write.
    if(fname == "DecimationFilter")
      flags |= DEPTHCAM_CONTROL_REQUIRES_RESTART;
    // Disparity-to-depth conversion is a format step, not a quality knob:
    // turning it off on a camera that sends disparity breaks the depth stream.
    if(fname == "DisparityTransform")
      flags |= DEPTHCAM_CONTROL_ADVANCED;

    {
      ControlEntry e;
      e.source = ControlEntry::FilterEnabled;
      e.filter = f;
      e.path = "filters/" + group + "/enabled";
      e.name = fname;
      auto& d = e.desc.base;
      d.kind = DEPTHCAM_CONTROL_BOOL;
      d.access = RW;
      d.min = 0;
      d.max = 1;
      d.step = 1;
      try
      {
        d.def = f->isEnabled() ? 1 : 0;
      }
      catch(...)
      {
      }
      e.desc.flags = flags;
      controls.push_back(std::move(e));
    }

    std::vector<OBFilterConfigSchemaItem> schema;
    try
    {
      schema = f->getConfigSchemaVec(); // Filter.hpp:233, item ObTypes.h:1729-1737
    }
    catch(...)
    {
      continue;
    }
    for(const auto& item : schema)
    {
      if(!item.name)
        continue;
      const auto param = snake_case(item.name);
      if(param.empty() || param == "enabled")
        continue;

      ControlEntry e;
      e.source = ControlEntry::FilterParam;
      e.filter = f;
      e.param = item.name;
      e.path = "filters/" + group + "/" + param;
      e.name = fname + " " + item.name;
      if(item.desc)
        e.description = item.desc;
      auto& d = e.desc.base;
      switch(item.type)
      {
        case OB_FILTER_CONFIG_VALUE_TYPE_INT:
          d.kind = DEPTHCAM_CONTROL_INT;
          break;
        case OB_FILTER_CONFIG_VALUE_TYPE_FLOAT:
          d.kind = DEPTHCAM_CONTROL_FLOAT;
          break;
        case OB_FILTER_CONFIG_VALUE_TYPE_BOOLEAN:
          d.kind = DEPTHCAM_CONTROL_BOOL;
          break;
        default:
          continue;
      }
      d.access = RW;
      d.min = item.min;
      d.max = item.max;
      d.step = item.step;
      d.def = item.def;
      e.desc.flags = flags;
      if(std::any_of(controls.begin(), controls.end(), [&](const ControlEntry& o) {
           return o.path == e.path;
         }))
        continue;
      controls.push_back(std::move(e));
    }
  }
}

void depthcam_device::notifyChanged(const char* id)
{
  std::lock_guard l{changed_mutex};
  if(changed_cb)
    changed_cb(id, changed_user);
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

void depthcam_device::handleImu(const std::shared_ptr<ob::FrameSet>& fs)
{
  if(!running.load(std::memory_order_acquire) || !fs || !on_frame)
    return;
  if(!(demand.load(std::memory_order_relaxed) & DEPTHCAM_STREAM_IMU))
    return;

  depthcam_imu_sample sample{};

  try
  {
    if(auto raw = fs->getFrame(OB_FRAME_ACCEL))
    {
      if(auto accel = raw->as<ob::AccelFrame>())
      {
        // Metres per second squared since SDK 2.9; the ABI wants SI too, so
        // there is nothing to convert.
        const auto v = accel->getValue();
        sample.accel[0] = v.x;
        sample.accel[1] = v.y;
        sample.accel[2] = v.z;
        sample.fields |= DEPTHCAM_IMU_ACCEL;

        sample.temperature_c = accel->getTemperature();
        sample.fields |= DEPTHCAM_IMU_TEMPERATURE;
      }
    }
  }
  catch(...)
  {
  }

  try
  {
    if(auto raw = fs->getFrame(OB_FRAME_GYRO))
    {
      if(auto gyro = raw->as<ob::GyroFrame>())
      {
        const auto v = gyro->getValue(); // radians per second
        sample.gyro[0] = v.x;
        sample.gyro[1] = v.y;
        sample.gyro[2] = v.z;
        sample.fields |= DEPTHCAM_IMU_GYRO;

        if(!(sample.fields & DEPTHCAM_IMU_TEMPERATURE))
        {
          sample.temperature_c = gyro->getTemperature();
          sample.fields |= DEPTHCAM_IMU_TEMPERATURE;
        }
      }
    }
  }
  catch(...)
  {
  }

  if(sample.fields == 0)
    return;

  depthcam_frame out{};
  out.stream = DEPTHCAM_STREAM_IMU;
  out.format = DEPTHCAM_FMT_IMU;
  out.timestamp_ns = uint64_t(fs->getTimeStampUs()) * 1000ull;
  out.data = &sample;
  out.bytes = sizeof(sample);
  // No owner: the host copies the sample out before returning.
  on_frame(&out, user);
}

void depthcam_device::handle(const std::shared_ptr<ob::FrameSet>& fs)
{
  if(!running.load(std::memory_order_acquire) || !fs)
    return;

  const uint32_t want = cfg.streams & demand.load(std::memory_order_relaxed);

  // Each stream independently: a frameset without colour must still yield
  // depth, IR and the point cloud.
  if(want & DEPTHCAM_STREAM_COLOR)
    deliver_video(DEPTHCAM_STREAM_COLOR, fs->colorFrame());
  if(want & DEPTHCAM_STREAM_IR)
    deliver_video(DEPTHCAM_STREAM_IR, fs->irFrame());

  if(!(want & (DEPTHCAM_STREAM_DEPTH | DEPTHCAM_STREAM_POINTCLOUD)))
    return;

  std::shared_ptr<ob::Frame> raw_depth = fs->depthFrame();
  std::shared_ptr<ob::Frame> depth = raw_depth;
  if(depth && !filters.empty())
    depth = filterDepth(depth);

  if(want & DEPTHCAM_STREAM_DEPTH)
    deliver_video(DEPTHCAM_STREAM_DEPTH, depth);

  if(want & DEPTHCAM_STREAM_POINTCLOUD)
  {
    auto source = fs;
    if(depth != raw_depth)
    {
      // The cloud and the colour-to-depth pass read depth out of the frameset,
      // so the filtered frame has to be in one. A fresh set rather than
      // replacing it in the SDK's (FrameSet::pushFrame replaces a frame of the
      // same type, Frame.hpp:985-991), which other holders may still read.
      try
      {
        source = ob::FrameFactory::createFrameSet(); // Frame.hpp:1257
        if(auto c = fs->colorFrame())
          source->pushFrame(c);
        if(auto ir = fs->irFrame())
          source->pushFrame(ir);
        source->pushFrame(depth);
      }
      catch(const std::exception& e)
      {
        set_error(std::string{"point cloud: "} + e.what());
        return;
      }
      catch(...)
      {
        return;
      }
    }
    deliver_pointcloud(source, depth);
  }
}

// ---------------------------------------------------------------------------

namespace
{

const char* backend_last_error()
{
  std::lock_guard lock{g_error_mutex};
  return g_last_error.empty() ? nullptr : g_last_error.c_str();
}

int backend_init(const char* resource_dir)
{
  try
  {
    // Must precede context construction: the SDK caches the extensions path in
    // a function-local static the first time anything asks for it.
    //
    // The *extensions* directory, not the package directory. SDK 2.5 appended
    // "extensions" itself; 2.9 takes the path literally and looked for
    // <package>/frameprocessor/libob_frame_processor.so, which does not exist.
    // The only symptom was a Femto Mega producing no point cloud at all -- the
    // failure is a warning the SDK logs and swallows.
    if(resource_dir && *resource_dir)
      ob::Context::setExtensionsDirectory(
          (std::string{resource_dir} + "/extensions").c_str());

    // Errors only, unless someone is debugging.
    //
    // The SDK's logging works again since the fork moved to 2.9.3 (the previous
    // one had every LOG_ macro defined to nothing), and it is chatty: opening a
    // Femto Mega alone prints a dozen "recoverable exception" warnings from
    // component probes that are entirely normal.
    const bool debug = std::getenv("SCORE_DEPTHCAM_DEBUG") != nullptr;
    ob::Context::setLoggerToConsole(
        debug ? OB_LOG_SEVERITY_DEBUG : OB_LOG_SEVERITY_ERROR);

    // And no log file. The SDK otherwise creates Log/OrbbecSDK.log.txt in the
    // process's working directory -- which for score is wherever the user
    // happened to launch it from -- and rotates 100MB of it.
    // "" rather than nullptr: the SDK builds a std::string from the argument
    // before looking at the severity, so a null directory throws.
    ob::Context::setLoggerToFile(OB_LOG_SEVERITY_OFF, "");

    g_context = std::make_unique<ob::Context>();

    try
    {
      // Off by default; without it a Femto Mega on the network is only
      // reachable by explicitly typing its address.
      g_context->enableNetDeviceEnumeration(true);
    }
    catch(const std::exception& e)
    {
      set_error(std::string{"network enumeration unavailable: "} + e.what());
    }

    g_context->setDeviceChangedCallback(
        [](std::shared_ptr<ob::DeviceList>, std::shared_ptr<ob::DeviceList>) {
      if(g_changed_cb)
        g_changed_cb(g_changed_user);
    });

    return 1;
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
    return 0;
  }
  catch(...)
  {
    set_error("unknown error during init");
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
    auto list = g_context->queryDeviceList();
    if(!list)
      return 0;

    const auto str = [](const char* s) { return s ? std::string{s} : std::string{}; };

    const uint32_t n = list->getCount();
    for(uint32_t i = 0; i < n; i++)
    {
      EnumEntry e;
      e.name = strip_vendor(str(list->getName(i)), "Orbbec ");
      e.serial = str(list->getSerialNumber(i));
      e.transport = str(list->getConnectionType(i));

      const bool network = e.transport == "Ethernet";
      if(network)
      {
        // Reachable by address whether or not it is currently broadcasting.
        const auto ip = str(list->getIpAddress(i));
        e.uri = ip.empty() ? ("orbbec:uid:" + str(list->getUid(i)))
                           : ("orbbec:net:" + ip + ":8090");
      }
      else if(!e.serial.empty())
      {
        e.uri = "orbbec:sn:" + e.serial;
      }
      else
      {
        e.uri = "orbbec:uid:" + str(list->getUid(i));
      }

      depthcam_device_info info{};
      info.backend = "orbbec";
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

void backend_set_changed_callback(depthcam_changed_cb cb, void* user)
{
  g_changed_cb = cb;
  g_changed_user = user;
}

bool has_sensor(ob::SensorList& sensors, OBSensorType type)
{
  for(uint32_t i = 0, n = sensors.getCount(); i < n; i++)
    if(sensors.getSensorType(i) == type)
      return true;
  return false;
}

/// Tolerates a device that advertises a sensor but rejects the requested
/// profile: a camera that cannot do 1080p colour should still give us depth.
bool try_enable(
    ob::Config& config, ob::SensorList& sensors, OBSensorType type, int w, int h,
    int fps, OBFormat format = OB_FORMAT_ANY)
{
  if(!has_sensor(sensors, type))
    return false;
  try
  {
    config.enableVideoStream(
        type, w > 0 ? uint32_t(w) : OB_WIDTH_ANY, h > 0 ? uint32_t(h) : OB_HEIGHT_ANY,
        fps > 0 ? uint32_t(fps) : OB_FPS_ANY, format);
    return true;
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
    return false;
  }
}

std::shared_ptr<ob::Device>
resolve_device(const Address& addr)
{
  if(addr.kind == Address::Network)
    return g_context->createNetDevice(addr.value.c_str(), addr.port);

  auto list = g_context->queryDeviceList();
  if(!list)
    throw std::runtime_error("could not query the device list");

  const uint32_t count = list->getCount();
  if(count == 0)
    throw std::runtime_error("no Orbbec device connected");

  if(addr.kind == Address::Any)
    return list->getDevice(0);

  for(uint32_t i = 0; i < count; i++)
  {
    const char* s = (addr.kind == Address::Serial) ? list->getSerialNumber(i)
                                                   : list->getUid(i);
    if(s && addr.value == s)
      return list->getDevice(i);
  }

  // Deliberately no fallback to "any camera": streaming from a different one
  // than the document names is worse than failing to connect.
  throw std::runtime_error("device '" + addr.value + "' is not connected");
}

/**
 * Builds dev.config from dev.cfg, the part of open() a depth mode switch has
 * to redo. any_depth_size drops the requested depth resolution, for a mode
 * that does not offer it.
 */
void build_config(depthcam_device& dev, bool any_depth_size)
{
  const depthcam_open_config* config = &dev.cfg;

  // A coloured cloud requires alignment; see color_matches_depth.
  dev.color_pointcloud = (config->streams & DEPTHCAM_STREAM_POINTCLOUD)
                         && config->color_pointcloud != 0
                         && config->align != DEPTHCAM_ALIGN_NONE;
  dev.color_on = dev.depth_on = dev.ir_on = dev.d2c = false;

  auto sensors = dev.device->getSensorList();
  dev.config = std::make_shared<ob::Config>();

  const bool want_cloud = (config->streams & DEPTHCAM_STREAM_POINTCLOUD) != 0;
  const bool need_color
      = (config->streams & DEPTHCAM_STREAM_COLOR) || dev.color_pointcloud;
  const bool need_depth = (config->streams & DEPTHCAM_STREAM_DEPTH) || want_cloud;

  // Raw RGB is needed whenever anything is going to *process* the colour
  // frame rather than just hand it over.
  //
  // The camera negotiates MJPG by default and neither ob::Align nor the
  // point-cloud filter will decode it: the align pass logs "Unsupported
  // format for C2D conversion yet!" once per frame and produces nothing,
  // which is a console full of errors and a dead depth stream. Any alignment
  // is enough to need it -- not just a coloured cloud, which is what this
  // used to check.
  const bool need_raw_color
      = dev.color_pointcloud || config->align != DEPTHCAM_ALIGN_NONE;

  if(sensors && need_color)
  {
    int cw = config->color_width, ch = config->color_height;

    if(need_raw_color)
    {
      // Pinning the resolution matters: an explicit format with
      // OB_WIDTH_ANY makes the SDK select the *largest* matching profile,
      // which is 3840x2160 here -- 8.3M points and ~199MB per frame.
      if(cw <= 0 && ch <= 0)
      {
        cw = default_raw_color_width;
        ch = default_raw_color_height;
      }

      dev.color_on = try_enable(
          *dev.config, *sensors, OB_SENSOR_COLOR, cw, ch, config->color_fps,
          OB_FORMAT_RGB);
      if(!dev.color_on)
      {
        set_error(
            "raw RGB colour unavailable; alignment and the coloured point "
            "cloud will not work on this camera");
        dev.color_pointcloud = false;
        dev.color_on = try_enable(
            *dev.config, *sensors, OB_SENSOR_COLOR, config->color_width,
            config->color_height, config->color_fps);
      }
    }
    else
    {
      dev.color_on = try_enable(
          *dev.config, *sensors, OB_SENSOR_COLOR, cw, ch, config->color_fps);
    }
  }

  if(sensors && need_depth)
    dev.depth_on = try_enable(
        *dev.config, *sensors, OB_SENSOR_DEPTH, any_depth_size ? 0 : config->depth_width,
        any_depth_size ? 0 : config->depth_height, config->depth_fps);

  if(sensors && (config->streams & DEPTHCAM_STREAM_IR))
  {
    // Stereo devices (Gemini 330 family) expose left/right rather than a
    // single IR sensor.
    const auto sensor = has_sensor(*sensors, OB_SENSOR_IR) ? OB_SENSOR_IR
                                                           : OB_SENSOR_IR_LEFT;
    dev.ir_sensor = sensor;
    dev.ir_on = try_enable(
        *dev.config, *sensors, sensor, config->ir_width, config->ir_height,
        config->ir_fps);
    if(!dev.ir_on)
    {
      // A requested profile the camera does not have should not cost the
      // stream: fall back to whatever it does offer.
      dev.ir_on = try_enable(*dev.config, *sensors, sensor, 0, 0, 0);
    }
  }

  if(need_color && config->align == DEPTHCAM_ALIGN_DEPTH_TO_COLOR)
  {
    // setAlignMode only records the mode (Config.cpp:76-78 in the SDK
    // sources); hardware D2C is checked when the pipeline starts. The SDK's
    // pipeline does not implement ALIGN_D2C_SW_MODE at all
    // (Pipeline::configAlignMode is empty), so there is no software fallback
    // to try here.
    try
    {
      dev.config->setAlignMode(ALIGN_D2C_HW_MODE);
      dev.d2c = dev.color_on && dev.depth_on;
    }
    catch(const std::exception& e)
    {
      set_error(std::string{"no D2C alignment available: "} + e.what());
    }
  }
}

/// Point format and colour normalisation, which follow build_config's
/// decision about the coloured cloud.
void configure_point_cloud(depthcam_device& dev)
{
  if(!dev.point_cloud)
    return;
  dev.point_cloud->setCreatePointFormat(
      dev.color_pointcloud ? OB_FORMAT_RGB_POINT : OB_FORMAT_POINT);
  // Without this the filter emits r/g/b in 0-255 (its documented default),
  // and every channel clamps to white downstream.
  dev.point_cloud->setColorDataNormalization(dev.color_pointcloud);
}

/**
 * Re-reads the calibration of the profiles dev.config resolves to.
 *
 * Pipeline::getCalibrationParam (Pipeline.hpp:516) matches the config against
 * the sensors' profiles exactly as start() will (Pipeline.cpp:516-523 calls the
 * same checkAndSetConfig), and returns every camera's intrinsics and
 * distortion and every sensor pair's extrinsics, IMU included -- without
 * streaming. It throws when the config matches no profile.
 */
bool refresh_calibration(depthcam_device& dev)
{
  std::unique_ptr<OBCalibrationParam> c;
  try
  {
    c = std::make_unique<OBCalibrationParam>(
        dev.pipeline->getCalibrationParam(dev.config));
  }
  catch(const std::exception& e)
  {
    set_error(std::string{"calibration: "} + e.what());
  }
  catch(...)
  {
  }
  std::lock_guard l{dev.calib_mutex};
  dev.calib = std::move(c);
  return dev.calib != nullptr;
}

void read_details(depthcam_device& dev)
{
  try
  {
    auto info = dev.device->getDeviceInfo(); // Device.hpp:99
    const auto str = [](const char* s) { return s ? std::string{s} : std::string{}; };
    dev.model = strip_vendor(str(info->getName()), "Orbbec ");
    dev.serial = str(info->getSerialNumber());
    dev.firmware = str(info->getFirmwareVersion());
    dev.hardware_rev = str(info->getHardwareVersion());
  }
  catch(...)
  {
  }
}

/// Presets, depth work modes and the recommended filters, once per open.
void read_modes_and_filters(depthcam_device& dev)
{
  try
  {
    auto list = dev.device->getAvailablePresetList(); // Device.hpp:863
    for(uint32_t i = 0, n = list->getCount(); i < n; i++)
      if(const char* name = list->getName(i))
        dev.presets.emplace_back(name);
  }
  catch(...)
  {
    // Not every model has presets; the SDK throws rather than return none.
  }

  try
  {
    auto list = dev.device->getDepthWorkModeList(); // Device.hpp:580
    for(uint32_t i = 0, n = list->getCount(); i < n; i++)
    {
      const auto mode = list->getOBDepthWorkMode(i);
      // OBDepthWorkMode::name is a fixed char[32] (ObTypes.h:1145).
      dev.depth_modes.emplace_back(mode.name, strnlen(mode.name, sizeof(mode.name)));
    }
  }
  catch(...)
  {
  }

  try
  {
    // The device caches this list per sensor (DeviceBase.cpp:482-504 in the
    // SDK sources), so these are the same objects a preset reconfigures.
    if(auto sensors = dev.device->getSensorList(); sensors && has_sensor(*sensors, OB_SENSOR_DEPTH))
      dev.filters = sensors->getSensor(OB_SENSOR_DEPTH)->createRecommendedFilters(); // Sensor.hpp:93
  }
  catch(const std::exception& e)
  {
    set_error(std::string{"no recommended filters: "} + e.what());
  }
  catch(...)
  {
  }
}

depthcam_device* backend_open(const char* uri, const depthcam_open_config* config)
{
  if(!g_context || !config)
    return nullptr;

  try
  {
    auto dev = std::make_unique<depthcam_device>();
    dev->cfg = *config;
    dev->device = resolve_device(parse_uri(uri));
    if(!dev->device)
      return nullptr;

    build_config(*dev, false);

    // Bound to *this* device: the no-argument ob::Pipeline constructor silently
    // takes the first device the SDK enumerates.
    dev->pipeline = std::make_unique<ob::Pipeline>(dev->device);

    if(config->streams & DEPTHCAM_STREAM_IMU)
    {
      // Tolerated rather than required: several Orbbec models have no IMU, and
      // a camera without one should still stream video.
      try
      {
        dev->imu_pipeline = std::make_unique<ob::Pipeline>(dev->device);
        dev->imu_config = std::make_shared<ob::Config>();
        dev->imu_config->enableAccelStream();
        dev->imu_config->enableGyroStream();
      }
      catch(const std::exception& e)
      {
        set_error(std::string{"no IMU on this camera: "} + e.what());
        dev->imu_pipeline.reset();
        dev->imu_config.reset();
      }
    }

    if(config->align == DEPTHCAM_ALIGN_COLOR_TO_DEPTH)
      dev->align = std::make_shared<ob::Align>(OB_STREAM_DEPTH);

    if(config->streams & DEPTHCAM_STREAM_POINTCLOUD)
    {
      dev->point_cloud = std::make_shared<ob::PointCloudFilter>();
      configure_point_cloud(*dev);
    }

    read_details(*dev);
    read_modes_and_filters(*dev);
    refresh_calibration(*dev);

    return dev.release();
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

/// The depth/mode chosen while stopped: switched now, before the pipeline is
/// (re)built from a config that matches the new mode's profiles.
void apply_depth_mode(depthcam_device& dev)
{
  const int idx = std::exchange(dev.pending_depth_mode, -1);
  if(idx < 0 || idx >= int(dev.depth_modes.size()))
    return;
  const auto& name = dev.depth_modes[std::size_t(idx)];
  try
  {
    const auto current = dev.device->getCurrentDepthWorkMode(); // Device.hpp:531
    if(name == std::string(current.name, strnlen(current.name, sizeof(current.name))))
      return;
    if(dev.device->switchDepthWorkMode(name.c_str()) != OB_STATUS_OK) // Device.hpp:568
    {
      set_error("could not switch to depth mode " + name);
      return;
    }
  }
  catch(const std::exception& e)
  {
    set_error("depth mode " + name + ": " + e.what());
    return;
  }
  catch(...)
  {
    return;
  }

  // The new mode may not offer the depth resolution the host asked for in the
  // old one; then the stream is kept with whatever the mode does offer rather
  // than lost.
  try
  {
    build_config(dev, false);
    configure_point_cloud(dev);
    if(!refresh_calibration(dev))
    {
      build_config(dev, true);
      configure_point_cloud(dev);
      refresh_calibration(dev);
    }
  }
  catch(const std::exception& e)
  {
    set_error("depth mode " + name + ": " + e.what());
  }
  catch(...)
  {
  }
}

void backend_close(depthcam_device* dev)
{
  if(!dev)
    return;
  backend_stop(dev);
  delete dev;
}

int backend_start(depthcam_device* dev, depthcam_frame_cb on_frame, void* user)
{
  if(!dev || !dev->pipeline)
    return 0;
  if(dev->running.load(std::memory_order_acquire))
    return 1;

  if(dev->pending_depth_mode >= 0)
    apply_depth_mode(*dev);

  dev->on_frame = on_frame;
  dev->user = user;
  dev->running.store(true, std::memory_order_release);

  try
  {
    dev->pipeline->start(dev->config, [dev](std::shared_ptr<ob::FrameSet> fs) {
      dev->handle(fs);
    });

    if(dev->imu_pipeline)
    {
      // Its own try: a camera whose IMU refuses to start still has video, and
      // that is the more important of the two.
      try
      {
        dev->imu_pipeline->start(
            dev->imu_config,
            [dev](std::shared_ptr<ob::FrameSet> fs) { dev->handleImu(fs); });
      }
      catch(const std::exception& e)
      {
        set_error(std::string{"could not start the IMU: "} + e.what());
        dev->imu_pipeline.reset();
      }
    }
    return 1;
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
    dev->running.store(false, std::memory_order_release);
    return 0;
  }
  catch(...)
  {
    dev->running.store(false, std::memory_order_release);
    return 0;
  }
}

void backend_stop(depthcam_device* dev)
{
  if(!dev)
    return;
  // The SDK throws when stopping a pipeline that was never started, and close()
  // reaches this unconditionally.
  if(!dev->running.exchange(false, std::memory_order_acq_rel))
    return;
  if(dev->imu_pipeline)
  {
    try
    {
      dev->imu_pipeline->stop();
    }
    catch(...)
    {
    }
  }

  try
  {
    dev->pipeline->stop();
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
  }
  catch(...)
  {
  }
}

int backend_list_controls(depthcam_device* dev, depthcam_control_cb cb, void* user)
{
  if(!dev || !cb || !dev->device)
    return 0;
  dev->scanControls();
  for(const auto& e : dev->controls)
    cb(&e.desc.base, user);
  return 1;
}

int backend_list_controls2(depthcam_device* dev, depthcam_control2_cb cb, void* user)
{
  if(!dev || !cb || !dev->device)
    return 0;
  dev->scanControls();
  for(const auto& e : dev->controls)
    cb(&e.desc, user);
  return 1;
}

int index_of(const std::vector<std::string>& v, const char* s)
{
  if(!s)
    return -1;
  const auto it = std::find(v.begin(), v.end(), s);
  return it == v.end() ? -1 : int(it - v.begin());
}

int backend_get_control(depthcam_device* dev, const char* id, double* out)
{
  if(!dev || !out || !dev->device)
    return 0;
  dev->scanControls();

  auto* e = dev->findControl(id);
  if(!e || !(e->desc.base.access & DEPTHCAM_ACCESS_READ))
    return 0;

  try
  {
    switch(e->source)
    {
      case ControlEntry::Preset: {
        const int i = index_of(e->labels, dev->device->getCurrentPresetName());
        if(i < 0)
          return 0;
        *out = i;
        return 1;
      }
      case ControlEntry::DepthMode: {
        if(dev->pending_depth_mode >= 0)
        {
          *out = dev->pending_depth_mode;
          return 1;
        }
        const auto mode = dev->device->getCurrentDepthWorkMode();
        const std::string name(mode.name, strnlen(mode.name, sizeof(mode.name)));
        const int i = index_of(e->labels, name.c_str());
        if(i < 0)
          return 0;
        *out = i;
        return 1;
      }
      case ControlEntry::FilterEnabled:
        *out = e->filter->isEnabled() ? 1.0 : 0.0;
        return 1;
      case ControlEntry::FilterParam:
        // Already a double whatever the item's type (Filter.hpp:252-262).
        *out = e->filter->getConfigValue(e->param);
        return 1;
      case ControlEntry::Property:
        break;
    }

    switch(e->type)
    {
      case OB_BOOL_PROPERTY:
        *out = dev->device->getBoolProperty(e->id) ? 1.0 : 0.0;
        return 1;
      case OB_INT_PROPERTY:
        *out = double(dev->device->getIntProperty(e->id));
        return 1;
      case OB_FLOAT_PROPERTY:
        *out = double(dev->device->getFloatProperty(e->id));
        return 1;
      default:
        return 0;
    }
  }
  catch(const std::exception& ex)
  {
    set_error(std::string{"get "} + id + ": " + ex.what());
    return 0;
  }
  catch(...)
  {
    return 0;
  }
}

int backend_set_control(depthcam_device* dev, const char* id, double value)
{
  if(!dev || !dev->device)
    return 0;
  dev->scanControls();

  auto* e = dev->findControl(id);
  if(!e || !(e->desc.base.access & DEPTHCAM_ACCESS_WRITE))
    return 0;

  const auto index = [&]() -> int {
    const int n = int(e->labels.size());
    if(n == 0 || !std::isfinite(value))
      return -1;
    return std::clamp(int(std::lround(value)), 0, n - 1);
  };

  try
  {
    switch(e->source)
    {
      case ControlEntry::Preset: {
        const int i = index();
        if(i < 0)
          return 0;
        dev->device->loadPreset(e->labels[std::size_t(i)].c_str()); // Device.hpp:850
        // "it is recommended to re-read the device settings" (Device.hpp:846):
        // a preset rewrites properties and filter settings alike.
        dev->notifyChanged(nullptr);
        return 1;
      }
      case ControlEntry::DepthMode: {
        const int i = index();
        if(i < 0)
          return 0;
        dev->pending_depth_mode = i;
        return 1;
      }
      case ControlEntry::FilterEnabled:
        e->filter->enable(value != 0.0);
        return 1;
      case ControlEntry::FilterParam:
        // Cast to the item's own type by the filter (Filter.hpp:238-250).
        e->filter->setConfigValue(e->param, value);
        return 1;
      case ControlEntry::Property:
        break;
    }

    switch(e->type)
    {
      case OB_BOOL_PROPERTY:
        dev->device->setBoolProperty(e->id, value != 0.0);
        return 1;
      case OB_INT_PROPERTY:
        dev->device->setIntProperty(e->id, int32_t(value));
        return 1;
      case OB_FLOAT_PROPERTY:
        dev->device->setFloatProperty(e->id, float(value));
        return 1;
      default:
        return 0;
    }
  }
  catch(const std::exception& ex)
  {
    set_error(std::string{"set "} + id + ": " + ex.what());
    return 0;
  }
  catch(...)
  {
    return 0;
  }
}

uint32_t backend_active_streams(depthcam_device* dev)
{
  if(!dev)
    return 0;
  uint32_t streams = dev->cfg.streams;
  // The IMU is the only one that can be asked for and quietly not granted:
  // several models have none, and the pipeline is only created when one does.
  if(!dev->imu_pipeline)
    streams &= ~uint32_t(DEPTHCAM_STREAM_IMU);
  return streams;
}

// --- calibration -------------------------------------------------------------

/**
 * The depth stream's intrinsics as delivered, from dev.calib. Called with
 * calib_mutex held.
 *
 * Aligned to colour, depth is in the colour camera's image: the hardware D2C
 * pass is given the colour profile's intrinsics as its target
 * (FrameProcessor.cpp:262 in the SDK sources). Whether it also leaves the
 * colour distortion in place happens in a closed plug-in and cannot be
 * checked; the colour camera's is reported, since the pixels are colour pixels.
 *
 * An enabled decimation filter divides the resolution and the focal lengths
 * and centre by its factor, padding the size to a multiple of 4, and keeps the
 * distortion; this is the SDK's own arithmetic (DecimationProcess.cpp:338-364),
 * which the frames it outputs carry.
 */
bool delivered_depth(
    const depthcam_device& dev, OBCameraIntrinsic& in, OBCameraDistortion& dist)
{
  if(!dev.calib || !dev.depth_on)
    return false;
  const auto s = dev.d2c ? OB_SENSOR_COLOR : OB_SENSOR_DEPTH;
  in = dev.calib->intrinsics[s];
  dist = dev.calib->distortion[s];
  if(in.width <= 0 || in.height <= 0 || !(in.fx > 0.f) || !(in.fy > 0.f))
    return false;

  for(const auto& f : dev.filters)
  {
    try
    {
      if(f->getName() != "DecimationFilter" || !f->isEnabled())
        continue;
      // "decimate, int, 1, 8, 1, 2" (DecimationProcess.cpp:247).
      const int k = std::clamp(int(std::lround(f->getConfigValue("decimate"))), 1, 8);
      const int rw = in.width / k, rh = in.height / k;
      in.width = int16_t((rw + 3) / 4 * 4);
      in.height = int16_t((rh + 3) / 4 * 4);
      in.fx /= float(k);
      in.fy /= float(k);
      in.cx /= float(k);
      in.cy /= float(k);
    }
    catch(...)
    {
    }
  }
  return true;
}

depthcam_distortion to_depthcam_distortion(const OBCameraDistortion& d)
{
  switch(d.model)
  {
    case OB_DISTORTION_MODIFIED_BROWN_CONRADY:
      return DEPTHCAM_DISTORTION_MODIFIED_BROWN_CONRADY;
    case OB_DISTORTION_INVERSE_BROWN_CONRADY:
      return DEPTHCAM_DISTORTION_INVERSE_BROWN_CONRADY;
    case OB_DISTORTION_BROWN_CONRADY:
    case OB_DISTORTION_BROWN_CONRADY_K6:
      return DEPTHCAM_DISTORTION_BROWN_CONRADY;
    case OB_DISTORTION_KANNALA_BRANDT4:
      return DEPTHCAM_DISTORTION_KANNALA_BRANDT4;
    case OB_DISTORTION_NONE:
    default: {
      // The SDK's own unprojection ignores the tag and applies the rational
      // Brown-Conrady polynomial to whatever coefficients there are
      // (CoordinateUtil.cpp:52-78, which its point cloud uses), so a profile
      // tagged NONE with coefficients is undistorted all the same.
      const float c[] = {d.k1, d.k2, d.k3, d.k4, d.k5, d.k6, d.p1, d.p2};
      for(float v : c)
        if(v != 0.f)
          return DEPTHCAM_DISTORTION_BROWN_CONRADY;
      return DEPTHCAM_DISTORTION_NONE;
    }
  }
}

void fill_intrinsics(
    const OBCameraIntrinsic& in, const OBCameraDistortion& d, depthcam_intrinsics* out)
{
  depthcam_intrinsics r{};
  r.struct_size = out->struct_size;
  r.width = in.width;
  r.height = in.height;
  // Already OpenCV's convention: the SDK unprojects pixel (u, v) as
  // (u - cx) / fx with integer u at the pixel centre (CoordinateUtil.cpp:44-45,
  // 477-483), so no half-pixel shift.
  r.fx = in.fx;
  r.fy = in.fy;
  r.cx = in.cx;
  r.cy = in.cy;
  r.model = to_depthcam_distortion(d);
  if(r.model == DEPTHCAM_DISTORTION_KANNALA_BRANDT4)
  {
    r.coeffs[0] = d.k1;
    r.coeffs[1] = d.k2;
    r.coeffs[2] = d.k3;
    r.coeffs[3] = d.k4;
  }
  else
  {
    // OBCameraDistortion is k1..k6 then p1, p2 (ObTypes.h:497-507): the
    // ABI's own order, k4..k6 being the rational denominator as in OpenCV
    // (CoordinateUtil.cpp:60,76).
    r.coeffs[0] = d.k1;
    r.coeffs[1] = d.k2;
    r.coeffs[2] = d.k3;
    r.coeffs[3] = d.k4;
    r.coeffs[4] = d.k5;
    r.coeffs[5] = d.k6;
    r.coeffs[6] = d.p1;
    r.coeffs[7] = d.p2;
  }
  std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
}

int backend_get_intrinsics(depthcam_device* dev, uint32_t stream, depthcam_intrinsics* out)
{
  if(!dev || !out)
    return 0;
  try
  {
    OBCameraIntrinsic in{};
    OBCameraDistortion dist{};
    {
      std::lock_guard l{dev->calib_mutex};
      if(!dev->calib)
        return 0;
      switch(stream)
      {
        case DEPTHCAM_STREAM_DEPTH:
        case DEPTHCAM_STREAM_POINTCLOUD:
          if(!delivered_depth(*dev, in, dist))
            return 0;
          break;
        case DEPTHCAM_STREAM_COLOR:
          if(!dev->color_on)
            return 0;
          in = dev->calib->intrinsics[OB_SENSOR_COLOR];
          dist = dev->calib->distortion[OB_SENSOR_COLOR];
          break;
        case DEPTHCAM_STREAM_IR:
          if(!dev->ir_on)
            return 0;
          in = dev->calib->intrinsics[dev->ir_sensor];
          dist = dev->calib->distortion[dev->ir_sensor];
          break;
        default:
          return 0;
      }
    }
    if(in.width <= 0 || in.height <= 0 || !(in.fx > 0.f) || !(in.fy > 0.f))
      return 0;
    fill_intrinsics(in, dist, out);
    return 1;
  }
  catch(...)
  {
    return 0;
  }
}

/// The SDK sensor whose camera frame a stream is expressed in.
bool frame_sensor(const depthcam_device& dev, uint32_t stream, OBSensorType& s)
{
  switch(stream)
  {
    case DEPTHCAM_STREAM_DEPTH:
    case DEPTHCAM_STREAM_POINTCLOUD:
      // Aligned to colour, depth (and the cloud made from it) is in the colour
      // camera's frame.
      s = dev.d2c ? OB_SENSOR_COLOR : OB_SENSOR_DEPTH;
      return true;
    case DEPTHCAM_STREAM_COLOR:
      s = OB_SENSOR_COLOR;
      return true;
    case DEPTHCAM_STREAM_IR:
      s = dev.ir_sensor;
      return true;
    case DEPTHCAM_STREAM_IMU:
      // The accelerometer's frame. The SDK calibrates the gyroscope
      // separately; on the cameras it supports both share one IMU die, but
      // that is not something the headers state.
      s = OB_SENSOR_ACCEL;
      return true;
    default:
      return false;
  }
}

int backend_get_extrinsics(
    depthcam_device* dev, uint32_t from, uint32_t to, depthcam_extrinsics* out)
{
  if(!dev || !out)
    return 0;
  try
  {
    OBSensorType a{}, b{};
    if(!frame_sensor(*dev, from, a) || !frame_sensor(*dev, to, b))
      return 0;

    // getCalibrationParam leaves the identity for a pair it has no sensor for
    // (Pipeline.cpp:528-535), which would read as "co-located".
    auto sensors = dev->device->getSensorList();
    if(!sensors || !has_sensor(*sensors, a) || !has_sensor(*sensors, b))
      return 0;

    OBExtrinsic x{};
    {
      std::lock_guard l{dev->calib_mutex};
      if(!dev->calib)
        return 0;
      if(a == b)
      {
        x.rot[0] = x.rot[4] = x.rot[8] = 1.f;
      }
      else
      {
        // extrinsics[source][target] (ObTypes.h:548-550).
        x = dev->calib->extrinsics[a][b];
      }
    }

    depthcam_extrinsics r{};
    r.struct_size = out->struct_size;
    // Row-major: the SDK applies it as rot[0..2] . p for x
    // (CoordinateUtil.cpp:195-197).
    for(int i = 0; i < 9; i++)
      r.rotation[i] = x.rot[i];
    // Millimetres (ObTypes.h:518-522).
    for(int i = 0; i < 3; i++)
      r.translation[i] = x.trans[i] * 0.001f;
    std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
    return 1;
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
  r.hardware_rev = dev->hardware_rev.c_str();
  // Unknown. OB_PROP_MIN_DEPTH_INT / OB_PROP_MAX_DEPTH_INT are thresholds the
  // user sets, without a documented unit (Property.h:87-95), not the mode's
  // range, and the depth work modes carry only a name (ObTypes.h:1136-1151).
  r.depth_min_m = 0.f;
  r.depth_max_m = 0.f;
  std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
  return 1;
}

/**
 * The SDK's own rays: the x/y tables its point cloud filter multiplies depth by
 * (PointCloudProcess.cpp:173-185 in the SDK sources), built by the same
 * function, CoordinateUtil::transformationInitXYTables, through its public
 * wrapper (Utils.hpp:148). That undistorts iteratively, which the host's
 * closed-form model does not reproduce exactly. transformation2dto3d
 * (Utils.hpp:49) is not used: its public form takes no distortion and is the
 * plain pinhole (CoordinateUtil.cpp:206-231).
 */
int backend_get_ray_table(
    depthcam_device* dev, uint32_t stream, float* xy, int32_t w, int32_t h)
{
  if(!dev || !xy || w <= 0 || h <= 0)
    return 0;
  if(stream != DEPTHCAM_STREAM_DEPTH && stream != DEPTHCAM_STREAM_POINTCLOUD)
    return 0;

  try
  {
    const std::size_t n = std::size_t(w) * std::size_t(h);
    OBCameraIntrinsic in{};
    OBCameraDistortion dist{};
    {
      std::lock_guard l{dev->calib_mutex};
      if(!delivered_depth(*dev, in, dist))
        return 0;
      if(in.width != w || in.height != h)
        return 0;
      if(dev->ray_table.size() == 2 * n
         && std::memcmp(&dev->ray_intrinsic, &in, sizeof(in)) == 0
         && std::memcmp(&dev->ray_distortion, &dist, sizeof(dist)) == 0)
      {
        std::copy(dev->ray_table.begin(), dev->ray_table.end(), xy);
        return 1;
      }
    }

    // Outside the lock: at 1280x720 with up to 20 iterations a pixel this is
    // not instant.
    auto param = std::make_unique<OBCalibrationParam>();
    param->intrinsics[OB_SENSOR_DEPTH] = in;
    param->distortion[OB_SENSOR_DEPTH] = dist;

    std::vector<float> planes(2 * n);
    uint32_t size = uint32_t(planes.size());
    OBXYTables tables{};
    if(!ob::CoordinateTransformHelper::transformationInitXYTables(
           *param, OB_SENSOR_DEPTH, planes.data(), &size, &tables))
      return 0;
    if(tables.width != w || tables.height != h || !tables.xTable || !tables.yTable)
      return 0;

    // Two planes, x then y; an x of NaN marks a pixel with no ray, with y set
    // to 0 (CoordinateUtil.cpp:483-488).
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    std::vector<float> table(2 * n);
    for(std::size_t i = 0; i < n; i++)
    {
      const float x = tables.xTable[i];
      const bool valid = !std::isnan(x);
      table[2 * i] = valid ? x : nan;
      table[2 * i + 1] = valid ? tables.yTable[i] : nan;
    }
    std::copy(table.begin(), table.end(), xy);

    std::lock_guard l{dev->calib_mutex};
    dev->ray_table = std::move(table);
    dev->ray_intrinsic = in;
    dev->ray_distortion = dist;
    return 1;
  }
  catch(const std::exception& e)
  {
    set_error(std::string{"ray table: "} + e.what());
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
  std::lock_guard l{dev->changed_mutex};
  dev->changed_cb = cb;
  dev->changed_user = user;
}

int backend_list_profiles(
    const char* uri, uint32_t stream, depthcam_profile_cb cb, void* user)
{
  if(!g_context || !cb)
    return 0;
  try
  {
    // An already-open camera is handed back rather than opened twice
    // (DeviceManager.cpp:146-169 in the SDK sources); otherwise this one is
    // released on return.
    auto device = resolve_device(parse_uri(uri));
    if(!device)
      return 0;
    auto sensors = device->getSensorList();
    if(!sensors)
      return 0;

    OBSensorType type{};
    switch(stream)
    {
      case DEPTHCAM_STREAM_DEPTH:
        type = OB_SENSOR_DEPTH;
        break;
      case DEPTHCAM_STREAM_COLOR:
        type = OB_SENSOR_COLOR;
        break;
      case DEPTHCAM_STREAM_IR:
        type = has_sensor(*sensors, OB_SENSOR_IR) ? OB_SENSOR_IR : OB_SENSOR_IR_LEFT;
        break;
      default:
        return 0;
    }
    if(!has_sensor(*sensors, type))
      return 0;

    auto list = sensors->getSensor(type)->getStreamProfileList(); // Sensor.hpp:81
    std::vector<std::tuple<int, int, int, int>> seen;
    for(uint32_t i = 0, n = list->getCount(); i < n; i++)
    {
      auto p = list->getProfile(i); // StreamProfile.hpp:505
      if(!p || !p->is<ob::VideoStreamProfile>())
        continue;
      auto vp = p->as<ob::VideoStreamProfile>();
      const int fmt = to_depthcam_format(vp->getFormat());
      if(fmt == DEPTHCAM_FMT_NONE)
        continue;
      // Several SDK formats map to one of ours (Y10/Y12/Y16 are all GRAY16).
      const auto key = std::make_tuple(
          int(vp->getWidth()), int(vp->getHeight()), int(vp->getFps()), fmt);
      if(std::find(seen.begin(), seen.end(), key) != seen.end())
        continue;
      seen.push_back(key);

      depthcam_profile prof{};
      prof.struct_size = sizeof(prof);
      prof.width = std::get<0>(key);
      prof.height = std::get<1>(key);
      prof.fps = std::get<2>(key);
      prof.format = fmt;
      cb(&prof, user);
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
  if(dev)
    dev->demand.store(streams, std::memory_order_relaxed);
}

const depthcam_backend_v1 g_backend{
    .abi_version = DEPTHCAM_ABI_VERSION,
    .name = "orbbec",
    .display_name = "Orbbec",
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
