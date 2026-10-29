/*
 * Luxonis OAK backend: depthai-core behind depthcam_abi.h.
 *
 * The pipeline is built out of dai::node::Depth, not dai::node::StereoDepth:
 * Depth picks stereo, ToF or on-device neural depth from what the camera
 * actually has, so one pipeline covers the whole OAK line. Hand-wired
 * StereoDepth works on an OAK-D and produces nothing on an OAK-D-SR-PoE.
 *
 * depthai-core is built as a nested CMake project rather than with
 * add_subdirectory (see Backends/CMakeLists.txt), and its telemetry is switched
 * off in init() below.
 */
#include <depthcam_abi.h>

#include <depthai/depthai.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
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

/// Only if the user has not already said otherwise: an explicit
/// DEPTHAI_TELEMETRY=1 is a deliberate opt-in and is left alone.
void set_env_default(const char* name, const char* value)
{
#if defined(_WIN32)
  size_t len = 0;
  if(getenv_s(&len, nullptr, 0, name) == 0 && len > 0)
    return;
  _putenv_s(name, value);
#else
  ::setenv(name, value, 0);
#endif
}

/// "depthai:id:14442C10D1...", "depthai:ip:10.0.0.5" or "depthai:index:0";
/// anything else is taken as a device id or an address, which is exactly what
/// dai::DeviceInfo's own string constructor accepts.
struct Address
{
  enum Kind
  {
    Any,
    Id,
    Name,
    Index
  } kind{Any};
  std::string text;
  int index{0};
};

Address parse_uri(const char* uri)
{
  Address a;
  if(!uri)
    return a;
  std::string s{uri};
  if(s.rfind("depthai:", 0) == 0)
    s = s.substr(8);
  if(s.empty())
    return a;

  if(s.rfind("id:", 0) == 0)
  {
    a.kind = Address::Id;
    a.text = s.substr(3);
  }
  else if(s.rfind("ip:", 0) == 0 || s.rfind("name:", 0) == 0)
  {
    a.kind = Address::Name;
    a.text = s.substr(s[0] == 'i' ? 3 : 5);
  }
  else if(s.rfind("index:", 0) == 0)
  {
    a.kind = Address::Index;
    try
    {
      a.index = std::stoi(s.substr(6));
    }
    catch(...)
    {
      a.kind = Address::Any;
    }
  }
  else
  {
    a.kind = Address::Id;
    a.text = s;
  }

  if(a.kind != Address::Index && a.kind != Address::Any && a.text.empty())
    a.kind = Address::Any;
  return a;
}

const char* transport_of(XLinkProtocol_t p)
{
  switch(p)
  {
    case X_LINK_USB_VSC:
    case X_LINK_USB_CDC:
      return "usb";
    case X_LINK_PCIE:
      return "pcie";
    case X_LINK_TCP_IP:
      return "ethernet";
    default:
      return "";
  }
}

/// The pipeline asks for RGB888i, GRAY8 and RAW16; the rest are here because
/// Camera::requestOutput may hand back something else when a sensor cannot
/// produce what was asked for.
int32_t format_of(dai::ImgFrame::Type t)
{
  using T = dai::ImgFrame::Type;
  switch(t)
  {
    case T::RGB888i:
      return DEPTHCAM_FMT_RGB24;
    case T::BGR888i:
      return DEPTHCAM_FMT_BGR24;
    case T::RGBA8888:
      return DEPTHCAM_FMT_RGBA;
    case T::GRAY8:
    case T::YUV400p:
      return DEPTHCAM_FMT_GRAY8;
    case T::RAW8:
      return DEPTHCAM_FMT_GRAY8;
    case T::RAW16:
    case T::RAW14:
    case T::RAW12:
    case T::RAW10:
      return DEPTHCAM_FMT_GRAY16;
    case T::NV12:
      return DEPTHCAM_FMT_NV12;
    case T::YUV420p:
      return DEPTHCAM_FMT_YUV420P;
    case T::YUV422i:
      return DEPTHCAM_FMT_YUYV422;
    default:
      return DEPTHCAM_FMT_NONE;
  }
}

uint64_t timestamp_ns(const std::chrono::steady_clock::time_point& tp)
{
  return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                      tp.time_since_epoch())
                      .count());
}

/// Keeps a depthai message alive while the host holds its frame. No copy:
/// messages are refcounted and their buffers are not recycled behind our back.
struct MessageHolder
{
  std::shared_ptr<dai::ADatatype> msg;
};

void release_message_holder(void* owner)
{
  delete static_cast<MessageHolder*>(owner);
}

/// Point clouds are converted rather than borrowed, so they carry their own
/// buffer.
struct CloudHolder
{
  std::vector<float> data;
};

void release_cloud_holder(void* owner)
{
  delete static_cast<CloudHolder*>(owner);
}

struct ImuHolder
{
  depthcam_imu_sample sample;
};

void release_imu_holder(void* owner)
{
  delete static_cast<ImuHolder*>(owner);
}

// --- controls ---------------------------------------------------------------

/**
 * depthai-core has no getters for camera settings -- dai::CameraControl is a
 * message you send into a node's input and nothing reads back -- so get()
 * returns what was last written, seeded with the SDK's defaults. The two
 * illumination controls and the chip temperature are device calls instead.
 */
enum class ControlId
{
  AutoExposure,
  ExposureTime,
  Iso,
  AutoFocus,
  Focus,
  AutoWhiteBalance,
  WhiteBalance,
  Brightness,
  Contrast,
  Saturation,
  Sharpness,
  LumaDenoise,
  ChromaDenoise,
  LaserPower,
  FloodLight,
  Temperature,
};

struct ControlDef
{
  ControlId id;
  const char* path;
  const char* name;
  const char* description;
  int32_t kind;
  int32_t access;
  double min, max, step, def;
};

/// Ranges from the doc comments on dai::CameraControl: 1us..33ms is what the
/// ISP accepts, 100..1600 what the IMX378/OV9282 sensors report.
const ControlDef control_defs[] = {
    {ControlId::AutoExposure, "color/auto_exposure", "auto exposure",
     "Let the camera choose exposure time and sensitivity",
     DEPTHCAM_CONTROL_BOOL, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 0, 1,
     1, 1},
    {ControlId::ExposureTime, "color/exposure_time", "exposure time",
     "Microseconds; writing it also turns auto exposure off",
     DEPTHCAM_CONTROL_INT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 1,
     33000, 1, 20000},
    {ControlId::Iso, "color/iso", "ISO",
     "Sensitivity; writing it also turns auto exposure off",
     DEPTHCAM_CONTROL_INT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 100,
     1600, 1, 800},
    {ControlId::AutoFocus, "color/auto_focus", "auto focus",
     "Continuous autofocus, on models with a focus motor",
     DEPTHCAM_CONTROL_BOOL, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 0, 1,
     1, 1},
    {ControlId::Focus, "color/focus", "focus",
     "Lens position; writing it also turns auto focus off",
     DEPTHCAM_CONTROL_INT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 0, 255,
     1, 130},
    {ControlId::AutoWhiteBalance, "color/auto_white_balance",
     "auto white balance", nullptr, DEPTHCAM_CONTROL_BOOL,
     DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 0, 1, 1, 1},
    {ControlId::WhiteBalance, "color/white_balance", "white balance",
     "Colour temperature in kelvin; writing it also turns auto white balance off",
     DEPTHCAM_CONTROL_INT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 1000,
     12000, 1, 4000},
    {ControlId::Brightness, "color/brightness", "brightness", nullptr,
     DEPTHCAM_CONTROL_INT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, -10, 10,
     1, 0},
    {ControlId::Contrast, "color/contrast", "contrast", nullptr,
     DEPTHCAM_CONTROL_INT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, -10, 10,
     1, 0},
    {ControlId::Saturation, "color/saturation", "saturation", nullptr,
     DEPTHCAM_CONTROL_INT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, -10, 10,
     1, 0},
    {ControlId::Sharpness, "color/sharpness", "sharpness", nullptr,
     DEPTHCAM_CONTROL_INT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 0, 4, 1,
     1},
    {ControlId::LumaDenoise, "color/luma_denoise", "luma denoise", nullptr,
     DEPTHCAM_CONTROL_INT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 0, 4, 1,
     1},
    {ControlId::ChromaDenoise, "color/chroma_denoise", "chroma denoise", nullptr,
     DEPTHCAM_CONTROL_INT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 0, 4, 1,
     1},
};

/// Published only where the hardware has the drivers; see getIrDrivers().
const ControlDef ir_control_defs[] = {
    {ControlId::LaserPower, "depth/laser_power", "laser dot projector",
     "Intensity of the structured-light projector, 0 to 1. It is what makes "
     "depth work on a blank wall, and it is off by default",
     DEPTHCAM_CONTROL_FLOAT, DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 0, 1,
     0, 0},
    {ControlId::FloodLight, "ir/flood_light", "infrared illumination",
     "Intensity of the infrared floodlight, 0 to 1", DEPTHCAM_CONTROL_FLOAT,
     DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE, 0, 1, 0, 0},
};

const ControlDef temperature_def{
    ControlId::Temperature, "sensors/temperature", "temperature",
    "Average of the on-chip sensors, in degrees Celsius",
    DEPTHCAM_CONTROL_FLOAT, DEPTHCAM_ACCESS_READ, -40, 125, 0, 0};

/// What list_controls2 adds to a ControlDef.
struct ControlMeta
{
  const char* unit{};
  uint32_t flags{};
  const char* auto_control{};
};

ControlMeta meta_of(ControlId id)
{
  switch(id)
  {
    case ControlId::ExposureTime:
      return {"us", 0, "color/auto_exposure"};
    case ControlId::Iso:
      return {nullptr, 0, "color/auto_exposure"};
    case ControlId::Focus:
      return {nullptr, 0, "color/auto_focus"};
    case ControlId::WhiteBalance:
      return {"K", 0, "color/auto_white_balance"};
    case ControlId::LumaDenoise:
    case ControlId::ChromaDenoise:
      return {nullptr, DEPTHCAM_CONTROL_ADVANCED, nullptr};
    case ControlId::Temperature:
      // Celsius, which is not one of the ABI's units.
      return {nullptr, DEPTHCAM_CONTROL_VOLATILE, nullptr};
    default:
      return {};
  }
}

depthcam_control base_of(const ControlDef& d)
{
  depthcam_control c{};
  c.id = d.path;
  c.name = d.name;
  c.description = d.description;
  c.kind = d.kind;
  c.access = d.access;
  c.min = d.min;
  c.max = d.max;
  c.step = d.step;
  c.def = d.def;
  return c;
}

// --- StereoDepth post-processing --------------------------------------------

/**
 * The StereoDepth configuration, published as filters/... controls.
 *
 * Everything here is a field of dai::StereoDepthConfig
 * (pipeline/datatype/StereoDepthConfig.hpp, filter parameters in
 * pipeline/FilterParams.hpp), sent at runtime through StereoDepth::inputConfig
 * ("Input StereoDepthConfig message with ability to modify parameters in
 * runtime", node/StereoDepth.hpp:80-84). The three that switch the stereo
 * algorithm itself -- left-right check, subpixel, extended disparity -- are
 * only switchable at runtime with setRuntimeModeSwitch(true)
 * (node/StereoDepth.hpp:288-293), which backend_open turns on.
 *
 * Decimation changes the size of the depth image, so it is marked as needing a
 * restart: it is held back and sent on the next start, where the host expects
 * the stream geometry to change.
 *
 * Ranges are the header's where it gives one; where it only gives a type and a
 * default, the range brackets the default and says so.
 */
struct StereoControl
{
  const char* id;
  const char* name;
  const char* description;
  int32_t kind;
  double min, max, step;
  const char* const* labels;
  int32_t label_count;
  const char* unit;
  uint32_t flags;
  double (*get)(const dai::StereoDepthConfig&);
  void (*set)(dai::StereoDepthConfig&, double);
};

const char* const median_labels[] = {"off", "3x3", "5x5", "7x7"};
const char* const persistency_labels[]
    = {"off",           "valid 8 out of 8", "valid 2 in last 3",
       "valid 2 in last 4", "valid 2 out of 8", "valid 1 in last 2",
       "valid 1 in last 5", "valid 1 in last 8", "indefinitely"};
const char* const decimation_mode_labels[]
    = {"pixel skipping", "non-zero median", "non-zero mean"};

constexpr uint32_t filter_flag = DEPTHCAM_CONTROL_FILTER;

using PP = dai::StereoDepthConfig::PostProcessing;
using Median = dai::StereoDepthConfig::MedianFilter;

int clamp_int(double v, int lo, int hi)
{
  return int(std::lround(std::clamp(v, double(lo), double(hi))));
}

const StereoControl stereo_controls[] = {
    {"filters/median/kernel", "median filter",
     "Kernel of the disparity median filter", DEPTHCAM_CONTROL_ENUM, 0, 3, 1,
     median_labels, 4, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) {
       switch(c.postProcessing.median)
       {
         case Median::KERNEL_3x3: return 1.;
         case Median::KERNEL_5x5: return 2.;
         case Median::KERNEL_7x7: return 3.;
         default: return 0.;
       }
     },
     [](dai::StereoDepthConfig& c, double v) {
       static constexpr Median m[]
           = {Median::MEDIAN_OFF, Median::KERNEL_3x3, Median::KERNEL_5x5,
              Median::KERNEL_7x7};
       c.setMedianFilter(m[clamp_int(v, 0, 3)]);
     }},
    // StereoDepthConfig.hpp:676-684: 0..255.
    {"filters/confidence/threshold", "confidence threshold",
     "Disparities whose cost-matching confidence is below this are discarded",
     DEPTHCAM_CONTROL_INT, 0, 255, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.getConfidenceThreshold()); },
     [](dai::StereoDepthConfig& c, double v) {
       c.setConfidenceThreshold(clamp_int(v, 0, 255));
     }},
    {"filters/left_right_check/enabled", "left-right check",
     "Computes disparity both ways and discards what disagrees: better occlusions",
     DEPTHCAM_CONTROL_BOOL, 0, 1, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return c.getLeftRightCheck() ? 1. : 0.; },
     [](dai::StereoDepthConfig& c, double v) { c.setLeftRightCheck(v != 0.); }},
    // StereoDepthConfig.hpp:706-709: 0..255.
    {"filters/left_right_check/threshold", "left-right check threshold",
     "Largest disagreement between the two directions that is still kept",
     DEPTHCAM_CONTROL_INT, 0, 255, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.getLeftRightCheckThreshold()); },
     [](dai::StereoDepthConfig& c, double v) {
       c.setLeftRightCheckThreshold(clamp_int(v, 0, 255));
     }},
    {"filters/subpixel/enabled", "subpixel",
     "Sub-pixel disparity interpolation, for long range; incompatible with "
     "extended disparity",
     DEPTHCAM_CONTROL_BOOL, 0, 1, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return c.getSubpixel() ? 1. : 0.; },
     [](dai::StereoDepthConfig& c, double v) { c.setSubpixel(v != 0.); }},
    // StereoDepthConfig.hpp:750-757: 3, 4 or 5.
    {"filters/subpixel/fractional_bits", "subpixel fractional bits",
     "The median filter only works with 3", DEPTHCAM_CONTROL_INT, 3, 5, 1,
     nullptr, 0, nullptr, filter_flag | DEPTHCAM_CONTROL_ADVANCED,
     [](const dai::StereoDepthConfig& c) { return double(c.getSubpixelFractionalBits()); },
     [](dai::StereoDepthConfig& c, double v) {
       c.setSubpixelFractionalBits(clamp_int(v, 3, 5));
     }},
    {"filters/extended_disparity/enabled", "extended disparity",
     "Doubles the disparity range, for close objects; incompatible with subpixel",
     DEPTHCAM_CONTROL_BOOL, 0, 1, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return c.getExtendedDisparity() ? 1. : 0.; },
     [](dai::StereoDepthConfig& c, double v) { c.setExtendedDisparity(v != 0.); }},
    // StereoDepthConfig.hpp:696-700: 0..65535, 0 disables. RVC2 only.
    {"filters/bilateral/sigma", "bilateral filter sigma",
     "5x5 bilateral filter on disparity; 0 disables it", DEPTHCAM_CONTROL_INT,
     0, 65535, 1, nullptr, 0, nullptr, filter_flag | DEPTHCAM_CONTROL_ADVANCED,
     [](const dai::StereoDepthConfig& c) { return double(c.getBilateralFilterSigma()); },
     [](dai::StereoDepthConfig& c, double v) {
       c.setBilateralFilterSigma(uint16_t(clamp_int(v, 0, 65535)));
     }},

    // FilterParams.hpp, SpeckleFilter. No ranges given: speckleRange defaults
    // to 50 and differenceThreshold to 2 disparity levels.
    {"filters/speckle/enabled", "speckle filter", "Removes speckle noise",
     DEPTHCAM_CONTROL_BOOL, 0, 1, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return c.postProcessing.speckleFilter.enable ? 1. : 0.; },
     [](dai::StereoDepthConfig& c, double v) { c.postProcessing.speckleFilter.enable = v != 0.; }},
    {"filters/speckle/range", "speckle search range", nullptr,
     DEPTHCAM_CONTROL_INT, 0, 256, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.postProcessing.speckleFilter.speckleRange); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.speckleFilter.speckleRange = uint32_t(clamp_int(v, 0, 256));
     }},
    {"filters/speckle/difference_threshold", "speckle difference threshold",
     "Largest disparity step, in integer levels, inside one speckle",
     DEPTHCAM_CONTROL_INT, 0, 64, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.postProcessing.speckleFilter.differenceThreshold); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.speckleFilter.differenceThreshold = uint32_t(clamp_int(v, 0, 64));
     }},

    // FilterParams.hpp, SpatialFilter. alpha is an EMA factor, so 0..1;
    // holeFillingRadius is a uint8; delta (default 3, 0 = auto) and
    // numIterations (default 1) have no stated range.
    {"filters/spatial/enabled", "spatial filter",
     "Edge-preserving smoothing", DEPTHCAM_CONTROL_BOOL, 0, 1, 1, nullptr, 0,
     nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return c.postProcessing.spatialFilter.enable ? 1. : 0.; },
     [](dai::StereoDepthConfig& c, double v) { c.postProcessing.spatialFilter.enable = v != 0.; }},
    {"filters/spatial/alpha", "spatial alpha",
     "1 is no filtering, 0 infinite filtering", DEPTHCAM_CONTROL_FLOAT, 0, 1, 0,
     nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.postProcessing.spatialFilter.alpha); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.spatialFilter.alpha = float(std::clamp(v, 0., 1.));
     }},
    {"filters/spatial/delta", "spatial delta",
     "Disparity step treated as an edge; 0 is automatic", DEPTHCAM_CONTROL_INT,
     0, 255, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.postProcessing.spatialFilter.delta); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.spatialFilter.delta = clamp_int(v, 0, 255);
     }},
    {"filters/spatial/iterations", "spatial iterations", nullptr,
     DEPTHCAM_CONTROL_INT, 1, 8, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.postProcessing.spatialFilter.numIterations); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.spatialFilter.numIterations = clamp_int(v, 1, 8);
     }},
    {"filters/spatial/hole_filling_radius", "spatial hole filling radius",
     nullptr, DEPTHCAM_CONTROL_INT, 0, 255, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.postProcessing.spatialFilter.holeFillingRadius); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.spatialFilter.holeFillingRadius = uint8_t(clamp_int(v, 0, 255));
     }},

    // FilterParams.hpp, TemporalFilter.
    {"filters/temporal/enabled", "temporal filter",
     "Smooths over time, optionally keeping old values where depth drops out",
     DEPTHCAM_CONTROL_BOOL, 0, 1, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return c.postProcessing.temporalFilter.enable ? 1. : 0.; },
     [](dai::StereoDepthConfig& c, double v) { c.postProcessing.temporalFilter.enable = v != 0.; }},
    {"filters/temporal/alpha", "temporal alpha",
     "1 is no filtering, 0 infinite filtering", DEPTHCAM_CONTROL_FLOAT, 0, 1, 0,
     nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.postProcessing.temporalFilter.alpha); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.temporalFilter.alpha = float(std::clamp(v, 0., 1.));
     }},
    {"filters/temporal/delta", "temporal delta",
     "Disparity step treated as an edge; 0 is automatic", DEPTHCAM_CONTROL_INT,
     0, 255, 1, nullptr, 0, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.postProcessing.temporalFilter.delta); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.temporalFilter.delta = clamp_int(v, 0, 255);
     }},
    {"filters/temporal/persistency", "temporal persistency",
     "When an invalid pixel is replaced by an older value",
     DEPTHCAM_CONTROL_ENUM, 0, 8, 1, persistency_labels, 9, nullptr, filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(int(c.postProcessing.temporalFilter.persistencyMode)); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.temporalFilter.persistencyMode
           = PP::TemporalFilter::PersistencyMode(clamp_int(v, 0, 8));
     }},

    // FilterParams.hpp, ThresholdFilter: "in depth units", which stay the
    // StereoDepthConfig default of millimetres (StereoDepthConfig.hpp:43).
    // Always on; the defaults keep everything.
    {"filters/threshold/min", "minimum depth",
     "Depth closer than this is invalidated", DEPTHCAM_CONTROL_INT, 0, 65535, 1,
     nullptr, 0, "mm", filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.postProcessing.thresholdFilter.minRange); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.thresholdFilter.minRange = clamp_int(v, 0, 65535);
     }},
    {"filters/threshold/max", "maximum depth",
     "Depth farther than this is invalidated", DEPTHCAM_CONTROL_INT, 0, 65535, 1,
     nullptr, 0, "mm", filter_flag,
     [](const dai::StereoDepthConfig& c) { return double(c.postProcessing.thresholdFilter.maxRange); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.thresholdFilter.maxRange = clamp_int(v, 0, 65535);
     }},

    // StereoDepthConfig.hpp, DecimationFilter: factor 1 to 4.
    {"filters/decimation/factor", "decimation factor",
     "Divides the depth resolution by this", DEPTHCAM_CONTROL_INT, 1, 4, 1,
     nullptr, 0, nullptr, filter_flag | DEPTHCAM_CONTROL_REQUIRES_RESTART,
     [](const dai::StereoDepthConfig& c) { return double(c.postProcessing.decimationFilter.decimationFactor); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.decimationFilter.decimationFactor = uint32_t(clamp_int(v, 1, 4));
     }},
    {"filters/decimation/mode", "decimation mode", nullptr,
     DEPTHCAM_CONTROL_ENUM, 0, 2, 1, decimation_mode_labels, 3, nullptr,
     filter_flag | DEPTHCAM_CONTROL_REQUIRES_RESTART,
     [](const dai::StereoDepthConfig& c) { return double(int(c.postProcessing.decimationFilter.decimationMode)); },
     [](dai::StereoDepthConfig& c, double v) {
       c.postProcessing.decimationFilter.decimationMode
           = PP::DecimationFilter::DecimationMode(clamp_int(v, 0, 2));
     }},
};

const StereoControl* find_stereo_control(const char* id)
{
  for(const auto& c : stereo_controls)
    if(std::strcmp(c.id, id) == 0)
      return &c;
  return nullptr;
}

depthcam_control base_of(const StereoControl& d, const dai::StereoDepthConfig& cfg)
{
  depthcam_control c{};
  c.id = d.id;
  c.name = d.name;
  c.description = d.description;
  c.kind = d.kind;
  c.access = DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE;
  c.min = d.min;
  c.max = d.max;
  c.step = d.step;
  // The values the Depth node's preset left in StereoDepth::initialConfig.
  c.def = d.get(cfg);
  c.enum_labels = d.labels;
  c.enum_count = d.label_count;
  return c;
}

// --- calibration helpers ----------------------------------------------------

using Mat4 = std::array<std::array<float, 4>, 4>;

Mat4 identity4()
{
  Mat4 m{};
  for(int i = 0; i < 4; i++)
    m[i][i] = 1.f;
  return m;
}

Mat4 mat4_of(const std::vector<std::vector<float>>& v)
{
  Mat4 m = identity4();
  for(std::size_t i = 0; i < 4 && i < v.size(); i++)
    for(std::size_t j = 0; j < 4 && j < v[i].size(); j++)
      m[i][j] = v[i][j];
  return m;
}

Mat4 mul(const Mat4& a, const Mat4& b)
{
  Mat4 r{};
  for(int i = 0; i < 4; i++)
    for(int j = 0; j < 4; j++)
      for(int k = 0; k < 4; k++)
        r[i][j] += a[i][k] * b[k][j];
  return r;
}

/// Inverse of a rigid transform: R^T, -R^T t.
Mat4 inverse_rigid(const Mat4& m)
{
  Mat4 r = identity4();
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      r[i][j] = m[j][i];
  for(int i = 0; i < 3; i++)
    r[i][3] = -(r[i][0] * m[0][3] + r[i][1] * m[1][3] + r[i][2] * m[2][3]);
  return r;
}

/**
 * @brief depthai's distortion model and coefficients in the ABI's layout.
 *
 * CalibrationHandler::getDistortionCoefficients documents the order
 * (device/CalibrationHandler.hpp:214-221): Perspective is OpenCV's rational
 * model k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, tau_x, tau_y; Fisheye is
 * OpenCV's k1..k4. ImgTransformation carries the same vector for the source
 * sensor, and depthai's own PointCloud node hands it to its undistortion with
 * the model as is (src/pipeline/node/PointCloud.cpp:664).
 *
 * The thin-prism (s1..s4) and tilt (tau) terms have no place in the ABI's
 * Brown-Conrady and are dropped; depthai calibrations leave them at zero
 * unless a camera was calibrated with those flags.
 *
 * @return false for a model the ABI cannot describe (equirectangular, radial
 *         division).
 */
bool distortion_of(dai::CameraModel model, const std::vector<float>& d, depthcam_intrinsics& r)
{
  const auto at = [&](std::size_t i) { return i < d.size() ? d[i] : 0.f; };
  const bool any = std::any_of(d.begin(), d.end(), [](float v) { return v != 0.f; });

  switch(model)
  {
    case dai::CameraModel::Perspective:
      if(!any)
      {
        r.model = DEPTHCAM_DISTORTION_NONE;
        return true;
      }
      r.model = DEPTHCAM_DISTORTION_BROWN_CONRADY;
      r.coeffs[0] = at(0); // k1
      r.coeffs[1] = at(1); // k2
      r.coeffs[2] = at(4); // k3
      r.coeffs[3] = at(5); // k4
      r.coeffs[4] = at(6); // k5
      r.coeffs[5] = at(7); // k6
      r.coeffs[6] = at(2); // p1
      r.coeffs[7] = at(3); // p2
      return true;
    case dai::CameraModel::Fisheye:
      if(!any)
      {
        r.model = DEPTHCAM_DISTORTION_NONE;
        return true;
      }
      r.model = DEPTHCAM_DISTORTION_KANNALA_BRANDT4;
      for(std::size_t i = 0; i < 4; i++)
        r.coeffs[i] = at(i);
      return true;
    default:
      return false;
  }
}

} // namespace

// ---------------------------------------------------------------------------

struct depthcam_device
{
  std::shared_ptr<dai::Device> device;
  std::unique_ptr<dai::Pipeline> pipeline;

  std::shared_ptr<dai::node::Camera> colorCam, irCam;
  std::shared_ptr<dai::node::Depth> depthNode;
  std::shared_ptr<dai::node::PointCloud> pcNode;
  std::shared_ptr<dai::node::IMU> imuNode;

  std::shared_ptr<dai::MessageQueue> colorQ, irQ, depthQ, pcQ, imuQ;
  std::shared_ptr<dai::InputQueue> colorControlQ;

  /// The StereoDepth inside the Depth node, when it picked stereo; null on a
  /// ToF or neural-depth camera, which then has no filters/ controls.
  std::shared_ptr<dai::node::StereoDepth> stereo;
  std::shared_ptr<dai::InputQueue> stereoConfigQ;
  /// What was last sent, under control_lock; depthai has no getter for it.
  dai::StereoDepthConfig stereoConfig;
  /// Decimation, held back until the next start (see stereo_controls).
  dai::StereoDepthConfig::PostProcessing::DecimationFilter pendingDecimation;

  std::optional<dai::CalibrationHandler> calib;
  dai::CameraBoardSocket colorSocket{dai::CameraBoardSocket::AUTO};
  dai::CameraBoardSocket irSocket{dai::CameraBoardSocket::AUTO};
  uint32_t colorW{}, colorH{}, irW{}, irH{};
  bool aligned{};
  std::vector<dai::CameraFeatures> features;
  std::string uri, model, serial, hardwareRev;

  /// The geometry of the last frame of each stream, as depthai describes it.
  /// This is what depthai's own PointCloud node deprojects with
  /// (src/pipeline/node/PointCloud.cpp:566-575,664), so once a frame has
  /// arrived it is authoritative; before that, the calibration is the
  /// estimate.
  std::mutex transform_lock;
  std::optional<dai::ImgTransformation> colorT, irT, depthT;

  /// Which opened streams the host consumes; see backend_set_stream_demand.
  std::atomic<uint32_t> demand{~0u};

  depthcam_open_config cfg{};
  /// What the pipeline actually wired up: an OAK-1 has no depth, a plain OAK-D
  /// no IMU.
  uint32_t active{};
  bool tinted_cloud{};

  std::atomic_bool running{};
  /// One-way, both of them: the pipeline is never torn down until close.
  /// See backend_stop.
  bool started{};
  bool callbacks_added{};
  // Read on the depthai queue threads, written by backend_start. stop() does
  // not join those threads -- see backend_stop -- so the two must be taken
  // together or a restart can pair a new callback with a freed user pointer.
  struct Sink
  {
    depthcam_frame_cb cb{};
    void* user{};
  };
  std::mutex sink_mutex;
  Sink sink;

  Sink currentSink()
  {
    std::lock_guard lock{sink_mutex};
    return running.load(std::memory_order_acquire) ? sink : Sink{};
  }

  std::vector<ControlDef> controls;

  /// Mirrors of what was last written; depthai has no getters.
  std::mutex control_lock;
  bool auto_exposure{true}, auto_focus{true}, auto_white_balance{true};
  double exposure_us{20000}, iso{800}, focus{130}, white_balance{4000};
  double brightness{0}, contrast{0}, saturation{0}, sharpness{1};
  double luma_denoise{1}, chroma_denoise{1};
  double laser_power{0}, flood_light{0};

  ~depthcam_device();

  void emitImage(uint32_t stream, const std::shared_ptr<dai::ImgFrame>& f, float unit_mm);
  void emitPointCloud(const std::shared_ptr<dai::PointCloudData>& pc);
  void emitImu(const std::shared_ptr<dai::IMUData>& imu);

  void applyCameraControl();
  void applyStereoConfig();
  const ControlDef* findControl(const char* id) const;
  void noteTransform(uint32_t stream, const std::shared_ptr<dai::ImgFrame>& f);
};

namespace
{
/// Every device open in this process, for list_profiles: depthai only knows a
/// camera's sensors once it is booted.
std::mutex g_open_mutex;
std::vector<depthcam_device*> g_open_devices;
}

depthcam_device::~depthcam_device()
{
  {
    std::lock_guard lock{g_open_mutex};
    g_open_devices.erase(
        std::remove(g_open_devices.begin(), g_open_devices.end(), this),
        g_open_devices.end());
  }

  try
  {
    if(pipeline)
    {
      // The queue callbacks capture `this`, so the pipeline has to be fully
      // stopped -- not merely asked to -- before this object goes away.
      if(pipeline->isRunning())
        pipeline->stop();
      pipeline->wait();
    }
  }
  catch(...)
  {
  }

  colorQ.reset();
  irQ.reset();
  depthQ.reset();
  pcQ.reset();
  imuQ.reset();
  colorControlQ.reset();
  stereoConfigQ.reset();
  stereo.reset();
  pipeline.reset();
  device.reset();
}

void depthcam_device::noteTransform(uint32_t stream, const std::shared_ptr<dai::ImgFrame>& f)
{
  if(!f)
    return;
  const auto& t = f->transformation;
  if(!t.isValid())
    return;

  std::lock_guard lock{transform_lock};
  auto& slot = stream == DEPTHCAM_STREAM_COLOR ? colorT
               : stream == DEPTHCAM_STREAM_IR  ? irT
                                               : depthT;
  // Only on a change of size, which is what decimation or a new sensor mode
  // brings: copying it per frame would cost an allocation per frame.
  if(!slot || slot->getSize() != t.getSize())
    slot = t;
}

void depthcam_device::emitImage(
    uint32_t stream, const std::shared_ptr<dai::ImgFrame>& f, float unit_mm)
{
  // Recorded even while stopped or undemanded: the frames keep coming (see
  // backend_stop) and get_intrinsics may be asked at any time.
  noteTransform(stream, f);

  if(!(demand.load(std::memory_order_relaxed) & stream))
    return;

  const auto out_sink = currentSink();
  if(!out_sink.cb || !f)
    return;

  const auto data = f->getData();
  if(data.empty())
    return;

  const int32_t format = format_of(f->getType());
  if(format == DEPTHCAM_FMT_NONE)
    return;

  auto* holder = new MessageHolder{f};

  depthcam_frame out{};
  out.stream = stream;
  out.format = format;
  out.width = int32_t(f->getWidth());
  out.height = int32_t(f->getHeight());
  out.stride = int32_t(f->getStride());
  out.timestamp_ns = timestamp_ns(f->getTimestamp());
  out.data = data.data();
  out.bytes = data.size();
  out.depth_unit_mm = unit_mm;
  out.owner = holder;
  out.release = &release_message_holder;

  out_sink.cb(&out, out_sink.user);
}

void depthcam_device::emitPointCloud(const std::shared_ptr<dai::PointCloudData>& pc)
{
  // The conversion below is a pass over every point; nobody reading the cloud
  // means it is not done.
  if(!(demand.load(std::memory_order_relaxed) & DEPTHCAM_STREAM_POINTCLOUD))
    return;

  const auto out_sink = currentSink();
  if(!out_sink.cb || !pc)
    return;

  // Already metres: the PointCloud node is built with LengthUnit::METER, which
  // is not its default. Axes are depthai's camera optical frame, which is the
  // +X right / +Y down / +Z forward depthcam_abi.h asks for.
  //
  // The message buffer is read directly rather than through getPoints(), which
  // materialises a std::vector per call (two, on a colour cloud). The layouts
  // are what PointCloudData itself casts to.
  const bool color = pc->isColor();
  const bool tinted = tinted_cloud && color;
  const int floats_per_point = tinted ? 6 : 3;

  const auto raw = pc->getData();
  const std::size_t point_bytes
      = color ? sizeof(dai::Point3fRGBA) : sizeof(dai::Point3f);
  const std::size_t n = raw.size() / point_bytes;
  if(n == 0)
    return;

  auto holder = std::make_unique<CloudHolder>();
  auto& out = holder->data;
  out.reserve(n * floats_per_point);

  // depthai writes an all-zero point where it had no depth.
  const auto keep = [](float z) { return std::isfinite(z) && z > 0.f; };

  if(color)
  {
    const auto* pts = reinterpret_cast<const dai::Point3fRGBA*>(raw.data());
    for(std::size_t i = 0; i < n; i++)
    {
      const auto& p = pts[i];
      if(!keep(p.z))
        continue;
      out.push_back(p.x);
      out.push_back(p.y);
      out.push_back(p.z);
      if(tinted)
      {
        out.push_back(p.r / 255.f);
        out.push_back(p.g / 255.f);
        out.push_back(p.b / 255.f);
      }
    }
  }
  else
  {
    const auto* pts = reinterpret_cast<const dai::Point3f*>(raw.data());
    for(std::size_t i = 0; i < n; i++)
    {
      const auto& p = pts[i];
      if(!keep(p.z))
        continue;
      out.push_back(p.x);
      out.push_back(p.y);
      out.push_back(p.z);
    }
  }

  if(out.empty())
    return;

  depthcam_frame frame{};
  frame.stream = DEPTHCAM_STREAM_POINTCLOUD;
  frame.format = tinted ? DEPTHCAM_FMT_XYZRGB : DEPTHCAM_FMT_XYZ;
  frame.point_count = int32_t(out.size() / floats_per_point);
  frame.timestamp_ns = timestamp_ns(pc->getTimestamp());
  frame.data = out.data();
  frame.bytes = out.size() * sizeof(float);
  frame.owner = holder.get();
  frame.release = &release_cloud_holder;

  holder.release();
  out_sink.cb(&frame, out_sink.user);
}

void depthcam_device::emitImu(const std::shared_ptr<dai::IMUData>& imu)
{
  const auto out_sink = currentSink();
  if(!out_sink.cb || !imu)
    return;

  for(const auto& packet : imu->packets)
  {
    auto holder = std::make_unique<ImuHolder>();
    auto& sample = holder->sample;

    // Both fields: the IMU node batches the two reports into one packet on the
    // device, so nothing is waited for here.
    //
    // No temperature: IMUPacket carries none, and getChipTemperature() is the
    // SoC's rather than the inertial sensor's -- published as the
    // sensors/temperature control instead.
    sample.fields = DEPTHCAM_IMU_ACCEL | DEPTHCAM_IMU_GYRO;

    // ACCELEROMETER_CALIBRATED is m/s^2 and GYROSCOPE_CALIBRATED rad/s, which
    // is what the ABI asks for.
    sample.accel[0] = packet.acceleroMeter.x;
    sample.accel[1] = packet.acceleroMeter.y;
    sample.accel[2] = packet.acceleroMeter.z;
    sample.gyro[0] = packet.gyroscope.x;
    sample.gyro[1] = packet.gyroscope.y;
    sample.gyro[2] = packet.gyroscope.z;

    depthcam_frame frame{};
    frame.stream = DEPTHCAM_STREAM_IMU;
    frame.format = DEPTHCAM_FMT_IMU;
    frame.timestamp_ns = timestamp_ns(packet.acceleroMeter.getTimestamp());
    frame.data = &sample;
    frame.bytes = sizeof(sample);
    frame.owner = holder.get();
    frame.release = &release_imu_holder;

    holder.release();
    out_sink.cb(&frame, out_sink.user);
  }
}

void depthcam_device::applyCameraControl()
{
  // Caller holds control_lock.
  if(!colorControlQ)
    return;

  try
  {
    auto ctrl = std::make_shared<dai::CameraControl>();

    if(auto_exposure)
      ctrl->setAutoExposureEnable();
    else
      ctrl->setManualExposure(uint32_t(exposure_us), uint32_t(iso));

    if(auto_focus)
      ctrl->setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_VIDEO);
    else
      ctrl->setManualFocus(uint8_t(std::clamp(focus, 0., 255.)));

    if(auto_white_balance)
      ctrl->setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
    else
      ctrl->setManualWhiteBalance(int(white_balance));

    ctrl->setBrightness(int(brightness));
    ctrl->setContrast(int(contrast));
    ctrl->setSaturation(int(saturation));
    ctrl->setSharpness(int(sharpness));
    ctrl->setLumaDenoise(int(luma_denoise));
    ctrl->setChromaDenoise(int(chroma_denoise));

    colorControlQ->send(ctrl);
  }
  catch(...)
  {
  }
}

/// The settings of `src` into `dst`, keeping `dst`'s own payload: a message
/// copied whole would share its ADatatype::data buffer with the original.
static void copy_config(dai::StereoDepthConfig& dst, const dai::StereoDepthConfig& src)
{
  auto payload = dst.data;
  dst = src;
  dst.data = std::move(payload);
}

void depthcam_device::applyStereoConfig()
{
  // Caller holds control_lock.
  if(!stereo)
    return;
  try
  {
    if(!started)
    {
      // Before the first start the configuration is simply the initial one.
      copy_config(*stereo->initialConfig, stereoConfig);
      return;
    }
    if(stereoConfigQ)
    {
      auto msg = std::make_shared<dai::StereoDepthConfig>();
      copy_config(*msg, stereoConfig);
      stereoConfigQ->send(msg);
    }
  }
  catch(...)
  {
  }
}

const ControlDef* depthcam_device::findControl(const char* id) const
{
  if(!id)
    return nullptr;
  for(const auto& c : controls)
    if(std::strcmp(c.path, id) == 0)
      return &c;
  return nullptr;
}

// ---------------------------------------------------------------------------

namespace
{

int backend_init([[maybe_unused]] const char* resource_dir)
{
  try
  {
#if defined(_WIN32)
    /*
     * depthai-core.dll is delay-loaded (see Backends/CMakeLists.txt) purely so
     * that this can run first: the process search order does not include the
     * directory this module came from, so the DLL sitting right beside it is
     * not found. LOAD_WITH_ALTERED_SEARCH_PATH covers the whole chain in one
     * call -- depthai's Windows triplet keeps libusb dynamic, and that DLL is
     * in the same directory.
     */
    if(resource_dir && *resource_dir)
    {
      const std::string dll = std::string{resource_dir} + "\\depthai-core.dll";
      if(!GetModuleHandleA("depthai-core.dll")
         && !LoadLibraryExA(dll.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH))
      {
        set_error(
            "could not load depthai-core.dll from the package directory (error "
            + std::to_string(GetLastError())
            + "). It is built against the Visual C++ runtime, so that "
              "redistributable has to be installed.");
        return 0;
      }
    }
#endif

    // depthai-core reports usage to Luxonis unless this says otherwise, from
    // inside a media application the user did not point at the internet.
    set_env_default("DEPTHAI_TELEMETRY", "0");

    // Some builds log to stderr at info level.
    set_env_default("DEPTHAI_LEVEL", "warn");

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

void backend_shutdown() { }

int backend_enumerate(depthcam_enumerate_cb cb, void* user)
{
  if(!cb)
    return 0;

  try
  {
    // No device is opened: this is an XLink discovery sweep over USB and the
    // local network.
    for(const auto& info : dai::Device::getAllAvailableDevices())
    {
      const std::string id = info.getDeviceId();
      const std::string uri
          = "depthai:id:" + (id.empty() ? info.name : id);
      // The model is only knowable once the device is booted, and info.name is
      // the USB path or the IP address, not a name. The transport and the
      // address are already carried by their own fields.
      const char* const name = "OAK";

      depthcam_device_info out{};
      out.backend = "depthai";
      out.uri = uri.c_str();
      out.name = name;
      out.serial = id.c_str();
      out.transport = transport_of(info.protocol);
      // A wish, not a promise: what a given OAK can deliver is only known once
      // it is open, and active_streams() reports it then.
      out.streams = DEPTHCAM_STREAM_COLOR | DEPTHCAM_STREAM_IR
                    | DEPTHCAM_STREAM_DEPTH | DEPTHCAM_STREAM_POINTCLOUD
                    | DEPTHCAM_STREAM_IMU;
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

std::optional<dai::DeviceInfo> find_device(const Address& addr)
{
  const auto all = dai::Device::getAllAvailableDevices();
  if(all.empty())
    return std::nullopt;

  switch(addr.kind)
  {
    case Address::Any:
      return all.front();
    case Address::Index:
      if(addr.index >= 0 && size_t(addr.index) < all.size())
        return all[size_t(addr.index)];
      return std::nullopt;
    case Address::Id:
      for(const auto& d : all)
        if(d.getDeviceId() == addr.text)
          return d;
      return std::nullopt;
    case Address::Name:
      for(const auto& d : all)
        if(d.name == addr.text)
          return d;
      return std::nullopt;
  }
  return std::nullopt;
}

depthcam_device* backend_open(const char* uri, const depthcam_open_config* config)
{
  if(!config)
    return nullptr;

  try
  {
    const auto addr = parse_uri(uri);

    auto dev = std::make_unique<depthcam_device>();
    dev->cfg = *config;

    if(const auto info = find_device(addr))
    {
      dev->device = std::make_shared<dai::Device>(*info);
    }
    else if(addr.kind == Address::Id || addr.kind == Address::Name)
    {
      // A PoE camera routed from another subnet does not answer discovery but
      // can still be reached by address, which DeviceInfo's string constructor
      // accepts. A wrong name throws and is reported below.
      dev->device = std::make_shared<dai::Device>(dai::DeviceInfo{addr.text});
    }
    else if(addr.kind == Address::Index)
    {
      set_error("no Luxonis OAK at index " + std::to_string(addr.index));
      return nullptr;
    }
    else
    {
      set_error("no Luxonis OAK camera connected");
      return nullptr;
    }

    auto& device = dev->device;
    dev->pipeline = std::make_unique<dai::Pipeline>(device);
    auto& pipeline = *dev->pipeline;

    const bool want_color = (config->streams & DEPTHCAM_STREAM_COLOR) != 0;
    const bool want_ir = (config->streams & DEPTHCAM_STREAM_IR) != 0;
    const bool want_depth = (config->streams & DEPTHCAM_STREAM_DEPTH) != 0;
    const bool want_cloud = (config->streams & DEPTHCAM_STREAM_POINTCLOUD) != 0;
    const bool want_imu = (config->streams & DEPTHCAM_STREAM_IMU) != 0;
    const bool tint = want_cloud && config->color_pointcloud != 0;

    /*
     * The IR camera must be created *before* the Depth node: Depth's
     * ensureStereoOutputs() reuses a Camera already on the stereo sockets, and
     * creating a second one on an occupied socket is an error. Claiming the
     * left socket here gets IR out of the same sensor at no extra bandwidth.
     */
    dai::Node::Output* irOut{};
    if(want_ir)
    {
      const auto pairs = device->getAvailableStereoPairs();
      if(!pairs.empty())
      {
        dev->irSocket = pairs.front().left;
        dev->irCam = pipeline.create<dai::node::Camera>()->build(pairs.front().left);
        const auto w = config->ir_width > 0 ? uint32_t(config->ir_width) : 640u;
        const auto h = config->ir_height > 0 ? uint32_t(config->ir_height) : 400u;
        dev->irW = w;
        dev->irH = h;
        irOut = dev->irCam->requestOutput(
            std::make_pair(w, h), dai::ImgFrame::Type::GRAY8,
            dai::ImgResizeMode::CROP,
            config->ir_fps > 0 ? std::optional<float>(float(config->ir_fps))
                               : std::nullopt);
        if(irOut)
          dev->active |= DEPTHCAM_STREAM_IR;
      }
    }

    dai::Node::Output* colorOut{};
    if(want_color || tint)
    {
      const auto colorSockets = device->getConnectedCameras(dai::CameraSensorType::COLOR);
      const auto socket = colorSockets.empty() ? dai::CameraBoardSocket::CAM_A
                                               : colorSockets.front();
      dev->colorCam = pipeline.create<dai::node::Camera>()->build(socket);
      dev->colorSocket = socket;

      const auto w = config->color_width > 0 ? uint32_t(config->color_width) : 1280u;
      const auto h = config->color_height > 0 ? uint32_t(config->color_height) : 720u;
      dev->colorW = w;
      dev->colorH = h;
      colorOut = dev->colorCam->requestOutput(
          std::make_pair(w, h), dai::ImgFrame::Type::RGB888i,
          dai::ImgResizeMode::CROP,
          config->color_fps > 0 ? std::optional<float>(float(config->color_fps))
                                : std::nullopt);
      if(colorOut && want_color)
        dev->active |= DEPTHCAM_STREAM_COLOR;
    }

    if(want_depth || want_cloud)
    {
      dev->depthNode = pipeline.create<dai::node::Depth>();

      const std::optional<std::pair<uint32_t, uint32_t>> size
          = (config->depth_width > 0 && config->depth_height > 0)
                ? std::optional<std::pair<uint32_t, uint32_t>>{{
                      uint32_t(config->depth_width),
                      uint32_t(config->depth_height)}}
                : std::nullopt;
      const std::optional<float> fps
          = config->depth_fps > 0 ? std::optional<float>(float(config->depth_fps))
                                  : std::nullopt;

      // AUTO: stereo on an OAK-D, ToF on an OAK-D-SR-PoE, neural depth on RVC4.
      dev->depthNode->build(dai::node::Depth::Algorithm::AUTO, fps, size);

      // Before the first depth() access, hence before the queues below.
      if(colorOut && config->align == DEPTHCAM_ALIGN_DEPTH_TO_COLOR)
      {
        dev->depthNode->setAlignTo(*colorOut);
        dev->aligned = true;
      }

      if(want_depth)
      {
        dev->depthQ = dev->depthNode->depth().createOutputQueue(4, false);
        dev->active |= DEPTHCAM_STREAM_DEPTH;
      }

      if(want_cloud)
      {
        dev->pcNode = pipeline.create<dai::node::PointCloud>();
        // Metres, not the millimetres depthai defaults to; the node scales on
        // the device, so this is free.
        dev->pcNode->initialConfig->setLengthUnit(dai::LengthUnit::METER);

        dev->depthNode->depth().link(dev->pcNode->inputDepth);
        if(tint && colorOut)
        {
          colorOut->link(dev->pcNode->getColorInput());
          dev->tinted_cloud = true;
        }

        dev->pcQ = dev->pcNode->outputPointCloud.createOutputQueue(4, false);
        dev->active |= DEPTHCAM_STREAM_POINTCLOUD;
      }

      // The graph is wired by the depth() accesses above; a StereoDepth
      // subnode is a child of the Depth node (Subnode.hpp, parent.add), and
      // Node::getAllNodes (pipeline/Node.hpp:709) is the public way to it.
      for(const auto& n : dev->depthNode->getAllNodes())
        if(auto st = std::dynamic_pointer_cast<dai::node::StereoDepth>(n))
        {
          dev->stereo = st;
          break;
        }

      if(dev->stereo)
      {
        dev->stereo->setRuntimeModeSwitch(true);
        dev->stereoConfig = *dev->stereo->initialConfig;
        dev->pendingDecimation = dev->stereoConfig.postProcessing.decimationFilter;
        dev->stereoConfigQ = dev->stereo->inputConfig.createInputQueue();
      }
    }

    if(want_imu)
    {
      // Guarded so a document that asked for everything still gets its video on
      // a camera with no IMU.
      try
      {
        dev->imuNode = pipeline.create<dai::node::IMU>();
        // CALIBRATED, not RAW: m/s^2 and rad/s with the stored calibration
        // applied, which is what depthcam_abi.h documents.
        dev->imuNode->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_CALIBRATED, 200);
        dev->imuNode->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED, 200);
        // One packet per report: batching would trade latency for throughput.
        dev->imuNode->setBatchReportThreshold(1);
        dev->imuNode->setMaxBatchReports(10);
        dev->imuQ = dev->imuNode->out.createOutputQueue(50, false);
        dev->active |= DEPTHCAM_STREAM_IMU;
      }
      catch(const std::exception& e)
      {
        set_error(std::string{"no IMU on this camera: "} + e.what());
        dev->imuNode.reset();
        dev->imuQ.reset();
      }
    }

    if(irOut)
      dev->irQ = irOut->createOutputQueue(4, false);
    if(colorOut && want_color)
      dev->colorQ = colorOut->createOutputQueue(4, false);

    if(dev->colorCam)
      dev->colorControlQ = dev->colorCam->inputControl.createInputQueue();

    if(dev->active == 0)
    {
      set_error("this camera could not provide any of the requested streams");
      return nullptr;
    }

    // --- controls ---------------------------------------------------------
    if(dev->colorCam)
      for(const auto& d : control_defs)
        dev->controls.push_back(d);

    // Only where the hardware has them: a plain OAK-D has neither.
    try
    {
      if(!device->getIrDrivers().empty())
        for(const auto& d : ir_control_defs)
          dev->controls.push_back(d);
    }
    catch(...)
    {
    }

    dev->controls.push_back(temperature_def);

    // --- what get_intrinsics / get_details fall back on --------------------
    // getCalibration is the calibration in use, which can differ from the
    // EEPROM's (device/DeviceBase.hpp:829-835); readCalibration is the EEPROM.
    try
    {
      dev->calib = device->getCalibration();
    }
    catch(...)
    {
      try
      {
        dev->calib = device->readCalibration();
      }
      catch(...)
      {
      }
    }

    try
    {
      dev->model = device->getProductName();
      if(dev->model.empty())
        dev->model = device->getDeviceName();
      dev->serial = device->getDeviceId();
    }
    catch(...)
    {
    }
    try
    {
      if(dev->calib)
      {
        const auto eeprom = dev->calib->getEepromData();
        dev->hardwareRev = eeprom.boardName;
        if(!eeprom.boardRev.empty())
          dev->hardwareRev += (dev->hardwareRev.empty() ? "" : " ") + eeprom.boardRev;
      }
    }
    catch(...)
    {
    }
    try
    {
      dev->features = device->getConnectedCameraFeatures();
    }
    catch(...)
    {
    }
    dev->uri = uri ? uri : "";

    {
      std::lock_guard lock{g_open_mutex};
      g_open_devices.push_back(dev.get());
    }
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

  try
  {
    {
      std::lock_guard lock{dev->sink_mutex};
      dev->sink = {on_frame, user};
      dev->running.store(true, std::memory_order_release);
    }

    if(dev->started)
    {
      // Second play: the pipeline was never torn down (see backend_stop), so
      // frames simply start being delivered again.
      std::lock_guard lock{dev->control_lock};
      dev->applyCameraControl();
      if(dev->stereo)
      {
        dev->stereoConfig.postProcessing.decimationFilter = dev->pendingDecimation;
        dev->applyStereoConfig();
      }
      return 1;
    }

    {
      std::lock_guard lock{dev->control_lock};
      if(dev->stereo)
      {
        dev->stereoConfig.postProcessing.decimationFilter = dev->pendingDecimation;
        dev->applyStereoConfig();
      }
    }

    // Once only, even if pipeline->start() below fails and the host retries:
    // MessageQueue::addCallback appends, so a second pass would deliver every
    // frame twice.
    if(!dev->callbacks_added)
    {
      dev->callbacks_added = true;

      // Callbacks rather than a polling thread: MessageQueue already runs one per
      // queue. Nothing may escape into those threads.
      if(dev->colorQ)
        dev->colorQ->addCallback([dev](std::shared_ptr<dai::ADatatype> msg) {
          try
          {
            dev->emitImage(
                DEPTHCAM_STREAM_COLOR,
                std::dynamic_pointer_cast<dai::ImgFrame>(msg), 0.f);
          }
          catch(...)
          {
          }
        });

      if(dev->irQ)
        dev->irQ->addCallback([dev](std::shared_ptr<dai::ADatatype> msg) {
          try
          {
            dev->emitImage(
                DEPTHCAM_STREAM_IR, std::dynamic_pointer_cast<dai::ImgFrame>(msg),
                0.f);
          }
          catch(...)
          {
          }
        });

      if(dev->depthQ)
        dev->depthQ->addCallback([dev](std::shared_ptr<dai::ADatatype> msg) {
          try
          {
            // Depth frames are uint16 millimetres, so the multiplier is 1.
            dev->emitImage(
                DEPTHCAM_STREAM_DEPTH,
                std::dynamic_pointer_cast<dai::ImgFrame>(msg), 1.f);
          }
          catch(...)
          {
          }
        });

      if(dev->pcQ)
        dev->pcQ->addCallback([dev](std::shared_ptr<dai::ADatatype> msg) {
          try
          {
            dev->emitPointCloud(std::dynamic_pointer_cast<dai::PointCloudData>(msg));
          }
          catch(...)
          {
          }
        });

      if(dev->imuQ)
        dev->imuQ->addCallback([dev](std::shared_ptr<dai::ADatatype> msg) {
          try
          {
            dev->emitImu(std::dynamic_pointer_cast<dai::IMUData>(msg));
          }
          catch(...)
          {
          }
        });
    }

    dev->pipeline->start();
    dev->started = true;

    // The control queue only reaches the device once the pipeline is running,
    // so anything set before play is applied here rather than lost.
    {
      std::lock_guard lock{dev->control_lock};
      dev->applyCameraControl();
      try
      {
        if(dev->laser_power > 0)
          dev->device->setIrLaserDotProjectorIntensity(float(dev->laser_power));
        if(dev->flood_light > 0)
          dev->device->setIrFloodLightIntensity(float(dev->flood_light));
      }
      catch(...)
      {
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
  /*
   * Stops *delivery*, not the pipeline.
   *
   * dai::Pipeline::stop() is one-way -- it closes every output queue and then
   * the device -- so a stopped pipeline cannot be restarted, and the host
   * calls stop()/start() around every play. Rebuilding it instead would mean
   * re-uploading firmware, which takes seconds.
   *
   * So the camera keeps streaming into callbacks that drop what they get, and
   * the pipeline is torn down in the destructor.
   */
  if(!dev)
    return;
  dev->running.store(false, std::memory_order_release);
}

int backend_list_controls(depthcam_device* dev, depthcam_control_cb cb, void* user)
{
  if(!dev || !cb)
    return 0;

  for(const auto& d : dev->controls)
  {
    const depthcam_control c = base_of(d);
    cb(&c, user);
  }

  if(dev->stereo)
  {
    // A copy: the callback must not run under control_lock.
    dai::StereoDepthConfig cfg;
    {
      std::lock_guard lock{dev->control_lock};
      cfg = dev->stereoConfig;
    }
    for(const auto& d : stereo_controls)
    {
      const depthcam_control c = base_of(d, cfg);
      cb(&c, user);
    }
  }
  return 1;
}

int backend_list_controls2(depthcam_device* dev, depthcam_control2_cb cb, void* user)
{
  if(!dev || !cb)
    return 0;

  for(const auto& d : dev->controls)
  {
    const auto meta = meta_of(d.id);
    depthcam_control2 c{};
    c.struct_size = sizeof(c);
    c.base = base_of(d);
    c.unit = meta.unit;
    c.flags = meta.flags;
    c.auto_control = meta.auto_control;
    cb(&c, user);
  }

  if(dev->stereo)
  {
    dai::StereoDepthConfig cfg;
    {
      std::lock_guard lock{dev->control_lock};
      cfg = dev->stereoConfig;
    }
    for(const auto& d : stereo_controls)
    {
      depthcam_control2 c{};
      c.struct_size = sizeof(c);
      c.base = base_of(d, cfg);
      c.unit = d.unit;
      c.flags = d.flags;
      cb(&c, user);
    }
  }
  return 1;
}

int backend_get_control(depthcam_device* dev, const char* id, double* out)
{
  if(!dev || !id || !out)
    return 0;

  if(dev->stereo)
    if(const auto* sc = find_stereo_control(id))
    {
      std::lock_guard lock{dev->control_lock};
      // Decimation reads back what will apply at the next start.
      if(sc->flags & DEPTHCAM_CONTROL_REQUIRES_RESTART)
      {
        auto cfg = dev->stereoConfig;
        cfg.postProcessing.decimationFilter = dev->pendingDecimation;
        *out = sc->get(cfg);
      }
      else
        *out = sc->get(dev->stereoConfig);
      return 1;
    }

  const ControlDef* def = dev->findControl(id);
  if(!def)
    return 0;

  if(def->id == ControlId::Temperature)
  {
    try
    {
      *out = dev->device->getChipTemperature().average;
      return 1;
    }
    catch(...)
    {
      return 0;
    }
  }

  std::lock_guard lock{dev->control_lock};
  switch(def->id)
  {
    case ControlId::AutoExposure:     *out = dev->auto_exposure ? 1. : 0.; break;
    case ControlId::ExposureTime:     *out = dev->exposure_us; break;
    case ControlId::Iso:              *out = dev->iso; break;
    case ControlId::AutoFocus:        *out = dev->auto_focus ? 1. : 0.; break;
    case ControlId::Focus:            *out = dev->focus; break;
    case ControlId::AutoWhiteBalance: *out = dev->auto_white_balance ? 1. : 0.; break;
    case ControlId::WhiteBalance:     *out = dev->white_balance; break;
    case ControlId::Brightness:       *out = dev->brightness; break;
    case ControlId::Contrast:         *out = dev->contrast; break;
    case ControlId::Saturation:       *out = dev->saturation; break;
    case ControlId::Sharpness:        *out = dev->sharpness; break;
    case ControlId::LumaDenoise:      *out = dev->luma_denoise; break;
    case ControlId::ChromaDenoise:    *out = dev->chroma_denoise; break;
    case ControlId::LaserPower:       *out = dev->laser_power; break;
    case ControlId::FloodLight:       *out = dev->flood_light; break;
    case ControlId::Temperature:      return 0; // handled above
  }
  return 1;
}

int backend_set_control(depthcam_device* dev, const char* id, double value)
{
  if(!dev || !id)
    return 0;

  if(dev->stereo)
    if(const auto* sc = find_stereo_control(id))
    {
      std::lock_guard lock{dev->control_lock};
      if(sc->flags & DEPTHCAM_CONTROL_REQUIRES_RESTART)
      {
        auto cfg = dev->stereoConfig;
        cfg.postProcessing.decimationFilter = dev->pendingDecimation;
        sc->set(cfg, value);
        dev->pendingDecimation = cfg.postProcessing.decimationFilter;
        // Sent by the next backend_start.
        return 1;
      }
      sc->set(dev->stereoConfig, value);
      dev->applyStereoConfig();
      return 1;
    }

  const ControlDef* def = dev->findControl(id);
  if(!def || !(def->access & DEPTHCAM_ACCESS_WRITE))
    return 0;

  // Device calls, not pipeline messages.
  if(def->id == ControlId::LaserPower || def->id == ControlId::FloodLight)
  {
    const float v = float(std::clamp(value, 0., 1.));
    try
    {
      std::lock_guard lock{dev->control_lock};
      if(def->id == ControlId::LaserPower)
      {
        if(!dev->device->setIrLaserDotProjectorIntensity(v))
          return 0;
        dev->laser_power = v;
      }
      else
      {
        if(!dev->device->setIrFloodLightIntensity(v))
          return 0;
        dev->flood_light = v;
      }
      return 1;
    }
    catch(...)
    {
      return 0;
    }
  }

  std::lock_guard lock{dev->control_lock};
  switch(def->id)
  {
    case ControlId::AutoExposure:
      dev->auto_exposure = value != 0.;
      break;
    // Writing a manual value implies manual mode.
    case ControlId::ExposureTime:
      dev->exposure_us = std::clamp(value, def->min, def->max);
      dev->auto_exposure = false;
      break;
    case ControlId::Iso:
      dev->iso = std::clamp(value, def->min, def->max);
      dev->auto_exposure = false;
      break;
    case ControlId::AutoFocus:
      dev->auto_focus = value != 0.;
      break;
    case ControlId::Focus:
      dev->focus = std::clamp(value, def->min, def->max);
      dev->auto_focus = false;
      break;
    case ControlId::AutoWhiteBalance:
      dev->auto_white_balance = value != 0.;
      break;
    case ControlId::WhiteBalance:
      dev->white_balance = std::clamp(value, def->min, def->max);
      dev->auto_white_balance = false;
      break;
    case ControlId::Brightness:
      dev->brightness = std::clamp(value, def->min, def->max);
      break;
    case ControlId::Contrast:
      dev->contrast = std::clamp(value, def->min, def->max);
      break;
    case ControlId::Saturation:
      dev->saturation = std::clamp(value, def->min, def->max);
      break;
    case ControlId::Sharpness:
      dev->sharpness = std::clamp(value, def->min, def->max);
      break;
    case ControlId::LumaDenoise:
      dev->luma_denoise = std::clamp(value, def->min, def->max);
      break;
    case ControlId::ChromaDenoise:
      dev->chroma_denoise = std::clamp(value, def->min, def->max);
      break;
    default:
      return 0;
  }

  dev->applyCameraControl();
  return 1;
}

uint32_t backend_active_streams(depthcam_device* dev)
{
  // What was actually wired, not what was asked for.
  return dev ? dev->active : 0;
}

// --- ABI 4 -------------------------------------------------------------------

bool is_depth(uint32_t s)
{
  return s == DEPTHCAM_STREAM_DEPTH || s == DEPTHCAM_STREAM_POINTCLOUD;
}

/// The intrinsics depthai itself attaches to a delivered frame:
/// getIntrinsicMatrix is the source sensor's matrix composed with every
/// crop/scale/warp applied since (common/ImgTransformations.hpp:171-175).
bool intrinsics_of(const dai::ImgTransformation& t, depthcam_intrinsics& r)
{
  const auto [w, h] = t.getSize();
  const auto k = t.getIntrinsicMatrix();
  if(w == 0 || h == 0 || k[0][0] <= 0.f || k[1][1] <= 0.f)
    return false;
  r.width = int32_t(w);
  r.height = int32_t(h);
  r.fx = k[0][0];
  r.fy = k[1][1];
  r.cx = k[0][2];
  r.cy = k[1][2];
  return distortion_of(t.getDistortionModel(), t.getDistortionCoefficients(), r);
}

/**
 * The estimate from the calibration, for before the first frame: the
 * camera's intrinsics rescaled to the requested size. The Camera outputs are
 * requested with ImgResizeMode::CROP, which keeps the aspect ratio and crops
 * the rest, and that is what getCameraIntrinsics computes with its default
 * keepAspectRatio = true (device/CalibrationHandler.hpp:101-127). It assumes a
 * centred crop of the calibrated full frame; a sensor mode with a smaller
 * field of view would not match, which the first frame then corrects.
 */
bool intrinsics_of(const dai::CalibrationHandler& calib, dai::CameraBoardSocket socket,
                   uint32_t w, uint32_t h, depthcam_intrinsics& r)
{
  if(socket == dai::CameraBoardSocket::AUTO || w == 0 || h == 0)
    return false;
  const auto k = calib.getCameraIntrinsics(socket, int(w), int(h));
  if(k.size() < 2 || k[0].size() < 3 || k[1].size() < 3)
    return false;
  r.width = int32_t(w);
  r.height = int32_t(h);
  r.fx = k[0][0];
  r.fy = k[1][1];
  r.cx = k[0][2];
  r.cy = k[1][2];
  return distortion_of(
      calib.getDistortionModel(socket), calib.getDistortionCoefficients(socket), r);
}

/**
 * Pixel convention: depthai's own deprojection is (col - cx) / fx with col
 * the integer pixel index (src/pipeline/node/PointCloud.cpp:436), which is the
 * ABI's convention as is.
 */
int backend_get_intrinsics(depthcam_device* dev, uint32_t stream, depthcam_intrinsics* out)
{
  if(!dev || !out)
    return 0;

  try
  {
    depthcam_intrinsics r{};
    r.struct_size = out->struct_size;

    std::optional<dai::ImgTransformation> t;
    {
      std::lock_guard lock{dev->transform_lock};
      if(stream == DEPTHCAM_STREAM_COLOR)
        t = dev->colorT;
      else if(stream == DEPTHCAM_STREAM_IR)
        t = dev->irT;
      else if(is_depth(stream))
        t = dev->depthT ? dev->depthT : (dev->aligned ? dev->colorT : std::nullopt);
      else
        return 0;
    }

    bool ok = false;
    if(t)
      ok = intrinsics_of(*t, r);
    else if(dev->calib)
    {
      if(stream == DEPTHCAM_STREAM_COLOR && dev->colorCam)
        ok = intrinsics_of(*dev->calib, dev->colorSocket, dev->colorW, dev->colorH, r);
      else if(stream == DEPTHCAM_STREAM_IR && dev->irCam)
        ok = intrinsics_of(*dev->calib, dev->irSocket, dev->irW, dev->irH, r);
      else if(is_depth(stream) && dev->aligned)
        // Aligned depth is delivered in the colour camera's image.
        ok = intrinsics_of(*dev->calib, dev->colorSocket, dev->colorW, dev->colorH, r);
      // Unaligned depth is rectified onto a camera and at a size the Depth
      // node picks internally (src/pipeline/node/Depth.cpp:686-726), and
      // nothing public says which before a frame arrives: no estimate.
    }
    if(!ok)
      return 0;

    std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
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

/**
 * @brief Where a stream's camera frame sits: a board socket, and the transform
 * from the stream's frame to that socket's camera frame, metres.
 */
std::optional<std::pair<dai::CameraBoardSocket, Mat4>>
frame_of(depthcam_device& dev, uint32_t stream)
{
  if(stream == DEPTHCAM_STREAM_COLOR && dev.colorCam)
    return std::make_pair(dev.colorSocket, identity4());
  if(stream == DEPTHCAM_STREAM_IR && dev.irCam)
    return std::make_pair(dev.irSocket, identity4());
  if(is_depth(stream))
  {
    if(dev.aligned && dev.colorCam)
      return std::make_pair(dev.colorSocket, identity4());

    // A rectified depth frame carries its pose relative to a physical camera,
    // the rectification rotation included (src/pipeline/node/PointCloud.cpp
    // :585-594 reads it exactly so). Extrinsics::getTransformationMatrix maps
    // into toCameraSocket (common/Extrinsics.hpp).
    std::lock_guard lock{dev.transform_lock};
    if(dev.depthT)
    {
      const auto e = dev.depthT->getExtrinsics();
      if(e.toCameraSocket != dai::CameraBoardSocket::AUTO)
        return std::make_pair(
            e.toCameraSocket, e.getTransformationMatrix(false, dai::LengthUnit::METER));
    }
  }
  return std::nullopt;
}

/// stream -> IMU, metres.
std::optional<Mat4> to_imu(depthcam_device& dev, uint32_t stream)
{
  const auto f = frame_of(dev, stream);
  if(!f || !dev.calib)
    return std::nullopt;
  // "from the camera to IMU" (device/CalibrationHandler.hpp:355-376).
  return mul(
      mat4_of(dev.calib->getCameraToImuExtrinsics(f->first, false, dai::LengthUnit::METER)),
      f->second);
}

int backend_get_extrinsics(
    depthcam_device* dev, uint32_t from, uint32_t to, depthcam_extrinsics* out)
{
  if(!dev || !out)
    return 0;

  try
  {
    Mat4 m = identity4();

    if(from == to || (is_depth(from) && is_depth(to)))
      ;
    else if(dev->aligned
            && ((is_depth(from) && to == DEPTHCAM_STREAM_COLOR)
                || (from == DEPTHCAM_STREAM_COLOR && is_depth(to))))
      ; // aligned depth is in the colour camera's frame
    else if(to == DEPTHCAM_STREAM_IMU)
    {
      const auto t = to_imu(*dev, from);
      if(!t)
        return 0;
      m = *t;
    }
    else if(from == DEPTHCAM_STREAM_IMU)
    {
      const auto t = to_imu(*dev, to);
      if(!t)
        return 0;
      m = inverse_rigid(*t);
    }
    else
    {
      const auto a = frame_of(*dev, from);
      const auto b = frame_of(*dev, to);
      if(!a || !b)
        return 0;

      Mat4 sockets = identity4();
      if(a->first != b->first)
      {
        if(!dev->calib)
          return 0;
        // getCameraExtrinsics(src, dst) composes src -> origin with the
        // inverse of dst -> origin, i.e. maps points from src's frame into
        // dst's (src/device/CalibrationHandler.cpp:518-556). Its translation
        // is centimetres unless a unit is asked for, and is rescaled to the
        // one passed here (CalibrationHandler.cpp:83-90).
        sockets = mat4_of(dev->calib->getCameraExtrinsics(
            a->first, b->first, false, dai::LengthUnit::METER));
      }
      m = mul(inverse_rigid(b->second), mul(sockets, a->second));
    }

    depthcam_extrinsics r{};
    r.struct_size = out->struct_size;
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        r.rotation[3 * i + j] = m[i][j];
    for(int i = 0; i < 3; i++)
      r.translation[i] = m[i][3];
    std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
    return 1;
  }
  catch(const std::exception& e)
  {
    // A camera pair the calibration does not link throws.
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
  // DeviceBase reports the bootloader's version, not the firmware's; the
  // firmware is the one embedded in depthai-core itself.
  r.firmware = "";
  r.hardware_rev = dev->hardwareRev.c_str();
  // Nothing depthai says gives the measuring range of the depth mode.
  r.depth_min_m = 0.f;
  r.depth_max_m = 0.f;
  std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
  return 1;
}

/**
 * Only for a camera already open in this process: what sensors an OAK has is
 * known once it has booted (DeviceBase::getConnectedCameraFeatures), and
 * dai::DeviceInfo, all that discovery yields, carries none of it
 * (xlink/XLinkConnection.hpp:27-47). Booting one just to answer would upload
 * the firmware and claim the device.
 *
 * The modes listed are the sensor's native ones (CameraFeatures::configs,
 * common/CameraFeatures.hpp:15-23,72-74) at their highest rate. Depth is not
 * listed: its size is chosen inside the Depth node.
 */
int backend_list_profiles(const char* uri, uint32_t stream, depthcam_profile_cb cb, void* user)
{
  if(!uri || !cb)
    return 0;
  if(stream != DEPTHCAM_STREAM_COLOR && stream != DEPTHCAM_STREAM_IR)
    return 0;

  try
  {
    std::lock_guard lock{g_open_mutex};
    for(auto* dev : g_open_devices)
    {
      if(!dev || dev->uri != uri)
        continue;

      const auto socket
          = stream == DEPTHCAM_STREAM_COLOR ? dev->colorSocket : dev->irSocket;
      if(socket == dai::CameraBoardSocket::AUTO)
        return 0;

      for(const auto& f : dev->features)
      {
        if(f.socket != socket)
          continue;
        for(const auto& c : f.configs)
        {
          if(c.width <= 0 || c.height <= 0)
            continue;
          depthcam_profile p{};
          p.struct_size = sizeof(p);
          p.width = c.width;
          p.height = c.height;
          p.fps = int32_t(std::lround(c.maxFps));
          // What this backend asks the Camera node for on each stream.
          p.format = stream == DEPTHCAM_STREAM_COLOR ? DEPTHCAM_FMT_RGB24
                                                     : DEPTHCAM_FMT_GRAY8;
          cb(&p, user);
        }
        return 1;
      }
      return 0;
    }
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
    .name = "depthai",
    .display_name = "Luxonis",
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
