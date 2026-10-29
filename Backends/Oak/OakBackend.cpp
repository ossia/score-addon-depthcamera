/*
 * Oak3DVision Oak CDK / QCDK backend: libPointCloud behind depthcam_abi.h.
 *
 * Melexis MLX7502x time-of-flight modules. No colour sensor and no IMU, so only
 * DEPTH, IR (the ToF amplitude image) and POINTCLOUD are published.
 *
 * libPointCloud is a vendored prebuilt binary (3rdparty/oaksdk); its camera
 * plug-ins and .conf/.dml files are loaded by path at runtime, which is what
 * init() below points the SDK at.
 */
#include <depthcam_abi.h>

#include <CameraSystem.h>
#include <DepthCamera.h>
#include <Frame.h>
#include <Logger.h>
#include <Parameter.h>
#include <Point.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#endif

/* The SDK's Windows binary is a release /MD build and passes std::vector and
 * std::string across the boundary, so a mismatched _ITERATOR_DEBUG_LEVEL is
 * silent memory corruption rather than a link error. */
#if defined(_MSC_VER) && defined(_ITERATOR_DEBUG_LEVEL) && _ITERATOR_DEBUG_LEVEL != 0
#error "The Oak backend must be built in a release configuration on MSVC: \
libPointCloud ships as a release /MD binary and std::vector's layout differs \
under _ITERATOR_DEBUG_LEVEL. Configure with -DCMAKE_BUILD_TYPE=Release, or \
turn the backend off with -DSCORE_DEPTHCAM_BUILD_OAK=OFF."
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

/**
 * One CameraSystem for the process: connect() only accepts a DevicePtr that
 * came out of that same object's scan().
 *
 * None of scan / connect / disconnect locks internally, hence g_system_mutex.
 * Recursive because ~depthcam_device disconnects and is run on backend_open's
 * failure path, which already holds the lock.
 */
std::unique_ptr<PointCloud::CameraSystem> g_system;
std::recursive_mutex g_system_mutex;

/// Every device open in this process, under g_system_mutex: list_profiles can
/// only ask a connected camera, and connecting is what open() is for.
std::vector<depthcam_device*> g_open_devices;

void set_env(const char* name, const std::string& value)
{
#if defined(_WIN32)
  _putenv_s(name, value.c_str());
#else
  ::setenv(name, value.c_str(), 1);
#endif
}

bool is_directory(const std::string& path)
{
  if(path.empty())
    return false;
#if defined(_WIN32)
  const DWORD a = GetFileAttributesA(path.c_str());
  return a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_DIRECTORY);
#else
  return PointCloud::isDirectory(path);
#endif
}

/// The SDK only knows a model's name once its plug-in has claimed the device,
/// which enumerate() must not do. These are the two ids its udev rules cover.
const char* model_for_product(const std::string& device_id)
{
  // Device::id() spells the USB device as "vendor:product", both lowercase hex.
  if(device_id.find(":9108") != std::string::npos)
    return "Oak CDK";
  if(device_id.find(":9107") != std::string::npos)
    return "Oak QCDK";
  return "Oak depth camera";
}

/// "oak:id:0::14b4:9107::0123", "oak:sn:0123" or "oak:index:0"; anything else
/// is taken as a bare serial.
struct Address
{
  enum Kind
  {
    Any,
    Id,
    Serial,
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
  if(s.rfind("oak:", 0) == 0)
    s = s.substr(4);
  if(s.empty())
    return a;

  if(s.rfind("id:", 0) == 0)
  {
    a.kind = Address::Id;
    a.text = s.substr(3);
  }
  else if(s.rfind("sn:", 0) == 0)
  {
    a.kind = Address::Serial;
    a.text = s.substr(3);
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
    a.kind = Address::Serial;
    a.text = s;
  }

  if(a.kind != Address::Index && a.kind != Address::Any && a.text.empty())
    a.kind = Address::Any;
  return a;
}

// --- controls ---------------------------------------------------------------

/**
 * The parameters worth publishing. getParameters() returns the model's whole
 * DML -- 307 raw register fields on an MLX75027, several of which stop the
 * sensor if written -- so this is a curated list, skipped where the model does
 * not have the parameter. SCORE_DEPTHCAM_OAK_ALL_CONTROLS=1 publishes all of
 * them, for bringing up a model this list predates.
 *
 * `sdk_name` is the SDK's spelling, typos included (`amp_threhold`); `id` is
 * the node path the host builds.
 */
struct CuratedControl
{
  const char* sdk_name;
  const char* id;
  const char* name;
  /// Used only when the SDK's own Parameter carries no unit (or cannot be
  /// reached; see describe_parameter).
  const char* unit;
  uint32_t flags;
};

/**
 * Units, where they are known rather than guessed. The SDK's DML files spell
 * none for these parameters (they are created by the camera plug-in, not the
 * DML), so they come from the shipped profiles: OakCDKCameraNormal.conf pairs
 * mod_freq1 = 100 and mod_freq2 = 80 with unambiguous_range = 7.45, which is
 * c / (2 * 20 MHz), the dealiased range of 100 and 80 MHz -- so the
 * frequencies are MHz and the ranges metres, and near_distance (0.3 there) is
 * metres with them.
 */
const CuratedControl curated_controls[] = {
    {"intg_time", "depth/integration_time", "integration time", nullptr, 0},
    {"intg_scale", "depth/integration_scale", "integration scale", nullptr,
     DEPTHCAM_CONTROL_ADVANCED},
    {"unambiguous_range", "depth/unambiguous_range", "unambiguous range", "m", 0},
    {"near_distance", "depth/near_distance", "near distance", "m", 0},
    {"measure_mode", "depth/measure_mode", "measure mode", nullptr,
     DEPTHCAM_CONTROL_ADVANCED},
    {"mod_freq1", "depth/modulation_frequency1", "modulation frequency 1", "MHz",
     DEPTHCAM_CONTROL_ADVANCED},
    {"mod_freq2", "depth/modulation_frequency2", "modulation frequency 2", "MHz",
     DEPTHCAM_CONTROL_ADVANCED},
    {"dealias_en", "depth/dealiasing", "dealiasing", nullptr,
     DEPTHCAM_CONTROL_ADVANCED},
    {"hdr_en", "depth/hdr", "HDR", nullptr, 0},
    // The profiles describe it as "enable Bilateral Filter and EdgeAware
    // Filter" (OakCDKCameraNormal.conf). The id predates the filters/ group and
    // is kept so saved documents still find it.
    {"filter_en", "depth/filtering", "on-camera filtering", nullptr,
     DEPTHCAM_CONTROL_FILTER},
    {"amp_threhold", "depth/amplitude_threshold", "amplitude threshold", nullptr, 0},
    {"binning_mode", "depth/binning_mode", "binning mode", nullptr, 0},
    // Under depth/, not a group of their own: the ToF sensor is the only one
    // on these modules, so the flip is a property of the depth stream (and of
    // the IR and the cloud derived from it) the way an Orbbec's depth/mirror
    // is.
    {"img_orientation_h", "depth/flip_horizontal", "flip horizontally", nullptr, 0},
    {"img_orientation_v", "depth/flip_vertical", "flip vertically", nullptr, 0},
    {"tsensor", "sensors/sensor_temperature", "sensor temperature", nullptr,
     DEPTHCAM_CONTROL_VOLATILE},
    {"tillum", "sensors/illumination_temperature", "illumination temperature",
     nullptr, DEPTHCAM_CONTROL_VOLATILE},
};

/// Not a Parameter: the SDK only exposes the rate through get/setFrameRate.
constexpr auto frame_rate_id = "depth/frame_rate";

/// One published control. Owns the storage its descriptor points into, because
/// depthcam_abi.h requires those strings to stay valid until close().
struct Control
{
  std::string sdk_name;
  std::string id;
  std::string name;
  std::string description;

  int32_t kind{DEPTHCAM_CONTROL_FLOAT};
  int32_t access{DEPTHCAM_ACCESS_READ};
  double min{}, max{}, step{}, def{};

  std::vector<std::string> labels;
  std::vector<const char*> label_ptrs;

  std::string unit;
  uint32_t flags{};

  /// Which DepthCamera::get<T>/set<T> reaches this parameter. The SDK compiled
  /// only these four instantiations.
  enum ValueType
  {
    Bool,
    Int,
    UInt,
    Float,
    /// Not a Parameter at all; see frame_rate_id.
    FrameRate
  } value_type{Float};
};

int32_t access_of(PointCloud::Parameter::IOType io)
{
  switch(io)
  {
    case PointCloud::Parameter::IO_READ_ONLY:
      return DEPTHCAM_ACCESS_READ;
    case PointCloud::Parameter::IO_WRITE_ONLY:
      return DEPTHCAM_ACCESS_WRITE;
    default:
      return DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE;
  }
}

/**
 * @brief Fill in kind, range and labels from the parameter's concrete type.
 *
 * Order matters: BoolParameter and EnumParameter both derive from
 * EnumParameterTemplate, and IntegerParameter from RangeParameterTemplate<int>,
 * so the most derived types have to be tried first.
 *
 * Every cast here returns null on x86_64 macOS: libc++ compares type_info by
 * address, and this module exports one symbol, so our copies never coalesce
 * with libpointcloud's. probe_parameter is the fallback.
 *
 * @return false if the type was not recognised; the caller probes instead of
 *         publishing a made-up range.
 */
bool describe_parameter(const PointCloud::ParameterPtr& p, Control& c)
{
  using namespace PointCloud;

  if(dynamic_cast<BoolParameter*>(p.get()))
  {
    c.kind = DEPTHCAM_CONTROL_BOOL;
    c.value_type = Control::Bool;
    c.min = 0;
    c.max = 1;
    c.step = 1;
    return true;
  }

  if(auto* e = dynamic_cast<EnumParameter*>(p.get()))
  {
    const auto& allowed = e->allowedValues();
    const auto& meanings = e->valueMeaning();

    // depthcam_abi.h indexes enum labels from `min` in steps of `step`, so a
    // set of allowed values with a hole in it cannot be an enum; those degrade
    // to a plain int with the choices spelled out in the description.
    const bool contiguous
        = !allowed.empty() && allowed.size() == meanings.size()
          && [&] {
               for(std::size_t i = 1; i < allowed.size(); i++)
                 if(allowed[i] != allowed[i - 1] + 1)
                   return false;
               return true;
             }();

    c.value_type = Control::Int;
    if(contiguous)
    {
      c.kind = DEPTHCAM_CONTROL_ENUM;
      c.min = allowed.front();
      c.max = allowed.back();
      c.step = 1;
      c.labels.assign(meanings.begin(), meanings.end());
      return true;
    }

    c.kind = DEPTHCAM_CONTROL_INT;
    c.step = 1;
    if(!allowed.empty())
    {
      c.min = *std::min_element(allowed.begin(), allowed.end());
      c.max = *std::max_element(allowed.begin(), allowed.end());
      for(std::size_t i = 0; i < allowed.size(); i++)
      {
        c.description += (i ? ", " : " (");
        c.description += std::to_string(allowed[i]);
        if(i < meanings.size() && !meanings[i].empty())
          c.description += " = " + meanings[i];
      }
      c.description += ")";
    }
    return true;
  }

  if(auto* r = dynamic_cast<RangeParameterTemplate<int>*>(p.get()))
  {
    c.kind = DEPTHCAM_CONTROL_INT;
    c.value_type = Control::Int;
    c.min = r->lowerLimit();
    c.max = r->upperLimit();
    c.step = 1;
    c.unit = r->unit();
    return true;
  }

  if(auto* r = dynamic_cast<RangeParameterTemplate<uint>*>(p.get()))
  {
    c.kind = DEPTHCAM_CONTROL_INT;
    c.value_type = Control::UInt;
    c.min = r->lowerLimit();
    c.max = r->upperLimit();
    c.step = 1;
    c.unit = r->unit();
    return true;
  }

  if(auto* r = dynamic_cast<RangeParameterTemplate<float>*>(p.get()))
  {
    c.kind = DEPTHCAM_CONTROL_FLOAT;
    c.value_type = Control::Float;
    c.min = r->lowerLimit();
    c.max = r->upperLimit();
    c.step = 0;
    c.unit = r->unit();
    return true;
  }

  // No range, but still a value get()/set() can carry: published unbounded.
  if(dynamic_cast<ParameterTemplate<bool>*>(p.get()))
  {
    c.kind = DEPTHCAM_CONTROL_BOOL;
    c.value_type = Control::Bool;
    c.min = 0;
    c.max = 1;
    c.step = 1;
    return true;
  }
  if(dynamic_cast<ParameterTemplate<int>*>(p.get()))
  {
    c.kind = DEPTHCAM_CONTROL_INT;
    c.value_type = Control::Int;
    c.step = 1;
    return true;
  }
  if(dynamic_cast<ParameterTemplate<uint>*>(p.get()))
  {
    c.kind = DEPTHCAM_CONTROL_INT;
    c.value_type = Control::UInt;
    c.step = 1;
    return true;
  }
  if(dynamic_cast<ParameterTemplate<float>*>(p.get()))
  {
    c.kind = DEPTHCAM_CONTROL_FLOAT;
    c.value_type = Control::Float;
    c.step = 0;
    return true;
  }

  return false;
}

/**
 * @brief Ask the SDK what type a parameter is, when our own RTTI cannot.
 *
 * DepthCamera::get<T> casts inside libpointcloud against its own type
 * information, so exactly one of these four succeeds. Range and enum labels are
 * lost -- they have no accessor on the base -- so the control is unbounded.
 *
 * @return false for a write-only parameter or a type get() cannot carry.
 */
bool probe_parameter(
    PointCloud::DepthCamera& cam, const PointCloud::String& name, Control& c)
{
  if(bool v{}; cam.get(name, v))
  {
    c.kind = DEPTHCAM_CONTROL_BOOL;
    c.value_type = Control::Bool;
    c.min = 0;
    c.max = 1;
    c.step = 1;
    return true;
  }
  if(int v{}; cam.get(name, v))
  {
    c.kind = DEPTHCAM_CONTROL_INT;
    c.value_type = Control::Int;
    c.step = 1;
    return true;
  }
  if(uint v{}; cam.get(name, v))
  {
    c.kind = DEPTHCAM_CONTROL_INT;
    c.value_type = Control::UInt;
    c.step = 1;
    return true;
  }
  if(float v{}; cam.get(name, v))
  {
    c.kind = DEPTHCAM_CONTROL_FLOAT;
    c.value_type = Control::Float;
    c.step = 0;
    return true;
  }
  return false;
}

/// Owns the buffer handed to the host for as long as it holds the frame. A
/// copy is unavoidable: the SDK recycles its FrameBufferManager slot as soon as
/// the callback returns, while the host may hold a frame for several passes.
struct VectorHolder
{
  std::vector<float> data;
};

void release_vector_holder(void* owner)
{
  delete static_cast<VectorHolder*>(owner);
}

} // namespace

// ---------------------------------------------------------------------------

struct depthcam_device
{
  PointCloud::DevicePtr device;
  PointCloud::DepthCameraPtr camera;

  std::string uri, name, serial;

  depthcam_open_config cfg{};
  std::atomic_bool running{};

  depthcam_frame_cb on_frame{};
  void* user{};

  int32_t width{}, height{};
  /// The size of the last depth frame, which binning_mode can change while
  /// streaming; 0 until one arrives.
  std::atomic<uint32_t> frame_w{}, frame_h{};

  /// Which opened streams the host consumes; see backend_set_stream_demand.
  std::atomic<uint32_t> demand{~0u};

  std::string chipset;

  std::vector<Control> controls;

  ~depthcam_device();

  void onDepthFrame(const PointCloud::Frame& frame);
  void onPointCloudFrame(const PointCloud::Frame& frame);

  void emit(uint32_t stream, int32_t format, std::vector<float>&& data,
            int32_t w, int32_t h, uint64_t timestamp_ns, float unit_mm,
            int32_t point_count);

  const Control* findControl(const char* id) const;
  void buildControls();
};

depthcam_device::~depthcam_device()
{
  try
  {
    std::lock_guard<std::recursive_mutex> lock{g_system_mutex};
    g_open_devices.erase(
        std::remove(g_open_devices.begin(), g_open_devices.end(), this),
        g_open_devices.end());
  }
  catch(...)
  {
  }

  if(!camera)
    return;

  // The callbacks capture `this`, and stop() only joins the capture thread --
  // it unregisters nothing. Both, in this order, before the object goes away.
  try
  {
    camera->stop();
    camera->clearAllCallbacks();
  }
  catch(...)
  {
  }

  try
  {
    std::lock_guard<std::recursive_mutex> lock{g_system_mutex};
    if(g_system)
      g_system->disconnect(camera, true);
  }
  catch(...)
  {
  }
  camera.reset();
}

void depthcam_device::emit(
    uint32_t stream, int32_t format, std::vector<float>&& data, int32_t w,
    int32_t h, uint64_t timestamp_ns, float unit_mm, int32_t point_count)
{
  if(data.empty() || !on_frame)
    return;

  auto* holder = new VectorHolder{std::move(data)};

  depthcam_frame out{};
  out.stream = stream;
  out.format = format;
  out.width = w;
  out.height = h;
  out.stride = (point_count == 0) ? int32_t(w * sizeof(float)) : 0;
  out.timestamp_ns = timestamp_ns;
  out.data = holder->data.data();
  out.bytes = holder->data.size() * sizeof(float);
  out.depth_unit_mm = unit_mm;
  out.point_count = point_count;
  out.owner = holder;
  out.release = &release_vector_holder;

  on_frame(&out, user);
}

void depthcam_device::onDepthFrame(const PointCloud::Frame& frame)
{
  if(!running.load(std::memory_order_acquire) || !on_frame)
    return;

  // static_cast, not dynamic_cast: on x86_64 macOS our type_info never
  // coalesces with libpointcloud's (see describe_parameter) and every frame
  // would silently be dropped. The caller checks the FrameType instead, and a
  // FRAME_DEPTH_FRAME is always a DepthFrame.
  const auto& d = static_cast<const PointCloud::DepthFrame&>(frame);

  const auto w = int32_t(d.size.width), h = int32_t(d.size.height);
  if(w <= 0 || h <= 0)
    return;

  frame_w.store(uint32_t(w), std::memory_order_relaxed);
  frame_h.store(uint32_t(h), std::memory_order_relaxed);

  // Frame::timestamp is documented as a Unix timestamp in microseconds.
  const uint64_t ts = uint64_t(d.timestamp) * 1000ull;
  const uint32_t wanted = demand.load(std::memory_order_relaxed);

  if((cfg.streams & DEPTHCAM_STREAM_DEPTH) && (wanted & DEPTHCAM_STREAM_DEPTH)
     && d.depth.size() == std::size_t(w) * h)
  {
    // 1000, not 1: DepthFrame::depth is in metres, and depth_unit_mm is what
    // the host multiplies a sample by to get millimetres.
    emit(DEPTHCAM_STREAM_DEPTH, DEPTHCAM_FMT_GRAYF32,
         std::vector<float>{d.depth}, w, h, ts, 1000.f, 0);
  }

  // The ToF amplitude image is this camera's infrared, normalised to 0..1 by
  // the SDK. There is no separate IR sensor, so cfg.ir_* are ignored.
  if((cfg.streams & DEPTHCAM_STREAM_IR) && (wanted & DEPTHCAM_STREAM_IR)
     && d.amplitude.size() == std::size_t(w) * h)
  {
    emit(DEPTHCAM_STREAM_IR, DEPTHCAM_FMT_GRAYF32,
         std::vector<float>{d.amplitude}, w, h, ts, 0.f, 0);
  }
}

void depthcam_device::onPointCloudFrame(const PointCloud::Frame& frame)
{
  if(!running.load(std::memory_order_acquire) || !on_frame)
    return;
  if(!(cfg.streams & DEPTHCAM_STREAM_POINTCLOUD)
     || !(demand.load(std::memory_order_relaxed) & DEPTHCAM_STREAM_POINTCLOUD))
    return;

  // static_cast for the same reason as in onDepthFrame:
  // _convertToPointCloudFrame fills an XYZIPointCloudFrame and the caller has
  // already checked the frame type.
  const auto& pc = static_cast<const PointCloud::XYZIPointCloudFrame&>(frame);
  if(pc.points.empty())
    return;

  // No colour sensor, so a "coloured" cloud is the ToF amplitude carried as
  // grey -- already normalised to 0..1, which is what XYZRGB asks for.
  const bool tinted = cfg.color_pointcloud != 0;
  const int floats_per_point = tinted ? 6 : 3;

  std::vector<float> out;
  out.reserve(pc.points.size() * floats_per_point);

  for(const auto& p : pc.points)
  {
    // PointCloudTransform marks an unsolved pixel POINT_INVALID (FLT_MAX, which
    // is finite) and an under-threshold amplitude NaN. Neither is a measurement.
    if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    if(p.z >= FLT_MAX || p.x >= FLT_MAX || p.y >= FLT_MAX)
      continue;

    // Already metres and already +X right / +Y down / +Z forward, unlike the
    // four SDKs here that work in millimetres.
    out.push_back(p.x);
    out.push_back(p.y);
    out.push_back(p.z);

    if(tinted)
    {
      const float i = std::isfinite(p.i) ? std::clamp(p.i, 0.f, 1.f) : 0.f;
      out.push_back(i);
      out.push_back(i);
      out.push_back(i);
    }
  }

  if(out.empty())
    return;

  const int32_t count = int32_t(out.size() / floats_per_point);
  emit(DEPTHCAM_STREAM_POINTCLOUD,
       tinted ? DEPTHCAM_FMT_XYZRGB : DEPTHCAM_FMT_XYZ, std::move(out), 0, 0,
       uint64_t(pc.timestamp) * 1000ull, 0.f, count);
}

const Control* depthcam_device::findControl(const char* id) const
{
  if(!id)
    return nullptr;
  for(const auto& c : controls)
    if(c.id == id)
      return &c;
  return nullptr;
}

void depthcam_device::buildControls()
{
  controls.clear();
  if(!camera)
    return;

  const auto& params = camera->getParameters();

  {
    Control c;
    c.id = frame_rate_id;
    c.name = "frame rate";
    c.description = "Frames per second";
    c.kind = DEPTHCAM_CONTROL_FLOAT;
    c.value_type = Control::FrameRate;
    c.access = DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE;
    c.min = 1;
    c.max = 60;
    c.step = 0;

    PointCloud::FrameRate r{};
    if(camera->getFrameRate(r))
      c.def = r.getFrameRate();

    // getMaximumFrameRate is per frame size.
    PointCloud::FrameSize s{};
    PointCloud::FrameRate max{};
    if(camera->getFrameSize(s) && camera->getMaximumFrameRate(max, s))
      if(const float f = max.getFrameRate(); f > 0)
        c.max = f;

    controls.push_back(std::move(c));
  }

  const bool all = [] {
    const char* e = std::getenv("SCORE_DEPTHCAM_OAK_ALL_CONTROLS");
    return e && *e && std::strcmp(e, "0") != 0;
  }();

  const auto add = [&](const std::string& sdk_name, const std::string& id,
                       const std::string& label, const char* unit,
                       uint32_t flags) {
    const auto it = params.find(sdk_name);
    if(it == params.end() || !it->second)
      return;

    Control c;
    c.sdk_name = sdk_name;
    c.id = id;
    c.name = label;
    c.access = access_of(it->second->ioType());

    if(!describe_parameter(it->second, c)
       && !probe_parameter(*camera, sdk_name, c))
      return;

    // Prepended: describe_parameter may already have appended the allowed
    // values of an enum it could not represent.
    const auto& sdk_desc = it->second->description();
    if(!sdk_desc.empty())
      c.description = sdk_desc + c.description;

    if(c.unit.empty() && unit)
      c.unit = unit;
    c.flags = flags;
    // A read-only register is a measurement: the temperatures, mostly.
    if(c.access == DEPTHCAM_ACCESS_READ)
      c.flags |= DEPTHCAM_CONTROL_VOLATILE;

    // After shrink_to_fit, which would otherwise move the strings the pointers
    // are taken from. Moving `c` into `controls` afterwards keeps them valid:
    // the labels vector's buffer moves as a whole.
    c.labels.shrink_to_fit();
    c.label_ptrs.reserve(c.labels.size());
    for(const auto& l : c.labels)
      c.label_ptrs.push_back(l.c_str());

    controls.push_back(std::move(c));
  };

  if(all)
  {
    for(const auto& [sdk_name, p] : params)
    {
      if(!p)
        continue;
      const auto& display = p->displayName();
      add(sdk_name, "advanced/" + sdk_name, display.empty() ? sdk_name : display,
          nullptr, DEPTHCAM_CONTROL_ADVANCED);
    }
  }
  else
  {
    for(const auto& d : curated_controls)
      add(d.sdk_name, d.id, d.name, d.unit, d.flags);
  }

  // Parameter holds the real default but does not expose it, so the host is
  // told the value at open time.
  for(auto& c : controls)
  {
    if(c.value_type == Control::FrameRate || !(c.access & DEPTHCAM_ACCESS_READ))
      continue;
    switch(c.value_type)
    {
      case Control::Bool:
      {
        bool v{};
        if(camera->get(c.sdk_name, v))
          c.def = v ? 1. : 0.;
        break;
      }
      case Control::Int:
      {
        int v{};
        if(camera->get(c.sdk_name, v))
          c.def = v;
        break;
      }
      case Control::UInt:
      {
        uint v{};
        if(camera->get(c.sdk_name, v))
          c.def = v;
        break;
      }
      case Control::Float:
      {
        float v{};
        if(camera->get(c.sdk_name, v))
          c.def = v;
        break;
      }
      default:
        break;
    }
  }
}

// ---------------------------------------------------------------------------

namespace
{

int backend_init(const char* resource_dir)
{
  try
  {
    /*
     * libPointCloud resolves its conf directory as
     * "<libpointcloud dir>/../share/pointcloud-1.0.0/conf", which in a package
     * where the SDK sits beside this module lands one level above the package.
     * Without the .conf/.dml files a camera enumerates and then fails to
     * connect. Must precede the CameraSystem below: that is what loads them.
     */
    if(resource_dir && *resource_dir)
    {
      const std::string root{resource_dir};
      if(const auto plugin = root + "/oak/plugin"; is_directory(plugin))
        set_env("POINTCLOUD_PLUGIN_PATH", plugin);
      if(const auto conf = root + "/oak/conf"; is_directory(conf))
        set_env("POINTCLOUD_CONF_PATH", conf);
    }

#if defined(_WIN32)
    /*
     * pointcloud.dll is delay-loaded (see Backends/CMakeLists.txt) purely so
     * that this can run first: the process search order does not include the
     * directory this module came from, so a DLL sitting right beside it is not
     * found. LOAD_WITH_ALTERED_SEARCH_PATH then covers its own dependencies.
     */
    if(resource_dir && *resource_dir)
    {
      const std::string dll = std::string{resource_dir} + "\\pointcloud.dll";
      if(!GetModuleHandleA("pointcloud.dll")
         && !LoadLibraryExA(dll.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH))
      {
        set_error(
            "could not load pointcloud.dll from the package directory (error "
            + std::to_string(GetLastError())
            + "). The Oak SDK is built against the Visual C++ runtime and needs "
              "OpenCL.dll from a graphics driver.");
        return 0;
      }
    }
#endif

    // The SDK logs to std::cerr and narrates every USB transaction at LOG_INFO;
    // stated so a stray environment cannot make the host chatty.
    PointCloud::logger.setDefaultLogLevel(PointCloud::LOG_ERROR);

    std::lock_guard<std::recursive_mutex> lock{g_system_mutex};
    g_system = std::make_unique<PointCloud::CameraSystem>();
    return 1;
  }
  catch(const std::exception& e)
  {
    set_error(e.what());
    return 0;
  }
  catch(...)
  {
    set_error("unknown error while initialising the Oak SDK");
    return 0;
  }
}

void backend_shutdown()
{
  try
  {
    std::lock_guard<std::recursive_mutex> lock{g_system_mutex};
    g_system.reset();
  }
  catch(...)
  {
  }
}

int backend_enumerate(depthcam_enumerate_cb cb, void* user)
{
  if(!cb)
    return 0;

  try
  {
    std::lock_guard<std::recursive_mutex> lock{g_system_mutex};
    if(!g_system)
      return 0;

    for(const auto& d : g_system->scan())
    {
      if(!d)
        continue;

      const std::string uri = "oak:id:" + d->id();
      const std::string serial = d->serialNumber();
      const std::string desc = d->description();
      const std::string name
          = desc.empty() ? model_for_product(d->deviceID()) : desc;

      depthcam_device_info info{};
      info.backend = "oak";
      info.uri = uri.c_str();
      info.name = name.c_str();
      info.serial = serial.c_str();
      // The SDK does not report the negotiated speed; both models are USB 2.0.
      info.transport = "usb2";
      info.streams = DEPTHCAM_STREAM_IR | DEPTHCAM_STREAM_DEPTH
                     | DEPTHCAM_STREAM_POINTCLOUD;
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

depthcam_device* backend_open(const char* uri, const depthcam_open_config* config)
{
  if(!config)
    return nullptr;

  try
  {
    const auto addr = parse_uri(uri);

    std::lock_guard<std::recursive_mutex> lock{g_system_mutex};
    if(!g_system)
    {
      set_error("the Oak SDK is not initialised");
      return nullptr;
    }

    // Rescanned rather than cached from enumerate(): connect() looks the device
    // up in the CameraSystem's factory map, and a DevicePtr from before a
    // replug names something that is no longer there.
    const auto devices = g_system->scan();
    if(devices.empty())
    {
      set_error("no Oak depth camera connected");
      return nullptr;
    }

    PointCloud::DevicePtr chosen;
    switch(addr.kind)
    {
      case Address::Id:
        for(const auto& d : devices)
          if(d && d->id() == addr.text)
            chosen = d;
        break;
      case Address::Serial:
        for(const auto& d : devices)
          if(d && d->serialNumber() == addr.text)
            chosen = d;
        break;
      case Address::Index:
        if(addr.index >= 0 && std::size_t(addr.index) < devices.size())
          chosen = devices[addr.index];
        break;
      case Address::Any:
        chosen = devices.front();
        break;
    }

    if(!chosen)
    {
      // No silent fallback to another camera. Worded as the other backends
      // word it, so the two ways of missing read the same everywhere.
      if(addr.kind == Address::Index)
        set_error("no Oak camera at index " + std::to_string(addr.index));
      else
        set_error("Oak camera '" + addr.text + "' is not connected");
      return nullptr;
    }

    auto dev = std::make_unique<depthcam_device>();
    dev->cfg = *config;
    dev->device = chosen;
    dev->uri = "oak:id:" + chosen->id();
    dev->serial = chosen->serialNumber();
    dev->name = model_for_product(chosen->deviceID());

    dev->camera = g_system->connect(chosen);
    if(!dev->camera)
    {
      set_error(
          "could not load a camera plug-in for " + chosen->id()
          + ". Check that the SDK's plugin/ and conf/ directories shipped with "
            "this package.");
      return nullptr;
    }

    if(!dev->camera->isInitialized())
    {
      set_error("the Oak camera " + chosen->id() + " did not initialise");
      return nullptr;
    }

    // Resolution before frame rate: the maximum rate depends on the frame size.
    if(config->depth_width > 0 && config->depth_height > 0)
    {
      PointCloud::FrameSize s{};
      s.width = uint32_t(config->depth_width);
      s.height = uint32_t(config->depth_height);
      if(!dev->camera->setFrameSize(s))
        set_error(
            "the Oak camera refused " + std::to_string(config->depth_width) + "x"
            + std::to_string(config->depth_height)
            + "; streaming at its configured size instead");
    }

    if(config->depth_fps > 0)
    {
      PointCloud::FrameRate r{};
      r.numerator = uint32_t(config->depth_fps);
      r.denominator = 1;
      if(!dev->camera->setFrameRate(r))
        set_error(
            "the Oak camera refused " + std::to_string(config->depth_fps)
            + " fps; streaming at its configured rate instead");
    }

    PointCloud::FrameSize actual{};
    if(dev->camera->getFrameSize(actual))
    {
      dev->width = int32_t(actual.width);
      dev->height = int32_t(actual.height);
    }

    dev->chipset = dev->camera->chipset();
    dev->buildControls();

    g_open_devices.push_back(dev.get());
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
  if(!dev || !dev->camera)
    return 0;
  if(dev->running.load(std::memory_order_acquire))
    return 1;

  try
  {
    dev->on_frame = on_frame;
    dev->user = user;
    dev->running.store(true, std::memory_order_release);

    // Only the callbacks the host asked for: _callbackAndContinue stops
    // processing a frame at the last stage anyone registered for, so a document
    // that wants only depth does not pay for the point-cloud transform. At
    // least one has to be registered before start().
    const bool want_depth
        = (dev->cfg.streams & (DEPTHCAM_STREAM_DEPTH | DEPTHCAM_STREAM_IR)) != 0;
    const bool want_cloud
        = (dev->cfg.streams & DEPTHCAM_STREAM_POINTCLOUD) != 0;

    if(!want_depth && !want_cloud)
    {
      set_error(
          "an Oak camera can only produce depth, infrared and point clouds; "
          "none of them was requested");
      dev->running.store(false, std::memory_order_release);
      return 0;
    }

    if(want_depth)
    {
      dev->camera->registerCallback(
          PointCloud::DepthCamera::FRAME_DEPTH_FRAME,
          [dev](PointCloud::DepthCamera&, const PointCloud::Frame& f,
                PointCloud::DepthCamera::FrameType t) {
            // Checked, not assumed from the registration: this is what makes
            // the static_cast inside onDepthFrame safe.
            if(t != PointCloud::DepthCamera::FRAME_DEPTH_FRAME)
              return;

            // Nothing may escape into the SDK's capture thread.
            try
            {
              dev->onDepthFrame(f);
            }
            catch(...)
            {
            }
          });
    }

    // Registered whatever the stream demand: the cloud has to come back the
    // moment the host wants it again, and nothing in DepthCamera.h says a
    // callback may be (un)registered while the capture thread runs. So with
    // the demand off, the SDK's transform still runs and only the conversion
    // in onPointCloudFrame is skipped.
    if(want_cloud)
    {
      dev->camera->registerCallback(
          PointCloud::DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME,
          [dev](PointCloud::DepthCamera&, const PointCloud::Frame& f,
                PointCloud::DepthCamera::FrameType t) {
            if(t != PointCloud::DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME)
              return;

            try
            {
              dev->onPointCloudFrame(f);
            }
            catch(...)
            {
            }
          });
    }

    if(!dev->camera->start())
    {
      set_error("could not start the Oak camera");
      dev->camera->clearAllCallbacks();
      dev->running.store(false, std::memory_order_release);
      return 0;
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
  if(!dev->running.exchange(false, std::memory_order_acq_rel))
    return;
  try
  {
    if(dev->camera)
    {
      dev->camera->stop();
      // stop() joins the capture thread but leaves the callbacks, which capture
      // `dev`. backend_start re-registers them.
      dev->camera->clearAllCallbacks();
    }
  }
  catch(...)
  {
  }
}

depthcam_control base_of(const Control& c)
{
  depthcam_control out{};
  out.id = c.id.c_str();
  out.name = c.name.c_str();
  out.description = c.description.empty() ? nullptr : c.description.c_str();
  out.kind = c.kind;
  out.access = c.access;
  out.min = c.min;
  out.max = c.max;
  out.step = c.step;
  out.def = c.def;
  out.enum_labels = c.label_ptrs.empty() ? nullptr : c.label_ptrs.data();
  out.enum_count = int32_t(c.label_ptrs.size());
  return out;
}

int backend_list_controls(depthcam_device* dev, depthcam_control_cb cb, void* user)
{
  if(!dev || !cb)
    return 0;

  for(const auto& c : dev->controls)
  {
    const depthcam_control out = base_of(c);
    cb(&out, user);
  }
  return 1;
}

int backend_list_controls2(depthcam_device* dev, depthcam_control2_cb cb, void* user)
{
  if(!dev || !cb)
    return 0;

  for(const auto& c : dev->controls)
  {
    depthcam_control2 out{};
    out.struct_size = sizeof(out);
    out.base = base_of(c);
    out.unit = c.unit.empty() ? nullptr : c.unit.c_str();
    out.flags = c.flags;
    cb(&out, user);
  }
  return 1;
}

int backend_get_control(depthcam_device* dev, const char* id, double* out)
{
  if(!dev || !dev->camera || !id || !out)
    return 0;

  const Control* c = dev->findControl(id);
  if(!c || !(c->access & DEPTHCAM_ACCESS_READ))
    return 0;

  try
  {
    switch(c->value_type)
    {
      case Control::FrameRate:
      {
        PointCloud::FrameRate r{};
        if(!dev->camera->getFrameRate(r))
          return 0;
        *out = r.getFrameRate();
        return 1;
      }
      case Control::Bool:
      {
        bool v{};
        // refresh=true throughout: these are device registers and the SDK's
        // cache is only as fresh as the last write.
        if(!dev->camera->get(c->sdk_name, v, true))
          return 0;
        *out = v ? 1. : 0.;
        return 1;
      }
      case Control::Int:
      {
        int v{};
        if(!dev->camera->get(c->sdk_name, v, true))
          return 0;
        *out = v;
        return 1;
      }
      case Control::UInt:
      {
        uint v{};
        if(!dev->camera->get(c->sdk_name, v, true))
          return 0;
        *out = v;
        return 1;
      }
      case Control::Float:
      {
        float v{};
        if(!dev->camera->get(c->sdk_name, v, true))
          return 0;
        *out = v;
        return 1;
      }
    }
    return 0;
  }
  catch(...)
  {
    return 0;
  }
}

int backend_set_control(depthcam_device* dev, const char* id, double value)
{
  if(!dev || !dev->camera || !id)
    return 0;

  const Control* c = dev->findControl(id);
  if(!c || !(c->access & DEPTHCAM_ACCESS_WRITE))
    return 0;

  try
  {
    switch(c->value_type)
    {
      case Control::FrameRate:
      {
        if(value < 1.)
          return 0;
        PointCloud::FrameRate r{};
        // The SDK models the rate as a rational; these cameras only do integers.
        r.numerator = uint32_t(std::lround(value));
        r.denominator = 1;
        return dev->camera->setFrameRate(r) ? 1 : 0;
      }
      case Control::Bool:
      {
        const bool v = value != 0.;
        return dev->camera->set(c->sdk_name, v) ? 1 : 0;
      }
      case Control::Int:
      {
        const int v = int(std::lround(value));
        return dev->camera->set(c->sdk_name, v) ? 1 : 0;
      }
      case Control::UInt:
      {
        if(value < 0.)
          return 0;
        const uint v = uint(std::lround(value));
        return dev->camera->set(c->sdk_name, v) ? 1 : 0;
      }
      case Control::Float:
      {
        const float v = float(value);
        return dev->camera->set(c->sdk_name, v) ? 1 : 0;
      }
    }
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
  // Neither model has a colour sensor or an IMU, whatever was asked for.
  return dev->cfg.streams
         & uint32_t(
             DEPTHCAM_STREAM_IR | DEPTHCAM_STREAM_DEPTH
             | DEPTHCAM_STREAM_POINTCLOUD);
}

// --- ABI 4 -------------------------------------------------------------------

/// A numeric parameter of whichever of the four types the SDK instantiates.
bool read_number(const PointCloud::DepthCamera& cam, const char* name, double& out)
{
  if(float v{}; cam.get(name, v))
  {
    out = v;
    return true;
  }
  if(uint v{}; cam.get(name, v))
  {
    out = v;
    return true;
  }
  if(int v{}; cam.get(name, v))
  {
    out = v;
    return true;
  }
  return false;
}

/**
 * @brief The lens calibration the SDK deprojects with.
 *
 * Best effort, and limited by what the vendored headers expose. The camera
 * profile's [calib] section holds fx, fy, cx, cy, k1, k2, k3, p1, p2
 * (OakCDKCameraNormal.conf, doc/ConfigurationFilesDescription.md), the same
 * nine numbers PointCloudFrameGenerator::setParameters takes
 * (PointCloudFrameGenerator.h:26-28) and that the vendor's ROS wrapper reads
 * back as parameters through DepthCamera::get and publishes as an OpenCV
 * "plumb_bob" camera_info (wrappers/pointcloud_ros/.../base_pointcloud_node.cpp
 * :452-475). That fixes the names and the model; three things no header in
 * 3rdparty/oaksdk states:
 *  - the resolution the numbers are for. They are taken to be the sensor's
 *    full frame (getMaximumFrameSize) and scaled to the delivered one, which
 *    is what PointCloudTransform::rescaleParameters exists for
 *    (PointCloudTransform.h:66);
 *  - whether they follow the img_orientation_h/v flips. They are calibrated
 *    with the profile that sets those flips, and are reported as they are;
 *  - whether DepthFrame::depth is a z-depth or a radial range: Frame.h:58
 *    says only "Unit: meters".
 */
int backend_get_intrinsics(depthcam_device* dev, uint32_t stream, depthcam_intrinsics* out)
{
  if(!dev || !dev->camera || !out)
    return 0;
  if(stream != DEPTHCAM_STREAM_DEPTH && stream != DEPTHCAM_STREAM_IR
     && stream != DEPTHCAM_STREAM_POINTCLOUD)
    return 0;

  try
  {
    double fx{}, fy{}, cx{}, cy{}, k1{}, k2{}, k3{}, p1{}, p2{};
    const auto& cam = *dev->camera;
    if(!read_number(cam, "fx", fx) || !read_number(cam, "fy", fy)
       || !read_number(cam, "cx", cx) || !read_number(cam, "cy", cy))
      return 0;
    if(fx <= 0. || fy <= 0.)
      return 0;
    // Absent coefficients are zero, as in the documentation's example profile.
    read_number(cam, "k1", k1);
    read_number(cam, "k2", k2);
    read_number(cam, "k3", k3);
    read_number(cam, "p1", p1);
    read_number(cam, "p2", p2);

    uint32_t w = dev->frame_w.load(std::memory_order_relaxed);
    uint32_t h = dev->frame_h.load(std::memory_order_relaxed);
    if(w == 0 || h == 0)
    {
      w = uint32_t(dev->width);
      h = uint32_t(dev->height);
    }
    if(w == 0 || h == 0)
      return 0;

    double sx = 1., sy = 1.;
    PointCloud::FrameSize full{};
    if(dev->camera->getMaximumFrameSize(full) && full.width > 0 && full.height > 0)
    {
      sx = double(w) / full.width;
      sy = double(h) / full.height;
    }

    depthcam_intrinsics r{};
    r.struct_size = out->struct_size;
    r.width = int32_t(w);
    r.height = int32_t(h);
    r.fx = float(fx * sx);
    r.fy = float(fy * sy);
    r.cx = float(cx * sx);
    r.cy = float(cy * sy);
    r.model = DEPTHCAM_DISTORTION_BROWN_CONRADY;
    r.coeffs[0] = float(k1);
    r.coeffs[1] = float(k2);
    r.coeffs[2] = float(k3);
    r.coeffs[6] = float(p1);
    r.coeffs[7] = float(p2);
    std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
    return 1;
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
  // One sensor: depth, IR (its amplitude) and the cloud share a frame. There
  // is nothing else on these modules to relate them to.
  const auto ours = [](uint32_t s) {
    return s == DEPTHCAM_STREAM_DEPTH || s == DEPTHCAM_STREAM_IR
           || s == DEPTHCAM_STREAM_POINTCLOUD;
  };
  if(!ours(from) || !ours(to))
    return 0;

  depthcam_extrinsics r{};
  r.struct_size = out->struct_size;
  r.rotation[0] = r.rotation[4] = r.rotation[8] = 1.f;
  std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
  return 1;
}

int backend_get_details(depthcam_device* dev, depthcam_device_details* out)
{
  if(!dev || !out)
    return 0;

  depthcam_device_details r{};
  r.struct_size = out->struct_size;
  r.model = dev->name.c_str();
  r.serial = dev->serial.c_str();
  // libPointCloud reports no firmware version.
  r.firmware = "";
  // The ToF sensor chip ("MLX75027"...), the closest thing to a hardware
  // revision DepthCamera exposes.
  r.hardware_rev = dev->chipset.c_str();

  // Both in metres; see curated_controls.
  try
  {
    if(dev->camera)
    {
      double near_m{}, range_m{};
      if(read_number(*dev->camera, "near_distance", near_m))
        r.depth_min_m = float(near_m);
      if(read_number(*dev->camera, "unambiguous_range", range_m))
        r.depth_max_m = float(range_m);
    }
  }
  catch(...)
  {
  }

  std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
  return 1;
}

/**
 * The SDK only lists modes for a connected camera
 * (DepthCamera::getSupportedVideoModes), and connecting claims it, so this
 * answers for a camera this process already has open and declines otherwise:
 * no table of modes ships with the SDK to fall back on.
 */
int backend_list_profiles(const char* uri, uint32_t stream, depthcam_profile_cb cb, void* user)
{
  if(!cb)
    return 0;
  if(stream != DEPTHCAM_STREAM_DEPTH && stream != DEPTHCAM_STREAM_IR)
    return 0;

  try
  {
    const auto addr = parse_uri(uri);

    std::lock_guard<std::recursive_mutex> lock{g_system_mutex};
    for(auto* dev : g_open_devices)
    {
      if(!dev || !dev->camera)
        continue;
      const bool match = (addr.kind == Address::Id && dev->uri == "oak:id:" + addr.text)
                         || (addr.kind == Address::Serial && dev->serial == addr.text);
      if(!match)
        continue;

      std::vector<PointCloud::SupportedVideoMode> modes;
      if(!dev->camera->getSupportedVideoModes(modes) || modes.empty())
        return 0;

      for(const auto& m : modes)
      {
        depthcam_profile p{};
        p.struct_size = sizeof(p);
        p.width = int32_t(m.frameSize.width);
        p.height = int32_t(m.frameSize.height);
        p.fps = int32_t(std::lround(m.getFrameRate()));
        // Depth and amplitude are both delivered as float images, whatever
        // the sensor's raw bytesPerPixel.
        p.format = DEPTHCAM_FMT_GRAYF32;
        cb(&p, user);
      }
      return 1;
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
    .name = "oak",
    .display_name = "Oak3DVision",
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
