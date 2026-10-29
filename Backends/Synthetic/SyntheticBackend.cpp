#include "SyntheticBackend.hpp"

#include "SyntheticScene.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#if !defined(SYNTHETIC_ABI_VERSION)
#define SYNTHETIC_ABI_VERSION DEPTHCAM_ABI_VERSION
#endif

namespace depthcam_synthetic
{
namespace
{
thread_local std::string g_error;

void setError(std::string e)
{
  g_error = std::move(e);
}

enum class Unit
{
  Mm16,
  TenthMm16,
  FloatMm,
  FloatM,
};

struct Options
{
  Unit unit{Unit::Mm16};
  bool irFloat{};
  bool distorted{};
  bool rayTable{};
  bool moving{};
  bool manualClock{};
  int fps{30};
};

bool parseOptions(std::string_view uri, Options& o)
{
  // "synthetic:..." -- or the ABI 3 stand-in's own name, "synthetic_abi3:...".
  const auto colon = uri.find(':');
  if(colon == std::string_view::npos || uri.substr(0, 9) != "synthetic")
    return false;
  uri.remove_prefix(colon + 1);

  while(!uri.empty())
  {
    const auto comma = uri.find(',');
    const auto item = uri.substr(0, comma);
    uri = comma == std::string_view::npos ? std::string_view{} : uri.substr(comma + 1);

    const auto eq = item.find('=');
    const auto key = item.substr(0, eq);
    const auto val = eq == std::string_view::npos ? std::string_view{} : item.substr(eq + 1);

    if(key == "unit")
    {
      if(val == "1mm")
        o.unit = Unit::Mm16;
      else if(val == "0.1mm")
        o.unit = Unit::TenthMm16;
      else if(val == "f32mm")
        o.unit = Unit::FloatMm;
      else if(val == "f32m")
        o.unit = Unit::FloatM;
      else
        return false;
    }
    else if(key == "ir")
      o.irFloat = val == "f32";
    else if(key == "distortion")
      o.distorted = val == "bc";
    else if(key == "raytable")
      o.rayTable = val == "1";
    else if(key == "motion")
      o.moving = val == "1";
    else if(key == "clock")
      o.manualClock = val == "manual";
    else if(key == "fps")
      o.fps = std::max(1, std::atoi(std::string{val}.c_str()));
    // "default" and anything unknown: ignored, so a URI can carry a label.
  }
  return true;
}

/// Millimetres per sample, as the frame reports it.
float unitMm(Unit u)
{
  switch(u)
  {
    case Unit::TenthMm16:
      return 0.1f;
    case Unit::FloatM:
      return 1000.f;
    default:
      return 1.f;
  }
}

constexpr int depth_modes[][2]{{640, 576}, {512, 512}, {1280, 720}};
const char* const depth_mode_labels[]{"640x576", "512x512", "1280x720"};
const char* const preset_labels[]{"Default", "High accuracy", "High density"};

/// One of each kind the ABI has, with the flags and units ABI 4 can say.
struct ControlDesc
{
  depthcam_control2 c;
};

std::vector<ControlDesc> makeControls()
{
  std::vector<ControlDesc> v;
  const auto add = [&](const char* id, const char* name, int kind, int access,
                       double min, double max, double step, double def,
                       const char* unit = nullptr, uint32_t flags = 0,
                       const char* autoc = nullptr,
                       const char* const* labels = nullptr, int count = 0) {
    depthcam_control2 c{};
    c.struct_size = sizeof(depthcam_control2);
    c.base.id = id;
    c.base.name = name;
    c.base.description = nullptr;
    c.base.kind = kind;
    c.base.access = access;
    c.base.min = min;
    c.base.max = max;
    c.base.step = step;
    c.base.def = def;
    c.base.enum_labels = labels;
    c.base.enum_count = count;
    c.unit = unit;
    c.flags = flags;
    c.auto_control = autoc;
    v.push_back({c});
  };
  constexpr int RW = DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE;
  add("depth/laser_power", "Laser power", DEPTHCAM_CONTROL_INT, RW, 0, 360, 30, 150,
      "mW");
  add("depth/mode", "Depth mode", DEPTHCAM_CONTROL_ENUM, RW, 0, 2, 1, 0, nullptr,
      DEPTHCAM_CONTROL_REQUIRES_RESTART, nullptr, depth_mode_labels, 3);
  add("color/auto_exposure", "Auto exposure", DEPTHCAM_CONTROL_BOOL, RW, 0, 1, 1, 1);
  add("color/exposure", "Exposure", DEPTHCAM_CONTROL_INT, RW, 1, 10000, 1, 1000, "us",
      DEPTHCAM_CONTROL_VOLATILE, "color/auto_exposure");
  add("color/gain", "Gain", DEPTHCAM_CONTROL_FLOAT, RW, 0, 1, 0.01, 0.5);
  add("device/preset", "Preset", DEPTHCAM_CONTROL_ENUM, RW, 0, 2, 1, 0, nullptr, 0,
      nullptr, preset_labels, 3);
  add("filters/spatial/enabled", "Spatial filter", DEPTHCAM_CONTROL_BOOL, RW, 0, 1, 1,
      0, nullptr, DEPTHCAM_CONTROL_FILTER);
  add("filters/spatial/alpha", "Spatial filter alpha", DEPTHCAM_CONTROL_FLOAT, RW, 0.25,
      1, 0.05, 0.5, nullptr, DEPTHCAM_CONTROL_FILTER);
  add("sensors/temperature", "Temperature", DEPTHCAM_CONTROL_FLOAT, DEPTHCAM_ACCESS_READ,
      -40, 125, 0.1, 35, "C", DEPTHCAM_CONTROL_VOLATILE);
  add("advanced/reboot", "Reboot", DEPTHCAM_CONTROL_ACTION, DEPTHCAM_ACCESS_WRITE, 0, 0,
      0, 0, nullptr, DEPTHCAM_CONTROL_ADVANCED);
  return v;
}

const std::vector<ControlDesc>& controls()
{
  static const auto c = makeControls();
  return c;
}

const depthcam_control2* findControl(std::string_view id)
{
  for(auto& d : controls())
    if(id == d.c.base.id)
      return &d.c;
  return nullptr;
}

struct FrameHolder
{
  std::vector<uint8_t> bytes;
};

void releaseFrame(void* owner)
{
  delete static_cast<FrameHolder*>(owner);
  counters().frames_released.fetch_add(1, std::memory_order_relaxed);
}

}

Counters& counters()
{
  static Counters c;
  return c;
}

}

struct depthcam_device
{
  depthcam_synthetic::Options opts;
  depthcam_open_config cfg{};
  std::string serial{"SYN-0001"};

  int depthMode{0};
  int pendingDepthMode{0};
  int depthW{640}, depthH{576};
  int irW{640}, irH{576};
  int colorW{1280}, colorH{720};
  uint32_t active{};

  // Control values, by id.
  std::mutex lock;
  double laserPower{150};
  bool autoExposure{true};
  double exposure{1000};
  double gain{0.5};
  int preset{0};
  bool spatial{};
  double spatialAlpha{0.5};

  /// Held while the change callback runs, so clearing it waits for a call in
  /// flight, as depthcam_abi.h promises.
  std::mutex changedLock;
  depthcam_control_changed_cb changedCb{};
  void* changedUser{};

  std::atomic<uint32_t> demand{~0u};

  std::thread thread;
  std::atomic_bool running{};
  std::mutex clockLock;
  std::condition_variable clockCv;
  int pendingSteps{0};
  int deliveredSteps{0};
  std::condition_variable deliveredCv;
  std::atomic<int64_t> frame{0};

  depthcam_frame_cb cb{};
  void* user{};

  void applyMode()
  {
    depthMode = pendingDepthMode;
    if(cfg.depth_width > 0 && cfg.depth_height > 0 && depthMode == 0)
    {
      depthW = cfg.depth_width;
      depthH = cfg.depth_height;
    }
    else
    {
      depthW = depthcam_synthetic::depth_modes[depthMode][0];
      depthH = depthcam_synthetic::depth_modes[depthMode][1];
    }
    // Infrared is the depth sensor's own picture here, as on a ToF camera.
    irW = depthW;
    irH = depthH;
  }

  depthcam_synthetic::Camera depthCam() const
  {
    return depthcam_synthetic::depthCamera(depthW, depthH, opts.distorted);
  }

  /// With depth aligned to colour, the depth stream is delivered in the colour
  /// camera's geometry. Only the undistorted path is modelled for that.
  bool alignedToColor() const { return cfg.align == DEPTHCAM_ALIGN_DEPTH_TO_COLOR; }
};

namespace depthcam_synthetic
{
namespace
{

void deliver(depthcam_device& d, uint32_t stream, int fmt, int w, int h,
             std::vector<uint8_t>&& bytes, uint64_t ts, float unit = 0.f, int points = 0)
{
  auto* holder = new FrameHolder{std::move(bytes)};
  depthcam_frame f{};
  f.stream = stream;
  f.format = fmt;
  f.width = w;
  f.height = h;
  f.stride = 0;
  f.timestamp_ns = ts;
  f.data = holder->bytes.data();
  f.bytes = holder->bytes.size();
  f.depth_unit_mm = unit;
  f.point_count = points;
  f.owner = holder;
  f.release = &releaseFrame;
  counters().frames_delivered.fetch_add(1, std::memory_order_relaxed);
  d.cb(&f, d.user);
}

/// Renders and delivers one frame of every opened stream.
void produce(depthcam_device& d)
{
  const int64_t frame = d.frame.fetch_add(1, std::memory_order_relaxed);
  const uint64_t ts = timestampNs(frame, d.opts.fps);
  const uint32_t want = d.active & d.demand.load(std::memory_order_relaxed);

  double laser{};
  {
    std::lock_guard l{d.lock};
    laser = d.laserPower;
  }

  // The depth camera's geometry, or the colour camera's when aligned to it.
  const Camera dc = d.alignedToColor() ? colorCamera(d.colorW, d.colorH) : d.depthCam();
  const int dw = dc.width, dh = dc.height;

  std::vector<double> metres;
  if(want & (DEPTHCAM_STREAM_DEPTH | DEPTHCAM_STREAM_POINTCLOUD))
  {
    metres.resize(std::size_t(dw) * dh);
    for(int v = 0; v < dh; v++)
      for(int u = 0; u < dw; u++)
        metres[std::size_t(v) * dw + u] = depthAt(dc, u, v, frame, d.opts.moving);
  }

  // The value each sample carries, and the metres it stands for once the unit
  // is applied -- which is what the point cloud is built from, so that it and
  // the host's own conversion agree to the last bit.
  const auto quantised = [&](double m) -> double {
    switch(d.opts.unit)
    {
      case Unit::Mm16:
        return std::min(65535., std::round(m * 1000.)) / 1000.;
      case Unit::TenthMm16:
        return std::min(65535., std::round(m * 10000.)) / 10000.;
      case Unit::FloatMm:
        return double(float(m * 1000.)) / 1000.;
      case Unit::FloatM:
        return double(float(m));
    }
    return m;
  };

  if(want & DEPTHCAM_STREAM_DEPTH)
  {
    const std::size_t n = std::size_t(dw) * dh;
    std::vector<uint8_t> bytes;
    int fmt{};
    if(d.opts.unit == Unit::Mm16 || d.opts.unit == Unit::TenthMm16)
    {
      const double scale = d.opts.unit == Unit::Mm16 ? 1000. : 10000.;
      bytes.resize(n * 2);
      auto* p = reinterpret_cast<uint16_t*>(bytes.data());
      for(std::size_t i = 0; i < n; i++)
        p[i] = uint16_t(std::min(65535., std::round(metres[i] * scale)));
      fmt = DEPTHCAM_FMT_GRAY16;
    }
    else
    {
      const double scale = d.opts.unit == Unit::FloatMm ? 1000. : 1.;
      bytes.resize(n * 4);
      auto* p = reinterpret_cast<float*>(bytes.data());
      for(std::size_t i = 0; i < n; i++)
        p[i] = float(metres[i] * scale);
      fmt = DEPTHCAM_FMT_GRAYF32;
    }
    deliver(d, DEPTHCAM_STREAM_DEPTH, fmt, dw, dh, std::move(bytes), ts,
            unitMm(d.opts.unit));
  }

  if(want & DEPTHCAM_STREAM_IR)
  {
    const std::size_t n = std::size_t(d.irW) * d.irH;
    std::vector<uint8_t> bytes(n * (d.opts.irFloat ? 4 : 2));
    for(int v = 0; v < d.irH; v++)
      for(int u = 0; u < d.irW; u++)
      {
        const auto i = std::size_t(v) * d.irW + u;
        const uint16_t ir = irAt(u, v, frame, laser);
        if(d.opts.irFloat)
          reinterpret_cast<float*>(bytes.data())[i] = float(ir);
        else
          reinterpret_cast<uint16_t*>(bytes.data())[i] = ir;
      }
    deliver(d, DEPTHCAM_STREAM_IR,
            d.opts.irFloat ? DEPTHCAM_FMT_GRAYF32 : DEPTHCAM_FMT_GRAY16, d.irW, d.irH,
            std::move(bytes), ts);
  }

  if(want & DEPTHCAM_STREAM_COLOR)
  {
    const Camera cc = colorCamera(d.colorW, d.colorH);
    std::vector<uint8_t> bytes(std::size_t(d.colorW) * d.colorH * 3);
    for(int v = 0; v < d.colorH; v++)
      for(int u = 0; u < d.colorW; u++)
      {
        const auto c = colorAt(cc, u, v, frame);
        std::memcpy(&bytes[(std::size_t(v) * d.colorW + u) * 3], c.data(), 3);
      }
    deliver(d, DEPTHCAM_STREAM_COLOR, DEPTHCAM_FMT_RGB24, d.colorW, d.colorH,
            std::move(bytes), ts);
  }

  if(want & DEPTHCAM_STREAM_POINTCLOUD)
  {
    // Organised: one point per depth pixel, zeros where there is no depth,
    // like the SDK clouds.
    counters().pointclouds_computed.fetch_add(1, std::memory_order_relaxed);
    const std::size_t n = std::size_t(dw) * dh;
    std::vector<uint8_t> bytes(n * 3 * sizeof(float));
    auto* p = reinterpret_cast<float*>(bytes.data());
    for(int v = 0; v < dh; v++)
      for(int u = 0; u < dw; u++)
      {
        const auto i = std::size_t(v) * dw + u;
        const double z = quantised(metres[i]);
        const auto r = ray(dc, u, v);
        p[3 * i + 0] = z > 0 ? float(r[0] * z) : 0.f;
        p[3 * i + 1] = z > 0 ? float(r[1] * z) : 0.f;
        p[3 * i + 2] = float(z);
      }
    deliver(d, DEPTHCAM_STREAM_POINTCLOUD, DEPTHCAM_FMT_XYZ, int(n), 1, std::move(bytes),
            ts, 0.f, int(n));
  }

  if(want & DEPTHCAM_STREAM_IMU)
  {
    // At rest, level: the accelerometer reads gravity's reaction, pointing up,
    // which is -Y in this frame.
    depthcam_imu_sample s{};
    s.fields = DEPTHCAM_IMU_ACCEL | DEPTHCAM_IMU_GYRO | DEPTHCAM_IMU_TEMPERATURE;
    s.accel[1] = -9.80665f;
    s.temperature_c = 35.f;
    std::vector<uint8_t> bytes(sizeof(s));
    std::memcpy(bytes.data(), &s, sizeof(s));
    deliver(d, DEPTHCAM_STREAM_IMU, DEPTHCAM_FMT_IMU, 1, 1, std::move(bytes), ts);
  }
}

void run(depthcam_device* d)
{
  using clk = std::chrono::steady_clock;
  auto next = clk::now();
  const auto period = std::chrono::nanoseconds(1'000'000'000 / d->opts.fps);

  while(d->running.load(std::memory_order_acquire))
  {
    if(d->opts.manualClock)
    {
      std::unique_lock l{d->clockLock};
      d->clockCv.wait(l, [&] {
        return d->pendingSteps > 0 || !d->running.load(std::memory_order_acquire);
      });
      if(!d->running.load(std::memory_order_acquire))
        break;
      d->pendingSteps--;
      l.unlock();
      produce(*d);
      l.lock();
      d->deliveredSteps++;
      d->deliveredCv.notify_all();
      continue;
    }

    produce(*d);
    next += period;
    std::this_thread::sleep_until(next);
  }
}

uint32_t supported()
{
  return DEPTHCAM_STREAM_COLOR | DEPTHCAM_STREAM_IR | DEPTHCAM_STREAM_DEPTH
         | DEPTHCAM_STREAM_POINTCLOUD | DEPTHCAM_STREAM_IMU;
}

const char* lastError()
{
  return g_error.empty() ? nullptr : g_error.c_str();
}

int init(const char*)
{
  return 1;
}

void shutdown() { }

int enumerate(depthcam_enumerate_cb cb, void* user)
{
  // Only on request: a camera that is not there has no business in the device
  // browser of someone who has a real one.
  if(!std::getenv("SCORE_DEPTHCAM_SYNTHETIC"))
    return 1;
  depthcam_device_info info{};
  info.backend = "synthetic";
  info.uri = "synthetic:default";
  info.name = "Synthetic depth camera";
  info.serial = "SYN-0001";
  info.transport = "memory";
  info.streams = supported();
  cb(&info, user);
  return 1;
}

depthcam_device* open(const char* uri, const depthcam_open_config* cfg)
{
  Options o;
  if(!uri || !parseOptions(uri, o))
  {
    setError(std::string{"not a synthetic: URI, or a bad option: "} + (uri ? uri : ""));
    return nullptr;
  }
  auto* d = new depthcam_device;
  d->opts = o;
  if(cfg)
    d->cfg = *cfg;
  else
    d->cfg.streams = supported();
  if(d->cfg.color_width > 0 && d->cfg.color_height > 0)
  {
    d->colorW = d->cfg.color_width;
    d->colorH = d->cfg.color_height;
  }
  d->active = d->cfg.streams & supported();
  d->applyMode();
  return d;
}

void stop(depthcam_device* d);

void close(depthcam_device* d)
{
  if(!d)
    return;
  stop(d);
  delete d;
}

int start(depthcam_device* d, depthcam_frame_cb cb, void* user)
{
  if(!d || !cb)
    return 0;
  if(d->running.load())
    return 1;
  d->applyMode();
  d->cb = cb;
  d->user = user;
  d->running.store(true, std::memory_order_release);
  counters().starts.fetch_add(1);
  d->thread = std::thread{[d] { run(d); }};
  return 1;
}

void stop(depthcam_device* d)
{
  if(!d || !d->running.exchange(false, std::memory_order_acq_rel))
    return;
  {
    std::lock_guard l{d->clockLock};
    d->clockCv.notify_all();
  }
  if(d->thread.joinable())
    d->thread.join();
  {
    std::lock_guard l{d->clockLock};
    d->pendingSteps = 0;
    d->deliveredCv.notify_all();
  }
  counters().stops.fetch_add(1);
}

int listControls(depthcam_device*, depthcam_control_cb cb, void* user)
{
  for(auto& c : controls())
    cb(&c.c.base, user);
  return 1;
}

int listControls2(depthcam_device*, depthcam_control2_cb cb, void* user)
{
  for(auto& c : controls())
    cb(&c.c, user);
  return 1;
}

int getControl(depthcam_device* d, const char* id, double* out)
{
  if(!d || !id || !out)
    return 0;
  const std::string_view s{id};
  std::lock_guard l{d->lock};
  if(s == "depth/laser_power")
    *out = d->laserPower;
  else if(s == "depth/mode")
    *out = d->pendingDepthMode;
  else if(s == "color/auto_exposure")
    *out = d->autoExposure;
  else if(s == "color/exposure")
    // Under auto the camera picks; it wanders a little with the frame.
    *out = d->autoExposure ? 1000. + double(d->frame.load() % 7) * 10. : d->exposure;
  else if(s == "color/gain")
    *out = d->gain;
  else if(s == "device/preset")
    *out = d->preset;
  else if(s == "filters/spatial/enabled")
    *out = d->spatial;
  else if(s == "filters/spatial/alpha")
    *out = d->spatialAlpha;
  else if(s == "sensors/temperature")
    *out = 35. + double(d->frame.load() % 10) * 0.1;
  else
    return 0;
  return 1;
}

int setControl(depthcam_device* d, const char* id, double v)
{
  if(!d || !id)
    return 0;
  counters().set_control_calls.fetch_add(1, std::memory_order_relaxed);
  const auto* desc = findControl(id);
  if(!desc || !(desc->base.access & DEPTHCAM_ACCESS_WRITE))
    return 0;

  const std::string_view s{id};
  const auto clampTo = [&](double x) {
    x = std::clamp(x, desc->base.min, desc->base.max);
    if(desc->base.step > 0)
      x = desc->base.min
          + std::round((x - desc->base.min) / desc->base.step) * desc->base.step;
    return x;
  };

  const char* notify = nullptr;
  {
    std::lock_guard l{d->lock};
    if(s == "depth/laser_power")
      d->laserPower = clampTo(v);
    else if(s == "depth/mode")
      // Takes effect at the next start, like a real depth mode.
      d->pendingDepthMode = int(clampTo(v));
    else if(s == "color/auto_exposure")
      d->autoExposure = v != 0.;
    else if(s == "color/exposure")
    {
      // Refused under auto, as the SDKs do.
      if(d->autoExposure)
        return 0;
      d->exposure = clampTo(v);
    }
    else if(s == "color/gain")
      d->gain = clampTo(v);
    else if(s == "device/preset")
    {
      d->preset = int(clampTo(v));
      // A preset rewrites other settings behind the host's back, which is what
      // the change callback exists to report.
      d->laserPower = d->preset == 1 ? 300. : 150.;
      notify = "depth/laser_power";
    }
    else if(s == "filters/spatial/enabled")
      d->spatial = v != 0.;
    else if(s == "filters/spatial/alpha")
      d->spatialAlpha = clampTo(v);
    else if(s == "advanced/reboot")
      ;
    else
      return 0;
  }
  if(notify)
  {
    std::lock_guard l{d->changedLock};
    if(d->changedCb)
      d->changedCb(notify, d->changedUser);
  }
  return 1;
}

uint32_t activeStreams(depthcam_device* d)
{
  return d ? d->active : 0;
}

void fill(const Camera& c, depthcam_intrinsics* out)
{
  depthcam_intrinsics r{};
  r.struct_size = out->struct_size;
  r.width = c.width;
  r.height = c.height;
  r.fx = float(c.fx);
  r.fy = float(c.fy);
  r.cx = float(c.cx);
  r.cy = float(c.cy);
  r.model = c.model;
  for(int i = 0; i < 6; i++)
    r.coeffs[i] = float(c.k[i]);
  r.coeffs[6] = float(c.p[0]);
  r.coeffs[7] = float(c.p[1]);
  std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
}

int getIntrinsics(depthcam_device* d, uint32_t stream, depthcam_intrinsics* out)
{
  if(!d || !out)
    return 0;
  switch(stream)
  {
    case DEPTHCAM_STREAM_DEPTH:
    case DEPTHCAM_STREAM_POINTCLOUD:
      fill(d->alignedToColor() ? colorCamera(d->colorW, d->colorH) : d->depthCam(), out);
      return 1;
    case DEPTHCAM_STREAM_IR:
      fill(d->depthCam(), out);
      return 1;
    case DEPTHCAM_STREAM_COLOR:
      fill(colorCamera(d->colorW, d->colorH), out);
      return 1;
    default:
      return 0;
  }
}

int getExtrinsics(depthcam_device* d, uint32_t from, uint32_t to, depthcam_extrinsics* out)
{
  if(!d || !out)
    return 0;
  const auto isDepth = [](uint32_t s) {
    return s == DEPTHCAM_STREAM_DEPTH || s == DEPTHCAM_STREAM_IR
           || s == DEPTHCAM_STREAM_POINTCLOUD || s == DEPTHCAM_STREAM_IMU;
  };
  Rigid x;
  if(from == to || (isDepth(from) && isDepth(to)))
    ; // identity: one sensor, and the IMU is taken to sit at its origin
  else if(isDepth(from) && to == DEPTHCAM_STREAM_COLOR)
    x = d->alignedToColor() ? Rigid{} : depthToColor();
  else if(from == DEPTHCAM_STREAM_COLOR && isDepth(to))
  {
    if(!d->alignedToColor())
    {
      // Inverse of a rigid transform: R^T, -R^T t.
      const Rigid f = depthToColor();
      for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
          x.r[3 * i + j] = f.r[3 * j + i];
      for(int i = 0; i < 3; i++)
        x.t[i] = -(x.r[3 * i] * f.t[0] + x.r[3 * i + 1] * f.t[1] + x.r[3 * i + 2] * f.t[2]);
    }
  }
  else
    return 0;

  depthcam_extrinsics r{};
  r.struct_size = out->struct_size;
  for(int i = 0; i < 9; i++)
    r.rotation[i] = float(x.r[i]);
  for(int i = 0; i < 3; i++)
    r.translation[i] = float(x.t[i]);
  std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
  return 1;
}

int getDetails(depthcam_device* d, depthcam_device_details* out)
{
  if(!d || !out)
    return 0;
  depthcam_device_details r{};
  r.struct_size = out->struct_size;
  r.model = "Synthetic depth camera";
  r.serial = d->serial.c_str();
  r.firmware = "1.2.3";
  r.hardware_rev = "A";
  r.depth_min_m = 0.25f;
  r.depth_max_m = 6.5f;
  std::memcpy(out, &r, std::min<std::size_t>(out->struct_size, sizeof(r)));
  return 1;
}

int getRayTable(depthcam_device* d, uint32_t stream, float* xy, int32_t w, int32_t h)
{
  if(!d || !xy || !d->opts.rayTable)
    return 0;
  if(stream != DEPTHCAM_STREAM_DEPTH)
    return 0;
  const Camera c
      = d->alignedToColor() ? colorCamera(d->colorW, d->colorH) : d->depthCam();
  if(w != c.width || h != c.height)
    return 0;
  for(int v = 0; v < h; v++)
    for(int u = 0; u < w; u++)
    {
      const auto r = ray(c, u, v);
      xy[2 * (std::size_t(v) * w + u)] = float(r[0]);
      xy[2 * (std::size_t(v) * w + u) + 1] = float(r[1]);
    }
  return 1;
}

void setControlChanged(depthcam_device* d, depthcam_control_changed_cb cb, void* user)
{
  if(!d)
    return;
  std::lock_guard l{d->changedLock};
  d->changedCb = cb;
  d->changedUser = user;
}

int listProfiles(const char* uri, uint32_t stream, depthcam_profile_cb cb, void* user)
{
  Options o;
  if(!uri || !parseOptions(uri, o) || !cb)
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
  const int depthFmt = (o.unit == Unit::Mm16 || o.unit == Unit::TenthMm16)
                           ? DEPTHCAM_FMT_GRAY16
                           : DEPTHCAM_FMT_GRAYF32;
  switch(stream)
  {
    case DEPTHCAM_STREAM_DEPTH:
      for(auto& m : depth_modes)
        for(int fps : {15, 30})
          emit(m[0], m[1], fps, depthFmt);
      return 1;
    case DEPTHCAM_STREAM_IR:
      for(auto& m : depth_modes)
        for(int fps : {15, 30})
          emit(m[0], m[1], fps, o.irFloat ? DEPTHCAM_FMT_GRAYF32 : DEPTHCAM_FMT_GRAY16);
      return 1;
    case DEPTHCAM_STREAM_COLOR:
      for(auto [w, h] : {std::pair{1280, 720}, std::pair{1920, 1080}})
        for(int fps : {15, 30})
          emit(w, h, fps, DEPTHCAM_FMT_RGB24);
      return 1;
    default:
      return 0;
  }
}

void setStreamDemand(depthcam_device* d, uint32_t streams)
{
  if(d)
    d->demand.store(streams, std::memory_order_relaxed);
}

depthcam_backend_v1 makeTable()
{
  depthcam_backend_v1 t{};
  t.abi_version = SYNTHETIC_ABI_VERSION;
  t.name = "synthetic";
  t.display_name = "Synthetic camera";
  t.last_error = &lastError;
  t.init = &init;
  t.shutdown = &shutdown;
  t.enumerate = &enumerate;
  t.set_changed_callback = nullptr;
  t.open = &open;
  t.close = &close;
  t.start = &start;
  t.stop = &stop;
  t.list_controls = &listControls;
  t.get_control = &getControl;
  t.set_control = &setControl;
  t.active_streams = &activeStreams;
#if SYNTHETIC_ABI_VERSION >= 4
  t.get_intrinsics = &getIntrinsics;
  t.get_extrinsics = &getExtrinsics;
  t.get_details = &getDetails;
  t.get_ray_table = &getRayTable;
  t.list_controls2 = &listControls2;
  t.set_control_changed_callback = &setControlChanged;
  t.list_profiles = &listProfiles;
  t.set_stream_demand = &setStreamDemand;
#else
  (void)&getIntrinsics;
  (void)&getExtrinsics;
  (void)&getDetails;
  (void)&getRayTable;
  (void)&listControls2;
  (void)&setControlChanged;
  (void)&listProfiles;
  (void)&setStreamDemand;
#endif
  return t;
}

}

const depthcam_backend_v1* backend()
{
  static const depthcam_backend_v1 table = makeTable();
  return &table;
}

void step(depthcam_device* d, int n)
{
  if(!d || n <= 0)
    return;
  std::unique_lock l{d->clockLock};
  const int target = d->deliveredSteps + n;
  d->pendingSteps += n;
  d->clockCv.notify_all();
  d->deliveredCv.wait(l, [&] {
    return d->deliveredSteps >= target || !d->running.load(std::memory_order_acquire);
  });
}

int64_t nextFrame(depthcam_device* d)
{
  return d ? d->frame.load() : 0;
}

}
