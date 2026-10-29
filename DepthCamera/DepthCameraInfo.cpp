#include "DepthCameraInfo.hpp"

#include <ossia/network/base/node_attributes.hpp>
#include <ossia/network/domain/domain.hpp>

#include <DepthCamera/DepthCameraStream.hpp>

#include <cmath>

namespace Gfx::DepthCamera
{
namespace
{

ossia::net::node_base* child(ossia::net::node_base& parent, std::string_view name)
{
  if(auto* n = parent.find_child(name))
    return n;
  return parent.create_child(std::string{name});
}

ossia::net::parameter_base* readOnly(
    ossia::net::node_base& parent, std::string_view name, ossia::val_type t,
    ossia::value init, const char* description = nullptr)
{
  auto* node = child(parent, name);
  if(!node)
    return nullptr;
  auto* p = node->get_parameter();
  if(!p)
    p = node->create_parameter(t);
  if(!p)
    return nullptr;
  p->set_access(ossia::access_mode::GET);
  p->set_value_quiet(std::move(init));
  if(description)
    ossia::net::set_description(*node, description);
  return p;
}

ossia::value listOf(const float* v, int n)
{
  std::vector<ossia::value> l;
  l.reserve(std::size_t(n));
  for(int i = 0; i < n; i++)
    l.emplace_back(v[i]);
  return ossia::value{std::move(l)};
}

const char* streamName(uint32_t s)
{
  switch(s)
  {
    case DEPTHCAM_STREAM_COLOR:
      return "color";
    case DEPTHCAM_STREAM_IR:
      return "ir";
    case DEPTHCAM_STREAM_DEPTH:
      return "depth";
    case DEPTHCAM_STREAM_POINTCLOUD:
      return "pointcloud";
    default:
      return "other";
  }
}

int statsIndex(uint32_t s)
{
  switch(s)
  {
    case DEPTHCAM_STREAM_COLOR:
      return 0;
    case DEPTHCAM_STREAM_IR:
      return 1;
    case DEPTHCAM_STREAM_DEPTH:
      return 2;
    case DEPTHCAM_STREAM_POINTCLOUD:
      return 3;
    default:
      return -1;
  }
}

std::array<ossia::net::parameter_base*, 4>
matrixRows(ossia::net::node_base& parent)
{
  std::array<ossia::net::parameter_base*, 4> rows{};
  for(int r = 0; r < 4; r++)
    rows[r] = readOnly(
        parent, "row" + std::to_string(r), ossia::val_type::VEC4F,
        ossia::vec4f{r == 0 ? 1.f : 0.f, r == 1 ? 1.f : 0.f, r == 2 ? 1.f : 0.f,
                     r == 3 ? 1.f : 0.f});
  return rows;
}

void setMatrix(
    ossia::net::parameter_base* m, const std::array<ossia::net::parameter_base*, 4>& rows,
    const Matrix4& v)
{
  if(m)
    m->set_value(listOf(v.data(), 16));
  for(int r = 0; r < 4; r++)
    if(rows[r])
      rows[r]->set_value(ossia::vec4f{v[r], v[4 + r], v[8 + r], v[12 + r]});
}
}

Matrix4 clipProjection(const depthcam_intrinsics& in, float n, float f) noexcept
{
  Matrix4 m{};
  if(in.width <= 0 || in.height <= 0 || f <= n || n <= 0)
    return identityMatrix();
  const float W = float(in.width), H = float(in.height);
  // Column-major: m[4 * col + row].
  // x_clip = 2fx/W X + (2(cx + 0.5)/W - 1) Z
  m[0] = 2.f * in.fx / W;
  m[8] = 2.f * (in.cx + 0.5f) / W - 1.f;
  // y_clip = -2fy/H Y + (1 - 2(cy + 0.5)/H) Z: the camera's +Y is down
  m[5] = -2.f * in.fy / H;
  m[9] = 1.f - 2.f * (in.cy + 0.5f) / H;
  // z_clip = (f + n)/(f - n) Z - 2fn/(f - n), w_clip = Z
  m[10] = (f + n) / (f - n);
  m[14] = -2.f * f * n / (f - n);
  m[11] = 1.f;
  return m;
}

InfoTree::InfoTree(
    const std::shared_ptr<InputStream>& stream, std::shared_ptr<Calibration> calib,
    ossia::net::node_base& root, QString uri, QString backendName, QString transport)
    : m_stream{stream}
    , m_calib{std::move(calib)}
    , m_root{root}
{
  m_info = child(root, "info");
  m_calibration = child(root, "calibration");
  if(!m_info || !m_calibration)
    return;

  readOnly(*m_info, "backend", ossia::val_type::STRING, backendName.toStdString());
  readOnly(*m_info, "uri", ossia::val_type::STRING, uri.toStdString());
  readOnly(*m_info, "transport", ossia::val_type::STRING, transport.toStdString());
  m_unit = readOnly(
      *m_info, "depth_unit_mm", ossia::val_type::FLOAT, 0.f,
      "Millimetres per sample of the raw depth stream, as the latest frame said");

  // One stats subtree per image stream the camera opened.
  if(m_stream)
  {
    const uint32_t active = m_stream->activeStreams();
    auto* statsRoot = child(root, "stats");
    for(uint32_t s : {uint32_t(DEPTHCAM_STREAM_COLOR), uint32_t(DEPTHCAM_STREAM_IR),
                      uint32_t(DEPTHCAM_STREAM_DEPTH), uint32_t(DEPTHCAM_STREAM_POINTCLOUD)})
    {
      if(!(active & s) || !statsRoot)
        continue;
      auto* n = child(*statsRoot, streamName(s));
      if(!n)
        continue;
      auto& st = m_stats[std::size_t(statsIndex(s))];
      st.frames = readOnly(*n, "frames", ossia::val_type::INT, 0, "Frames since start");
      st.dropped = readOnly(
          *n, "dropped", ossia::val_type::INT, 0,
          "Frames discarded because the graph did not take them in time");
      st.fps = readOnly(*n, "fps", ossia::val_type::FLOAT, 0.f, "From the device clock");
      st.device_time = readOnly(
          *n, "device_time_ms", ossia::val_type::FLOAT, 0.f,
          "Capture time of the latest frame on the device clock, since start. Updated "
          "at every frame");
      st.host_time = readOnly(
          *n, "host_time_ms", ossia::val_type::FLOAT, 0.f,
          "Arrival time of the latest frame on the host clock, since start");
      st.interval = readOnly(
          *n, "interval_ms", ossia::val_type::FLOAT, 0.f,
          "Device time between the latest two frames");
    }

    m_stream->setStatsCallback(
        [this](const StreamOutput& out, const StreamStats& st) { onStats(out, st); });
  }

  refresh();
}

InfoTree::~InfoTree()
{
  // Blocks until a callback in flight has returned; see setStatsCallback.
  if(m_stream)
    m_stream->setStatsCallback({});
}

InfoTree::StreamNodes& InfoTree::streamNodes(uint32_t stream)
{
  for(auto& s : m_streams)
    if(s.stream == stream)
      return s;

  StreamNodes s;
  s.stream = stream;
  if(auto* n = child(*m_calibration, streamName(stream)))
  {
    s.width = readOnly(*n, "width", ossia::val_type::INT, 0);
    s.height = readOnly(*n, "height", ossia::val_type::INT, 0);
    s.intrinsics = readOnly(
        *n, "intrinsics", ossia::val_type::VEC4F, ossia::vec4f{},
        "fx, fy, cx, cy in pixels of the stream as delivered; the centre of pixel "
        "(u, v) is at (u, v)");
    if(auto* dist = child(*n, "distortion"))
    {
      s.model = readOnly(*dist, "model", ossia::val_type::STRING, std::string{"none"});
      const float zero[8]{};
      s.coeffs = readOnly(
          *dist, "coeffs", ossia::val_type::LIST, listOf(zero, 8), "k1 k2 k3 k4 k5 k6 p1 p2");
    }
    const auto id = identityMatrix();
    s.projection = readOnly(
        *n, "projection", ossia::val_type::LIST, listOf(id.data(), 16),
        "Column-major: this camera's frame (+Y down, +Z forward, metres) to OpenGL clip "
        "space, for render/near and render/far");
  }
  m_streams.push_back(s);
  return m_streams.back();
}

void InfoTree::refresh()
{
  if(!m_calib || !m_info || !m_calibration)
    return;

  if(m_stream && m_stream->backend() && m_stream->device())
  {
    const int w = m_stream->m_depth.width.load();
    const int h = m_stream->m_depth.height.load();
    m_calib->set(Calibration::query(
        *m_stream->backend(), *m_stream->device(), m_stream->activeStreams(), w, h));
  }
  publish(m_calib->get());
}

void InfoTree::publish(const CalibrationData& d)
{
  const auto str = [&](const char* name, const std::string& v) {
    if(auto* p = readOnly(*m_info, name, ossia::val_type::STRING, std::string{}))
      p->set_value(v);
  };
  str("model", d.model);
  str("serial", d.serial);
  str("firmware", d.firmware);
  str("hardware_rev", d.hardwareRev);
  if(auto* p = readOnly(
         *m_info, "depth_range", ossia::val_type::VEC2F, ossia::vec2f{},
         "Metres the current depth mode measures; 0 if the camera does not say"))
    p->set_value(ossia::vec2f{d.depthMin, d.depthMax});
  if(auto* p = readOnly(
         *m_info, "calibration_estimated", ossia::val_type::BOOL, false,
         "The camera reported no calibration; a 60-degree pinhole stands in"))
    p->set_value(d.estimated);

  for(const auto& s : d.streams)
  {
    auto& n = streamNodes(s.stream);
    const auto& in = s.intrinsics;
    if(n.width)
      n.width->set_value(int(in.width));
    if(n.height)
      n.height->set_value(int(in.height));
    if(n.intrinsics)
      n.intrinsics->set_value(ossia::vec4f{in.fx, in.fy, in.cx, in.cy});
    if(n.model)
      n.model->set_value(std::string{distortionName(in.model)});
    if(n.coeffs)
      n.coeffs->set_value(listOf(in.coeffs, 8));
  }

  if(d.depthToColor)
  {
    if(!m_depthToColor)
    {
      const auto id = identityMatrix();
      m_depthToColor = readOnly(
          *m_calibration, "depth_to_color", ossia::val_type::LIST, listOf(id.data(), 16),
          "Column-major 4x4, metres: a point in the depth camera's frame to the colour "
          "camera's");
      if(auto* n = m_calibration->find_child(std::string_view{"depth_to_color"}))
        m_depthToColorRows = matrixRows(*n);
    }
    setMatrix(m_depthToColor, m_depthToColorRows, *d.depthToColor);
  }
  if(d.depthToImu)
  {
    if(!m_depthToImu)
    {
      const auto id = identityMatrix();
      m_depthToImu = readOnly(
          *m_calibration, "depth_to_imu", ossia::val_type::LIST, listOf(id.data(), 16),
          "Column-major 4x4, metres: a point in the depth camera's frame to the IMU's");
      if(auto* n = m_calibration->find_child(std::string_view{"depth_to_imu"}))
        m_depthToImuRows = matrixRows(*n);
    }
    setMatrix(m_depthToImu, m_depthToImuRows, *d.depthToImu);
  }

  publishProjections(d);
}

void InfoTree::setClipPlanes(float near_m, float far_m)
{
  m_near = near_m;
  m_far = far_m;
  if(m_calib)
    publishProjections(m_calib->get());
}

void InfoTree::publishProjections(const CalibrationData& d)
{
  float n = m_near > 0 ? m_near : (d.depthMin > 0 ? d.depthMin : 0.1f);
  float f = m_far > 0 ? m_far : (d.depthMax > 0 ? d.depthMax : 10.f);
  if(f <= n)
    f = n + 1.f;
  for(const auto& s : d.streams)
  {
    auto& nodes = streamNodes(s.stream);
    if(nodes.projection)
    {
      const auto m = clipProjection(s.intrinsics, n, f);
      nodes.projection->set_value(listOf(m.data(), 16));
    }
  }
}

void InfoTree::onStats(const StreamOutput& out, const StreamStats& st)
{
  // Camera thread. set_value: the camera is telling us.
  const int i = statsIndex(out.stream);
  if(i < 0)
    return;
  auto& n = m_stats[std::size_t(i)];

  if(n.device_time)
    n.device_time->set_value(float(st.device_time_ms));

  if(out.stream == DEPTHCAM_STREAM_DEPTH && m_unit)
  {
    const float u = out.unit_mm.load(std::memory_order_relaxed);
    if(u > 0 && u != m_lastUnit)
    {
      m_lastUnit = u;
      m_unit->set_value(u);
    }
  }

  const auto now = std::chrono::steady_clock::now();
  if(st.frames > 1 && now - n.last_publish < std::chrono::milliseconds(100))
    return;
  n.last_publish = now;

  if(n.frames)
    n.frames->set_value(int(st.frames));
  if(n.dropped)
    n.dropped->set_value(int(st.dropped));
  if(n.fps)
    n.fps->set_value(float(st.fps));
  if(n.host_time)
    n.host_time->set_value(float(st.host_time_ms));
  if(n.interval)
    n.interval->set_value(float(st.interval_ms));
}

}
