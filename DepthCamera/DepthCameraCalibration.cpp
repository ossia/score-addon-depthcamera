#include "DepthCameraCalibration.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace Gfx::DepthCamera
{

Matrix4 identityMatrix() noexcept
{
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

Matrix4 toMatrix(const depthcam_extrinsics& e) noexcept
{
  // Rotation arrives row-major; a column-major matrix stores column j at
  // [4j .. 4j+3], so element (i, j) goes to [4j + i].
  Matrix4 m = identityMatrix();
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      m[4 * j + i] = e.rotation[3 * i + j];
  for(int i = 0; i < 3; i++)
    m[12 + i] = e.translation[i];
  return m;
}

const char* distortionName(int model) noexcept
{
  switch(model)
  {
    case DEPTHCAM_DISTORTION_NONE:
      return "none";
    case DEPTHCAM_DISTORTION_BROWN_CONRADY:
      return "brown_conrady";
    case DEPTHCAM_DISTORTION_INVERSE_BROWN_CONRADY:
      return "inverse_brown_conrady";
    case DEPTHCAM_DISTORTION_MODIFIED_BROWN_CONRADY:
      return "modified_brown_conrady";
    case DEPTHCAM_DISTORTION_KANNALA_BRANDT4:
      return "kannala_brandt4";
    case DEPTHCAM_DISTORTION_FTHETA:
      return "ftheta";
    default:
      return "unknown";
  }
}

namespace
{
struct Model
{
  double fx, fy, cx, cy;
  int model;
  double k1, k2, k3, k4, k5, k6, p1, p2;
};

Model scaled(const depthcam_intrinsics& in, int w, int h)
{
  Model m{};
  const double sx = in.width > 0 ? double(w) / in.width : 1.;
  const double sy = in.height > 0 ? double(h) / in.height : 1.;
  m.fx = in.fx * sx;
  m.fy = in.fy * sy;
  // Pixel centres at integers: the image spans [-0.5, w - 0.5], so a point at
  // c stays at the same fraction of that span.
  m.cx = (in.cx + 0.5) * sx - 0.5;
  m.cy = (in.cy + 0.5) * sy - 0.5;
  m.model = in.model;
  m.k1 = in.coeffs[0];
  m.k2 = in.coeffs[1];
  m.k3 = in.coeffs[2];
  m.k4 = in.coeffs[3];
  m.k5 = in.coeffs[4];
  m.k6 = in.coeffs[5];
  m.p1 = in.coeffs[6];
  m.p2 = in.coeffs[7];
  return m;
}

/// OpenCV, undistorted -> distorted.
void forwardBC(const Model& m, double x, double y, double& xd, double& yd)
{
  const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
  const double radial
      = (1 + m.k1 * r2 + m.k2 * r4 + m.k3 * r6) / (1 + m.k4 * r2 + m.k5 * r4 + m.k6 * r6);
  xd = x * radial + 2 * m.p1 * x * y + m.p2 * (r2 + 2 * x * x);
  yd = y * radial + m.p1 * (r2 + 2 * y * y) + 2 * m.p2 * x * y;
}

/// librealsense's forward model (rs2_project_point_to_pixel).
void forwardModifiedBC(const Model& m, double x, double y, double& xd, double& yd)
{
  const double r2 = x * x + y * y;
  const double f = 1 + m.k1 * r2 + m.k2 * r2 * r2 + m.k3 * r2 * r2 * r2;
  x *= f;
  y *= f;
  xd = x + 2 * m.p1 * x * y + m.p2 * (r2 + 2 * x * x);
  yd = y + 2 * m.p2 * x * y + m.p1 * (r2 + 2 * y * y);
}

template <typename F>
std::array<double, 2> invert(F&& forward, double xd, double yd)
{
  // Fixed-point iteration on x = xd - (distort(x) - x), started at the
  // distorted point. Converges for every lens a depth camera ships with; 50
  // rounds is well past float precision.
  double x = xd, y = yd;
  for(int i = 0; i < 50; i++)
  {
    double fx{}, fy{};
    forward(x, y, fx, fy);
    x -= fx - xd;
    y -= fy - yd;
  }
  return {x, y};
}
}

std::array<double, 2> pixelRay(const depthcam_intrinsics& in, double u, double v)
{
  const Model m = scaled(in, in.width, in.height);
  const double xd = (u - m.cx) / m.fx;
  const double yd = (v - m.cy) / m.fy;

  switch(m.model)
  {
    case DEPTHCAM_DISTORTION_BROWN_CONRADY:
      return invert(
          [&](double x, double y, double& a, double& b) { forwardBC(m, x, y, a, b); }, xd,
          yd);

    case DEPTHCAM_DISTORTION_MODIFIED_BROWN_CONRADY:
      return invert(
          [&](double x, double y, double& a, double& b) {
        forwardModifiedBC(m, x, y, a, b);
      }, xd, yd);

    case DEPTHCAM_DISTORTION_INVERSE_BROWN_CONRADY: {
      // The coefficients describe distorted -> undistorted directly, which is
      // how librealsense applies them in rs2_deproject_pixel_to_point.
      const double r2 = xd * xd + yd * yd;
      const double f = 1 + m.k1 * r2 + m.k2 * r2 * r2 + m.k3 * r2 * r2 * r2;
      return {
          xd * f + 2 * m.p1 * xd * yd + m.p2 * (r2 + 2 * xd * xd),
          yd * f + 2 * m.p2 * xd * yd + m.p1 * (r2 + 2 * yd * yd)};
    }

    case DEPTHCAM_DISTORTION_KANNALA_BRANDT4: {
      const double rd = std::sqrt(xd * xd + yd * yd);
      if(rd < 1e-12)
        return {xd, yd};
      // theta_d = theta (1 + k1 t^2 + k2 t^4 + k3 t^6 + k4 t^8); Newton for theta.
      double theta = rd;
      for(int i = 0; i < 20; i++)
      {
        const double t2 = theta * theta;
        const double f
            = theta * (1 + t2 * (m.k1 + t2 * (m.k2 + t2 * (m.k3 + t2 * m.k4)))) - rd;
        const double df
            = 1 + t2 * (3 * m.k1 + t2 * (5 * m.k2 + t2 * (7 * m.k3 + 9 * t2 * m.k4)));
        theta -= f / df;
      }
      if(!(theta >= 0) || theta >= 1.5707963267948966)
        return {std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN()};
      const double r = std::tan(theta);
      return {xd * r / rd, yd * r / rd};
    }

    case DEPTHCAM_DISTORTION_FTHETA: {
      // As librealsense: r = tan(k1 rd) / atan(2 tan(k1 / 2)).
      double rd = std::sqrt(xd * xd + yd * yd);
      if(rd < 1e-12)
        rd = 1e-12;
      const double r = std::tan(m.k1 * rd) / std::atan(2 * std::tan(m.k1 / 2.));
      return {xd * r / rd, yd * r / rd};
    }

    case DEPTHCAM_DISTORTION_NONE:
    default:
      return {xd, yd};
  }
}

void computeRayTable(const depthcam_intrinsics& in, int w, int h, float* xy)
{
  if(w <= 0 || h <= 0 || !xy)
    return;

  depthcam_intrinsics at = in;
  if(in.width != w || in.height != h)
  {
    const Model m = scaled(in, w, h);
    at.width = w;
    at.height = h;
    at.fx = float(m.fx);
    at.fy = float(m.fy);
    at.cx = float(m.cx);
    at.cy = float(m.cy);
  }

  for(int v = 0; v < h; v++)
    for(int u = 0; u < w; u++)
    {
      const auto r = pixelRay(at, u, v);
      const std::size_t i = 2 * (std::size_t(v) * w + u);
      xy[i] = float(r[0]);
      xy[i + 1] = float(r[1]);
    }
}

depthcam_intrinsics estimatedIntrinsics(int w, int h) noexcept
{
  depthcam_intrinsics in{};
  in.struct_size = sizeof(in);
  in.width = w;
  in.height = h;
  // fx = (w / 2) / tan(30 degrees)
  in.fx = float(w * 0.5 / 0.57735026918962576);
  in.fy = in.fx;
  in.cx = float((w - 1) * 0.5);
  in.cy = float((h - 1) * 0.5);
  in.model = DEPTHCAM_DISTORTION_NONE;
  return in;
}

const depthcam_intrinsics* CalibrationData::intrinsics(uint32_t stream) const noexcept
{
  for(const auto& s : streams)
    if(s.stream == stream)
      return &s.intrinsics;
  return nullptr;
}

CalibrationData Calibration::query(
    const depthcam_backend_v1& b, depthcam_device& dev, uint32_t streams, int depthW,
    int depthH)
{
  CalibrationData d;

  if(b.get_details)
  {
    depthcam_device_details det{};
    det.struct_size = sizeof(det);
    if(b.get_details(&dev, &det))
    {
      const auto str = [](const char* s) { return s ? std::string{s} : std::string{}; };
      d.model = str(det.model);
      d.serial = str(det.serial);
      d.firmware = str(det.firmware);
      d.hardwareRev = str(det.hardware_rev);
      d.depthMin = det.depth_min_m;
      d.depthMax = det.depth_max_m;
    }
  }

  for(uint32_t s : {uint32_t(DEPTHCAM_STREAM_DEPTH), uint32_t(DEPTHCAM_STREAM_COLOR),
                    uint32_t(DEPTHCAM_STREAM_IR)})
  {
    if(!(streams & s))
      continue;
    depthcam_intrinsics in{};
    in.struct_size = sizeof(in);
    if(b.get_intrinsics && b.get_intrinsics(&dev, s, &in) && in.width > 0
       && in.height > 0 && in.fx > 0 && in.fy > 0)
      d.streams.push_back({s, in});
  }

  // Depth is the one the GPU outputs cannot do without.
  if((streams & DEPTHCAM_STREAM_DEPTH) && !d.intrinsics(DEPTHCAM_STREAM_DEPTH)
     && depthW > 0 && depthH > 0)
  {
    d.streams.push_back({DEPTHCAM_STREAM_DEPTH, estimatedIntrinsics(depthW, depthH)});
    d.estimated = true;
  }

  if(b.get_extrinsics)
  {
    depthcam_extrinsics e{};
    e.struct_size = sizeof(e);
    if((streams & DEPTHCAM_STREAM_COLOR)
       && b.get_extrinsics(&dev, DEPTHCAM_STREAM_DEPTH, DEPTHCAM_STREAM_COLOR, &e))
      d.depthToColor = toMatrix(e);

    e = {};
    e.struct_size = sizeof(e);
    if((streams & DEPTHCAM_STREAM_IMU)
       && b.get_extrinsics(&dev, DEPTHCAM_STREAM_DEPTH, DEPTHCAM_STREAM_IMU, &e))
      d.depthToImu = toMatrix(e);
  }

  return d;
}

void Calibration::set(CalibrationData d)
{
  std::lock_guard l{m_lock};
  m_data = std::move(d);
  m_rays.reset();
  m_version.fetch_add(1, std::memory_order_acq_rel);
}

CalibrationData Calibration::get() const
{
  std::lock_guard l{m_lock};
  return m_data;
}

void Calibration::setRaySource(const depthcam_backend_v1* backend, depthcam_device* device)
{
  std::lock_guard l{m_lock};
  m_backend = backend;
  m_device = device;
  m_rays.reset();
}

std::shared_ptr<const std::vector<float>> Calibration::depthRays(int w, int h)
{
  std::lock_guard l{m_lock};
  const auto version = m_version.load(std::memory_order_acquire);
  if(m_rays && m_raysW == w && m_raysH == h && m_raysVersion == version)
    return m_rays;

  auto table = std::make_shared<std::vector<float>>(std::size_t(w) * h * 2);
  bool done = false;
  if(m_backend && m_device && m_backend->get_ray_table)
    done = m_backend->get_ray_table(m_device, DEPTHCAM_STREAM_DEPTH, table->data(), w, h)
           != 0;

  if(!done)
  {
    const auto* in = m_data.intrinsics(DEPTHCAM_STREAM_DEPTH);
    const auto guess = estimatedIntrinsics(w, h);
    computeRayTable(in ? *in : guess, w, h, table->data());
  }

  m_rays = std::move(table);
  m_raysW = w;
  m_raysH = h;
  m_raysVersion = version;
  return m_rays;
}

}
