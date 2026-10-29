#pragma once

/**
 * @file SyntheticScene.hpp
 * @brief The scene the synthetic camera films, in closed form.
 *
 * Shared by the backend, which renders it, and by the tests, which compute
 * what every texel, vertex and calibration value must be from the same
 * functions. Everything is in double precision and depends only on the pixel
 * and the frame number, so two runs produce the same bytes.
 *
 * Camera frame: +X right, +Y down, +Z forward, metres, as depthcam_abi.h
 * states. Depth is the Z of the first surface hit, not the range.
 *
 *   - a wall, z = 3 m, behind everything;
 *   - a floor, y = +0.9 m (below the camera);
 *   - a sphere of radius 0.3 m centred on (sx, 0, 1.8): its nearest point on
 *     the optical axis is at exactly 1.5 m when sx = 0;
 *   - a box from (-1.1, -0.75, 1.9) to (-0.5, -0.15, 2.3), up and to the left,
 *     whose edges are depth discontinuities against the wall;
 *   - a 16 x 16 pixel dropout in the top-left corner, where the sensor reports
 *     nothing (0).
 *
 * Pixel coordinates follow depthcam_abi.h: the centre of pixel (u, v) is at
 * (u, v).
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace depthcam_synthetic
{

/// Same values as depthcam_distortion.
enum Distortion
{
  None = 0,
  BrownConrady = 1,
};

struct Camera
{
  int width{}, height{};
  double fx{}, fy{}, cx{}, cy{};
  int model{None};
  double k[6]{}; ///< k1..k6
  double p[2]{}; ///< p1, p2
};

/// Row-major rotation and a translation in metres: p_to = R p_from + t.
struct Rigid
{
  double r[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  double t[3]{};
};

inline Camera depthCamera(int w, int h, bool distorted)
{
  Camera c;
  c.width = w;
  c.height = h;
  c.fx = 0.8 * w;
  c.fy = 0.8 * w;
  c.cx = (w - 1) * 0.5;
  c.cy = (h - 1) * 0.5;
  if(distorted)
  {
    c.model = BrownConrady;
    c.k[0] = -0.12;
    c.k[1] = 0.05;
    c.p[0] = 0.001;
    c.p[1] = -0.0005;
  }
  return c;
}

/// Off-centre on purpose, so a test that forgets cx or cy gets it wrong.
inline Camera colorCamera(int w, int h)
{
  Camera c;
  c.width = w;
  c.height = h;
  c.fx = 0.75 * w;
  c.fy = 0.75 * w;
  c.cx = (w - 1) * 0.5 + 3.5;
  c.cy = (h - 1) * 0.5 - 2.25;
  return c;
}

/// Depth to colour: 32 mm to the left, a degree of yaw.
inline Rigid depthToColor()
{
  Rigid x;
  const double a = 1.0 * 3.14159265358979323846 / 180.;
  x.r[0] = std::cos(a);
  x.r[2] = std::sin(a);
  x.r[6] = -std::sin(a);
  x.r[8] = std::cos(a);
  x.t[0] = -0.032;
  x.t[1] = 0.001;
  x.t[2] = 0.002;
  return x;
}

/// OpenCV's forward model, normalised undistorted -> normalised distorted.
inline void distort(const Camera& c, double x, double y, double& xd, double& yd)
{
  if(c.model == None)
  {
    xd = x;
    yd = y;
    return;
  }
  const double r2 = x * x + y * y;
  const double r4 = r2 * r2, r6 = r4 * r2;
  const double radial = (1 + c.k[0] * r2 + c.k[1] * r4 + c.k[2] * r6)
                        / (1 + c.k[3] * r2 + c.k[4] * r4 + c.k[5] * r6);
  xd = x * radial + 2 * c.p[0] * x * y + c.p[1] * (r2 + 2 * x * x);
  yd = y * radial + c.p[0] * (r2 + 2 * y * y) + 2 * c.p[1] * x * y;
}

/// Inverts distort() by fixed-point iteration, far past convergence.
inline void undistort(const Camera& c, double xd, double yd, double& x, double& y)
{
  x = xd;
  y = yd;
  if(c.model == None)
    return;
  for(int i = 0; i < 200; i++)
  {
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2, r6 = r4 * r2;
    const double radial = (1 + c.k[0] * r2 + c.k[1] * r4 + c.k[2] * r6)
                          / (1 + c.k[3] * r2 + c.k[4] * r4 + c.k[5] * r6);
    const double dx = 2 * c.p[0] * x * y + c.p[1] * (r2 + 2 * x * x);
    const double dy = c.p[0] * (r2 + 2 * y * y) + 2 * c.p[1] * x * y;
    x = (xd - dx) / radial;
    y = (yd - dy) / radial;
  }
}

/// (x/z, y/z) of the ray through the centre of pixel (u, v).
inline std::array<double, 2> ray(const Camera& c, double u, double v)
{
  double x{}, y{};
  undistort(c, (u - c.cx) / c.fx, (v - c.cy) / c.fy, x, y);
  return {x, y};
}

/// Pixel a camera-frame point projects to, distortion included.
inline std::array<double, 2> project(const Camera& c, const double p[3])
{
  double xd{}, yd{};
  distort(c, p[0] / p[2], p[1] / p[2], xd, yd);
  return {xd * c.fx + c.cx, yd * c.fy + c.cy};
}

inline void transform(const Rigid& x, const double p[3], double out[3])
{
  for(int i = 0; i < 3; i++)
    out[i] = x.r[3 * i] * p[0] + x.r[3 * i + 1] * p[1] + x.r[3 * i + 2] * p[2] + x.t[i];
}

constexpr double wall_z = 3.0;
constexpr double floor_y = 0.9;
constexpr double sphere_r = 0.3;
constexpr double sphere_z = 1.8;
constexpr double box_min[3]{-1.1, -0.75, 1.9};
constexpr double box_max[3]{-0.5, -0.15, 2.3};
constexpr int dropout = 16;

/// The sphere sways when `moving`; still otherwise, centred on the axis.
inline double sphereX(int64_t frame, bool moving)
{
  return moving ? 0.25 * std::sin(2. * 3.14159265358979323846 * double(frame) / 90.)
                : 0.;
}

/// Z of the first hit along (x, y, 1), in metres.
inline double hitDepth(double x, double y, double sx)
{
  double best = wall_z;

  if(y > 0)
    best = std::min(best, floor_y / y);

  {
    // |t d - c|^2 = r^2, d = (x, y, 1), c = (sx, 0, sphere_z)
    const double a = x * x + y * y + 1;
    const double b = -2 * (x * sx + sphere_z);
    const double cc = sx * sx + sphere_z * sphere_z - sphere_r * sphere_r;
    const double disc = b * b - 4 * a * cc;
    if(disc >= 0)
    {
      const double t = (-b - std::sqrt(disc)) / (2 * a);
      if(t > 0)
        best = std::min(best, t);
    }
  }

  {
    const double d[3]{x, y, 1};
    double t0 = 0, t1 = 1e30;
    bool hit = true;
    for(int i = 0; i < 3 && hit; i++)
    {
      if(std::abs(d[i]) < 1e-12)
      {
        hit = box_min[i] <= 0 && 0 <= box_max[i];
        continue;
      }
      double a = box_min[i] / d[i], b = box_max[i] / d[i];
      if(a > b)
        std::swap(a, b);
      t0 = std::max(t0, a);
      t1 = std::min(t1, b);
      hit = t0 <= t1;
    }
    if(hit && t0 > 0)
      best = std::min(best, t0);
  }
  return best;
}

/// Depth at pixel (u, v), in metres, 0 where the sensor reports nothing.
inline double depthAt(const Camera& c, int u, int v, int64_t frame, bool moving)
{
  if(u < dropout && v < dropout)
    return 0.;
  const auto r = ray(c, u, v);
  return hitDepth(r[0], r[1], sphereX(frame, moving));
}

/// The infrared picture: a ramp that moves with the frame number and scales
/// with the laser power, so a control write is visible in the image.
inline uint16_t irAt(int u, int v, int64_t frame, double laser_mw)
{
  const int base = int((u * 97 + v * 61 + frame * 13) % 4096) * 16;
  const double scaled = base * laser_mw / 150.;
  return uint16_t(std::min(65535., std::max(0., std::round(scaled))));
}

/// Colour: red and green encode the pixel, so a texture coordinate can be
/// checked from the colour it samples; blue is the frame number.
inline std::array<uint8_t, 3> colorAt(const Camera& c, int u, int v, int64_t frame)
{
  return {
      uint8_t(std::lround(u * 255. / (c.width - 1))),
      uint8_t(std::lround(v * 255. / (c.height - 1))), uint8_t(frame & 0xFF)};
}

/// Device clock, nanoseconds: starts at one second, one period per frame.
inline uint64_t timestampNs(int64_t frame, int fps)
{
  return 1'000'000'000ull + uint64_t(frame) * 1'000'000'000ull / uint64_t(fps);
}

}
