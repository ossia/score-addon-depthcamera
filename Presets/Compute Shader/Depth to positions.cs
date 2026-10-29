/*{
  "DESCRIPTION": "Turns a depth image into a position map: one RGBA32F texel per pixel, xyz in metres in the camera's frame (+X right, +Y down, +Z forward), w = 1 where the depth is valid and 0 elsewhere. For depth from anything other than the Depth Camera Input device, which outputs this as its positions node: a Kinect v2 device, a capture card, a monocular depth model. Give it the camera's intrinsics in pixels of the depth image (the device publishes them under calibration/depth/intrinsics); with a focal length of 0 it assumes the horizontal field of view instead. Lens distortion is not undone.",
  "CREDIT": "ossia score",
  "ISFVSN": "2.0",
  "MODE": "COMPUTE_SHADER",
  "CATEGORIES": ["Depth"],
  "RESOURCES": [
    { "NAME": "depth", "TYPE": "image", "ACCESS": "read_only", "FORMAT": "r32f" },
    { "NAME": "focal", "TYPE": "point2D", "DEFAULT": [0, 0], "MIN": [0, 0], "MAX": [10000, 10000],
      "LABEL": "Focal length (fx, fy), pixels" },
    { "NAME": "principal", "TYPE": "point2D", "DEFAULT": [-1, -1], "MIN": [-1, -1], "MAX": [10000, 10000],
      "LABEL": "Principal point (cx, cy), pixels; negative for the centre" },
    { "NAME": "fov", "TYPE": "float", "DEFAULT": 60, "MIN": 1, "MAX": 170,
      "LABEL": "Horizontal field of view when the focal length is 0, degrees" },
    { "NAME": "scale", "TYPE": "float", "DEFAULT": 1, "MIN": 0, "MAX": 1000,
      "LABEL": "Metres per unit of the depth image" },
    { "NAME": "near", "TYPE": "float", "DEFAULT": 0, "MIN": 0, "MAX": 100, "LABEL": "Near limit, metres; 0 for none" },
    { "NAME": "far", "TYPE": "float", "DEFAULT": 0, "MIN": 0, "MAX": 100, "LABEL": "Far limit, metres; 0 for none" },
    { "NAME": "positions", "TYPE": "image", "ACCESS": "write_only", "FORMAT": "rgba32f",
      "WIDTH": "$WIDTH_depth", "HEIGHT": "$HEIGHT_depth" }
  ],
  "PASSES": [{
    "LOCAL_SIZE": [16, 16, 1],
    "EXECUTION_MODEL": { "TYPE": "2D_IMAGE", "TARGET": "positions" }
  }]
}*/

void main()
{
  ivec2 p = ivec2(gl_GlobalInvocationID.xy);
  ivec2 size = imageSize(positions);
  if(any(greaterThanEqual(p, size)))
    return;

  // IMG_LOAD and IMG_STORE address the picture with row 0 at the top on every
  // backend, which is where the intrinsics count rows from.
  float z = IMG_LOAD(depth, p).r * scale;
  bool valid = z > 0.0 && !isnan(z) && !isinf(z);
  if(near > 0.0 && z < near) valid = false;
  if(far > 0.0 && z > far) valid = false;

  vec2 f = focal;
  if(f.x <= 0.0)
    f = vec2(0.5 * float(size.x) / tan(radians(fov) * 0.5));
  if(f.y <= 0.0)
    f.y = f.x;
  // Pixel centres are at integers, as OpenCV and the depth camera device have
  // them: the centre of the image is at (size - 1) / 2.
  vec2 c = principal.x < 0.0 ? 0.5 * vec2(size - 1) : principal;

  vec2 ray = (vec2(p) - c) / f;
  IMG_STORE(positions, p, valid ? vec4(ray * z, z, 1.0) : vec4(0.0));
}
