#pragma once

/**
 * @file DepthCameraGpu.hpp
 * @brief Depth and infrared, from the sensor's raw samples to metric outputs,
 * on the GPU.
 *
 * One node per camera for depth, with four outlets that share a single upload
 * and a single processing pass per frame:
 *
 *   0 depth      R32F, metres along +Z, 0 where invalid; or the 0-4.5 m grey
 *                preview, RGBA8, for documents from before metric depth
 *   1 positions  RGBA32F, xyz in metres in the depth camera's frame, w = 1
 *                where valid. One texel per depth pixel, so N vertices of a
 *                particle system map onto N texels
 *   2 normals    RGBA16F, from neighbouring positions, facing the camera
 *   3 mesh       geometry: the organised grid of positions as a GPU vertex
 *                buffer (position, normal, texcoord0 into the colour image),
 *                with triangles across depth discontinuities collapsed
 *
 * and one node for infrared (R32F, scaled by render/ir_gain).
 *
 * The raw frame is uploaded as it arrives, R16 or R32F, with no CPU-side
 * conversion; the unit, the clip planes, the stage box and the temporal
 * smoothing are uniforms. An outlet that nothing is connected to costs
 * nothing: the passes run only for what is read.
 *
 * Orientation follows score's render targets, so the textures sample the
 * same way as any other node's output; the mesh is in image order, row-major
 * from the top-left pixel.
 */

#include <Gfx/Graph/Node.hpp>
#include <Gfx/Graph/NodeRenderer.hpp>
#include <Gfx/Graph/VideoNode.hpp>

#include <DepthCamera/DepthCameraCalibration.hpp>
#include <DepthCamera/DepthCameraSettings.hpp>

#include <array>
#include <atomic>
#include <memory>

namespace Gfx::DepthCamera
{
class InputStream;
class InputStreamExtractor;
struct StreamOutput;

/**
 * @brief Host-side processing knobs: `<cam>/controls/render/...`.
 *
 * Written from whichever thread a control write arrives on, read by the
 * renderers once per frame. Works the same on every backend, including the
 * ones whose SDK has no filters at all.
 */
struct RenderParams
{
  std::atomic<float> near_m{0.f};  ///< 0: no near limit
  std::atomic<float> far_m{0.f};   ///< 0: no far limit
  std::atomic<float> ir_gain{1.f};
  std::atomic<float> temporal{0.f}; ///< weight of the history, 0 = off
  /// A triangle whose longest edge exceeds this times its depth is dropped
  /// from the mesh. 0 keeps every triangle with three valid corners.
  std::atomic<float> edge_cull{0.05f};
  std::atomic<bool> box_enabled{false};
  std::array<std::atomic<float>, 3> box_min{-1.f, -1.f, 0.f};
  std::array<std::atomic<float>, 3> box_max{1.f, 1.f, 4.f};
  std::atomic<int> topology{0}; ///< 0 triangles, 1 points
};

class DepthStreamNode final : public score::gfx::ProcessNode
{
public:
  enum class Kind
  {
    Depth,
    Infrared,
  };
  enum Outlet
  {
    DepthOut = 0,
    PositionsOut = 1,
    NormalsOut = 2,
    MeshOut = 3,
  };

  DepthStreamNode(
      Kind kind, std::shared_ptr<InputStream> stream, StreamOutput& out,
      std::shared_ptr<InputStreamExtractor> extractor,
      std::shared_ptr<RenderParams> params, std::shared_ptr<Calibration> calib,
      DepthCameraSettings::DepthOutput mode);
  ~DepthStreamNode();

  score::gfx::NodeRenderer* createRenderer(score::gfx::RenderList& r) const noexcept override;
  void process(score::gfx::Message&& msg) override;
  void renderedNodesChanged() override;

  const Kind kind;
  const DepthCameraSettings::DepthOutput mode;

  std::shared_ptr<InputStream> stream;
  StreamOutput& out;
  std::shared_ptr<RenderParams> params;
  std::shared_ptr<Calibration> calib;

  /// Frames shared by every RenderList this node is drawn in.
  score::gfx::VideoFrameShare frames;

private:
  bool m_consuming{};
};

}
