#pragma once

/**
 * @file DepthCameraPointCloud.hpp
 * @brief The CPU point cloud of a backend, as a renderable geometry.
 *
 * In a header rather than in the protocol so the performance harness can
 * build the CPU path next to the GPU one.
 */

#include <Crousti/Executor.hpp>

#include <halp/controls.hpp>
#include <halp/geometry.hpp>
#include <halp/meta.hpp>

#include <DepthCamera/DepthCameraStream.hpp>

#include <QDebug>

#include <chrono>
#include <functional>

namespace Gfx::DepthCamera
{

/**
 * @brief Turns the camera's point-cloud buffer into a renderable geometry.
 *
 * Note this is *not* Threedim::PCLToMesh2, which reads from a gpu_buffer_input
 * port: a device root node has no inputs, the frames arrive from the camera
 * thread instead. It also carries its own c_name/uuid -- the version this
 * replaced had copied Threedim's, which would have collided the moment either
 * was registered as a process factory.
 */
struct PointCloudMesh
{
  halp_meta(name, "Depth camera point cloud")
  halp_meta(category, "Visuals/Meshes")
  halp_meta(c_name, "depthcam_pointcloud")
  halp_meta(uuid, "6ff23ad1-1ec3-4d1a-b0a2-2d7e57b30f8e")

  struct
  {
  } inputs;

  struct
  {
    struct
    {
      halp_meta(name, "Geometry");
      halp::dynamic_geometry mesh;
      float transform[16]{};
      bool dirty_mesh = false;
      bool dirty_transform = false;
    } geometry;
  } outputs;

  /// Owned by the renderer, valid for the duration of operator().
  AVFrame* pcl{};

  PointCloudMesh()
  {
    // Identity; the device node has no control inputs to build one from.
    outputs.geometry.transform[0] = 1.f;
    outputs.geometry.transform[5] = 1.f;
    outputs.geometry.transform[10] = 1.f;
    outputs.geometry.transform[15] = 1.f;
    outputs.geometry.dirty_transform = true;
  }

  void operator()()
  {
    if(!pcl)
      return;

    // InputStream packs the point count into width and the floats per point
    // into height: 3 for OB_FORMAT_POINT, 6 for OB_FORMAT_RGB_POINT.
    const int point_count = pcl->width;
    const int stride_floats = pcl->height;
    const int byte_size = pcl->linesize[0];

    if(point_count <= 0 || byte_size <= 0)
      return;
    if(stride_floats != 3 && stride_floats != 6)
      return;

    auto& mesh = outputs.geometry.mesh;
    mesh.topology = halp::primitive_topology::points;
    mesh.cull_mode = halp::cull_mode::none;
    mesh.front_face = halp::front_face::counter_clockwise;

    mesh.buffers.clear();
    mesh.bindings.clear();
    mesh.attributes.clear();
    mesh.input.clear();

    // No copy here: avnd's load_buffer copies into the ossia geometry during
    // upload, and the renderer keeps the frame alive until that has happened.
    halp::geometry_cpu_buffer buf{};
    buf.raw_data = pcl->data[0];
    buf.byte_size = byte_size;
    buf.dirty = true;
    mesh.buffers.push_back(buf);

    mesh.attributes.push_back(
        {.binding = 0,
         .semantic = halp::attribute_semantic::position,
         .format = halp::attribute_format::float3,
         .byte_offset = 0});

    if(stride_floats == 6)
    {
      // OBColorPoint is { x, y, z, r, g, b }; InputStream turns on the filter's
      // colorDataNormalization so the channels arrive in 0-1.
      mesh.attributes.push_back(
          {.binding = 0,
           .semantic = halp::attribute_semantic::color0,
           .format = halp::attribute_format::float3,
           .byte_offset = 3 * int(sizeof(float))});
    }

    mesh.bindings.push_back(
        {.stride = stride_floats * int(sizeof(float)),
         .step_rate = 1,
         .classification = halp::binding_classification::per_vertex});

    struct halp::geometry_input in;
    in.buffer = 0;
    in.byte_offset = 0;
    mesh.input.push_back(in);

    mesh.vertices = point_count;
    outputs.geometry.dirty_mesh = true;
  }
};

}

namespace oscr
{
/**
 * @brief avnd gfx node fed by a device rather than by a process.
 *
 * The body mirrors oscr::GfxRenderer, but that specialisation cannot be reused:
 * it is tied to oscr::GfxNode, which requires an oscr::ProcessModel and an
 * execution command queue. A device root node has neither -- which is also why
 * processControlIn/Out are absent here rather than merely disabled, and why
 * there is no "skip if the transport did not advance" check: a camera pushes
 * frames whether or not the transport is running.
 */
template <typename Node_T>
struct DeviceNode;

/// Set SCORE_DEPTHCAM_DEBUG=1 to have each geometry node report, once a
/// second, how many frames it pulled and how many uploads it performed. That
/// separates "the camera is not delivering" from "the graph is not asking",
/// which look identical from the outside.
inline bool depthcam_debug()
{
  static const bool on = qEnvironmentVariableIsSet("SCORE_DEPTHCAM_DEBUG");
  return on;
}

template <typename Node_T>
struct DeviceRenderer final : score::gfx::GenericNodeRenderer
{
  std::shared_ptr<Node_T> state;

  AVND_NO_UNIQUE_ADDRESS geometry_outputs_storage<Node_T> geometry_outs;

  const DeviceNode<Node_T>& node() const noexcept
  {
    return static_cast<const DeviceNode<Node_T>&>(score::gfx::NodeRenderer::node);
  }

  DeviceRenderer(const DeviceNode<Node_T>& p)
      : score::gfx::GenericNodeRenderer{p}
      , state{std::make_shared<Node_T>()}
  {
    prepareNewState<Node_T>(state, p);
  }

  ~DeviceRenderer() { releaseFrame(); }

  void init(score::gfx::RenderList& renderer, QRhiResourceUpdateBatch& res) override
  {
    if_possible(state->init(renderer, res));
  }

  void update(
      score::gfx::RenderList& renderer, QRhiResourceUpdateBatch& res,
      score::gfx::Edge* e) override
  {
    if_possible(state->update(renderer, res, e));
  }

  void release(score::gfx::RenderList& r) override
  {
    releaseFrame();
    if_possible(state->release(r));
  }

  void runInitialPasses(
      score::gfx::RenderList& renderer, QRhiCommandBuffer& commands,
      QRhiResourceUpdateBatch*& res, score::gfx::Edge& edge) override
  {
    auto& parent = node();

    // Hold on to the previous frame until the new one is in: releasing it
    // before we have a replacement would leave the geometry pointing at a
    // recycled buffer for one pass.
    if(AVFrame* next = parent.dec->dequeue_frame())
    {
      releaseFrame();
      m_frame = next;
      m_new_frames++;
    }
    m_uploads++;

    state->pcl = m_frame;
    if_possible((*state)());

    // load_buffer copies the point data into the ossia geometry here, so the
    // frame is free to go back to the pool afterwards.
    if constexpr(avnd::geometry_output_introspection<Node_T>::size > 0)
      geometry_outs.upload(renderer, *this->state, edge);

    if(depthcam_debug())
      reportDebug();
  }

  void reportDebug()
  {
    m_passes++;
    const auto now = std::chrono::steady_clock::now();
    if(m_last_report == std::chrono::steady_clock::time_point{})
      m_last_report = now;
    if(now - m_last_report < std::chrono::seconds(1))
      return;

    qDebug().noquote() << QStringLiteral(
                              "[depthcam] geometry: %1 render passes, %2 new "
                              "frames, %3 uploads in the last second")
                              .arg(m_passes)
                              .arg(m_new_frames)
                              .arg(m_uploads);
    m_passes = m_new_frames = m_uploads = 0;
    m_last_report = now;
  }

private:
  void releaseFrame()
  {
    if(m_frame)
    {
      node().dec->release_frame(m_frame);
      m_frame = nullptr;
    }
    if(state)
      state->pcl = nullptr;
  }

  AVFrame* m_frame{};

  int m_passes{}, m_new_frames{}, m_uploads{};
  std::chrono::steady_clock::time_point m_last_report{};
};

template <typename Node_T>
struct DeviceNode final
    : score::gfx::NodeModel
    , GpuNodeElements<Node_T>
{
  std::shared_ptr<Gfx::DepthCamera::InputStreamExtractor> dec;

  /// Told when the node starts and stops being drawn, so the camera can stop
  /// computing a point cloud nobody reads.
  std::function<void(bool)> consuming;
  bool m_consuming{};

  explicit DeviceNode(std::shared_ptr<Gfx::DepthCamera::InputStreamExtractor> d)
      : dec{std::move(d)}
  {
    initGfxPorts<Node_T>(this, this->input, this->output);
  }

  ~DeviceNode()
  {
    if(m_consuming && consuming)
      consuming(false);
  }

  void renderedNodesChanged() override
  {
    const bool now = !this->renderedNodes.empty();
    if(now != m_consuming)
    {
      m_consuming = now;
      if(consuming)
        consuming(now);
    }
  }

  score::gfx::NodeRenderer*
  createRenderer(score::gfx::RenderList& r) const noexcept override
  {
    return new DeviceRenderer<Node_T>{*this};
  }
};
}
