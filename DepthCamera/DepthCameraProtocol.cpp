#include "DepthCameraProtocol.hpp"

#include <Crousti/Executor.hpp>

#include <halp/controls.hpp>
#include <halp/geometry.hpp>
#include <halp/meta.hpp>

#include <ossia/detail/algorithms.hpp>
#include <ossia/network/base/osc_address.hpp>

#include <chrono>

namespace Gfx::DepthCamera
{
namespace
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

namespace Gfx::DepthCamera
{

namespace
{
/// The colour stream, through score's own video path, which reports when it
/// starts and stops being drawn.
class ColorNode final : public score::gfx::CameraNode
{
public:
  ColorNode(
      std::shared_ptr<InputStream> s, std::shared_ptr<InputStreamExtractor> dec)
      : score::gfx::CameraNode{dec, {}}
      , m_stream{std::move(s)}
  {
  }
  ~ColorNode() override
  {
    if(m_consuming && m_stream)
      m_stream->addConsumer(m_stream->m_rgb, -1);
  }

  void renderedNodesChanged() override
  {
    score::gfx::CameraNode::renderedNodesChanged();
    const bool now = !renderedNodes.empty();
    if(now != m_consuming && m_stream)
    {
      m_consuming = now;
      m_stream->addConsumer(m_stream->m_rgb, now ? 1 : -1);
    }
  }

private:
  std::shared_ptr<InputStream> m_stream;
  bool m_consuming{};
};
}

registered_node::registered_node(
    GfxExecutionAction& ctx, std::unique_ptr<score::gfx::Node> n)
    : context{&ctx}
    , node{n.get()}
{
  id = context->ui->register_node(std::move(n));
}

registered_node::~registered_node()
{
  context->ui->unregister_node(id);
}

depthcam_parameter::depthcam_parameter(
    std::shared_ptr<registered_node> nd, int out, ossia::net::node_base& n)
    : ossia::gfx::texture_parameter{n}
    , node{std::move(nd)}
    , outlet{out}
{
}

depthcam_parameter::~depthcam_parameter() = default;

void depthcam_parameter::pull_texture(port_index idx)
{
  node->context->setEdge(
      port_index{node->id, outlet}, idx, Process::CableType::ImmediateGlutton);

  score::gfx::Message m;
  m.node_id = node->id;
  node->context->ui->send_message(std::move(m));
}

depthcam_geometry_parameter::depthcam_geometry_parameter(
    std::shared_ptr<registered_node> nd, int out, ossia::net::node_base& n)
    : ossia::gfx::geometry_parameter{n}
    , node{std::move(nd)}
    , outlet{out}
{
}

depthcam_geometry_parameter::~depthcam_geometry_parameter() = default;

void depthcam_geometry_parameter::pull_geometry(port_index idx)
{
  node->context->setEdge(
      port_index{node->id, outlet}, idx, Process::CableType::ImmediateGlutton);

  score::gfx::Message m;
  m.node_id = node->id;
  node->context->ui->send_message(std::move(m));
}

depthcam_node::depthcam_node(
    ossia::net::device_base& dev, std::string name, const parameter_factory& make)
    : m_device{dev}
{
  // Name first: the parameter below captures *this, and the address has to be
  // valid by then.
  m_name = std::move(name);

  // node_base leaves m_oscAddressCache empty -- only generic_node fills it --
  // so a node deriving straight from node_base reports no address at all.
  // osc_address() came back as "", making every stream parameter look like
  // "cam:" rather than "cam:/depth", which breaks everything addressed by
  // name: OSC exposure, remote control, Score.iterateDevice in a script.
  m_oscAddressCache = ossia::net::osc_parameter_string(*this);

  m_parameter = make(*this);
}

depthcam_protocol::depthcam_protocol(
    const depthcam_backend_v1* backend, const QString& uri,
    const DepthCameraSettings& stgs)
    : ossia::net::protocol_base{flags{}}
    , stream{std::make_shared<InputStream>(backend, uri, stgs)}
{
}

depthcam_protocol::~depthcam_protocol()
{
  // Not left to the shared_ptr: see InputStream::close. This runs from
  // ~generic_device, after the node tree is cleared, which is the last
  // deterministic point in a device's life.
  if(stream)
    stream->close();
}

// The four hooks below only ever concern the `controls` subtree: the stream
// parameters are textures and geometry, which the graphics graph pulls
// directly and which carry no value for the network layer to move.
bool depthcam_protocol::pull(ossia::net::parameter_base& p)
{
  return m_controls && m_controls->read(p);
}

bool depthcam_protocol::push(
    const ossia::net::parameter_base& p, const ossia::value& v)
{
  return m_controls && m_controls->write(p, v);
}

bool depthcam_protocol::push_raw(const ossia::net::full_parameter_data&)
{
  // Nothing here is addressable without its parameter: a raw push carries an
  // address and a value but no descriptor, and every control needs its kind and
  // range to be interpreted.
  return false;
}

bool depthcam_protocol::observe(ossia::net::parameter_base& p, bool enable)
{
  // ossia calls this when a parameter gains its first callback and loses its
  // last, which is exactly the signal needed to decide whether a read-only
  // sensor is worth polling.
  return m_controls && m_controls->observe(p, enable);
}

bool depthcam_protocol::update(ossia::net::node_base&)
{
  // The tree is fixed once the camera is open, so there is nothing to explore;
  // re-reading the settings is still the useful thing to do on a refresh.
  if(m_controls)
    m_controls->refresh();
  return false;
}

void depthcam_protocol::registerExtractor(std::shared_ptr<InputStreamExtractor> dec)
{
  m_extractors.push_back(std::move(dec));
}

void depthcam_protocol::start_execution()
{
  stream->start();

  // The negotiated resolutions are only known once the pipeline is running.
  for(auto& dec : m_extractors)
    dec->refreshMetadata();
}

void depthcam_protocol::stop_execution()
{
  // Just stop. There is deliberately no "wait until no renderer is holding a
  // frame" dance here, unlike Gfx::video_texture_input_protocol: that exists so
  // avformat_close_input runs after every frame is freed, and we close no
  // demuxer. Frames a renderer has already dequeued are in neither queue, so
  // FrameQueue::drain() does not touch them, and point-cloud frames keep their
  // backend buffer alive through their own AVBufferRef regardless of the camera.
  //
  // Reaching into the gfx nodes from here is not an option: they belong to the
  // GfxContext, which can already be gone on a document switch.
  stream->stop();
}

depthcam_device_impl::depthcam_device_impl(
    const DepthCameraSettings& settings, GfxExecutionAction& ctx,
    std::unique_ptr<depthcam_protocol> proto, std::string name,
    ControlTree::StoreCallback onStore)
    : ossia::net::generic_device{std::move(proto), std::move(name)}
    , m_params{std::make_shared<RenderParams>()}
    , m_calib{std::make_shared<Calibration>()}
{
  auto& protocol = static_cast<depthcam_protocol&>(*m_protocol);
  auto& stream = protocol.stream;

  const auto extractor = [&](StreamOutput& out) {
    auto dec = std::make_shared<InputStreamExtractor>(stream, out, QString{});
    protocol.registerExtractor(dec);
    return dec;
  };
  const auto texture = [&](const std::shared_ptr<registered_node>& node, int outlet,
                           const char* child_name) {
    this->add_child(std::make_unique<depthcam_node>(
        *this, child_name, [&](ossia::net::node_base& n) {
      return std::make_unique<depthcam_parameter>(node, outlet, n);
    }));
  };

  if(settings.rgb)
  {
    auto dec = extractor(stream->m_rgb);
    auto node = std::make_shared<registered_node>(
        ctx, std::make_unique<ColorNode>(stream, dec));
    texture(node, 0, "rgb");
  }

  if(settings.ir)
  {
    auto dec = extractor(stream->m_ir);
    auto node = std::make_shared<registered_node>(
        ctx, std::make_unique<DepthStreamNode>(
                 DepthStreamNode::Kind::Infrared, stream, stream->m_ir, dec, m_params,
                 m_calib, settings.depthOutput));
    texture(node, 0, "ir");
  }

  // One node for every output computed from depth: the frame is uploaded and
  // turned into positions once, whichever of them are read.
  if(settings.depth || settings.positions || settings.normals || settings.mesh)
  {
    auto dec = extractor(stream->m_depth);
    auto node = std::make_shared<registered_node>(
        ctx, std::make_unique<DepthStreamNode>(
                 DepthStreamNode::Kind::Depth, stream, stream->m_depth, dec, m_params,
                 m_calib, settings.depthOutput));
    if(settings.depth)
      texture(node, DepthStreamNode::DepthOut, "depth");
    if(settings.positions)
      texture(node, DepthStreamNode::PositionsOut, "positions");
    if(settings.normals)
      texture(node, DepthStreamNode::NormalsOut, "normals");
    if(settings.mesh)
      this->add_child(std::make_unique<depthcam_node>(
          *this, "mesh", [&](ossia::net::node_base& n) {
        return std::make_unique<depthcam_geometry_parameter>(
            node, DepthStreamNode::MeshOut, n);
      }));
  }

  if(settings.pointcloud)
  {
    auto dec = extractor(stream->m_pcl);
    auto pcl = std::make_unique<oscr::DeviceNode<PointCloudMesh>>(dec);
    pcl->consuming = [s = std::weak_ptr<InputStream>{stream}](bool on) {
      if(auto st = s.lock())
        st->addConsumer(st->m_pcl, on ? 1 : -1);
    };
    auto node = std::make_shared<registered_node>(ctx, std::move(pcl));
    this->add_child(std::make_unique<depthcam_node>(
        *this, "pointcloud", [&](ossia::net::node_base& n) {
      return std::make_unique<depthcam_geometry_parameter>(node, 0, n);
    }));
  }

  // Only if the camera really has one: a Kinect v2 does not, and three nodes
  // that never move are worse than none.
  if(settings.imu && (stream->activeStreams() & DEPTHCAM_STREAM_IMU))
    m_imu = std::make_unique<ImuTree>(stream, *this, *this);

  // Calibration and identity: what the GPU outputs are computed with, and what
  // a shader binds to reproject them.
  if(auto* backend = stream->backend(); backend && stream->device())
  {
    m_calib->setRaySource(backend, stream->device());
    QString backendName = QString::fromUtf8(backend->name ? backend->name : "");
    m_info = std::make_unique<InfoTree>(
        stream, m_calib, *this, settings.device, backendName, QString{});
  }

  // Last, so the streams stay at the top of the explorer: a camera can publish
  // fifty settings and the four things a user is looking for should not be at
  // the bottom of that list.
  if(auto* backend = stream->backend(); backend && stream->device())
  {
    auto tree = std::make_unique<ControlTree>(
        *backend, *stream->device(), stream, *this, *this, m_params, settings.controls,
        std::move(onStore), [this] {
      if(m_info)
        m_info->refresh();
    });
    if(!tree->empty())
    {
      protocol.setControls(tree.get());
      m_controls = std::move(tree);
    }
  }
}

void depthcam_device_impl::releaseControls() noexcept
{
  // The protocol must stop routing writes here before the tree goes.
  static_cast<depthcam_protocol&>(*m_protocol).setControls(nullptr);
  m_controls.reset();

  // Stops the camera thread's stats callback, and the renderers' ray lookups.
  m_info.reset();
  if(m_calib)
    m_calib->setRaySource(nullptr, nullptr);

  // And the camera must stop pushing samples at parameters that are about to
  // be freed. ~ImuTree does not return until any callback in flight has.
  m_imu.reset();
}

depthcam_device_impl::~depthcam_device_impl()
{
  // Usually already done from InputDevice::disconnect(); this covers the paths
  // that destroy the device without going through it.
  releaseControls();
}

}
