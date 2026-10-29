#include "DepthCameraControls.hpp"

#include <ossia/network/base/node_attributes.hpp>
#include <ossia/network/common/complex_type.hpp>
#include <ossia/network/dataspace/dataspace_visitors.hpp>
#include <ossia/network/domain/domain.hpp>

#include <ossia-qt/invoke.hpp>

#include <DepthCamera/DepthCameraGpu.hpp>
#include <DepthCamera/DepthCameraStream.hpp>

#include <QDebug>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace Gfx::DepthCamera
{
namespace
{

/// How often observed read-only controls are re-read.
///
/// Slow on purpose. Everything in this category is a temperature, a supply
/// voltage or an exposure under auto, none of which move fast, and each read is
/// a transfer competing with the video stream -- a TCP round trip, for a camera
/// on the network. A camera nobody is listening to costs nothing at all: the
/// worker sleeps until something is observed.
constexpr auto poll_interval = std::chrono::milliseconds{500};

ossia::val_type type_for(const depthcam_control& c) noexcept
{
  switch(c.kind)
  {
    case DEPTHCAM_CONTROL_BOOL:
      return ossia::val_type::BOOL;
    case DEPTHCAM_CONTROL_ACTION:
      return ossia::val_type::IMPULSE;
    case DEPTHCAM_CONTROL_ENUM:
      // A named enum reads and writes as its label; an unnamed one is just an
      // int with a restricted range.
      return c.enum_labels && c.enum_count > 0 ? ossia::val_type::STRING
                                               : ossia::val_type::INT;
    case DEPTHCAM_CONTROL_FLOAT:
      return ossia::val_type::FLOAT;
    case DEPTHCAM_CONTROL_INT:
    default:
      return ossia::val_type::INT;
  }
}

ossia::domain domain_for(const depthcam_control& c)
{
  switch(c.kind)
  {
    case DEPTHCAM_CONTROL_ENUM: {
      if(c.enum_labels && c.enum_count > 0)
      {
        std::vector<std::string> labels;
        labels.reserve(std::size_t(c.enum_count));
        for(int i = 0; i < c.enum_count; i++)
          labels.push_back(c.enum_labels[i] ? c.enum_labels[i] : "");
        return ossia::make_domain(std::move(labels));
      }
      return ossia::make_domain(int(c.min), int(c.max));
    }
    case DEPTHCAM_CONTROL_INT:
      return ossia::make_domain(int(c.min), int(c.max));
    case DEPTHCAM_CONTROL_FLOAT:
      return ossia::make_domain(float(c.min), float(c.max));
    case DEPTHCAM_CONTROL_BOOL:
    case DEPTHCAM_CONTROL_ACTION:
    default:
      return {};
  }
}

ossia::access_mode access_for(int access) noexcept
{
  const bool r = (access & DEPTHCAM_ACCESS_READ) != 0;
  const bool w = (access & DEPTHCAM_ACCESS_WRITE) != 0;
  if(r && w)
    return ossia::access_mode::BI;
  return w ? ossia::access_mode::SET : ossia::access_mode::GET;
}

/// The index an enum label stands for, given that labels are indexed from `min`
/// in steps of `step`.
int enum_index_of(const depthcam_control& c, int value) noexcept
{
  const double step = c.step > 0 ? c.step : 1;
  const int i = int(std::lround((value - c.min) / step));
  return (i >= 0 && i < c.enum_count) ? i : -1;
}

/// The units ossia can convert between. Anything else -- microseconds,
/// milliwatts, kelvin, percent -- is stated in the description instead: tagging
/// a temperature as a distance so the field is filled in would let a consumer
/// convert it as one.
ossia::unit_t unit_for(std::string_view u)
{
  if(u == "m")
    return ossia::parse_pretty_unit("distance.m");
  if(u == "cm")
    return ossia::parse_pretty_unit("distance.cm");
  if(u == "mm")
    return ossia::parse_pretty_unit("distance.mm");
  if(u == "s")
    return ossia::parse_pretty_unit("time.s");
  if(u == "ms")
    return ossia::parse_pretty_unit("time.ms");
  if(u == "dB")
    return ossia::parse_pretty_unit("gain.dB");
  return {};
}

std::string description_for(const depthcam_control2& c2)
{
  const auto& c = c2.base;
  std::string d = c.name ? c.name : "";
  if(c.description && *c.description)
  {
    if(!d.empty())
      d += " -- ";
    d += c.description;
  }
  if(c2.unit && *c2.unit)
    d += std::string{" ["} + c2.unit + "]";
  if((c.kind == DEPTHCAM_CONTROL_INT || c.kind == DEPTHCAM_CONTROL_FLOAT) && c.step > 0
     && c.step != 1)
  {
    char buf[64];
    std::snprintf(buf, sizeof(buf), " (step %g)", c.step);
    d += buf;
  }
  if(c.kind == DEPTHCAM_CONTROL_ACTION)
    d += d.empty() ? "trigger" : " (trigger)";
  if(c2.flags & DEPTHCAM_CONTROL_REQUIRES_RESTART)
    d += " (restarts the stream)";
  if(c2.auto_control && *c2.auto_control)
    d += std::string{" (read-only while "} + c2.auto_control + " is on)";
  return d;
}

/// Walks `a/b/c`, creating what is missing. Returns the node the leaf goes in
/// and writes the leaf name to @p leaf.
ossia::net::node_base*
resolve_group(ossia::net::node_base& root, std::string_view id, std::string& leaf)
{
  ossia::net::node_base* cur = &root;

  for(;;)
  {
    const auto slash = id.find('/');
    if(slash == std::string_view::npos)
    {
      leaf = std::string{id};
      return leaf.empty() ? nullptr : cur;
    }

    const auto segment = id.substr(0, slash);
    id = id.substr(slash + 1);
    if(segment.empty())
      continue;

    // find_child before create_child: create_child uniquifies a name that is
    // already taken, so every control in a group would otherwise land in
    // "color", "color.1", "color.2"...
    auto* next = cur->find_child(segment);
    if(!next)
      next = cur->create_child(std::string{segment});
    if(!next)
      return nullptr;
    cur = next;
  }
}

/// The host-side controls, in the order they are published.
struct RenderControl
{
  const char* id;
  const char* description;
  ossia::val_type type;
  float min, max;
};

constexpr RenderControl render_controls[]{
    {"render/near", "Depth closer than this is invalid; 0 for no limit [m]",
     ossia::val_type::FLOAT, 0.f, 20.f},
    {"render/far", "Depth farther than this is invalid; 0 for no limit [m]",
     ossia::val_type::FLOAT, 0.f, 20.f},
    {"render/ir_gain", "Infrared brightness", ossia::val_type::FLOAT, 0.f, 64.f},
    {"render/temporal",
     "Temporal smoothing: the weight of the previous frames, 0 for none. A change "
     "of more than 5 % restarts the average",
     ossia::val_type::FLOAT, 0.f, 0.99f},
    {"render/edge_cull",
     "Mesh triangles whose longest edge is more than this times their depth are "
     "dropped; 0 keeps every triangle",
     ossia::val_type::FLOAT, 0.f, 1.f},
    {"render/stage_box/enabled", "Only keep what lies inside the box",
     ossia::val_type::BOOL, 0.f, 1.f},
    {"render/stage_box/min", "Corner of the box, in the depth camera's frame [m]",
     ossia::val_type::VEC3F, -20.f, 20.f},
    {"render/stage_box/max", "Opposite corner of the box [m]", ossia::val_type::VEC3F,
     -20.f, 20.f},
    {"render/topology", "How the mesh output connects its vertices",
     ossia::val_type::STRING, 0.f, 0.f},
};

const char* const topology_labels[]{"triangles", "points"};

std::vector<double> renderValue(const RenderParams& p, std::string_view id)
{
  if(id == "render/near")
    return {p.near_m.load()};
  if(id == "render/far")
    return {p.far_m.load()};
  if(id == "render/ir_gain")
    return {p.ir_gain.load()};
  if(id == "render/temporal")
    return {p.temporal.load()};
  if(id == "render/edge_cull")
    return {p.edge_cull.load()};
  if(id == "render/stage_box/enabled")
    return {p.box_enabled.load() ? 1. : 0.};
  if(id == "render/stage_box/min")
    return {p.box_min[0].load(), p.box_min[1].load(), p.box_min[2].load()};
  if(id == "render/stage_box/max")
    return {p.box_max[0].load(), p.box_max[1].load(), p.box_max[2].load()};
  if(id == "render/topology")
    return {double(p.topology.load())};
  return {};
}

void setRender(RenderParams& p, std::string_view id, const std::vector<double>& v)
{
  if(v.empty())
    return;
  const auto f = [&](int i) { return float(v[std::size_t(std::min<int>(i, int(v.size()) - 1))]); };
  if(id == "render/near")
    p.near_m = std::max(0.f, f(0));
  else if(id == "render/far")
    p.far_m = std::max(0.f, f(0));
  else if(id == "render/ir_gain")
    p.ir_gain = std::max(0.f, f(0));
  else if(id == "render/temporal")
    p.temporal = std::clamp(f(0), 0.f, 0.99f);
  else if(id == "render/edge_cull")
    p.edge_cull = std::max(0.f, f(0));
  else if(id == "render/stage_box/enabled")
    p.box_enabled = v[0] != 0.;
  else if(id == "render/stage_box/min")
    for(int i = 0; i < 3; i++)
      p.box_min[std::size_t(i)] = f(i);
  else if(id == "render/stage_box/max")
    for(int i = 0; i < 3; i++)
      p.box_max[std::size_t(i)] = f(i);
  else if(id == "render/topology")
    p.topology = std::clamp(int(v[0]), 0, 1);
}

ossia::value renderAsValue(const RenderControl& rc, const std::vector<double>& v)
{
  switch(rc.type)
  {
    case ossia::val_type::BOOL:
      return v.empty() ? false : v[0] != 0.;
    case ossia::val_type::VEC3F:
      if(v.size() >= 3)
        return ossia::vec3f{float(v[0]), float(v[1]), float(v[2])};
      return ossia::vec3f{};
    case ossia::val_type::STRING:
      return std::string{topology_labels[std::clamp(v.empty() ? 0 : int(v[0]), 0, 1)]};
    default:
      return v.empty() ? 0.f : float(v[0]);
  }
}

const RenderControl* renderControl(std::string_view id)
{
  for(auto& rc : render_controls)
    if(id == rc.id)
      return &rc;
  return nullptr;
}

} // namespace

ControlTree::ControlTree(
    const depthcam_backend_v1& backend, depthcam_device& device,
    std::shared_ptr<InputStream> stream, ossia::net::device_base&,
    ossia::net::node_base& parent, std::shared_ptr<RenderParams> params,
    ControlValues saved, StoreCallback onStore, std::function<void()> onRestarted)
    : m_backend{backend}
    , m_device{device}
    , m_stream{std::move(stream)}
    , m_params{std::move(params)}
    , m_onStore{std::move(onStore)}
    , m_onRestarted{std::move(onRestarted)}
    , m_replayed{m_replayedPromise.get_future().share()}
{
  auto* root = parent.find_child(std::string_view{"controls"});
  if(!root)
    root = parent.create_child("controls");
  if(!root)
  {
    m_replayedPromise.set_value();
    return;
  }

  buildBackendControls(*root);
  if(m_params)
    buildRenderControls(*root);
  if(!m_entries.empty())
    buildActions(*root);

  // Which toggle rules which control.
  for(std::size_t i = 0; i < m_entries.size(); i++)
  {
    auto& e = *m_entries[i];
    if(e.desc < 0 || m_descs[std::size_t(e.desc)].autoControl.empty())
      continue;
    const int a = indexOf(m_descs[std::size_t(e.desc)].autoControl);
    if(a >= 0 && m_entries[std::size_t(a)]->kind == DEPTHCAM_CONTROL_BOOL)
    {
      e.autoEntry = a;
      m_entries[std::size_t(a)]->ruled.push_back(int(i));
    }
  }

  for(auto& e : m_entries)
    if(e->flags & DEPTHCAM_CONTROL_INACTIVE)
      setInactive(*e, true);

  if(m_backend.set_control_changed_callback)
    m_backend.set_control_changed_callback(&m_device, &ControlTree::onChangedThunk, this);

  m_running.store(true, std::memory_order_release);
  m_worker = std::thread{[this] { run(); }};

  // First the record, in its order, then the gate the stream waits on, then
  // the values the constructor did not wait for.
  applySaved(saved);
  post({.kind = Request::Gate});
  if(m_stream)
    m_stream->setStartGate(m_replayed);
  refresh();
}

ControlTree::~ControlTree()
{
  // Nothing may call back into a tree that is going away.
  if(m_backend.set_control_changed_callback)
    m_backend.set_control_changed_callback(&m_device, nullptr, nullptr);

  // The worker first, and joined, not just signalled: it reads through
  // m_backend at a device that is closed as soon as this returns.
  if(m_running.exchange(false, std::memory_order_acq_rel))
  {
    wake();
    if(m_worker.joinable())
      m_worker.join();
  }

  // A stream still waiting for the replay must not wait for a tree that is
  // gone.
  try
  {
    m_replayedPromise.set_value();
  }
  catch(...)
  {
  }

  // Deliberately *not* callbacks_clear() on the parameters: nothing here ever
  // installs a callback. Writes arrive through depthcam_protocol::push, which
  // the device clears before destroying this object. And the parameters cannot
  // be touched here anyway -- Device::DeviceInterface::disconnect() calls
  // root.clear_children() on the way down, so by the time this runs on a
  // reconnect or a device removal they are already gone.
  //
  // Anything the worker posted to m_context and that has not run yet dies with
  // it: ~QObject drops a QObject's undelivered events.
}

void ControlTree::buildBackendControls(ossia::net::node_base& root)
{
  // Collected first, published second: list_controls hands out pointers into
  // the backend's own storage, and creating a node in the callback would mean
  // reentering the tree while the backend is walking its property list.
  std::vector<Desc> descs;
  if(m_backend.list_controls2)
  {
    m_backend.list_controls2(
        &m_device,
        [](const depthcam_control2* c, void* user) {
      if(!c || !c->base.id || !*c->base.id)
        return;
      Desc d;
      // Only what both sides know about: a newer backend's struct is longer.
      std::memcpy(&d.c, c, std::min<std::size_t>(c->struct_size, sizeof(d.c)));
      d.c.struct_size = sizeof(d.c);
      d.unit = c->unit ? c->unit : "";
      d.autoControl = c->auto_control ? c->auto_control : "";
      // The copies outlive whatever the backend's pointers were valid for.
      static_cast<std::vector<Desc>*>(user)->push_back(std::move(d));
        },
        &descs);
  }
  else if(m_backend.list_controls)
  {
    m_backend.list_controls(
        &m_device,
        [](const depthcam_control* c, void* user) {
      if(!c || !c->id || !*c->id)
        return;
      Desc d;
      d.c.struct_size = sizeof(d.c);
      d.c.base = *c;
      static_cast<std::vector<Desc>*>(user)->push_back(std::move(d));
        },
        &descs);
  }

  m_descs.reserve(descs.size());
  for(auto& d : descs)
  {
    d.c.unit = nullptr;
    d.c.auto_control = nullptr;
    const auto& c = d.c.base;

    std::string leaf;
    auto* group = resolve_group(root, c.id, leaf);
    if(!group)
      continue;
    auto* node = group->create_child(leaf);
    if(!node)
      continue;
    auto* param = node->create_parameter(type_for(c));
    if(!param)
      continue;

    param->set_domain(domain_for(c));
    param->set_access(access_for(c.access));
    param->set_bounding(ossia::bounding_mode::CLIP);
    if(auto u = unit_for(d.unit))
      param->set_unit(u);

    {
      depthcam_control2 forDescription = d.c;
      forDescription.unit = d.unit.empty() ? nullptr : d.unit.c_str();
      forDescription.auto_control = d.autoControl.empty() ? nullptr : d.autoControl.c_str();
      if(auto s = description_for(forDescription); !s.empty())
        ossia::net::set_description(*node, std::move(s));
    }

    ossia::net::tags tags;
    if(d.c.flags & DEPTHCAM_CONTROL_ADVANCED)
      tags.push_back("advanced");
    if(d.c.flags & DEPTHCAM_CONTROL_FILTER)
      tags.push_back("filter");
    if(d.c.flags & DEPTHCAM_CONTROL_REQUIRES_RESTART)
      tags.push_back("restart");
    if(d.c.flags & DEPTHCAM_CONTROL_VOLATILE)
      tags.push_back("volatile");
    if(!tags.empty())
      ossia::net::set_tags(*node, std::move(tags));

    auto e = std::make_unique<Entry>();
    e->id = c.id;
    e->param = param;
    e->origin = Origin::Backend;
    e->kind = c.kind;
    e->access = c.access;
    e->flags = d.c.flags;

    // Worth watching: what we cannot write, and what changes on its own.
    // Everything else we already know the value of, because we are the one who
    // set it, and re-reading forty of those twice a second is a stream of
    // transfers competing with the video for no information at all.
    const bool readable = c.access & DEPTHCAM_ACCESS_READ;
    const bool writable = c.access & DEPTHCAM_ACCESS_WRITE;
    e->pollable = readable && c.kind != DEPTHCAM_CONTROL_ACTION
                  && (!writable || (d.c.flags & DEPTHCAM_CONTROL_VOLATILE)
                      || !d.autoControl.empty());

    // The backend's default, not the camera's current value: the worker reads
    // those as its first job. Every parameter gets *a* value, without
    // exception: a parameter holding ossia::value{} has no type, and anything
    // that walks the whole tree -- deviceToJson, an OSCquery export, a preset
    // save -- throws "value_to_json_value: no type" on the first one it meets.
    const double v = c.def;
    switch(type_for(c))
    {
      case ossia::val_type::IMPULSE:
        param->set_value_quiet(ossia::value{ossia::impulse{}});
        break;
      case ossia::val_type::BOOL:
        param->set_value_quiet(ossia::value{v != 0.});
        break;
      case ossia::val_type::FLOAT:
        param->set_value_quiet(ossia::value{float(v)});
        break;
      case ossia::val_type::STRING: {
        const auto i = enum_index_of(c, int(v));
        param->set_value_quiet(ossia::value{
            std::string{i >= 0 && c.enum_labels[i] ? c.enum_labels[i] : ""}});
        break;
      }
      default:
        param->set_value_quiet(ossia::value{int(v)});
        break;
    }

    m_descs.push_back(std::move(d));
    e->desc = int(m_descs.size()) - 1;
    m_entries.push_back(std::move(e));
  }
}

void ControlTree::buildRenderControls(ossia::net::node_base& root)
{
  for(const auto& rc : render_controls)
  {
    std::string leaf;
    auto* group = resolve_group(root, rc.id, leaf);
    if(!group)
      continue;
    auto* node = group->create_child(leaf);
    if(!node)
      continue;
    auto* param = node->create_parameter(rc.type);
    if(!param)
      continue;
    param->set_access(ossia::access_mode::BI);
    switch(rc.type)
    {
      case ossia::val_type::FLOAT:
        param->set_domain(ossia::make_domain(rc.min, rc.max));
        param->set_bounding(ossia::bounding_mode::CLIP);
        break;
      case ossia::val_type::VEC3F:
        param->set_domain(ossia::make_domain(rc.min, rc.max));
        break;
      case ossia::val_type::STRING:
        param->set_domain(
            ossia::make_domain(std::vector<std::string>{topology_labels[0], topology_labels[1]}));
        break;
      default:
        break;
    }
    if(std::string_view{rc.id} == "render/near" || std::string_view{rc.id} == "render/far")
      param->set_unit(ossia::parse_pretty_unit("distance.m"));
    ossia::net::set_description(*node, rc.description);
    param->set_value_quiet(renderAsValue(rc, renderValue(*m_params, rc.id)));

    auto e = std::make_unique<Entry>();
    e->id = rc.id;
    e->param = param;
    e->origin = Origin::Render;
    e->access = DEPTHCAM_ACCESS_READ | DEPTHCAM_ACCESS_WRITE;
    m_entries.push_back(std::move(e));
  }
}

void ControlTree::buildActions(ossia::net::node_base& root)
{
  const auto add = [&](const char* id, const char* description) {
    std::string leaf;
    auto* group = resolve_group(root, id, leaf);
    if(!group)
      return;
    auto* node = group->create_child(leaf);
    if(!node)
      return;
    auto* param = node->create_parameter(ossia::val_type::IMPULSE);
    if(!param)
      return;
    param->set_access(ossia::access_mode::SET);
    param->set_value_quiet(ossia::impulse{});
    ossia::net::set_description(*node, description);
    auto e = std::make_unique<Entry>();
    e->id = id;
    e->param = param;
    e->origin = Origin::Action;
    e->kind = DEPTHCAM_CONTROL_ACTION;
    e->access = DEPTHCAM_ACCESS_WRITE;
    m_entries.push_back(std::move(e));
  };
  add("advanced/store_current_settings",
      "Save every current setting with the document, including ones changed in "
      "another program (trigger)");
  add("advanced/reset_to_defaults",
      "Every setting back to the camera's default, and forget the saved ones "
      "(trigger)");
}

int ControlTree::indexOf(const ossia::net::parameter_base& param) noexcept
{
  for(std::size_t i = 0; i < m_entries.size(); i++)
    if(m_entries[i]->param == &param)
      return int(i);
  return -1;
}

int ControlTree::indexOf(std::string_view id) const noexcept
{
  for(std::size_t i = 0; i < m_entries.size(); i++)
    if(m_entries[i]->id == id)
      return int(i);
  return -1;
}

ControlTree::Entry* ControlTree::find(const ossia::net::parameter_base& param) noexcept
{
  const int i = indexOf(param);
  return i >= 0 ? m_entries[std::size_t(i)].get() : nullptr;
}

ossia::net::parameter_base* ControlTree::parameter(std::string_view id) const noexcept
{
  const int i = indexOf(id);
  return i >= 0 ? m_entries[std::size_t(i)]->param : nullptr;
}

void ControlTree::applySaved(const ControlValues& saved)
{
  for(const auto& s : saved)
  {
    if(s.value.empty())
      continue;
    const auto id = s.id.toStdString();
    const int idx = indexOf(id);
    if(idx < 0)
      continue; // a control this camera does not have, or no longer has
    auto& e = *m_entries[std::size_t(idx)];

    {
      std::lock_guard l{m_store_lock};
      std::erase_if(m_store, [&](const auto& c) { return c.id == s.id; });
      m_store.push_back(s);
    }

    switch(e.origin)
    {
      case Origin::Render:
        if(m_params)
        {
          setRender(*m_params, id, s.value);
          if(const auto* rc = renderControl(id))
            e.param->set_value_quiet(renderAsValue(*rc, renderValue(*m_params, id)));
        }
        break;
      case Origin::Backend:
        if((e.access & DEPTHCAM_ACCESS_WRITE) && e.kind != DEPTHCAM_CONTROL_ACTION)
          post({.kind = Request::Write, .entry = idx, .value = s.value[0]});
        break;
      case Origin::Action:
        break;
    }
  }
}

void ControlTree::post(Request r)
{
  {
    std::lock_guard lk{m_queue_lock};
    if(!m_running.load(std::memory_order_acquire))
      return;

    // Collapse repeats. An automation curve writing at audio rate would
    // otherwise pile up thousands of requests the camera can never keep up
    // with, and only the last one was ever going to matter.
    if(r.kind == Request::Read || r.kind == Request::Write)
    {
      for(auto& p : m_pending)
      {
        if(p.entry == r.entry && p.kind == r.kind)
        {
          p.value = r.value;
          return;
        }
      }
    }
    m_pending.push_back(r);
  }
  m_queue_cv.notify_one();
}

void ControlTree::wake()
{
  std::lock_guard lk{m_queue_lock};
  m_queue_cv.notify_one();
}

void ControlTree::waitIdle()
{
  int64_t ticket{};
  {
    std::lock_guard l{m_idle_lock};
    ticket = ++m_idleAsked;
  }
  post({.kind = Request::Wait, .value = double(ticket)});
  std::unique_lock l{m_idle_lock};
  m_idle_cv.wait_for(l, std::chrono::seconds(10), [&] {
    return m_idleDone >= ticket || !m_running.load();
  });
}

void ControlTree::onChangedThunk(const char* id, void* user)
{
  // Any backend thread. Only queue: the worker reads it back.
  auto* self = static_cast<ControlTree*>(user);
  if(!self)
    return;
  if(!id)
  {
    self->refresh();
    return;
  }
  if(const int i = self->indexOf(std::string_view{id}); i >= 0)
    self->post({.kind = Request::Read, .entry = i});
}

void ControlTree::writeOne(
    Entry& e, double value, std::vector<std::pair<int, double>>& results)
{
  if(!m_backend.set_control)
    return;

  // A depth mode or a resolution takes effect on a restart only. Stopping and
  // starting from here keeps the GUI thread out of it, which matters for a
  // camera whose pipeline takes a second to come back.
  const bool restart = (e.flags & DEPTHCAM_CONTROL_REQUIRES_RESTART) && m_stream
                       && m_stream->running();
  if(restart)
    m_stream->stop();
  m_backend.set_control(&m_device, e.id.c_str(), value);
  if(restart)
  {
    m_stream->start();
    if(m_onRestarted)
      ossia::qt::run_async(&m_context, [this] { m_onRestarted(); });
  }

  // Read back what the camera settled on rather than trusting the write:
  // ranges are quantised (an Orbbec white balance snaps to 100K steps) and some
  // settings are refused outright while an `auto` is on.
  double got{};
  if((e.access & DEPTHCAM_ACCESS_READ) && m_backend.get_control
     && m_backend.get_control(&m_device, e.id.c_str(), &got))
  {
    const int idx = indexOf(e.id);
    results.emplace_back(idx, got);
  }
}

void ControlTree::run()
{
  std::vector<Request> batch;
  std::vector<std::pair<int, double>> results;

  while(m_running.load(std::memory_order_acquire))
  {
    {
      std::unique_lock lk{m_queue_lock};
      if(m_pending.empty())
      {
        // Sleep until there is something to do. With nothing observed there is
        // no timeout at all, so an idle camera is not touched.
        if(m_polled.load(std::memory_order_relaxed) > 0)
          m_queue_cv.wait_for(lk, poll_interval);
        else
          m_queue_cv.wait(lk);
      }
      batch.swap(m_pending);
      m_pending.clear();
    }

    if(!m_running.load(std::memory_order_acquire))
      break;

    results.clear();
    std::vector<int64_t> idleTickets;

    for(const auto& r : batch)
    {
      switch(r.kind)
      {
        case Request::Write:
          writeOne(*m_entries[std::size_t(r.entry)], r.value, results);
          break;

        case Request::Read: {
          auto& e = *m_entries[std::size_t(r.entry)];
          double got{};
          if(e.origin == Origin::Backend && (e.access & DEPTHCAM_ACCESS_READ)
             && m_backend.get_control
             && m_backend.get_control(&m_device, e.id.c_str(), &got))
            results.emplace_back(r.entry, got);
          break;
        }

        case Request::Gate:
          try
          {
            m_replayedPromise.set_value();
          }
          catch(...)
          {
          }
          break;

        case Request::StoreAll: {
          // Toggles first: replaying a manual exposure before auto-exposure is
          // off would be refused.
          ControlValues all;
          std::vector<int> order;
          for(std::size_t i = 0; i < m_entries.size(); i++)
            if(!m_entries[i]->ruled.empty())
              order.push_back(int(i));
          for(std::size_t i = 0; i < m_entries.size(); i++)
            if(m_entries[i]->ruled.empty())
              order.push_back(int(i));

          for(int i : order)
          {
            auto& e = *m_entries[std::size_t(i)];
            if(e.origin == Origin::Render && m_params)
            {
              all.push_back({QString::fromStdString(e.id), renderValue(*m_params, e.id)});
              continue;
            }
            if(e.origin != Origin::Backend || e.kind == DEPTHCAM_CONTROL_ACTION
               || !(e.access & DEPTHCAM_ACCESS_READ) || !(e.access & DEPTHCAM_ACCESS_WRITE)
               || !m_backend.get_control)
              continue;
            // A control its toggle has made read-only is the camera's choice,
            // not a setting.
            if(e.autoEntry >= 0)
            {
              double a{};
              if(m_backend.get_control(
                     &m_device, m_entries[std::size_t(e.autoEntry)]->id.c_str(), &a)
                 && a != 0.)
                continue;
            }
            double got{};
            if(m_backend.get_control(&m_device, e.id.c_str(), &got))
            {
              all.push_back({QString::fromStdString(e.id), {got}});
              results.emplace_back(i, got);
            }
          }
          {
            std::lock_guard l{m_store_lock};
            m_store = std::move(all);
          }
          scheduleStore();
          break;
        }

        case Request::Reset: {
          for(std::size_t i = 0; i < m_entries.size(); i++)
          {
            auto& e = *m_entries[i];
            if(e.origin != Origin::Backend || e.kind == DEPTHCAM_CONTROL_ACTION
               || !(e.access & DEPTHCAM_ACCESS_WRITE) || e.desc < 0)
              continue;
            writeOne(e, m_descs[std::size_t(e.desc)].c.base.def, results);
          }
          if(m_params)
          {
            const RenderParams defaults;
            for(const auto& rc : render_controls)
              setRender(*m_params, rc.id, renderValue(defaults, rc.id));
            ossia::qt::run_async(&m_context, [this] {
              for(auto& e : m_entries)
                if(e->origin == Origin::Render)
                  if(const auto* rc = renderControl(e->id))
                    e->param->set_value(renderAsValue(*rc, renderValue(*m_params, e->id)));
            });
          }
          {
            std::lock_guard l{m_store_lock};
            m_store.clear();
          }
          scheduleStore();
          break;
        }

        case Request::Wait:
          idleTickets.push_back(int64_t(r.value));
          break;
      }
    }
    batch.clear();

    // Then the sensors anything is listening to.
    if(m_polled.load(std::memory_order_relaxed) > 0 && m_backend.get_control)
    {
      for(std::size_t i = 0; i < m_entries.size(); i++)
      {
        auto& e = *m_entries[i];
        if(!e.pollable || e.observers.load(std::memory_order_relaxed) <= 0)
          continue;
        // A writable control with an auto toggle only moves while the toggle
        // is on.
        if(e.autoEntry >= 0 && (e.access & DEPTHCAM_ACCESS_WRITE)
           && !(e.flags & DEPTHCAM_CONTROL_VOLATILE) && !e.inactive.load())
          continue;
        double got{};
        if(m_backend.get_control(&m_device, e.id.c_str(), &got))
          results.emplace_back(int(i), got);
      }
    }

    if(!results.empty())
    {
      // Published from the Qt thread: this runs the parameters' callbacks, which
      // reach the explorer, the execution engine and anything listening over
      // OSC.
      ossia::qt::run_async(&m_context, [this, values = results] {
        for(auto& [idx, v] : values)
          if(idx >= 0)
            publish(*m_entries[std::size_t(idx)], v);
      });
    }

    if(!idleTickets.empty())
    {
      std::lock_guard l{m_idle_lock};
      for(auto t : idleTickets)
        m_idleDone = std::max(m_idleDone, t);
      m_idle_cv.notify_all();
    }
  }

  // Wake anyone waiting for a worker that is gone.
  std::lock_guard l{m_idle_lock};
  m_idleDone = m_idleAsked;
  m_idle_cv.notify_all();
}

void ControlTree::record(const std::string& id, std::vector<double> v)
{
  {
    const auto qid = QString::fromStdString(id);
    std::lock_guard l{m_store_lock};
    std::erase_if(m_store, [&](const auto& c) { return c.id == qid; });
    m_store.push_back({qid, std::move(v)});
  }
  scheduleStore();
}

void ControlTree::scheduleStore()
{
  if(!m_onStore)
    return;
  // One hand-over per burst: an automation writing every tick would otherwise
  // copy the whole record into the document's settings every tick.
  if(m_storePending.exchange(true))
    return;
  ossia::qt::run_async(&m_context, [this] {
    m_storePending.store(false);
    m_onStore(stored());
  });
}

ControlTree::ControlValues ControlTree::stored() const
{
  std::lock_guard l{m_store_lock};
  return m_store;
}

bool ControlTree::writeRender(Entry& e, const ossia::value& v)
{
  if(!m_params)
    return false;
  std::vector<double> d;
  if(e.id == "render/stage_box/min" || e.id == "render/stage_box/max")
  {
    const auto x = ossia::convert<ossia::vec3f>(v);
    d = {x[0], x[1], x[2]};
  }
  else if(e.id == "render/stage_box/enabled")
    d = {ossia::convert<bool>(v) ? 1. : 0.};
  else if(e.id == "render/topology")
  {
    int t = 0;
    if(auto s = v.target<std::string>())
      t = *s == topology_labels[1] ? 1 : 0;
    else
      t = std::clamp(ossia::convert<int>(v), 0, 1);
    d = {double(t)};
  }
  else
    d = {double(ossia::convert<float>(v))};

  // Straight into the atomics the renderers read: no camera involved, so no
  // worker either.
  setRender(*m_params, e.id, d);
  record(e.id, std::move(d));
  return true;
}

bool ControlTree::write(const ossia::net::parameter_base& param, const ossia::value& v)
{
  const int idx = indexOf(param);
  if(idx < 0)
    return false;

  auto& e = *m_entries[std::size_t(idx)];
  switch(e.origin)
  {
    case Origin::Render:
      return writeRender(e, v);
    case Origin::Action:
      if(e.id == "advanced/store_current_settings")
        post({.kind = Request::StoreAll});
      else if(e.id == "advanced/reset_to_defaults")
        post({.kind = Request::Reset});
      return true;
    case Origin::Backend:
      break;
  }

  if(!(e.access & DEPTHCAM_ACCESS_WRITE) || !m_backend.set_control)
    return false;

  // Its auto toggle is on: the camera would refuse it, so do not ask, and put
  // back what it really is.
  if(e.inactive.load())
  {
    // The parameter took the value already; forget what was last published so
    // the read puts the real one back even if it did not change.
    e.has_last = false;
    post({.kind = Request::Read, .entry = idx});
    return true;
  }

  const auto& c = m_descs[std::size_t(e.desc)].c.base;

  double raw{};
  switch(e.kind)
  {
    case DEPTHCAM_CONTROL_ACTION:
      // No payload: any write fires it.
      raw = 1.;
      break;
    case DEPTHCAM_CONTROL_BOOL:
      raw = ossia::convert<bool>(v) ? 1. : 0.;
      break;
    case DEPTHCAM_CONTROL_ENUM: {
      if(c.enum_labels && c.enum_count > 0)
      {
        // A label is what the explorer offers, but an index is what OSC and
        // scripts tend to send; accept either.
        if(auto s = v.target<std::string>())
        {
          int found = -1;
          for(int i = 0; i < c.enum_count && found < 0; i++)
            if(c.enum_labels[i] && *s == c.enum_labels[i])
              found = i;
          if(found < 0)
            return false;
          raw = c.min + found * (c.step > 0 ? c.step : 1);
          break;
        }
      }
      raw = double(ossia::convert<int>(v));
      break;
    }
    case DEPTHCAM_CONTROL_FLOAT:
      raw = double(ossia::convert<float>(v));
      break;
    case DEPTHCAM_CONTROL_INT:
    default:
      raw = double(ossia::convert<int>(v));
      break;
  }

  // Queued, not sent. This is called from the GUI thread, from the execution
  // engine and from whichever thread an OSC message arrived on, and a write is
  // a synchronous transfer to the camera.
  post({.kind = Request::Write, .entry = idx, .value = raw});

  // Remembered for the next open. Not an action: replaying a reboot would be a
  // poor way to open a document.
  if(e.kind != DEPTHCAM_CONTROL_ACTION)
    record(e.id, {raw});

  // The tree now shows what was asked for. If the camera clamps or rounds it,
  // the worker's read-back corrects it -- which cannot be done here, because
  // this runs inside the parameter's own callback and callback_container::send
  // holds a non-recursive mutex across it.
  e.last = raw;
  e.has_last = true;
  return true;
}

bool ControlTree::read(ossia::net::parameter_base& param)
{
  const int idx = indexOf(param);
  if(idx < 0)
    return false;

  auto& e = *m_entries[std::size_t(idx)];
  if(e.origin != Origin::Backend)
    return e.origin == Origin::Render;
  if(!(e.access & DEPTHCAM_ACCESS_READ) || !m_backend.get_control)
    return false;

  // The parameter already holds the last known value, so a pull has nothing to
  // wait for; this only asks for a fresher one.
  post({.kind = Request::Read, .entry = idx});
  return true;
}

void ControlTree::setInactive(Entry& e, bool inactive)
{
  if(e.origin != Origin::Backend)
    return;
  e.inactive.store(inactive);
  // V4L2's rule: an inactive control is shown read-only.
  e.param->set_access(inactive ? ossia::access_mode::GET : access_for(e.access));
}

void ControlTree::publish(Entry& e, double v)
{
  if(e.origin != Origin::Backend)
    return;

  // A toggle decides whether what it rules can be written.
  for(int r : e.ruled)
  {
    auto& ruled = *m_entries[std::size_t(r)];
    if(ruled.inactive.load() != (v != 0.))
      setInactive(ruled, v != 0.);
  }

  if(e.has_last && e.last == v)
    return;
  e.last = v;
  e.has_last = true;

  const auto& c = m_descs[std::size_t(e.desc)].c.base;

  // set_value rather than push_value: this is the camera telling us, so the
  // value must reach the listeners without being sent straight back.
  switch(e.kind)
  {
    case DEPTHCAM_CONTROL_BOOL:
      e.param->set_value(ossia::value{v != 0.});
      break;
    case DEPTHCAM_CONTROL_FLOAT:
      e.param->set_value(ossia::value{float(v)});
      break;
    case DEPTHCAM_CONTROL_ENUM:
      if(c.enum_labels && c.enum_count > 0)
      {
        const auto i = enum_index_of(c, int(v));
        if(i >= 0 && c.enum_labels[i])
          e.param->set_value(ossia::value{std::string{c.enum_labels[i]}});
        break;
      }
      [[fallthrough]];
    case DEPTHCAM_CONTROL_ACTION:
      break;
    default:
      e.param->set_value(ossia::value{int(v)});
      break;
  }
}

bool ControlTree::observe(const ossia::net::parameter_base& param, bool enable)
{
  auto* e = find(param);
  if(!e)
    return false;
  if(!e->pollable)
    return true; // ours, but nothing to watch for

  const int before = e->observers.load(std::memory_order_relaxed);
  const int after = std::max(0, before + (enable ? 1 : -1));
  e->observers.store(after, std::memory_order_relaxed);

  if(before == 0 && after > 0)
  {
    if(m_polled.fetch_add(1, std::memory_order_relaxed) == 0)
      wake();
  }
  else if(before > 0 && after == 0)
  {
    m_polled.fetch_sub(1, std::memory_order_relaxed);
  }
  return true;
}

void ControlTree::refresh()
{
  for(std::size_t i = 0; i < m_entries.size(); i++)
  {
    const auto& e = *m_entries[i];
    if(e.origin == Origin::Backend && (e.access & DEPTHCAM_ACCESS_READ)
       && e.kind != DEPTHCAM_CONTROL_ACTION)
      post({.kind = Request::Read, .entry = int(i)});
  }
}

}
