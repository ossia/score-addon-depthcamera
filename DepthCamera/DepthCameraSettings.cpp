#include "DepthCameraSettings.hpp"

#include <score/serialization/DataStreamVisitor.hpp>
#include <score/serialization/JSONVisitor.hpp>

#include <QIODevice>
#include <QtEndian>

namespace Gfx::DepthCamera
{

QString DepthCameraSettings::backend() const
{
  const auto s = device.trimmed();
  const int colon = s.indexOf(':');
  return colon > 0 ? s.left(colon) : QString{};
}

void DepthCameraSettings::storeControl(const QString& id, std::vector<double> value)
{
  std::erase_if(controls, [&](const ControlValue& c) { return c.id == id; });
  controls.push_back({id, std::move(value)});
}

}

namespace
{
/// Leads a versioned stream. Odd, so it cannot be the byte length QDataStream
/// writes before a QString, which is what an unversioned stream starts with:
/// those are always even, or 0xFFFFFFFF for a null string.
constexpr quint32 settings_stream_marker = 0x44430001u;

/// 1: depthOutput, positions, normals, mesh, controls.
constexpr int32_t settings_stream_version = 1;
}

template <>
void DataStreamReader::read(const Gfx::DepthCamera::DepthCameraSettings& n)
{
  m_stream << settings_stream_marker << settings_stream_version;
  m_stream << n.device << n.rgb << n.ir << n.depth << n.pointcloud
           << n.colorPointcloud << static_cast<int32_t>(n.align) << n.colorWidth
           << n.colorHeight << n.colorFps << n.depthWidth << n.depthHeight
           << n.depthFps << n.imu << n.irWidth << n.irHeight
           << n.irFps;
  m_stream << static_cast<int32_t>(n.depthOutput) << n.positions << n.normals
           << n.mesh << int32_t(n.controls.size());
  for(const auto& c : n.controls)
  {
    m_stream << c.id << int32_t(c.value.size());
    for(double d : c.value)
      m_stream << d;
  }
  insertDelimiter();
}

template <>
void DataStreamWriter::write(Gfx::DepthCamera::DepthCameraSettings& n)
{
  using S = Gfx::DepthCamera::DepthCameraSettings;

  // A stream written before the version existed starts straight with the
  // device string. It is read as version 0, whose output was the preview.
  int32_t version = 0;
  if(auto* dev = m_stream.stream.device())
  {
    const QByteArray head = dev->peek(sizeof(quint32));
    if(head.size() == sizeof(quint32))
    {
      const auto* p = reinterpret_cast<const uchar*>(head.constData());
      const quint32 v = m_stream.stream.byteOrder() == QDataStream::BigEndian
                            ? qFromBigEndian<quint32>(p)
                            : qFromLittleEndian<quint32>(p);
      if(v == settings_stream_marker)
      {
        quint32 marker{};
        m_stream >> marker >> version;
      }
    }
  }

  int32_t align{};
  m_stream >> n.device >> n.rgb >> n.ir >> n.depth >> n.pointcloud
      >> n.colorPointcloud >> align >> n.colorWidth >> n.colorHeight >> n.colorFps
      >> n.depthWidth >> n.depthHeight >> n.depthFps >> n.imu >> n.irWidth >> n.irHeight
      >> n.irFps;
  n.align = static_cast<S::AlignMode>(align);

  if(version >= 1)
  {
    int32_t output{}, count{};
    m_stream >> output >> n.positions >> n.normals >> n.mesh >> count;
    n.depthOutput = static_cast<S::DepthOutput>(output);
    n.controls.clear();
    for(int32_t i = 0; i < count; i++)
    {
      S::ControlValue c;
      int32_t k{};
      m_stream >> c.id >> k;
      for(int32_t j = 0; j < k; j++)
      {
        double d{};
        m_stream >> d;
        c.value.push_back(d);
      }
      n.controls.push_back(std::move(c));
    }
  }
  else
  {
    n.depthOutput = S::DepthOutput::Preview;
    n.positions = n.normals = n.mesh = false;
    n.controls.clear();
  }
  checkDelimiter();
}

template <>
void JSONReader::read(const Gfx::DepthCamera::DepthCameraSettings& n)
{
  obj["Device"] = n.device;
  obj["RGB"] = n.rgb;
  obj["IR"] = n.ir;
  obj["Depth"] = n.depth;
  obj["Pointcloud"] = n.pointcloud;
  obj["ColorPointcloud"] = n.colorPointcloud;
  obj["Align"] = static_cast<int>(n.align);
  obj["ColorWidth"] = n.colorWidth;
  obj["ColorHeight"] = n.colorHeight;
  obj["ColorFps"] = n.colorFps;
  obj["DepthWidth"] = n.depthWidth;
  obj["DepthHeight"] = n.depthHeight;
  obj["DepthFps"] = n.depthFps;
  obj["Imu"] = n.imu;
  obj["IRWidth"] = n.irWidth;
  obj["IRHeight"] = n.irHeight;
  obj["IRFps"] = n.irFps;
  obj["DepthOutput"] = static_cast<int>(n.depthOutput);
  obj["Positions"] = n.positions;
  obj["Normals"] = n.normals;
  obj["Mesh"] = n.mesh;

  // {"color/exposure": [120], "render/stage_min": [-1, -1, 0.5]}: an object
  // keeps the file readable, and rapidjson keeps the members in order.
  stream.Key("Controls");
  stream.StartObject();
  for(const auto& c : n.controls)
  {
    const auto id = c.id.toUtf8();
    stream.Key(id.constData(), id.size());
    stream.StartArray();
    for(double d : c.value)
      stream.Double(d);
    stream.EndArray();
  }
  stream.EndObject();
}

template <>
void JSONWriter::write(Gfx::DepthCamera::DepthCameraSettings& n)
{
  // tryGet, not obj[...]: rapidjson's operator[] *asserts* on a missing key,
  // which aborts the whole process. That is reachable from ordinary use --
  // Score.createDevice() in a JS script passes whatever subset of fields the
  // author wrote, and a document saved by an older build has no Align key at
  // all. Anything absent keeps the default from the struct.
  if(auto v = obj.tryGet("Device"))
    n.device = v->toString();
  if(auto v = obj.tryGet("RGB"))
    n.rgb = v->toBool();
  if(auto v = obj.tryGet("IR"))
    n.ir = v->toBool();
  if(auto v = obj.tryGet("Depth"))
    n.depth = v->toBool();
  if(auto v = obj.tryGet("Pointcloud"))
    n.pointcloud = v->toBool();
  if(auto v = obj.tryGet("ColorPointcloud"))
    n.colorPointcloud = v->toBool();
  if(auto v = obj.tryGet("Align"))
    n.align
        = static_cast<Gfx::DepthCamera::DepthCameraSettings::AlignMode>(v->toInt());
  if(auto v = obj.tryGet("ColorWidth"))
    n.colorWidth = v->toInt();
  if(auto v = obj.tryGet("ColorHeight"))
    n.colorHeight = v->toInt();
  if(auto v = obj.tryGet("ColorFps"))
    n.colorFps = v->toInt();
  if(auto v = obj.tryGet("DepthWidth"))
    n.depthWidth = v->toInt();
  if(auto v = obj.tryGet("DepthHeight"))
    n.depthHeight = v->toInt();
  if(auto v = obj.tryGet("DepthFps"))
    n.depthFps = v->toInt();
  if(auto v = obj.tryGet("Imu"))
    n.imu = v->toBool();
  if(auto v = obj.tryGet("IRWidth"))
    n.irWidth = v->toInt();
  if(auto v = obj.tryGet("IRHeight"))
    n.irHeight = v->toInt();
  if(auto v = obj.tryGet("IRFps"))
    n.irFps = v->toInt();

  // A document from before the setting rendered the preview, and must keep
  // doing so: anything downstream was built for 0-1 grey.
  n.depthOutput = Gfx::DepthCamera::DepthCameraSettings::DepthOutput::Preview;
  if(auto v = obj.tryGet("DepthOutput"))
    n.depthOutput
        = static_cast<Gfx::DepthCamera::DepthCameraSettings::DepthOutput>(v->toInt());
  if(auto v = obj.tryGet("Positions"))
    n.positions = v->toBool();
  if(auto v = obj.tryGet("Normals"))
    n.normals = v->toBool();
  if(auto v = obj.tryGet("Mesh"))
    n.mesh = v->toBool();
  n.controls.clear();
  if(auto v = obj.tryGet("Controls"); v && v->obj.IsObject())
  {
    for(const auto& m : v->obj.GetObject())
    {
      if(!m.name.IsString() || !m.value.IsArray())
        continue;
      Gfx::DepthCamera::DepthCameraSettings::ControlValue c;
      c.id = QString::fromUtf8(m.name.GetString(), m.name.GetStringLength());
      for(const auto& d : m.value.GetArray())
        if(d.IsNumber())
          c.value.push_back(d.GetDouble());
      if(!c.value.empty())
        n.controls.push_back(std::move(c));
    }
  }
}
