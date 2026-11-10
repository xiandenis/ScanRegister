#include "pcse_reader.hpp"

#include "point_cloud_io.hpp"
#include "point_cloud_traits.hpp"

#include <logging/logging.hpp>

#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace Ddx::point_cloud_io {
namespace {

namespace fs = std::filesystem;

constexpr std::size_t kChunkBytesTarget = 256ull * 1024ull * 1024ull;

std::string basename_lower(const std::string& path)
{
  const fs::path p(path);
  std::string name = p.filename().string();
  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return name;
}

bool read_json_file(const fs::path& path, rapidjson::Document& doc)
{
#ifdef _WIN32
  FILE* fp = nullptr;
  if (_wfopen_s(&fp, path.wstring().c_str(), L"rb") != 0 || fp == nullptr) {
    return false;
  }
#else
  FILE* fp = std::fopen(path.string().c_str(), "rb");
  if (fp == nullptr) {
    return false;
  }
#endif
  char buffer[65536];
  rapidjson::FileReadStream is(fp, buffer, sizeof(buffer));
  doc.ParseStream(is);
  std::fclose(fp);
  return !doc.HasParseError() && doc.IsObject();
}

bool read_xyz3(const rapidjson::Value& v, double& x, double& y, double& z)
{
  if (!v.IsArray() || v.Size() < 3) {
    return false;
  }
  if (!v[0].IsNumber() || !v[1].IsNumber() || !v[2].IsNumber()) {
    return false;
  }
  x = v[0].GetDouble();
  y = v[1].GetDouble();
  z = v[2].GetDouble();
  return true;
}

PcseAttrType parse_attr_type(const std::string& type)
{
  if (type == "uint8" || type == "uchar") {
    return PcseAttrType::UInt8;
  }
  if (type == "uint16") {
    return PcseAttrType::UInt16;
  }
  if (type == "int32") {
    return PcseAttrType::Int32;
  }
  if (type == "float" || type == "float32") {
    return PcseAttrType::Float32;
  }
  return PcseAttrType::Unknown;
}

bool parse_manifest_object(const rapidjson::Document& doc, const fs::path& dataset_dir,
                           PcseManifest& out)
{
  if (!doc.HasMember("format") || !doc["format"].IsString() ||
      std::string(doc["format"].GetString()) != "pcse") {
    LOG_ERROR("PCSE manifest format != \"pcse\": {}", dataset_dir.string());
    return false;
  }

  double scale_x = 1.0;
  double scale_y = 1.0;
  double scale_z = 1.0;
  double offset_x = 0.0;
  double offset_y = 0.0;
  double offset_z = 0.0;
  bool have_top_scale = false;
  bool have_top_offset = false;
  if (doc.HasMember("scale") && read_xyz3(doc["scale"], scale_x, scale_y, scale_z)) {
    have_top_scale = true;
  }
  if (doc.HasMember("offset") && read_xyz3(doc["offset"], offset_x, offset_y, offset_z)) {
    have_top_offset = true;
  }

  if (!doc.HasMember("attributes") || !doc["attributes"].IsArray()) {
    LOG_ERROR("PCSE manifest missing attributes: {}", dataset_dir.string());
    return false;
  }

  std::uint32_t attr_sum = 0;
  std::uint32_t position_off = 0;
  bool found_position = false;
  bool has_intensity = false;
  std::uint32_t intensity_off = 0;
  PcseAttrType intensity_type = PcseAttrType::Unknown;
  bool has_rgb = false;
  std::uint32_t rgb_off = 0;
  PcseAttrType rgb_type = PcseAttrType::Unknown;
  double pos_scale_x = 1.0;
  double pos_scale_y = 1.0;
  double pos_scale_z = 1.0;
  double pos_offset_x = 0.0;
  double pos_offset_y = 0.0;
  double pos_offset_z = 0.0;
  bool have_pos_scale = false;
  bool have_pos_offset = false;

  for (auto const& attr : doc["attributes"].GetArray()) {
    if (!attr.IsObject() || !attr.HasMember("name") || !attr["name"].IsString() ||
        !attr.HasMember("size") || !attr["size"].IsUint()) {
      LOG_ERROR("PCSE manifest attributes entry invalid: {}", dataset_dir.string());
      return false;
    }
    const std::string name = attr["name"].GetString();
    const std::uint32_t size = attr["size"].GetUint();
    const std::string type_str =
        (attr.HasMember("type") && attr["type"].IsString()) ? attr["type"].GetString() : "";
    const PcseAttrType atype = parse_attr_type(type_str);
    const std::uint32_t num_elements =
        (attr.HasMember("numElements") && attr["numElements"].IsUint())
            ? attr["numElements"].GetUint()
            : 1u;

    if (name == "position") {
      found_position = true;
      position_off = attr_sum;
      if (atype != PcseAttrType::Int32 && !type_str.empty()) {
        LOG_ERROR("PCSE position type must be int32, got {}: {}", type_str,
                  dataset_dir.string());
        return false;
      }
      if (size < 12) {
        LOG_ERROR("PCSE position size < 12: {}", dataset_dir.string());
        return false;
      }
      if (attr.HasMember("scale") &&
          read_xyz3(attr["scale"], pos_scale_x, pos_scale_y, pos_scale_z)) {
        have_pos_scale = true;
      }
      if (attr.HasMember("offset") &&
          read_xyz3(attr["offset"], pos_offset_x, pos_offset_y, pos_offset_z)) {
        have_pos_offset = true;
      }
    } else if (name == "intensity") {
      // Common: uint16; also accept float on disk.
      if (atype == PcseAttrType::UInt16 || atype == PcseAttrType::Float32 ||
          atype == PcseAttrType::UInt8) {
        has_intensity = true;
        intensity_off = attr_sum;
        intensity_type = atype;
      } else {
        LOG_WARN("PCSE intensity type {} unsupported; skipping intensity", type_str);
      }
    } else if (name == "rgb") {
      // Common: uint16×3; also accept uint8×3.
      if (num_elements >= 3 &&
          (atype == PcseAttrType::UInt16 || atype == PcseAttrType::UInt8) &&
          size >= (atype == PcseAttrType::UInt16 ? 6u : 3u)) {
        has_rgb = true;
        rgb_off = attr_sum;
        rgb_type = atype;
      } else {
        LOG_WARN("PCSE rgb layout unsupported (type={} ne={} size={}); skipping rgb",
                 type_str, num_elements, size);
      }
    }
    attr_sum += size;
  }

  if (!found_position) {
    LOG_ERROR("PCSE manifest has no position attribute: {}", dataset_dir.string());
    return false;
  }

  if (!have_top_scale) {
    if (!have_pos_scale) {
      LOG_ERROR("PCSE manifest missing scale: {}", dataset_dir.string());
      return false;
    }
    scale_x = pos_scale_x;
    scale_y = pos_scale_y;
    scale_z = pos_scale_z;
  }
  if (!have_top_offset) {
    if (!have_pos_offset) {
      LOG_ERROR("PCSE manifest missing offset: {}", dataset_dir.string());
      return false;
    }
    offset_x = pos_offset_x;
    offset_y = pos_offset_y;
    offset_z = pos_offset_z;
  }

  if (!doc.HasMember("types") || !doc["types"].IsObject() || !doc["types"].HasMember("points") ||
      !doc["types"]["points"].IsObject()) {
    LOG_ERROR("PCSE manifest missing types.points: {}", dataset_dir.string());
    return false;
  }
  const rapidjson::Value& points_type = doc["types"]["points"];
  if (!points_type.HasMember("bytesPerPoint") || !points_type["bytesPerPoint"].IsUint()) {
    LOG_ERROR("PCSE types.points.bytesPerPoint missing: {}", dataset_dir.string());
    return false;
  }
  const std::uint32_t bpp = points_type["bytesPerPoint"].GetUint();
  if (bpp == 0 || bpp < position_off + 12) {
    LOG_ERROR("PCSE bytesPerPoint invalid ({}): {}", bpp, dataset_dir.string());
    return false;
  }
  if (bpp != attr_sum) {
    LOG_ERROR("PCSE bytesPerPoint {} != sum(attributes.size) {}: {}", bpp, attr_sum,
              dataset_dir.string());
    return false;
  }
  if (points_type.HasMember("codec") && points_type["codec"].IsString()) {
    const std::string codec = points_type["codec"].GetString();
    if (codec != "UNCOMPRESSED") {
      LOG_ERROR("PCSE codec not UNCOMPRESSED ({}): {}", codec, dataset_dir.string());
      return false;
    }
  }

  std::string data_rel = "points.bin";
  if (points_type.HasMember("data") && points_type["data"].IsString()) {
    data_rel = points_type["data"].GetString();
  }
  const fs::path bin_path = dataset_dir / data_rel;
  if (!fs::is_regular_file(bin_path)) {
    LOG_ERROR("PCSE points.bin missing: {}", bin_path.string());
    return false;
  }

  std::uint64_t total_points = 0;
  if (doc.HasMember("totalPoints") && doc["totalPoints"].IsNumber()) {
    total_points = static_cast<std::uint64_t>(doc["totalPoints"].GetDouble());
  }

  out.scale_x = scale_x;
  out.scale_y = scale_y;
  out.scale_z = scale_z;
  out.offset_x = offset_x;
  out.offset_y = offset_y;
  out.offset_z = offset_z;
  out.bytes_per_point = bpp;
  out.position_byte_offset = position_off;
  out.has_intensity = has_intensity;
  out.intensity_byte_offset = intensity_off;
  out.intensity_type = intensity_type;
  out.has_rgb = has_rgb;
  out.rgb_byte_offset = rgb_off;
  out.rgb_type = rgb_type;
  out.total_points = total_points;
  out.dataset_dir = dataset_dir.string();
  out.points_bin_path = bin_path.string();
  return true;
}

std::int32_t read_i32_le(const std::uint8_t* p)
{
  std::int32_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

std::uint16_t read_u16_le(const std::uint8_t* p)
{
  std::uint16_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

float read_f32_le(const std::uint8_t* p)
{
  float v = 0.f;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

float decode_intensity(const std::uint8_t* rec, const PcseManifest& meta)
{
  if (!meta.has_intensity) {
    return 0.f;
  }
  const std::uint8_t* p = rec + meta.intensity_byte_offset;
  switch (meta.intensity_type) {
    case PcseAttrType::UInt8:
      return static_cast<float>(p[0]);
    case PcseAttrType::UInt16:
      return static_cast<float>(read_u16_le(p));
    case PcseAttrType::Float32:
      return read_f32_le(p);
    default:
      return 0.f;
  }
}

void decode_rgb8(const std::uint8_t* rec, const PcseManifest& meta, std::uint8_t& r,
                 std::uint8_t& g, std::uint8_t& b)
{
  r = g = b = 0;
  if (!meta.has_rgb) {
    return;
  }
  const std::uint8_t* p = rec + meta.rgb_byte_offset;
  if (meta.rgb_type == PcseAttrType::UInt16) {
    // Same convention as LAS: high byte of 16-bit channel.
    r = static_cast<std::uint8_t>(read_u16_le(p + 0) >> 8);
    g = static_cast<std::uint8_t>(read_u16_le(p + 2) >> 8);
    b = static_cast<std::uint8_t>(read_u16_le(p + 4) >> 8);
  } else if (meta.rgb_type == PcseAttrType::UInt8) {
    r = p[0];
    g = p[1];
    b = p[2];
  }
}

void world_xyz_from_pcse_record(const std::uint8_t* rec, const PcseManifest& meta, double& x,
                                double& y, double& z)
{
  const std::uint8_t* pos = rec + meta.position_byte_offset;
  x = static_cast<double>(read_i32_le(pos + 0)) * meta.scale_x + meta.offset_x;
  y = static_cast<double>(read_i32_le(pos + 4)) * meta.scale_y + meta.offset_y;
  z = static_cast<double>(read_i32_le(pos + 8)) * meta.scale_z + meta.offset_z;
}

bool find_origin_in_pcse_records(const std::uint8_t* data, std::size_t n_records,
                                 const PcseManifest& meta, Eigen::Vector3d& origin)
{
  const std::uint32_t bpp = meta.bytes_per_point;
  for (std::size_t i = 0; i < n_records; ++i) {
    double x = 0, y = 0, z = 0;
    world_xyz_from_pcse_record(data + i * bpp, meta, x, y, z);
    if (is_valid_origin_candidate(x, y, z)) {
      origin = Eigen::Vector3d(x, y, z);
      return true;
    }
  }
  return false;
}

template <typename PointT>
void decode_pcse_records_local(const std::uint8_t* data, std::size_t n_records,
                               std::uint64_t base_index, const PcseManifest& meta,
                               Eigen::Vector3d const& origin, pcl::PointCloud<PointT>& cloud)
{
  const std::uint32_t bpp = meta.bytes_per_point;
  constexpr bool want_i = detail::PointCloudPointTraits<PointT>::has_intensity;
  constexpr bool want_rgb = detail::PointCloudPointTraits<PointT>::has_rgb;
  const bool file_has_color = meta.has_rgb;

  for (std::size_t i = 0; i < n_records; ++i) {
    const std::uint8_t* rec = data + i * bpp;
    double x_abs = 0, y_abs = 0, z_abs = 0;
    world_xyz_from_pcse_record(rec, meta, x_abs, y_abs, z_abs);

    float intensity = 0.f;
    if constexpr (want_i) {
      intensity = decode_intensity(rec, meta);
    }
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
    if constexpr (want_rgb) {
      decode_rgb8(rec, meta, r, g, b);
    }

    PointT& pt = cloud.points[static_cast<std::size_t>(base_index + i)];
    detail::assign_from_las(pt, static_cast<float>(x_abs - origin.x()),
                            static_cast<float>(y_abs - origin.y()),
                            static_cast<float>(z_abs - origin.z()), intensity, file_has_color, r,
                            g, b);
  }
}

template <typename PointT>
int load_pcse_impl(const std::string& path, FramedCloud<PointT>& framed)
{
  static_assert(detail::PointCloudPointTraits<PointT>::has_xyz,
                "PCSE loader requires XYZ point type");

  PcseManifest meta;
  if (read_pcse_manifest(path, meta) != 0) {
    return -1;
  }

  std::ifstream in(meta.points_bin_path, std::ios::binary | std::ios::ate);
  if (!in) {
    LOG_ERROR("PCSE cannot open points.bin: {}", meta.points_bin_path);
    return -1;
  }
  const auto file_size = static_cast<std::uint64_t>(in.tellg());
  const std::uint32_t bpp = meta.bytes_per_point;
  if (bpp == 0 || file_size % bpp != 0) {
    LOG_ERROR("PCSE points.bin size {} not divisible by bpp {}: {}", file_size, bpp,
              meta.points_bin_path);
    return -1;
  }
  const std::uint64_t n = file_size / bpp;
  if (meta.total_points != 0 && meta.total_points != n) {
    LOG_ERROR("PCSE totalPoints {} != points.bin count {}: {}", meta.total_points, n,
              meta.points_bin_path);
    return -1;
  }
  if (n == 0) {
    LOG_WARN("PCSE points.bin empty: {}", meta.points_bin_path);
    return -1;
  }

  std::size_t chunk_bytes = kChunkBytesTarget;
  if (chunk_bytes < bpp) {
    chunk_bytes = bpp;
  }
  chunk_bytes = (chunk_bytes / bpp) * bpp;
  if (chunk_bytes == 0) {
    chunk_bytes = bpp;
  }
  if (static_cast<std::uint64_t>(chunk_bytes) > file_size) {
    chunk_bytes = static_cast<std::size_t>(file_size);
  }

  std::vector<std::uint8_t> chunk(chunk_bytes);
  in.seekg(0, std::ios::beg);

  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  bool found_origin = false;
  std::uint64_t points_done = 0;
  while (points_done < n && !found_origin) {
    const std::uint64_t remaining = n - points_done;
    const std::size_t this_records = static_cast<std::size_t>(
        std::min<std::uint64_t>(remaining, chunk_bytes / bpp));
    const std::size_t this_bytes = this_records * bpp;
    in.read(reinterpret_cast<char*>(chunk.data()), static_cast<std::streamsize>(this_bytes));
    if (!in || static_cast<std::size_t>(in.gcount()) != this_bytes) {
      LOG_ERROR("PCSE failed reading points.bin: {}", meta.points_bin_path);
      return -1;
    }
    if (find_origin_in_pcse_records(chunk.data(), this_records, meta, origin)) {
      found_origin = true;
    }
    points_done += this_records;
  }
  if (!found_origin) {
    LOG_ERROR("PCSE: no valid origin candidate: {}", meta.points_bin_path);
    return -1;
  }

  framed.origin = origin;
  framed.cloud.clear();
  framed.cloud.points.resize(static_cast<std::size_t>(n));
  framed.cloud.width = static_cast<std::uint32_t>(n);
  framed.cloud.height = 1;
  framed.cloud.is_dense = true;

  in.clear();
  in.seekg(0, std::ios::beg);
  points_done = 0;
  while (points_done < n) {
    const std::uint64_t remaining = n - points_done;
    const std::size_t this_records = static_cast<std::size_t>(
        std::min<std::uint64_t>(remaining, chunk_bytes / bpp));
    const std::size_t this_bytes = this_records * bpp;
    in.read(reinterpret_cast<char*>(chunk.data()), static_cast<std::streamsize>(this_bytes));
    if (!in || static_cast<std::size_t>(in.gcount()) != this_bytes) {
      LOG_ERROR("PCSE failed reading points.bin: {}", meta.points_bin_path);
      framed.cloud.clear();
      return -1;
    }
    decode_pcse_records_local(chunk.data(), this_records, points_done, meta, origin,
                              framed.cloud);
    points_done += this_records;
  }
  return 0;
}

}  // namespace

std::string resolve_pcse_dataset_dir(const std::string& path)
{
  if (path.empty()) {
    return {};
  }
  std::error_code ec;
  fs::path p(path);
  if (fs::is_directory(p, ec)) {
    const fs::path manifest = p / "manifest.json";
    if (fs::is_regular_file(manifest, ec)) {
      return p.string();
    }
    return {};
  }
  if (fs::is_regular_file(p, ec) && basename_lower(path) == "manifest.json") {
    return p.parent_path().string();
  }
  return {};
}

bool is_pcse_dataset(const std::string& path)
{
  const std::string dir = resolve_pcse_dataset_dir(path);
  if (dir.empty()) {
    return false;
  }
  rapidjson::Document doc;
  if (!read_json_file(fs::path(dir) / "manifest.json", doc)) {
    return false;
  }
  return doc.HasMember("format") && doc["format"].IsString() &&
         std::string(doc["format"].GetString()) == "pcse";
}

int read_pcse_manifest(const std::string& path, PcseManifest& out)
{
  const std::string dir = resolve_pcse_dataset_dir(path);
  if (dir.empty()) {
    LOG_ERROR("PCSE path is not a dataset directory or manifest.json: {}", path);
    return -1;
  }
  rapidjson::Document doc;
  const fs::path manifest_path = fs::path(dir) / "manifest.json";
  if (!read_json_file(manifest_path, doc)) {
    LOG_ERROR("PCSE failed to parse manifest.json: {}", manifest_path.string());
    return -1;
  }
  if (!parse_manifest_object(doc, fs::path(dir), out)) {
    return -1;
  }
  return 0;
}

template <>
int load_pcse<pcl::PointXYZ>(const std::string& path, FramedCloud<pcl::PointXYZ>& framed)
{
  return load_pcse_impl(path, framed);
}

template <>
int load_pcse<pcl::PointXYZI>(const std::string& path, FramedCloud<pcl::PointXYZI>& framed)
{
  return load_pcse_impl(path, framed);
}

template <>
int load_pcse<pcl::PointXYZRGB>(const std::string& path, FramedCloud<pcl::PointXYZRGB>& framed)
{
  return load_pcse_impl(path, framed);
}

}  // namespace Ddx::point_cloud_io
