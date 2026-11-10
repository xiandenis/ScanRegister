#include "point_cloud_io.hpp"

#include "las_fast_reader.hpp"
#include "point_cloud_io_impl.hpp"
#include "point_cloud_traits.hpp"

#include <logging/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace Ddx::point_cloud_io {
namespace {

using detail::PointCloudPointTraits;

template <typename PointT>
bool las_point_type_supported()
{
  return PointCloudPointTraits<PointT>::has_xyz;
}

/**
 * LAS load uses only las_fast (uncompressed formats 0–3 / 6–8).
 * PDAL was removed for speed; .laz / waveform formats fail until a dedicated backend returns.
 */
template <typename PointT>
int load_las_dispatch(const std::string& file_path, FramedCloud<PointT>& framed)
{
  if constexpr (!PointCloudPointTraits<PointT>::has_xyz ||
                PointCloudPointTraits<PointT>::has_normal) {
    LOG_ERROR(
        "LAS reader requires a point type with x/y/z and without Normal-only layout "
        "(e.g. PointXYZ, PointXYZI, PointXYZRGB).");
    return -1;
  }

  if (file_extension(file_path) == "laz") {
    LOG_ERROR(
        "LAS .laz is not supported without PDAL (removed). Convert to uncompressed .las "
        "or restore a LAZ backend later: {}",
        file_path);
    return -1;
  }

  LasPublicHeader header;
  if (read_las_public_header(file_path, header) != 0) {
    return -1;
  }
  if (!las_fast_eligible(file_path, header)) {
    LOG_ERROR(
        "las_fast not eligible (point format={}); PDAL fallback removed. "
        "Use uncompressed LAS formats 0–3 or 6–8: {}",
        static_cast<unsigned>(header.point_data_record_format), file_path);
    return -1;
  }
  return load_las_fast(file_path, framed);
}

void append_le(std::vector<char>& buf, const void* data, std::size_t n)
{
  const auto* bytes = static_cast<const char*>(data);
  buf.insert(buf.end(), bytes, bytes + n);
}

template <typename T>
void append_le(std::vector<char>& buf, T value)
{
  append_le(buf, &value, sizeof(T));
}

std::int32_t quantize_las(double value, double scale)
{
  return static_cast<std::int32_t>(std::llround(value / scale));
}

template <typename PointT>
int save_las_impl(const std::string& file_path, const FramedCloud<PointT>& framed, double scale)
{
  if (!las_point_type_supported<PointT>()) {
    LOG_ERROR(
        "LAS writer requires a point type with x/y/z fields "
        "(e.g. PointXYZ, PointXYZI, PointXYZRGB).");
    return -1;
  }
  if (!(scale > 0.0) || !std::isfinite(scale)) {
    LOG_ERROR("LAS writer scale must be finite and > 0: {}", scale);
    return -1;
  }
  const auto& cloud = framed.cloud;
  if (cloud.empty()) {
    LOG_ERROR("LAS writer refusing empty cloud: {}", file_path);
    return -1;
  }
  if (cloud.size() > std::numeric_limits<std::uint32_t>::max()) {
    LOG_ERROR("LAS writer point count exceeds uint32: {}", file_path);
    return -1;
  }

  constexpr bool kRgb = PointCloudPointTraits<PointT>::has_rgb;
  constexpr bool kIntensity = PointCloudPointTraits<PointT>::has_intensity;
  const auto rec_len = static_cast<std::uint16_t>(kRgb ? 26 : 20);
  const auto count = static_cast<std::uint32_t>(cloud.size());
  const double header_offset_x = framed.origin.x();
  const double header_offset_y = framed.origin.y();
  const double header_offset_z = framed.origin.z();

  std::ofstream out(file_path, std::ios::binary);
  if (!out) {
    LOG_ERROR("LAS writer cannot create {}", file_path);
    return -1;
  }

  std::vector<char> header(227, 0);
  std::memcpy(header.data(), "LASF", 4);
  header[24] = 1;
  header[25] = 2;
  const auto header_size = static_cast<std::uint16_t>(227);
  std::memcpy(header.data() + 94, &header_size, 2);
  const auto point_offset = static_cast<std::uint32_t>(227);
  std::memcpy(header.data() + 96, &point_offset, 4);
  header[104] = static_cast<char>(kRgb ? 2 : 0);
  std::memcpy(header.data() + 105, &rec_len, 2);
  std::memcpy(header.data() + 107, &count, 4);
  std::memcpy(header.data() + 111, &count, 4);
  std::memcpy(header.data() + 131, &scale, 8);
  std::memcpy(header.data() + 139, &scale, 8);
  std::memcpy(header.data() + 147, &scale, 8);
  std::memcpy(header.data() + 155, &header_offset_x, 8);
  std::memcpy(header.data() + 163, &header_offset_y, 8);
  std::memcpy(header.data() + 171, &header_offset_z, 8);
  out.write(header.data(), static_cast<std::streamsize>(header.size()));

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double min_z = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();

  std::vector<char> records;
  records.reserve(static_cast<std::size_t>(rec_len) * 4096);
  auto flush_records = [&]() {
    if (!records.empty()) {
      out.write(records.data(), static_cast<std::streamsize>(records.size()));
      records.clear();
    }
  };

  for (const PointT& pt : cloud) {
    const double x = static_cast<double>(pt.x);
    const double y = static_cast<double>(pt.y);
    const double z = static_cast<double>(pt.z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      LOG_ERROR("LAS writer refusing non-finite coordinate: {}", file_path);
      out.close();
      std::error_code ec;
      std::filesystem::remove(file_path, ec);
      return -1;
    }
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    min_z = std::min(min_z, z);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
    max_z = std::max(max_z, z);

    append_le<std::int32_t>(records, quantize_las(x, scale));
    append_le<std::int32_t>(records, quantize_las(y, scale));
    append_le<std::int32_t>(records, quantize_las(z, scale));
    std::uint16_t intensity = 0;
    if constexpr (kIntensity) {
      const float raw = pt.intensity;
      if (raw > 0.f) {
        intensity = raw >= 65535.f ? static_cast<std::uint16_t>(65535)
                                    : static_cast<std::uint16_t>(raw);
      }
    }
    append_le<std::uint16_t>(records, intensity);
    records.insert(records.end(), 4, 0);
    append_le<std::uint16_t>(records, 0);
    if constexpr (kRgb) {
      append_le<std::uint16_t>(records, static_cast<std::uint16_t>(pt.r) << 8);
      append_le<std::uint16_t>(records, static_cast<std::uint16_t>(pt.g) << 8);
      append_le<std::uint16_t>(records, static_cast<std::uint16_t>(pt.b) << 8);
    }
    if (records.size() >= static_cast<std::size_t>(rec_len) * 4096) {
      flush_records();
    }
  }
  flush_records();

  auto patch = [&](int at, double value) {
    out.seekp(at, std::ios::beg);
    out.write(reinterpret_cast<const char*>(&value), static_cast<std::streamsize>(sizeof(value)));
  };
  patch(179, max_x);
  patch(187, min_x);
  patch(195, max_y);
  patch(203, min_y);
  patch(211, max_z);
  patch(219, min_z);
  if (!out) {
    LOG_ERROR("LAS writer failed while writing {}", file_path);
    out.close();
    std::error_code ec;
    std::filesystem::remove(file_path, ec);
    return -1;
  }
  return 0;
}

}  // namespace

int read_las_header_offset(const std::string& file_path, LasHeaderOffset& offset)
{
  const PointCloudFormat fmt = detect_format(file_path);
  if (fmt != PointCloudFormat::Las && fmt != PointCloudFormat::Slas) {
    LOG_ERROR("read_las_header_offset requires a .las or .slas file: {}", file_path);
    return -1;
  }

  LasPublicHeader header;
  if (read_las_public_header(file_path, header) != 0) {
    LOG_ERROR("failed to read LAS/SLAS public header offset: {}", file_path);
    return -1;
  }
  offset.x = header.offset_x;
  offset.y = header.offset_y;
  offset.z = header.offset_z;
  return 0;
}

template <>
int load_las<pcl::PointXYZ>(const std::string& file_path, FramedCloud<pcl::PointXYZ>& framed)
{
  return load_las_dispatch(file_path, framed);
}

template <>
int load_las<pcl::PointXYZI>(const std::string& file_path, FramedCloud<pcl::PointXYZI>& framed)
{
  return load_las_dispatch(file_path, framed);
}

template <>
int load_las<pcl::PointXYZRGB>(const std::string& file_path,
                               FramedCloud<pcl::PointXYZRGB>& framed)
{
  return load_las_dispatch(file_path, framed);
}

template <>
int save_las<pcl::PointXYZ>(const std::string& file_path,
                            const FramedCloud<pcl::PointXYZ>& framed, double scale)
{
  return save_las_impl(file_path, framed, scale);
}

template <>
int save_las<pcl::PointXYZI>(const std::string& file_path,
                             const FramedCloud<pcl::PointXYZI>& framed, double scale)
{
  return save_las_impl(file_path, framed, scale);
}

template <>
int save_las<pcl::PointXYZRGB>(const std::string& file_path,
                               const FramedCloud<pcl::PointXYZRGB>& framed, double scale)
{
  return save_las_impl(file_path, framed, scale);
}

}  // namespace Ddx::point_cloud_io
