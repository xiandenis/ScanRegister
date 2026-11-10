#include "point_cloud_io.hpp"

#include "las_fast_reader.hpp"
#include "point_cloud_io_impl.hpp"
#include "point_cloud_traits.hpp"

#include <logging/logging.hpp>

#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/io/LasReader.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace Ddx::point_cloud_io {
namespace {

using detail::PointCloudPointTraits;

template <typename PointT>
bool las_point_type_supported()
{
  return PointCloudPointTraits<PointT>::has_xyz;
}

std::uint8_t las_color_to_u8(std::uint16_t value)
{
  return static_cast<std::uint8_t>(value >> 8);
}

#ifdef _WIN32
/**
 * PDAL (Windows) treats filename as UTF-8 via MultiByteToWideChar(CP_UTF8).
 * Callers in this repo typically pass ACP/GBK paths (MSVC main argv, path.string()).
 * Convert to UTF-8 before handing the path to PDAL; leave the original string unchanged
 * for stlplus/ifstream which still expect ACP.
 */
std::string path_for_pdal(const std::string& file_path)
{
  auto to_wide = [](const std::string& s, UINT cp, DWORD flags) -> std::wstring {
    if (s.empty()) {
      return {};
    }
    int n = MultiByteToWideChar(cp, flags, s.data(), static_cast<int>(s.size()), nullptr, 0);
    if (n <= 0) {
      return {};
    }
    std::wstring w(static_cast<std::size_t>(n), L'\0');
    if (MultiByteToWideChar(cp, flags, s.data(), static_cast<int>(s.size()), w.data(), n) <= 0) {
      return {};
    }
    return w;
  };

  auto exists_w = [](const std::wstring& w) {
    return !w.empty() && GetFileAttributesW(w.c_str()) != INVALID_FILE_ATTRIBUTES;
  };

  std::wstring wide = to_wide(file_path, CP_ACP, 0);
  if (!exists_w(wide)) {
    std::wstring const u8 = to_wide(file_path, CP_UTF8, MB_ERR_INVALID_CHARS);
    if (exists_w(u8)) {
      wide = u8;
    } else if (wide.empty()) {
      wide = u8.empty() ? to_wide(file_path, CP_UTF8, 0) : u8;
    }
  }
  if (wide.empty()) {
    return file_path;
  }

  int utf8_len =
      WideCharToMultiByte(CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()), nullptr, 0,
                          nullptr, nullptr);
  if (utf8_len <= 0) {
    return file_path;
  }
  std::string utf8(static_cast<std::size_t>(utf8_len), '\0');
  if (WideCharToMultiByte(CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()), utf8.data(),
                          utf8_len, nullptr, nullptr) <= 0) {
    return file_path;
  }
  return utf8;
}
#else
std::string path_for_pdal(const std::string& file_path)
{
  return file_path;
}
#endif

bool prepare_las_reader(const std::string& file_path, pdal::LasReader& reader,
                        pdal::PointTable& table)
{
  pdal::Options options;
  options.add("filename", path_for_pdal(file_path));
  reader.setOptions(options);
  reader.prepare(table);
  return true;
}

template <typename PointT>
int load_las_pdal_impl(const std::string& file_path, pcl::PointCloud<PointT>& cloud,
                       bool subtract_header_offset)
{
  if (!las_point_type_supported<PointT>()) {
    LOG_ERROR(
        "LAS reader requires a point type with x/y/z fields "
        "(e.g. PointXYZ, PointXYZI, PointXYZRGB).");
    return -1;
  }

  try {
    pdal::LasReader reader;
    pdal::PointTable table;
    prepare_las_reader(file_path, reader, table);

    const pdal::LasHeader& las_header = reader.header();
    const double header_offset_x = las_header.offsetX();
    const double header_offset_y = las_header.offsetY();
    const double header_offset_z = las_header.offsetZ();

    const pdal::PointViewSet view_set = reader.execute(table);
    if (view_set.empty()) {
      LOG_WARN("LAS/SLAS file contains no point views: {}", file_path);
      return -1;
    }

    const pdal::PointViewPtr view = *view_set.begin();
    if (!view || view->empty()) {
      LOG_WARN("LAS/SLAS file contains no points: {}", file_path);
      return -1;
    }

    const bool file_has_color = view->hasDim(pdal::Dimension::Id::Red) &&
                                view->hasDim(pdal::Dimension::Id::Green) &&
                                view->hasDim(pdal::Dimension::Id::Blue);
    const bool file_has_intensity = view->hasDim(pdal::Dimension::Id::Intensity);

    cloud.clear();
    cloud.points.reserve(view->size());
    cloud.height = 1;
    cloud.is_dense = true;

    for (pdal::PointId i = 0; i < view->size(); ++i) {
      const double x_abs = view->getFieldAs<double>(pdal::Dimension::Id::X, i);
      const double y_abs = view->getFieldAs<double>(pdal::Dimension::Id::Y, i);
      const double z_abs = view->getFieldAs<double>(pdal::Dimension::Id::Z, i);

      const double x_d = x_abs - (subtract_header_offset ? header_offset_x : 0.0);
      const double y_d = y_abs - (subtract_header_offset ? header_offset_y : 0.0);
      const double z_d = z_abs - (subtract_header_offset ? header_offset_z : 0.0);

      float intensity = 0.f;
      if (file_has_intensity) {
        intensity = view->getFieldAs<float>(pdal::Dimension::Id::Intensity, i);
      }

      std::uint8_t r = 0;
      std::uint8_t g = 0;
      std::uint8_t b = 0;
      if (file_has_color && PointCloudPointTraits<PointT>::has_rgb) {
        r = las_color_to_u8(view->getFieldAs<std::uint16_t>(pdal::Dimension::Id::Red, i));
        g = las_color_to_u8(view->getFieldAs<std::uint16_t>(pdal::Dimension::Id::Green, i));
        b = las_color_to_u8(view->getFieldAs<std::uint16_t>(pdal::Dimension::Id::Blue, i));
      }

      PointT pcl_pt{};
      detail::assign_from_las(pcl_pt, static_cast<float>(x_d), static_cast<float>(y_d),
                              static_cast<float>(z_d), intensity, file_has_color, r, g, b);
      cloud.push_back(pcl_pt);
    }

    cloud.width = static_cast<std::uint32_t>(cloud.size());
    return 0;
  } catch (const std::exception& ex) {
    LOG_ERROR("PDAL failed to read LAS/SLAS {}: {}", file_path, ex.what());
    return -1;
  }
}

template <typename PointT>
int load_las_dispatch(const std::string& file_path, pcl::PointCloud<PointT>& cloud,
                      bool subtract_header_offset)
{
  if constexpr (PointCloudPointTraits<PointT>::has_xyz &&
                !PointCloudPointTraits<PointT>::has_normal) {
    LasPublicHeader header;
    if (read_las_public_header(file_path, header) == 0) {
      if (las_fast_eligible(file_path, header)) {
        const int rc = load_las_fast(file_path, cloud, subtract_header_offset);
        if (rc == 0) {
          return 0;
        }
        LOG_WARN("las_fast failed for {}; falling back to PDAL", file_path);
      } else {
        LOG_WARN(
            "las_fast not eligible (format={}) for {}; falling back to PDAL",
            static_cast<unsigned>(header.point_data_record_format), file_path);
      }
    }
  }
  return load_las_pdal_impl(file_path, cloud, subtract_header_offset);
}

}  // namespace

int read_las_header_offset(const std::string& file_path, LasHeaderOffset& offset)
{
  const PointCloudFormat fmt = detect_format(file_path);
  if (fmt != PointCloudFormat::Las && fmt != PointCloudFormat::Slas) {
    LOG_ERROR("read_las_header_offset requires a .las or .slas file: {}", file_path);
    return -1;
  }

  {
    LasPublicHeader header;
    if (read_las_public_header(file_path, header) == 0) {
      offset.x = header.offset_x;
      offset.y = header.offset_y;
      offset.z = header.offset_z;
      return 0;
    }
  }

  try {
    pdal::LasReader reader;
    pdal::PointTable table;
    prepare_las_reader(file_path, reader, table);

    const pdal::LasHeader& header = reader.header();
    offset.x = header.offsetX();
    offset.y = header.offsetY();
    offset.z = header.offsetZ();
    return 0;
  } catch (const std::exception& ex) {
    LOG_ERROR("PDAL failed to read LAS/SLAS header offset {}: {}", file_path, ex.what());
    return -1;
  }
}

template <>
int load_las<pcl::PointXYZ>(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>& cloud,
                            bool subtract_header_offset)
{
  return load_las_dispatch(file_path, cloud, subtract_header_offset);
}

template <>
int load_las<pcl::PointXYZI>(const std::string& file_path, pcl::PointCloud<pcl::PointXYZI>& cloud,
                             bool subtract_header_offset)
{
  return load_las_dispatch(file_path, cloud, subtract_header_offset);
}

template <>
int load_las<pcl::PointXYZRGB>(const std::string& file_path,
                               pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                               bool subtract_header_offset)
{
  return load_las_dispatch(file_path, cloud, subtract_header_offset);
}

template <>
int load_las<pcl::Normal>(const std::string& file_path, pcl::PointCloud<pcl::Normal>& cloud,
                          bool subtract_header_offset)
{
  return load_las_pdal_impl(file_path, cloud, subtract_header_offset);
}

template <>
int load_las_pdal<pcl::PointXYZ>(const std::string& file_path,
                                 pcl::PointCloud<pcl::PointXYZ>& cloud,
                                 bool subtract_header_offset)
{
  return load_las_pdal_impl(file_path, cloud, subtract_header_offset);
}

template <>
int load_las_pdal<pcl::PointXYZI>(const std::string& file_path,
                                  pcl::PointCloud<pcl::PointXYZI>& cloud,
                                  bool subtract_header_offset)
{
  return load_las_pdal_impl(file_path, cloud, subtract_header_offset);
}

template <>
int load_las_pdal<pcl::PointXYZRGB>(const std::string& file_path,
                                    pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                                    bool subtract_header_offset)
{
  return load_las_pdal_impl(file_path, cloud, subtract_header_offset);
}

template <>
int load_las_pdal<pcl::Normal>(const std::string& file_path,
                               pcl::PointCloud<pcl::Normal>& cloud,
                               bool subtract_header_offset)
{
  return load_las_pdal_impl(file_path, cloud, subtract_header_offset);
}

namespace {

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
int save_las_impl(const std::string& file_path, const pcl::PointCloud<PointT>& cloud,
                  const LasHeaderOffset& header_offset, double scale)
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
  std::memcpy(header.data() + 155, &header_offset.x, 8);
  std::memcpy(header.data() + 163, &header_offset.y, 8);
  std::memcpy(header.data() + 171, &header_offset.z, 8);
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

template <>
int save_las<pcl::PointXYZ>(const std::string& file_path,
                            const pcl::PointCloud<pcl::PointXYZ>& cloud,
                            const LasHeaderOffset& header_offset, double scale)
{
  return save_las_impl(file_path, cloud, header_offset, scale);
}

template <>
int save_las<pcl::PointXYZI>(const std::string& file_path,
                             const pcl::PointCloud<pcl::PointXYZI>& cloud,
                             const LasHeaderOffset& header_offset, double scale)
{
  return save_las_impl(file_path, cloud, header_offset, scale);
}

template <>
int save_las<pcl::PointXYZRGB>(const std::string& file_path,
                               const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                               const LasHeaderOffset& header_offset, double scale)
{
  return save_las_impl(file_path, cloud, header_offset, scale);
}

}  // namespace Ddx::point_cloud_io
