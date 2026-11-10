#pragma once

#include <pcl/point_types.h>
#include <cstdint>
#include <type_traits>

namespace Ddx::point_cloud_io::detail {

template <typename PointT>
struct PointCloudPointTraits {
  static constexpr bool has_xyz = false;
  static constexpr bool has_intensity = false;
  static constexpr bool has_rgb = false;
  static constexpr bool has_normal = false;
};

template <>
struct PointCloudPointTraits<pcl::PointXYZ> {
  static constexpr bool has_xyz = true;
  static constexpr bool has_intensity = false;
  static constexpr bool has_rgb = false;
  static constexpr bool has_normal = false;
};

template <>
struct PointCloudPointTraits<pcl::PointXYZI> {
  static constexpr bool has_xyz = true;
  static constexpr bool has_intensity = true;
  static constexpr bool has_rgb = false;
  static constexpr bool has_normal = false;
};

template <>
struct PointCloudPointTraits<pcl::PointXYZRGB> {
  static constexpr bool has_xyz = true;
  static constexpr bool has_intensity = false;
  static constexpr bool has_rgb = true;
  static constexpr bool has_normal = false;
};

template <>
struct PointCloudPointTraits<pcl::Normal> {
  static constexpr bool has_xyz = false;
  static constexpr bool has_intensity = false;
  static constexpr bool has_rgb = false;
  static constexpr bool has_normal = true;
};

inline void set_xyz(pcl::PointXYZ& pt, float x, float y, float z)
{
  pt.x = x;
  pt.y = y;
  pt.z = z;
}

inline void set_xyz(pcl::PointXYZI& pt, float x, float y, float z)
{
  pt.x = x;
  pt.y = y;
  pt.z = z;
}

inline void set_xyz(pcl::PointXYZRGB& pt, float x, float y, float z)
{
  pt.x = x;
  pt.y = y;
  pt.z = z;
}

inline void set_intensity(pcl::PointXYZI& pt, float intensity) { pt.intensity = intensity; }

inline void set_rgb8(pcl::PointXYZRGB& pt, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  pt.r = r;
  pt.g = g;
  pt.b = b;
}

inline void set_normal(pcl::Normal& pt, float nx, float ny, float nz, float curvature = 0.f)
{
  pt.normal_x = nx;
  pt.normal_y = ny;
  pt.normal_z = nz;
  pt.curvature = curvature;
}

template <typename PointT>
inline void assign_from_las(PointT& pt, float x, float y, float z, float intensity,
                            bool has_color, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if constexpr (PointCloudPointTraits<PointT>::has_xyz) {
    set_xyz(pt, x, y, z);
  }
  if constexpr (PointCloudPointTraits<PointT>::has_intensity) {
    set_intensity(pt, intensity);
  }
  if constexpr (PointCloudPointTraits<PointT>::has_rgb) {
    if (has_color) {
      set_rgb8(pt, r, g, b);
    }
  }
}

}  // namespace Ddx::point_cloud_io::detail
