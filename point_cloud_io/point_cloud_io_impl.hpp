#pragma once

#include "point_cloud_io.hpp"
#include "pcse_reader.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <logging/logging.hpp>

#include <algorithm>
#include <cctype>

namespace Ddx::point_cloud_io {

inline std::string file_extension(const std::string& file_path)
{
  const auto dot = file_path.find_last_of(".\\/");
  if (dot == std::string::npos || file_path[dot] != '.') {
    return {};
  }
  std::string ext = file_path.substr(dot + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return ext;
}

inline PointCloudFormat detect_format(const std::string& file_path)
{
  if (is_pcse_dataset(file_path)) {
    return PointCloudFormat::Pcse;
  }
  const std::string ext = file_extension(file_path);
  if (ext == "las" || ext == "laz") {
    return PointCloudFormat::Las;
  }
  if (ext == "slas") {
    return PointCloudFormat::Slas;
  }
  if (ext == "ply") {
    return PointCloudFormat::Ply;
  }
  if (ext == "pcd") {
    return PointCloudFormat::Pcd;
  }
  return PointCloudFormat::Unknown;
}

template <typename PointT>
int load_slas(const std::string& file_path, pcl::PointCloud<PointT>& cloud,
              bool subtract_header_offset)
{
  return load_las<PointT>(file_path, cloud, subtract_header_offset);
}

inline int read_slas_header_offset(const std::string& file_path, LasHeaderOffset& offset)
{
  return read_las_header_offset(file_path, offset);
}

template <typename PointT>
int load_ply(const std::string& file_path, pcl::PointCloud<PointT>& cloud)
{
  return pcl::io::loadPLYFile(file_path, cloud);
}

template <typename PointT>
int load_pcd(const std::string& file_path, pcl::PointCloud<PointT>& cloud)
{
  return pcl::io::loadPCDFile(file_path, cloud);
}

template <typename PointT>
int load(const std::string& file_path, pcl::PointCloud<PointT>& cloud,
         bool subtract_las_header_offset)
{
  switch (detect_format(file_path)) {
    case PointCloudFormat::Las:
    case PointCloudFormat::Slas:
      return load_las<PointT>(file_path, cloud, subtract_las_header_offset);
    case PointCloudFormat::Ply:
      return load_ply<PointT>(file_path, cloud);
    case PointCloudFormat::Pcd:
      return load_pcd<PointT>(file_path, cloud);
    case PointCloudFormat::Pcse:
      // PCSE manifest offset 是 int32 量化锚点（常≈bbox.min），不是 LAS 式「大坐标系头偏移」。
      // 解码 x=i*scale+offset 已是数据集坐标系（站心 PCSE 即站心）；再减 offset 会把原点挪到包围盒角。
      (void)subtract_las_header_offset;
      return load_pcse<PointT>(file_path, cloud, /*subtract_offset=*/false);
    default:
      return -1;
  }
}

template <typename PointT>
int save_slas(const std::string& file_path, const pcl::PointCloud<PointT>& cloud,
              const LasHeaderOffset& header_offset)
{
  return save_las<PointT>(file_path, cloud, header_offset);
}

template <typename PointT>
int save_pcd(const std::string& file_path, const pcl::PointCloud<PointT>& cloud)
{
  if (cloud.empty()) {
    LOG_ERROR("PCD writer refusing empty cloud: {}", file_path);
    return -1;
  }
  if (pcl::io::savePCDFileBinary(file_path, cloud) < 0) {
    LOG_ERROR("PCL failed to write PCD {}", file_path);
    return -1;
  }
  return 0;
}

template <typename PointT>
int save_ply(const std::string& file_path, const pcl::PointCloud<PointT>& cloud)
{
  if (cloud.empty()) {
    LOG_ERROR("PLY writer refusing empty cloud: {}", file_path);
    return -1;
  }
  if (pcl::io::savePLYFileBinary(file_path, cloud) < 0) {
    LOG_ERROR("PCL failed to write PLY {}", file_path);
    return -1;
  }
  return 0;
}

template <typename PointT>
int save(const std::string& file_path, const pcl::PointCloud<PointT>& cloud,
         const LasHeaderOffset& header_offset)
{
  switch (detect_format(file_path)) {
    case PointCloudFormat::Las:
    case PointCloudFormat::Slas:
      return save_las<PointT>(file_path, cloud, header_offset);
    case PointCloudFormat::Ply:
      return save_ply<PointT>(file_path, cloud);
    case PointCloudFormat::Pcd:
      return save_pcd<PointT>(file_path, cloud);
    default:
      LOG_ERROR("unsupported point cloud extension for save: {}", file_path);
      return -1;
  }
}

}  // namespace Ddx::point_cloud_io
