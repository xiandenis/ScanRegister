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
int load_slas(const std::string& file_path, FramedCloud<PointT>& framed)
{
  return load_las<PointT>(file_path, framed);
}

inline int read_slas_header_offset(const std::string& file_path, LasHeaderOffset& offset)
{
  return read_las_header_offset(file_path, offset);
}

template <typename PointT>
int load_ply(const std::string& file_path, FramedCloud<PointT>& framed)
{
  framed.origin = Eigen::Vector3d::Zero();
  framed.cloud.clear();
  if (pcl::io::loadPLYFile(file_path, framed.cloud) < 0) {
    return -1;
  }
  return localize_world_cloud(framed);
}

template <typename PointT>
int load_pcd(const std::string& file_path, FramedCloud<PointT>& framed)
{
  framed.origin = Eigen::Vector3d::Zero();
  framed.cloud.clear();
  if (pcl::io::loadPCDFile(file_path, framed.cloud) < 0) {
    return -1;
  }
  return localize_world_cloud(framed);
}

template <typename PointT>
int load(const std::string& file_path, FramedCloud<PointT>& framed)
{
  switch (detect_format(file_path)) {
    case PointCloudFormat::Las:
    case PointCloudFormat::Slas:
      return load_las<PointT>(file_path, framed);
    case PointCloudFormat::Ply:
      return load_ply<PointT>(file_path, framed);
    case PointCloudFormat::Pcd:
      return load_pcd<PointT>(file_path, framed);
    case PointCloudFormat::Pcse:
      return load_pcse<PointT>(file_path, framed);
    default:
      return -1;
  }
}

template <typename PointT>
int save_slas(const std::string& file_path, const FramedCloud<PointT>& framed)
{
  return save_las<PointT>(file_path, framed);
}

template <typename PointT>
int save_pcd(const std::string& file_path, const FramedCloud<PointT>& framed)
{
  if (framed.cloud.empty()) {
    LOG_ERROR("PCD writer refusing empty cloud: {}", file_path);
    return -1;
  }
  if (pcl::io::savePCDFileBinary(file_path, framed.cloud) < 0) {
    LOG_ERROR("PCL failed to write PCD {}", file_path);
    return -1;
  }
  return 0;
}

template <typename PointT>
int save_ply(const std::string& file_path, const FramedCloud<PointT>& framed)
{
  if (framed.cloud.empty()) {
    LOG_ERROR("PLY writer refusing empty cloud: {}", file_path);
    return -1;
  }
  if (pcl::io::savePLYFileBinary(file_path, framed.cloud) < 0) {
    LOG_ERROR("PCL failed to write PLY {}", file_path);
    return -1;
  }
  return 0;
}

template <typename PointT>
int save(const std::string& file_path, const FramedCloud<PointT>& framed)
{
  switch (detect_format(file_path)) {
    case PointCloudFormat::Las:
    case PointCloudFormat::Slas:
      return save_las<PointT>(file_path, framed);
    case PointCloudFormat::Ply:
      return save_ply<PointT>(file_path, framed);
    case PointCloudFormat::Pcd:
      return save_pcd<PointT>(file_path, framed);
    default:
      LOG_ERROR("unsupported point cloud extension for save: {}", file_path);
      return -1;
  }
}

}  // namespace Ddx::point_cloud_io
