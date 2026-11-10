#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

namespace Ddx::point_cloud_io {

enum class PointCloudFormat {
  Unknown,
  Las,
  Slas,
  Ply,
  Pcd,
  Pcse,
};

/** @brief Detect format from extension, or PCSE dataset directory / manifest.json. */
PointCloudFormat detect_format(const std::string& file_path);

/** @brief Return a lowercase extension without the leading dot. */
std::string file_extension(const std::string& file_path);

/** @brief X/Y/Z offset fields from a LAS/SLAS file header (LAS spec, not world coordinates). */
struct LasHeaderOffset {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/**
 * @brief Read X/Y/Z offset from a LAS or SLAS file header (via PDAL LasReader).
 * @return 0 on success, -1 on failure.
 */
int read_las_header_offset(const std::string& file_path, LasHeaderOffset& offset);

/** @brief Same as read_las_header_offset; provided for SLAS naming symmetry. */
int read_slas_header_offset(const std::string& file_path, LasHeaderOffset& offset);

/**
 * @brief Load a point cloud from LAS / SLAS / PLY / PCD / PCSE.
 * @param subtract_las_header_offset When loading LAS/SLAS, subtract header X/Y/Z offset
 *        before storing as float (preserves precision for large coordinates).
 *        Default true. Pass false to keep absolute coordinates. Ignored for PLY/PCD.
 *        PCSE ignores this flag: manifest offset is a quantization anchor; decoded coords
 *        are already in the dataset frame (station-centered PCSE stays station-centered).
 * @return 0 on success, -1 on failure (same convention as pcl::io::load*).
 */
template <typename PointT>
int load(const std::string& file_path, pcl::PointCloud<PointT>& cloud,
         bool subtract_las_header_offset = true);

/**
 * @brief Load PCSE dataset (directory or manifest.json): authoritative points.bin.
 *        Fills fields present on @p PointT (XYZ / intensity / RGB), same pattern as LAS.
 *        Ignores lod/tree and other attributes.
 * @param subtract_offset If true, subtracts manifest quantization offset after decode
 *        (generally wrong for station-centered PCSE). Default false: keep decoded frame.
 */
template <typename PointT>
int load_pcse(const std::string& file_path, pcl::PointCloud<PointT>& cloud,
              bool subtract_offset = false);

/**
 * @brief Load LAS/LAZ. Prefers fast uncompressed reader (formats 0–3); falls back to PDAL
 *        for .laz / unsupported formats / Normal point type.
 */
template <typename PointT>
int load_las(const std::string& file_path, pcl::PointCloud<PointT>& cloud,
             bool subtract_header_offset = true);

/** @brief Force PDAL LasReader path (benchmark / explicit fallback). */
template <typename PointT>
int load_las_pdal(const std::string& file_path, pcl::PointCloud<PointT>& cloud,
                  bool subtract_header_offset = true);

/** @brief Load SLAS (LAS binary layout; forced through PDAL LasReader). */
template <typename PointT>
int load_slas(const std::string& file_path, pcl::PointCloud<PointT>& cloud,
              bool subtract_header_offset = true);

template <typename PointT>
int load_ply(const std::string& file_path, pcl::PointCloud<PointT>& cloud);

template <typename PointT>
int load_pcd(const std::string& file_path, pcl::PointCloud<PointT>& cloud);

/**
 * @brief Save a point cloud as PCD / PLY / LAS (.las / .slas) by extension.
 *        PCD and PLY are binary PCL io. LAS is raw point records (same layout as load_las_fast).
 * @param header_offset LAS/SLAS header X/Y/Z offset only (default 0). Ignored for PCD/PLY.
 *        Points are stored as-is (not shifted by this offset).
 * @return 0 on success, -1 on failure (empty cloud, unknown extension, or writer error).
 */
template <typename PointT>
int save(const std::string& file_path, const pcl::PointCloud<PointT>& cloud,
         const LasHeaderOffset& header_offset = {});

/** @brief Save binary PCD via pcl::io::savePCDFileBinary. */
template <typename PointT>
int save_pcd(const std::string& file_path, const pcl::PointCloud<PointT>& cloud);

/** @brief Save binary PLY via pcl::io::savePLYFileBinary. */
template <typename PointT>
int save_ply(const std::string& file_path, const pcl::PointCloud<PointT>& cloud);

/**
 * @brief Save uncompressed LAS 1.2 (format 0, or format 2 when the point type has RGB).
 *        Same point-record layout as load_las_fast; does not use PDAL.
 *        Default coordinate scale is 0.001. Pass a finer scale when point spacing is at or below that.
 * @param header_offset Written into the LAS header X/Y/Z offset fields (default 0).
 *        Points in @p cloud are stored as-is (not shifted by this offset).
 * @param scale LAS X/Y/Z scale. Must be finite and > 0. Default 0.001.
 * @return 0 on success, -1 on failure.
 */
template <typename PointT>
int save_las(const std::string& file_path, const pcl::PointCloud<PointT>& cloud,
             const LasHeaderOffset& header_offset = {}, double scale = 0.001);

/** @brief Same as save_las; provided for SLAS naming symmetry (.slas extension). */
template <typename PointT>
int save_slas(const std::string& file_path, const pcl::PointCloud<PointT>& cloud,
              const LasHeaderOffset& header_offset = {});

}  // namespace Ddx::point_cloud_io

#include "point_cloud_io_impl.hpp"
