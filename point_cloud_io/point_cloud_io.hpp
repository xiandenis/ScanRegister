#pragma once

#include "framed_cloud.hpp"

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

/**
 * @brief X/Y/Z offset fields from a LAS/SLAS file header (LAS quantization anchor).
 *        Diagnostic only — not the same as FramedCloud.origin unless they happen to match.
 */
struct LasHeaderOffset {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/**
 * @brief Read X/Y/Z offset from a LAS or SLAS file header (diagnostic).
 * @return 0 on success, -1 on failure.
 */
int read_las_header_offset(const std::string& file_path, LasHeaderOffset& offset);

/** @brief Same as read_las_header_offset; provided for SLAS naming symmetry. */
int read_slas_header_offset(const std::string& file_path, LasHeaderOffset& offset);

/**
 * @brief Load LAS / SLAS / PLY / PCD / PCSE into a FramedCloud.
 *        World coordinates are recovered first; origin is the first valid world point;
 *        cloud stores float local coordinates (world - origin).
 * @return 0 on success, -1 on failure.
 */
template <typename PointT>
int load(const std::string& file_path, FramedCloud<PointT>& framed);

/**
 * @brief Load PCSE dataset (directory or manifest.json): authoritative points.bin.
 *        Fills fields present on @p PointT (XYZ / intensity / RGB).
 */
template <typename PointT>
int load_pcse(const std::string& file_path, FramedCloud<PointT>& framed);

/**
 * @brief Load uncompressed LAS via las_fast (point formats 0–3, 6–8).
 *        .laz and waveform formats are not supported (PDAL removed; may return later).
 */
template <typename PointT>
int load_las(const std::string& file_path, FramedCloud<PointT>& framed);

/** @brief Load SLAS (same LAS binary layout as load_las / las_fast). */
template <typename PointT>
int load_slas(const std::string& file_path, FramedCloud<PointT>& framed);

template <typename PointT>
int load_ply(const std::string& file_path, FramedCloud<PointT>& framed);

template <typename PointT>
int load_pcd(const std::string& file_path, FramedCloud<PointT>& framed);

/**
 * @brief Save FramedCloud as PCD / PLY / LAS (.las / .slas) by extension.
 *        LAS/SLAS: points written as local coords; header offset = framed.origin.
 *        PCD/PLY: local coords only (origin not embedded; no sidecar this round).
 * @return 0 on success, -1 on failure.
 */
template <typename PointT>
int save(const std::string& file_path, const FramedCloud<PointT>& framed);

template <typename PointT>
int save_pcd(const std::string& file_path, const FramedCloud<PointT>& framed);

template <typename PointT>
int save_ply(const std::string& file_path, const FramedCloud<PointT>& framed);

/**
 * @brief Save uncompressed LAS 1.2 (format 0, or format 2 when PointT has RGB).
 *        Header X/Y/Z offset = framed.origin; points are local.
 * @param scale LAS X/Y/Z scale. Must be finite and > 0. Default 0.001.
 */
template <typename PointT>
int save_las(const std::string& file_path, const FramedCloud<PointT>& framed,
             double scale = 0.001);

template <typename PointT>
int save_slas(const std::string& file_path, const FramedCloud<PointT>& framed);

}  // namespace Ddx::point_cloud_io

#include "point_cloud_io_impl.hpp"
