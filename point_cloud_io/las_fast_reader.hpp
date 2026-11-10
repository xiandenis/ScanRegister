#pragma once

#include "point_cloud_io.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <string>

namespace Ddx::point_cloud_io {

/** Parsed LAS public header fields needed for fast decode (uncompressed only). */
struct LasPublicHeader {
  std::uint16_t header_size = 0;
  std::uint32_t offset_to_point_data = 0;
  std::uint8_t point_data_record_format = 0;
  std::uint16_t point_data_record_length = 0;
  std::uint64_t point_count = 0;
  double scale_x = 1.0;
  double scale_y = 1.0;
  double scale_z = 1.0;
  double offset_x = 0.0;
  double offset_y = 0.0;
  double offset_z = 0.0;
};

/**
 * @brief Read LAS public header fields (does not require format 0–3).
 * @return 0 on success, -1 on failure.
 */
int read_las_public_header(const std::string& file_path, LasPublicHeader& header);

/** @brief True when extension is not .laz and format is 0–3 or 6–8 (no waveform). */
bool las_fast_eligible(const std::string& file_path, const LasPublicHeader& header);

/**
 * @brief Fast uncompressed LAS loader (point record formats 0–3, 6–8).
 *        Does not use PDAL. LAZ / waveform formats 4/5/9/10 are rejected.
 *        Peak strategy: allocate PCL cloud first; Windows maps the point block
 *        (no private full-file raw buffer); other platforms / mmap failure use
 *        record-aligned ~256 MiB chunked fread.
 */
int load_las_fast(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>& cloud,
                  bool subtract_header_offset = true);
int load_las_fast(const std::string& file_path, pcl::PointCloud<pcl::PointXYZI>& cloud,
                  bool subtract_header_offset = true);
int load_las_fast(const std::string& file_path, pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                  bool subtract_header_offset = true);

}  // namespace Ddx::point_cloud_io
