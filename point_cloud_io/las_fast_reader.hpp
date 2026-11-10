#pragma once

#include "point_cloud_io.hpp"

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
 * @brief Fast uncompressed LAS loader into FramedCloud (formats 0–3, 6–8).
 *        World = scale*i + header_offset; origin = first valid world point;
 *        cloud stores float(world - origin).
 */
int load_las_fast(const std::string& file_path, FramedCloud<pcl::PointXYZ>& framed);
int load_las_fast(const std::string& file_path, FramedCloud<pcl::PointXYZI>& framed);
int load_las_fast(const std::string& file_path, FramedCloud<pcl::PointXYZRGB>& framed);

}  // namespace Ddx::point_cloud_io
