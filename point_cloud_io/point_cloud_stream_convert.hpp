#pragma once

#include "point_cloud_io.hpp"

#include <cstddef>
#include <string>

namespace Ddx::point_cloud_io {

/**
 * @brief Options for streaming PCD / PLY / LAS conversion.
 *
 * Coordinates follow load(): LAS/SLAS header offset is subtracted by default so values
 * stay in a float-friendly local frame. When the destination is LAS/SLAS and
 * preserve_source_las_offset is true, that same offset is written back into the
 * destination header (points themselves stay de-offset, scale 0.001).
 *
 * Memory: one chunk is resident (default 1<<20 points, hard-capped at 64MB).
 * Compressed PCD is decompressed to a temporary file, not a full in-memory cloud.
 * .laz is rejected so PDAL cannot pull the whole file into RAM.
 */
struct StreamConvertOptions {
  bool subtract_las_header_offset = true;
  bool preserve_source_las_offset = true;
  std::size_t chunk_points = 1u << 20;
};

/**
 * @brief Stream-convert between .pcd / .ply / .las / .slas.
 * @return 0 on success, -1 on failure.
 */
int convert_point_cloud(const std::string& src_path, const std::string& dst_path,
                        const StreamConvertOptions& options = {});

}  // namespace Ddx::point_cloud_io
