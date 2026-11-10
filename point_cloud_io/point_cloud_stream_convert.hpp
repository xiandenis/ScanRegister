#pragma once

#include "point_cloud_io.hpp"

#include <cstddef>
#include <string>

namespace Ddx::point_cloud_io {

/**
 * @brief Options for streaming PCD / PLY / LAS conversion.
 *
 * Coordinates follow FramedCloud rules: discover a working origin (first valid world
 * point), emit local float points, and when the destination is LAS/SLAS write that
 * origin into the header offset fields.
 *
 * Memory: one chunk is resident (default 1<<20 points, hard-capped at 64MB).
 * Compressed PCD is decompressed to a temporary file, not a full in-memory cloud.
 * .laz is rejected (no LAZ backend; PDAL removed).
 */
struct StreamConvertOptions {
  std::size_t chunk_points = 1u << 20;
};

/**
 * @brief Stream-convert between .pcd / .ply / .las / .slas.
 * @return 0 on success, -1 on failure.
 */
int convert_point_cloud(const std::string& src_path, const std::string& dst_path,
                        const StreamConvertOptions& options = {});

}  // namespace Ddx::point_cloud_io
