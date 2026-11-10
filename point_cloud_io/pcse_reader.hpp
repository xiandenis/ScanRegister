#pragma once

#include <cstdint>
#include <string>

namespace Ddx::point_cloud_io {

/** Attribute element type on disk (authoritative points schema). */
enum class PcseAttrType : std::uint8_t {
  Unknown = 0,
  UInt8,
  UInt16,
  Int32,
  Float32,
};

/** Parsed PCSE manifest fields for decoding points.bin (XYZ + optional intensity/RGB). */
struct PcseManifest {
  double scale_x = 1.0;
  double scale_y = 1.0;
  double scale_z = 1.0;
  double offset_x = 0.0;
  double offset_y = 0.0;
  double offset_z = 0.0;
  std::uint32_t bytes_per_point = 0;
  std::uint32_t position_byte_offset = 0;
  bool has_intensity = false;
  std::uint32_t intensity_byte_offset = 0;
  PcseAttrType intensity_type = PcseAttrType::Unknown;
  bool has_rgb = false;
  std::uint32_t rgb_byte_offset = 0;
  PcseAttrType rgb_type = PcseAttrType::Unknown;  // per-channel type
  std::uint64_t total_points = 0;                 // from manifest when present; else 0
  std::string dataset_dir;
  std::string points_bin_path;
};

/**
 * @brief Resolve a dataset directory from a PCSE folder or a path to its manifest.json.
 * @return Empty string on failure.
 */
std::string resolve_pcse_dataset_dir(const std::string& path);

/** @brief True when @p path is a PCSE dataset directory or its manifest.json. */
bool is_pcse_dataset(const std::string& path);

/**
 * @brief Parse manifest.json and locate points.bin under the dataset.
 * @return 0 on success, -1 on failure.
 */
int read_pcse_manifest(const std::string& path, PcseManifest& out);

}  // namespace Ddx::point_cloud_io
