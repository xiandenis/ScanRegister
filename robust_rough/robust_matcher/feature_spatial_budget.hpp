/**
 * Soft feature-count budget via spatial bins + quality scores.
 * Only runs when N > soft_limit; never pads across bins to hit the limit.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace robust_matcher {

struct FeatSpatialBudgetParams {
  int soft_limit         = 80000;
  float bin_voxel_gain   = 6.0f;
};

struct FeatSpatialBudgetResult {
  std::size_t n_before      = 0;
  std::size_t n_after       = 0;
  std::size_t occupied_bins = 0;
  bool applied              = false;
};

namespace detail {

struct VoxelKeyHash {
  std::size_t operator()(const Eigen::Vector3i& v) const noexcept {
    const std::size_t x = static_cast<std::size_t>(static_cast<uint32_t>(v.x()));
    const std::size_t y = static_cast<std::size_t>(static_cast<uint32_t>(v.y()));
    const std::size_t z = static_cast<std::size_t>(static_cast<uint32_t>(v.z()));
    return x * 73856093u ^ y * 19349663u ^ z * 83492791u;
  }
};

}  // namespace detail

/**
 * In-place filter of keypoints / descriptors / scores.
 * No-op when points.size() <= soft_limit (or invalid params / empty).
 */
inline FeatSpatialBudgetResult ApplySpatialFeatureBudget(
    std::vector<Eigen::Vector3f>& points,
    std::vector<Eigen::VectorXf>& descriptors,
    std::vector<float>& scores,
    float voxel_size,
    const FeatSpatialBudgetParams& params) {
  FeatSpatialBudgetResult result;
  result.n_before = points.size();
  result.n_after  = points.size();

  if (points.empty() || params.soft_limit <= 0) {
    return result;
  }
  if (descriptors.size() != points.size() || scores.size() != points.size()) {
    throw std::runtime_error(
        "ApplySpatialFeatureBudget: points/descriptors/scores size mismatch");
  }
  if (static_cast<int>(points.size()) <= params.soft_limit) {
    return result;
  }
  if (!(voxel_size > 0.0f) || !(params.bin_voxel_gain > 0.0f)) {
    return result;
  }

  const float bin_size = params.bin_voxel_gain * voxel_size;
  const float inv_bin  = 1.0f / bin_size;

  std::unordered_map<Eigen::Vector3i, std::vector<std::size_t>, detail::VoxelKeyHash> bins;
  bins.reserve(points.size() / 4 + 16);

  for (std::size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3f& p = points[i];
    Eigen::Vector3i key(static_cast<int>(std::floor(p.x() * inv_bin)),
                        static_cast<int>(std::floor(p.y() * inv_bin)),
                        static_cast<int>(std::floor(p.z() * inv_bin)));
    bins[key].push_back(i);
  }

  result.occupied_bins = bins.size();
  const int k_per_bin =
      std::max(1,
               static_cast<int>(std::ceil(static_cast<double>(params.soft_limit) /
                                          static_cast<double>(std::max<std::size_t>(
                                              1, result.occupied_bins)))));

  std::vector<std::size_t> keep;
  keep.reserve(static_cast<std::size_t>(params.soft_limit));

  for (auto& kv : bins) {
    auto& idxs = kv.second;
    const std::size_t take =
        std::min(idxs.size(), static_cast<std::size_t>(k_per_bin));
    if (take < idxs.size()) {
      std::partial_sort(
          idxs.begin(), idxs.begin() + static_cast<std::ptrdiff_t>(take), idxs.end(),
          [&](std::size_t a, std::size_t b) { return scores[a] > scores[b]; });
      idxs.resize(take);
    } else {
      std::sort(idxs.begin(), idxs.end(),
                [&](std::size_t a, std::size_t b) { return scores[a] > scores[b]; });
    }
    keep.insert(keep.end(), idxs.begin(), idxs.end());
  }

  std::sort(keep.begin(), keep.end());

  std::vector<Eigen::Vector3f> out_pts;
  std::vector<Eigen::VectorXf> out_desc;
  std::vector<float> out_scores;
  out_pts.reserve(keep.size());
  out_desc.reserve(keep.size());
  out_scores.reserve(keep.size());
  for (std::size_t idx : keep) {
    out_pts.push_back(std::move(points[idx]));
    out_desc.push_back(std::move(descriptors[idx]));
    out_scores.push_back(scores[idx]);
  }

  points      = std::move(out_pts);
  descriptors = std::move(out_desc);
  scores      = std::move(out_scores);

  result.n_after = points.size();
  result.applied = true;
  return result;
}

}  // namespace robust_matcher
