#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <cmath>
#include <logging/logging.hpp>

namespace Ddx::point_cloud_io {

/** Minimum squared length for a world point to be used as FramedCloud.origin. */
inline constexpr double kOriginCandidateMinSquaredNorm = 1e-12;

/**
 * @brief True when (x,y,z) may define a recoverable origin: finite and not near-zero.
 */
inline bool is_valid_origin_candidate(double x, double y, double z)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  return (x * x + y * y + z * z) >= kOriginCandidateMinSquaredNorm;
}

inline bool is_valid_origin_candidate(Eigen::Vector3d const& p)
{
  return is_valid_origin_candidate(p.x(), p.y(), p.z());
}

/**
 * @brief Point cloud in a local float frame plus recoverable world origin.
 *        Invariant: world ≈ local + origin (within float precision).
 *        Does not embed WorkingFrame (job-level shared frame is a separate concern).
 */
template <typename PointT>
struct FramedCloud {
  pcl::PointCloud<PointT> cloud;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
};

/**
 * @brief Treat @p framed.cloud XYZ as world-frame values: lock origin to the first valid
 *        candidate, subtract it in place, and store it in @p framed.origin.
 * @return 0 on success, -1 if no valid origin candidate exists.
 */
template <typename PointT>
int localize_world_cloud(FramedCloud<PointT>& framed)
{
  auto& cloud = framed.cloud;
  if (cloud.empty()) {
    LOG_ERROR("localize_world_cloud: empty cloud");
    return -1;
  }

  bool found = false;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  for (PointT const& pt : cloud.points) {
    const double x = static_cast<double>(pt.x);
    const double y = static_cast<double>(pt.y);
    const double z = static_cast<double>(pt.z);
    if (!is_valid_origin_candidate(x, y, z)) {
      continue;
    }
    origin = Eigen::Vector3d(x, y, z);
    found = true;
    break;
  }
  if (!found) {
    LOG_ERROR("localize_world_cloud: no valid origin candidate");
    cloud.clear();
    framed.origin = Eigen::Vector3d::Zero();
    return -1;
  }

  framed.origin = origin;
  for (PointT& pt : cloud.points) {
    pt.x = static_cast<float>(static_cast<double>(pt.x) - origin.x());
    pt.y = static_cast<float>(static_cast<double>(pt.y) - origin.y());
    pt.z = static_cast<float>(static_cast<double>(pt.z) - origin.z());
  }
  return 0;
}

}  // namespace Ddx::point_cloud_io
