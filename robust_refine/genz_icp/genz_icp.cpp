#include "genz_icp/genz_icp.hpp"

#include "genz_icp/core/Preprocessing.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"

#include <sophus/se3.hpp>

#include <cmath>
#include <vector>

namespace genz_icp {
namespace {

std::vector<Eigen::Vector3d> CloudToEigen(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    std::vector<Eigen::Vector3d> out;
    out.reserve(cloud.size());
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        out.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y),
                         static_cast<double>(p.z));
    }
    return out;
}

Sophus::SE3d MatrixToSE3(const Eigen::Matrix4d& M)
{
    return Sophus::SE3d(Sophus::SO3d::fitToSO3(M.block<3, 3>(0, 0)), M.block<3, 1>(0, 3));
}

}  // namespace

GenZICP::Config GenZICP::Config::Indoor()
{
    Config c;
    c.voxel_size = 0.05;
    c.max_distance = 1.0;
    return c;
}

GenZICP::Config GenZICP::Config::Outdoor()
{
    Config c;
    c.voxel_size = 0.25;
    c.max_distance = 1.0;
    return c;
}

bool GenZICP::align(const Eigen::Matrix4d& initial)
{
    result_ = Result{};

    if (!source_ || !target_ || source_->empty() || target_->empty()) {
        return false;
    }

    auto source = CloudToEigen(*source_);
    auto target = CloudToEigen(*target_);
    if (source.empty() || target.empty()) {
        return false;
    }

    if (config_.use_range_filter) {
        source = Preprocess(source, config_.max_range, config_.min_range);
        target = Preprocess(target, config_.max_range, config_.min_range);
        if (source.empty() || target.empty()) {
            return false;
        }
    }

    // No VoxelDownsample: voxel_size only sets the hash-map cell size.
    VoxelHashMap map(config_.voxel_size, /*max_distance=*/100.0, /*map_cleanup_radius=*/400.0,
                     config_.planarity_threshold, config_.max_points_per_voxel);
    map.AddPoints(target);

    Registration registration(config_.max_num_iterations, config_.convergence_criterion);
    registration.SetTerminalStatusEnabled(false);

    const Sophus::SE3d T0 = MatrixToSE3(initial);
    const auto [pose, planar, non_planar] =
        registration.RegisterFrame(source, map, T0, config_.max_distance, config_.kernel);

    if (!pose.matrix().allFinite() || (planar.empty() && non_planar.empty())) {
        return false;
    }

    result_.ok = true;
    result_.T = pose.matrix();
    result_.planar_count = planar.size();
    result_.non_planar_count = non_planar.size();
    return true;
}

}  // namespace genz_icp
