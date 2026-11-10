#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>

namespace genz_icp {

/// Station-pair registration (TrimICP-style). Target builds VoxelHashMap; source
/// is registered with an external initial guess. Distinct from
/// `pipeline::GenZICP` (sequential odometry).
class GenZICP {
public:
    struct Config {
        double voxel_size = 0.05;  ///< Hash-map cell size only (no input downsample).
        double max_distance = 1.0;
        double kernel = 2.0 / 3.0;
        int max_num_iterations = 150;
        double convergence_criterion = 1e-4;
        double planarity_threshold = 0.1;
        int max_points_per_voxel = 1;
        bool use_range_filter = false;
        double min_range = 0.5;
        double max_range = 100.0;

        static Config Indoor();   ///< voxel 0.05, max_distance 1.0
        static Config Outdoor();  ///< voxel 0.25, max_distance 1.0
    };

    struct Result {
        bool ok = false;
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();  ///< p_tgt = T * p_src
        std::size_t planar_count = 0;
        std::size_t non_planar_count = 0;
    };

    GenZICP() = default;
    explicit GenZICP(const Config& config) : config_(config) {}

    void setConfig(const Config& config) { config_ = config; }
    const Config& config() const { return config_; }

    void setSourceCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source) { source_ = source; }
    void setTargetCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target) { target_ = target; }

    /// Align source to target. Returns true iff lastResult().ok.
    bool align(const Eigen::Matrix4d& initial = Eigen::Matrix4d::Identity());

    Eigen::Matrix4d getFinalTransformation() const { return result_.T; }
    const Result& lastResult() const { return result_; }

private:
    Config config_{};
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr source_{};
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_{};
    Result result_{};
};

}  // namespace genz_icp
