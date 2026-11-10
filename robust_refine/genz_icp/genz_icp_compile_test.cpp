#include <genz_icp/genz_icp.hpp>
#include <genz_icp/pipeline/GenZICP.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <iostream>

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr MakeSyntheticCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(500);
    for (double x = 0.0; x <= 6.0 + 1e-9; x += 0.3) {
        for (double y = 0.0; y <= 6.0 + 1e-9; y += 0.3) {
            cloud->emplace_back(static_cast<float>(x), static_cast<float>(y), 0.f);
        }
    }
    for (double z = 0.3; z <= 1.5 + 1e-9; z += 0.3) {
        cloud->emplace_back(3.f, 3.f, static_cast<float>(z));
    }
    return cloud;
}

}  // namespace

int main()
{
    // Ensure pipeline object still links (odometry API, unused for station-pair).
    genz_icp::pipeline::GenZICP pipeline;
    (void)pipeline;

    const auto cloud = MakeSyntheticCloud();
    auto cfg = genz_icp::GenZICP::Config::Outdoor();
    // Synthetic grid is sparse; Outdoor voxel is fine for identity self-align.

    genz_icp::GenZICP icp(cfg);
    icp.setSourceCloud(cloud);
    icp.setTargetCloud(cloud);
    if (!icp.align()) {
        std::cerr << "genz_icp_compile_test: GenZICP::align failed\n";
        return 1;
    }

    const Eigen::Matrix4d T = icp.getFinalTransformation();
    if (!T.allFinite()) {
        std::cerr << "genz_icp_compile_test: non-finite pose\n";
        return 1;
    }
    const double tnorm = T.block<3, 1>(0, 3).norm();
    if (tnorm >= 0.05) {
        std::cerr << "genz_icp_compile_test: translation too large: " << tnorm << "\n";
        return 1;
    }

    std::cout << "genz_icp_compile_test: ok\n";
    return 0;
}
