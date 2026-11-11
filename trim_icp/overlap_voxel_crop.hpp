#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace Ddx {

struct OverlapVoxelCropParams {
	float voxel_size = 0.75f;
	/// Chebyshev radius in voxel units (query-time; occupancy is not materialized dilated).
	int dilation = 1;
	int min_points = 10;
	/// Build occupancy from every N-th point (N>=1). Filtering still visits all points.
	int occupancy_stride = 1;
};

/// Mutual overlap crop via hash occupancy + query-time dilation (geometry only; no normals).
/// T_src_to_tgt maps source points into the target frame.
bool cropMutualOverlapByVoxel(
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& src,
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& tgt,
	const Eigen::Matrix4d& T_src_to_tgt,
	const OverlapVoxelCropParams& params,
	std::vector<int>& src_indices_out,
	std::vector<int>& tgt_indices_out);

} // namespace Ddx
