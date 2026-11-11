#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>

namespace Ddx {

struct KnnNormalOverlapCropParams {
	float max_search_radius = 0.75f;
	int min_points = 10;
	/// South-homologous: keep when n_s · n_t >= dir_thr (reject when dot < dir_thr).
	float dir_thr = -0.1f;
};

/// Bidirectional 1-NN + normal dot prefilter (south optimizeTransformation geometry/normal gate).
bool cropMutualOverlapByKnnNormal(
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& src,
	const pcl::PointCloud<pcl::Normal>::ConstPtr& src_normals,
	const pcl::search::KdTree<pcl::PointXYZ>::Ptr& src_tree,
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& tgt,
	const pcl::PointCloud<pcl::Normal>::ConstPtr& tgt_normals,
	const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tgt_tree,
	const Eigen::Matrix4d& T_src_to_tgt,
	const KnnNormalOverlapCropParams& params,
	std::vector<int>& src_indices_out,
	std::vector<int>& tgt_indices_out);

} // namespace Ddx
