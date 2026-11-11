#include "knn_normal_overlap_crop.hpp"

namespace Ddx {
namespace {

using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;
using NormalCloud = pcl::PointCloud<pcl::Normal>;
using KdTree = pcl::search::KdTree<PointType>;

} // namespace

bool cropMutualOverlapByKnnNormal(
	const PointCloud::ConstPtr& src,
	const NormalCloud::ConstPtr& src_normals,
	const KdTree::Ptr& src_tree,
	const PointCloud::ConstPtr& tgt,
	const NormalCloud::ConstPtr& tgt_normals,
	const KdTree::Ptr& tgt_tree,
	const Eigen::Matrix4d& T_src_to_tgt,
	const KnnNormalOverlapCropParams& params,
	std::vector<int>& src_indices_out,
	std::vector<int>& tgt_indices_out)
{
	src_indices_out.clear();
	tgt_indices_out.clear();

	if (!src || !tgt || !src_normals || !tgt_normals || !src_tree || !tgt_tree) {
		return false;
	}
	if (src->size() != src_normals->size() || tgt->size() != tgt_normals->size()) {
		return false;
	}
	if (params.max_search_radius <= 0.f || params.min_points < 1) {
		return false;
	}

	Eigen::Matrix4f const T = T_src_to_tgt.cast<float>();
	Eigen::Matrix4f const T_inv = T.inverse();
	float const max_distance = params.max_search_radius * params.max_search_radius;
	float const dir_thr = params.dir_thr;

	int const s_point_number = static_cast<int>(src->size());
	std::vector<char> source_keep(static_cast<std::size_t>(s_point_number), 0);
	{
		std::vector<int> indices(1);
		std::vector<float> distances(1);
#ifdef _OPENMP
#pragma omp parallel for private(indices, distances)
#endif
		for (int i = 0; i < s_point_number; ++i) {
			Eigen::Vector4f sp = T * src->points[static_cast<std::size_t>(i)].getVector4fMap();
			if (!sp.allFinite()) {
				continue;
			}
			Eigen::Vector4f sp_normal =
				T * src_normals->points[static_cast<std::size_t>(i)].getNormalVector4fMap();
			if (!sp_normal.allFinite()) {
				continue;
			}

			PointType pt;
			pt.x = sp(0);
			pt.y = sp(1);
			pt.z = sp(2);
			if (tgt_tree->nearestKSearch(pt, 1, indices, distances) < 1) {
				continue;
			}
			if (distances[0] > max_distance) {
				continue;
			}

			Eigen::Vector3f const sp_n = sp_normal.head<3>();
			Eigen::Vector3f const tp_n =
				tgt_normals->points[static_cast<std::size_t>(indices[0])].getNormalVector3fMap();
			if (sp_n.dot(tp_n) < dir_thr) {
				continue;
			}
			source_keep[static_cast<std::size_t>(i)] = 1;
		}
	}

	src_indices_out.reserve(static_cast<std::size_t>(s_point_number));
	for (int i = 0; i < s_point_number; ++i) {
		if (source_keep[static_cast<std::size_t>(i)]) {
			src_indices_out.push_back(i);
		}
	}

	int const t_point_number = static_cast<int>(tgt->size());
	std::vector<char> target_keep(static_cast<std::size_t>(t_point_number), 0);
	{
		std::vector<int> indices(1);
		std::vector<float> distances(1);
#ifdef _OPENMP
#pragma omp parallel for private(indices, distances)
#endif
		for (int i = 0; i < t_point_number; ++i) {
			Eigen::Vector4f tp = T_inv * tgt->points[static_cast<std::size_t>(i)].getVector4fMap();
			if (!tp.allFinite()) {
				continue;
			}
			Eigen::Vector4f tp_normal =
				T_inv * tgt_normals->points[static_cast<std::size_t>(i)].getNormalVector4fMap();
			if (!tp_normal.allFinite()) {
				continue;
			}

			PointType pt;
			pt.x = tp(0);
			pt.y = tp(1);
			pt.z = tp(2);
			if (src_tree->nearestKSearch(pt, 1, indices, distances) < 1) {
				continue;
			}
			if (distances[0] > max_distance) {
				continue;
			}

			Eigen::Vector3f const sp_n =
				src_normals->points[static_cast<std::size_t>(indices[0])].getNormalVector3fMap();
			Eigen::Vector3f const tp_n = tp_normal.head<3>();
			if (sp_n.dot(tp_n) < dir_thr) {
				continue;
			}
			target_keep[static_cast<std::size_t>(i)] = 1;
		}
	}

	tgt_indices_out.reserve(static_cast<std::size_t>(t_point_number));
	for (int i = 0; i < t_point_number; ++i) {
		if (target_keep[static_cast<std::size_t>(i)]) {
			tgt_indices_out.push_back(i);
		}
	}

	if (static_cast<int>(src_indices_out.size()) < params.min_points
		|| static_cast<int>(tgt_indices_out.size()) < params.min_points) {
		return false;
	}
	return true;
}

} // namespace Ddx
