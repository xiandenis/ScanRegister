#include "overlap_estimator.hpp"
#include <octree/fast_octree.h>
#include <octree/fast_octree.hpp>
#include <pcl/common/io.h>
#include <pcl/search/kdtree.h>

namespace Ddx {

	bool
	OverlapEstimator::estimateOverlap(const Eigen::Matrix4d& transformation_matrix,
		const double distance_threshold, const double leaf_size)
	{
		std::vector<std::vector<uint32_t>> source_leaf_indices;
		PointCloud::Ptr source_down_sampled(new PointCloud());
		if (true)
		{
			Ddx::octree::FastOctree<pcl::PointXYZ> fast_octree;
			std::vector<int> indices;
			Ddx::octree::OctreeParams params;
			params.bucketSize = 1;
			params.minExtent = leaf_size / 2.0f;
			fast_octree.SetMatrixData(m_source_cloud, params);
			fast_octree.getMedoidCenters(indices);
			fast_octree.getLeafSampling(source_leaf_indices);
			pcl::copyPointCloud(*m_source_cloud, indices, *source_down_sampled);
		}

		std::vector<std::vector<uint32_t>> target_leaf_indices;
		PointCloud::Ptr target_down_sampled(new PointCloud());
		if (true)
		{
			Ddx::octree::FastOctree<pcl::PointXYZ> fast_octree;
			std::vector<int> indices;
			Ddx::octree::OctreeParams params;
			params.bucketSize = 1;
			params.minExtent = leaf_size / 2.0f;
			fast_octree.SetMatrixData(m_target_cloud, params);
			fast_octree.getMedoidCenters(indices);
			fast_octree.getLeafSampling(target_leaf_indices);
			pcl::copyPointCloud(*m_target_cloud, indices, *target_down_sampled);
		}

		// source overlap estimation
		if (true)
		{
			Eigen::Matrix4f transformation = transformation_matrix.cast<float>();
			double dist_thre = distance_threshold + leaf_size;
			KdTree::Ptr target_kdtree(new KdTree());
			target_kdtree->setInputCloud(target_down_sampled);
			std::vector<bool> result_indices = overlapEstimation(source_down_sampled, target_kdtree, transformation, dist_thre * dist_thre);
			m_source_indices = mergeIndices(result_indices, source_leaf_indices, m_source_cloud->size());
		}


		// target overlap estimation
		if (true)
		{
			Eigen::Matrix4f transformation = transformation_matrix.inverse().cast<float>();

			double dist_thre = distance_threshold + leaf_size;
			KdTree::Ptr source_kdtree(new KdTree());
			source_kdtree->setInputCloud(source_down_sampled);
			std::vector<bool> result_indices = overlapEstimation(target_down_sampled, source_kdtree, transformation, dist_thre * dist_thre);
			m_target_indices = mergeIndices(result_indices, target_leaf_indices, m_target_cloud->size());
		}


		// overlap estimation result
		int total = m_source_indices.size() + m_target_indices.size();
		m_overlap = double(total) / (m_source_cloud->size() + m_target_cloud->size());

		return true;
	}

	std::vector<bool>
	OverlapEstimator::overlapEstimation(
		PointCloud::Ptr cloud, 
		KdTree::Ptr& kdtree, 
		const Eigen::Matrix4f& transformation_matrix, 
		const double distance_threshold)
	{
		const int point_number = cloud->size();
		std::vector<bool> overlap_results(point_number, false);
		std::vector<int> indices(1);
		std::vector<float> distances(1);

#ifdef _OPENMP
#pragma omp parallel for private(indices, distances) shared(overlap_results) 
#endif
		for (int i = 0; i < point_number; ++i)
		{
			Eigen::Vector4f r11x = cloud->points[i].getVector4fMap();
			Eigen::Vector4f point = transformation_matrix * r11x;
			pcl::PointXYZ search_point;
			search_point.x = point[0];
			search_point.y = point[1];
			search_point.z = point[2];
			kdtree->nearestKSearch(search_point, 1, indices, distances);
			if (distances[0] < distance_threshold) {
				overlap_results[i] = true;
			}
		}

		return overlap_results;
	}

	std::vector<int>
	OverlapEstimator::mergeIndices(
			const std::vector<bool>& source_indices,
			const std::vector<std::vector<uint32_t>>& leaf_sample_indices,
			const int max_indices)
	{
		std::vector<int> indices;
		indices.reserve(max_indices);

		for (int i = 0; i < source_indices.size(); ++i) {
			if (source_indices[i] == false) { continue; }
			for (int j = 0; j < leaf_sample_indices[i].size(); ++j) {
				indices.push_back(leaf_sample_indices[i][j]);
			}
		}

		return indices;
	}
}