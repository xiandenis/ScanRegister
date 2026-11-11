
#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>

namespace Ddx
{
	class OverlapEstimator
	{
	private:
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
		typedef pcl::search::KdTree<pcl::PointXYZ> KdTree;
	public:
		void setSourceCloud(PointCloud::Ptr source_cloud) { m_source_cloud = source_cloud; }
		void setTargetCloud(PointCloud::Ptr target_cloud) { m_target_cloud = target_cloud; }

		bool estimateOverlap(const Eigen::Matrix4d& transformation_matrix, const double distance_threshold, const double leaf_size = 0.2);

		double getOverlap() const { return m_overlap; }
		std::vector<int> getSourceIndices() const { return m_source_indices; }
		std::vector<int> getTargetIndices() const { return m_target_indices; }
	private:

		std::vector<bool> overlapEstimation(
			PointCloud::Ptr cloud, KdTree::Ptr& kdtree,
			const Eigen::Matrix4f& transformation_matrix,
			const double distance_threshold);

		std::vector<int> mergeIndices(
			const std::vector<bool>& source_indices,
			const std::vector<std::vector<uint32_t>>& leaf_sample_indices,
			const int max_indices);
	private:
		PointCloud::Ptr m_source_cloud;
		PointCloud::Ptr m_target_cloud;

		std::vector<int> m_source_indices;
		std::vector<int> m_target_indices;
		double m_overlap;
	};
}

