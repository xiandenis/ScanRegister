#pragma once

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <vector>

namespace Ddx
{
	class RegistrationReport
	{
	private:
		typedef pcl::PointXYZ PointType;
		typedef pcl::Normal NormalType;
		typedef pcl::PointCloud<PointType> PointCloud;
		typedef pcl::search::KdTree<PointType> kdTree;
		typedef pcl::PointCloud<NormalType> NormalCloud;
	public:
		RegistrationReport() 
			: m_max_search_radius(0.04)
			, m_finalRms(-1.f)
			, m_inlieNumber(0)
			, m_src_points(nullptr)
			, m_tgt_points(nullptr)
			, m_tgt_tree(nullptr)
			, m_transformation(Eigen::Matrix4f::Identity())
		{}


		void setSourceCloud(PointCloud::Ptr src) { m_src_points = src; }
		void setTargetCloud(PointCloud::Ptr tgt) { m_tgt_points = tgt; }
		void setTargetTree(kdTree::Ptr tgt_tree) { m_tgt_tree = tgt_tree; }
		void setTransformationMatrix(const Eigen::Matrix4f& matrix) { m_transformation = matrix; }
		void setMaxSearchRadius(float max_radius) { m_max_search_radius = max_radius * max_radius; }


		bool compute();
		bool compute(std::vector<int> const& vaild_indices);

		float getFinalRms() const { return m_finalRms; }
		int getInlierNumber() const { return m_inlieNumber; }

		std::vector<Eigen::Vector3f> getPointPairSource(float threshold = 0.03) const;
		std::vector<Eigen::Vector3f> getPointPairTarget(float threshold = 0.03) const;

	private:
		bool checkSoucre();

		bool checkTarget();

	private:
		PointCloud::Ptr m_src_points;
		Eigen::Matrix4f m_transformation;

		PointCloud::Ptr m_tgt_points;
		kdTree::Ptr m_tgt_tree;

		float m_max_search_radius;

		float m_finalRms;
		int m_inlieNumber;
		std::vector<int> m_tgt_indices;
		std::vector<float> m_distances;

	};

}

