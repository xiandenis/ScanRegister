#pragma once
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <vector>

namespace Ddx
{
	class ImplicitSurfaceReport
	{
	private:
		typedef pcl::PointXYZ PointType;
		typedef pcl::Normal NormalType;
		typedef pcl::PointCloud<PointType> PointCloud;
		typedef pcl::search::KdTree<PointType> KdTree;
		typedef pcl::PointCloud<NormalType> NormalCloud;
	public:

		ImplicitSurfaceReport() 
			: m_search_maximum(20)
			, m_search_radius(0.1)
			, m_gaussian_sigma(0.05f)
			, m_finalRms(-1.0f)
			, m_inlieNumber(0)
			, m_src_points(nullptr)
			, m_tgt_points(nullptr)
			, m_src_normals(nullptr)
			, m_tgt_normals(nullptr)
			, m_tgt_tree(new KdTree())
			, m_transformation(Eigen::Matrix4f::Identity())
		{ }
		
		void setSourceCloud(PointCloud::Ptr src_points) { m_src_points = src_points; }
		void setTargetCloud(PointCloud::Ptr tgt_points) { m_tgt_points = tgt_points; }
		void setSourceNormals(NormalCloud::Ptr src_normals) { m_src_normals = src_normals; }
		void setTargetNormals(NormalCloud::Ptr tgt_normals) { m_tgt_normals = tgt_normals; }
		void setTargetTree(KdTree::Ptr tgt_tree) { m_tgt_tree = tgt_tree; }
		void setTransformation(const Eigen::Matrix4f& transformation) { m_transformation = transformation; }
		void setSearchMaximum(int maximum) { m_search_maximum = maximum; }
		void setSearchRadius(float radius) { m_search_radius = radius; }

		bool compute();
		bool compute(std::vector<int> const& vaild_indices);

		float getFinalRms() const { return m_finalRms; }
		int getInlierNumber() const { return m_inlieNumber; }

		std::vector<Eigen::Vector3f> getPointPairSource(float threshold = 0.03) const;
		std::vector<Eigen::Vector3f> getPointPairTarget(float threshold = 0.03) const;

	private:
		bool checkSource();
		bool checkTarget();
		float _gaussianFunction(const float& distance, const float& theta);
	private:
		PointCloud::Ptr m_src_points;
		NormalCloud::Ptr m_src_normals;
		Eigen::Matrix4f m_transformation;

		PointCloud::Ptr  m_tgt_points;
		NormalCloud::Ptr m_tgt_normals;
		KdTree::Ptr m_tgt_tree;

		// parameters
		int m_search_maximum;
		float m_search_radius;
		float m_gaussian_sigma;

		
		float m_finalRms;
		int m_inlieNumber;
		std::vector<Eigen::Vector3f> m_surface_points;
		std::vector<float> m_surface_points_distance;
	};
}


