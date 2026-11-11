#pragma once
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <vector>

namespace Ddx
{
	class ExplicitSurfaceReport
	{
	private:
		typedef pcl::PointXYZ PointType;
		typedef pcl::Normal NormalType;
		typedef pcl::PointCloud<PointType> PointCloud;
		typedef pcl::search::KdTree<PointType> KdTree;

	public:
		ExplicitSurfaceReport()
			: m_search_neighbors(10)
			, m_search_max_radius(0.2)
			, m_plane_distance(0.1)
			, m_finalRms(-1.0f)
			, m_inlieNumber(0)
			, m_tgt_tree(nullptr)
			, m_src_points(nullptr)
			, m_tgt_points(nullptr)
			, m_transformation(Eigen::Matrix4f::Identity())
		{
		}

		void setSourceCloud(PointCloud::Ptr src_points) { m_src_points = src_points; }
		void setTargetCloud(PointCloud::Ptr tgt_points) { m_tgt_points = tgt_points; }
		void setTargetTree(KdTree::Ptr tgt_tree) { m_tgt_tree = tgt_tree; }
		void setTransformation(const Eigen::Matrix4f& transformation) { m_transformation = transformation; }
		void setSearchNeighbors(int search_neighbors) { m_search_neighbors = search_neighbors; }
		void setSearchMaxRadius(float search_max_radius) { m_search_max_radius = search_max_radius; }
		void setPlaneDistance(float plane_distance) { m_plane_distance = plane_distance; }

		bool compute();
		bool compute(std::vector<int> const& vaild_indices);

		float getFinalRms() const { return m_finalRms; }
		int getInlierNumber() const { return m_inlieNumber; }

		std::vector<Eigen::Vector3f> getPointPairSource(float threshold = 0.03) const;
		std::vector<Eigen::Vector3f> getPointPairTarget(float threshold = 0.03) const;

	private:
		bool checkSource();
		bool checkTarget();
		bool _estiPlane(Eigen::Vector3f& normal,
			Eigen::Vector3f& center,
			PointCloud::Ptr src_points,
			std::vector<int> const& indices,
			const double& threshold = 0.1);
	private:
		PointCloud::Ptr m_src_points;
		Eigen::Matrix4f m_transformation;

		PointCloud::Ptr m_tgt_points;
		KdTree::Ptr m_tgt_tree;


		// params
		int m_search_neighbors;
		float m_search_max_radius;
		float m_plane_distance;


		float m_finalRms;
		int m_inlieNumber;
		std::vector<Eigen::Vector3f> m_surface_points;
		std::vector<float> m_surface_points_distance;
	};
}


