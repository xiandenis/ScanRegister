#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace Ddx
{
	namespace pcl_type {
		typedef pcl::PointXYZ PointType;
		typedef pcl::PointCloud<PointType> PointCloud;
		typedef PointCloud::Ptr PointCloudPtr;

		typedef pcl::PointCloud<pcl::Normal> Normal;
		typedef Normal::Ptr NormalPtr;

		typedef pcl::search::KdTree<PointType> KdTree;
		typedef KdTree::Ptr KdTreePtr;
	}
}