#pragma once
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace teaser
{
	class TeaserRespondence
	{
	public:

		typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EPointCloud;
		typedef pcl::PointCloud<pcl::PointXYZ>::Ptr  PointCloudPtr;
		typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr  FPFHDescriptorPtr;

		std::vector<std::pair<int, int>>
			calculateCorrespondences(const EPointCloud& source_points, const EPointCloud& target_points,
				FPFHDescriptorPtr& source_features, FPFHDescriptorPtr& target_features);

		std::vector<std::pair<int, int>>
			calculateCorrespondences(const PointCloudPtr& source_points, const PointCloudPtr& target_points,
				FPFHDescriptorPtr& source_features, FPFHDescriptorPtr& target_features);

	};

}
