#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace Ddx
{
	class ResolutionEstimate
	{
	public:
		static float estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string const& name = "");
		static float estimate(std::vector<Eigen::Vector3f> const& cloud, std::string const& name = "");
	private:

	};

}