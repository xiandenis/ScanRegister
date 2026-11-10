#pragma once

#include <registration_type/pcl_type.hpp>

#include <vector>

namespace Ddx
{
	/** 全景角栅格最近点赢家下采样（站心系径向深度）；原 cloud_sample::AngleDownsample。 */
	class AngleDownsample
	{
	public:
		static bool downsampling(pcl_type::PointCloudPtr const& cloud, std::vector<int>& indices,
		                         int width = 5000, int height = 2500);

		static bool downsampling(pcl_type::PointCloudPtr const& cloud,
		                         pcl_type::PointCloudPtr& ds_cloud, int width = 5000,
		                         int height = 2500);
	};

}
