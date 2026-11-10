#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

namespace Ddx
{
	class DepthMapTool
	{
	public:
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

		// float to 4uchar  and 4uchar to float , 是把float类型的深度图转成4char的png格式保存，
		static bool convert_float_to_4uchar(const cv::Mat& src, cv::Mat& dst);

		static bool convert_4uchar_to_float(const cv::Mat& src, cv::Mat& dst);

		static bool convert_32FC1_to_16UC1(const cv::Mat& src, cv::Mat& dest, float scale);

		static bool image_float_extend(const cv::Mat& src, cv::Mat& dest, int extend_width);

		static bool image_float_extend_restore(const cv::Mat& src, cv::Mat& dest, int extend_width);

		static bool image_value_segmentd_to_uchar(const cv::Mat& src, cv::Mat& dest, float min_depth, float max_depth);

		static cv::Mat depth_create_tbb(PointCloudXYZ::Ptr cloud, int width, int height);

		static cv::Mat depth_create_tbb(PointCloudXYZ::Ptr cloud, Eigen::Matrix4d const& transforma, int width, int height);

		static cv::Mat depth_create(PointCloudXYZ::Ptr cloud, int width, int height);

		static cv::Mat depth_create(PointCloudXYZ::Ptr cloud, Eigen::Matrix4d const& transforma, int width, int height);
	};
}

