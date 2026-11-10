#include "depth_map_tool.hpp"
#include <common/panorama_util.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

namespace Ddx
{

namespace {

constexpr float kEmptyDepth = 0.0f;
constexpr float kInitDepth = std::numeric_limits<float>::max();

struct LocalDepthMap
{
	std::vector<float> depth;

	void reset(std::size_t cells)
	{
		depth.assign(cells, kInitDepth);
	}
};

template <typename ProjectFn>
cv::Mat depth_create_tbb_impl(DepthMapTool::PointCloudXYZ::Ptr cloud, int width, int height, ProjectFn&& project)
{
	std::size_t const cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
	int const n = static_cast<int>(cloud->size());
	int const grain = std::max(1, n / (8 * std::max(1, tbb::this_task_arena::max_concurrency())));

	tbb::enumerable_thread_specific<LocalDepthMap> tls;
	tbb::parallel_for(
		tbb::blocked_range<int>(0, n, grain),
		[&](tbb::blocked_range<int> const& range) {
			LocalDepthMap& local = tls.local();
			if (local.depth.size() != cells) {
				local.reset(cells);
			}
			for (int i = range.begin(); i < range.end(); ++i) {
				Eigen::Vector3f const p = project(i);
				float const depth = p.norm();
				if (!(depth > 0.f) || !std::isfinite(depth)) {
					continue;
				}
				Eigen::Vector2i const pixel = PanoramaUtil::math_pixel_from_xyz(p, width, height);
				if (pixel.x() < 0 || pixel.x() >= width || pixel.y() < 0 || pixel.y() >= height) {
					continue;
				}
				float& cell = local.depth[static_cast<std::size_t>(pixel.y() * width + pixel.x())];
				if (depth < cell) {
					cell = depth;
				}
			}
		});

	cv::Mat depth_map(height, width, CV_32FC1);
	float* merged = reinterpret_cast<float*>(depth_map.data);
	std::fill(merged, merged + cells, kInitDepth);
	for (auto it = tls.begin(); it != tls.end(); ++it) {
		for (std::size_t c = 0; c < cells; ++c) {
			if (it->depth[c] < merged[c]) {
				merged[c] = it->depth[c];
			}
		}
	}
	for (std::size_t c = 0; c < cells; ++c) {
		if (merged[c] == kInitDepth) {
			merged[c] = kEmptyDepth;
		}
	}
	return depth_map;
}

} // namespace
	bool 
	DepthMapTool::convert_float_to_4uchar(const cv::Mat& src, cv::Mat& dst)
	{
		if (src.type() != CV_32FC1)
			return false;

		cv::Mat m2(src.size(), CV_8UC4, src.data); //将float型转为8UC4类型
		dst = m2.clone();

		return true;
	}

	bool 
	DepthMapTool::convert_4uchar_to_float(const cv::Mat& src, cv::Mat& dst)
	{
		if (src.type() != CV_8UC4)
			return false;

		cv::Mat DepthImage1(src.size(), CV_32FC1, src.data);   //再将4char图片转为float型

		dst = DepthImage1.clone();
		return true;
	}

	bool 
	DepthMapTool::convert_32FC1_to_16UC1(const cv::Mat& src, cv::Mat& dest, float scale)
	{
		if (src.type() != CV_32FC1 || src.empty())
			return false;

		cv::Mat ret(src.rows, src.cols, CV_16UC1, cv::Scalar(0));
		{
			float max_float = 15000.0f;
			const float* sptr = (const float*)src.data;
			int size = src.rows * src.cols;
			const float* send = sptr + size;
			unsigned short* dptr = (unsigned short*)ret.data;
			while (sptr < send)
			{
				if (*sptr < max_float)
				{
					float temp = scale * (*sptr);
					*dptr = temp >= 65535 ? 65535 : static_cast<unsigned short>(temp);
				}
				dptr++;
				sptr++;
			}
		}
		dest = ret.clone();
		return true;
	}

	bool
	DepthMapTool::image_float_extend(const cv::Mat& src, cv::Mat& dest, int extend_width)
	{
		if (src.type() != CV_32FC1 || src.empty() || extend_width < 0)
			return false;

		int s_width = src.cols;
		int s_height = src.rows;
		// 创建一张扩展图像
		int t_width = s_width + 1 * extend_width;
		int t_height = s_height;
		cv::Mat ret(t_height, t_width, CV_32FC1);
		for (int i = 0; i < t_height; i++)
		{
			for (int j = 0; j < t_width; ++j)
			{
				int x = (j - extend_width + s_width) % s_width;
				ret.at<float>(i, j) = src.at<float>(i, x);
			}
		}
		dest = ret.clone();
		return true;
	}

	bool
	DepthMapTool::image_float_extend_restore(const cv::Mat& src, cv::Mat& dest, int extend_width)
	{
		if (src.type() != CV_32FC1 || src.empty() || extend_width < 0)
			return false;
		if (src.cols <= extend_width)
			return false;

		int s_width = src.cols;
		int s_height = src.rows;
		// 创建一张扩展图像
		int t_width = s_width - 1 * extend_width;
		int t_height = s_height;
		cv::Mat ret(t_height, t_width, CV_32FC1);
		for (int i = 0; i < t_height; i++)
		{
			for (int j = 0; j < t_width; ++j)
			{
				int x = j + extend_width;
				ret.at<float>(i, j) = src.at<float>(i, x);
			}
		}
		dest = ret.clone();
		return true;
	}

	bool 
	DepthMapTool::image_value_segmentd_to_uchar(const cv::Mat& src, cv::Mat& dest, float min_depth, float max_depth)
	{
		if (src.type() != CV_32FC1 || src.empty() || !(max_depth > min_depth))
			return false;

		cv::Mat ret(src.rows, src.cols, CV_8UC1, cv::Scalar(0));
		{
			const float* sptr = (const float*)src.data;
			int size = src.rows * src.cols;
			const float* send = sptr + size;

			unsigned char* dptr = (unsigned char*)ret.data;

			float scaler = 255.0f / (max_depth - min_depth);
			while (sptr < send)
			{
				if (*sptr > min_depth && *sptr < max_depth)
					*dptr = static_cast<uchar>(scaler * (*sptr - min_depth));
				dptr++;
				sptr++;
			}
		}
		dest = ret.clone();
		return true;
	}

	cv::Mat
	DepthMapTool::depth_create_tbb(PointCloudXYZ::Ptr cloud, int width, int height)
	{
		if (!cloud || cloud->empty() || width <= 0 || height <= 0)
			return cv::Mat();

		return depth_create_tbb_impl(cloud, width, height, [&](int i) {
			return cloud->points[i].getVector3fMap();
		});
	}

	cv::Mat 
	DepthMapTool::depth_create_tbb(PointCloudXYZ::Ptr cloud, Eigen::Matrix4d const& transforma, int width, int height)
	{
		if (!cloud || cloud->empty() || width <= 0 || height <= 0)
			return cv::Mat();

		Eigen::Matrix3f const R = transforma.block<3, 3>(0, 0).cast<float>();
		Eigen::Vector3f const t = transforma.block<3, 1>(0, 3).cast<float>();
		return depth_create_tbb_impl(cloud, width, height, [&](int i) {
			return Eigen::Vector3f(R * cloud->points[i].getVector3fMap() + t);
		});
	}

	cv::Mat
	DepthMapTool::depth_create(PointCloudXYZ::Ptr cloud, int width, int height)
	{
		if (!cloud || cloud->empty() || width <= 0 || height <= 0)
			return cv::Mat();

		cv::Mat depth_map = cv::Mat::zeros(height, width, CV_32FC1);

		for (int i = 0; i < static_cast<int>(cloud->size()); ++i)
		{
			const Eigen::Vector3f r11x = cloud->points[i].getVector3fMap();
			float depth = r11x.norm();

			Eigen::Vector2i pixel = Ddx::PanoramaUtil::math_pixel_from_xyz(r11x, width, height);
			if (pixel.x() < 0 || pixel.x() >= width || pixel.y() < 0 || pixel.y() >= height)
				continue;

			float& depth_value = depth_map.at<float>(pixel.y(), pixel.x());
			if (depth_value == 0 || depth < depth_value)
			{
				depth_value = depth;
			}
		}

		return depth_map;
	}

	cv::Mat 
	DepthMapTool::depth_create(PointCloudXYZ::Ptr cloud, Eigen::Matrix4d const& transforma, int width, int height)
	{
		if (!cloud || cloud->empty() || width <= 0 || height <= 0)
			return cv::Mat();

		cv::Mat depth_map = cv::Mat::zeros(height, width, CV_32FC1);
		Eigen::Matrix3f R = transforma.block<3, 3>(0, 0).cast<float>();
		Eigen::Vector3f t = transforma.block<3, 1>(0, 3).cast<float>();

		for (int i = 0; i < static_cast<int>(cloud->size()); ++i)
		{
			Eigen::Vector3f r11x = R * cloud->points[i].getVector3fMap() + t;
			float depth = r11x.norm();

			Eigen::Vector2i pixel = Ddx::PanoramaUtil::math_pixel_from_xyz(r11x, width, height);
			if (pixel.x() < 0 || pixel.x() >= width || pixel.y() < 0 || pixel.y() >= height)
				continue;

			float& depth_value = depth_map.at<float>(pixel.y(), pixel.x());
			if (depth_value == 0 || depth < depth_value)
			{
				depth_value = depth;
			}
		}

		return depth_map;
	}

}
