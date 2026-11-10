#pragma once

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <string>

namespace Ddx
{
	class PanoramaFilter
	{
	public:
		PanoramaFilter()
			: abnormal_rate_(-1.0f)
			, overlap_rate_(-1.0f)
			, tolerance_(0.015f)
			, threshold_(0.06f)
			, verbose_(false)
		{}
		void setVerbose(bool open) { verbose_ = open; }
		bool difference(cv::Mat const& sourceMat, cv::Mat const& targetMat, std::string const& diff_path = "");

		float getAbnormalRate() const { return abnormal_rate_; }
		float getOverlapRate() const { return overlap_rate_; }
		float getDepthValidRate() const { return depth_valid_rate_; }
	private:
		bool checkSize(cv::Mat const& sourceMat, cv::Mat const& targetMat);

		float depth_valid_rate_;  // 深度有效率
		float abnormal_rate_;	// 异常率
		float overlap_rate_;	// 重叠率

		float tolerance_; // 阈值
		float threshold_; // 差值阈值

		bool verbose_;
	};
}
