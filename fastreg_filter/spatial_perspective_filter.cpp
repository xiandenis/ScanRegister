#include "spatial_perspective_filter.hpp"
#include <logging/logging.hpp>

#define MATH_PI  3.14159265358979323846264338327950288   // pi

namespace Ddx
{
	bool
	SpatialPerspectiveFilter::difference(cv::Mat const& sourceMat, cv::Mat const& targetMat, std::string const& diff_path)
	{
		if (!checkSize(sourceMat, targetMat)) {
			return false;
		}
		int rows = sourceMat.rows;
		int cols = sourceMat.cols;
		cv::Mat difference_binary(rows, cols, CV_8UC1, cv::Scalar(0));

		constexpr int radius = 2;  // 5x5 neighborhood
		constexpr int border = radius;
		int r_threshold = static_cast<int>(rows * (1 - 0.12));
		int overlap_number = 0;
		int vail_number = 0;
		// 边缘扩充
		cv::Mat paddedImage;
		cv::copyMakeBorder(targetMat, paddedImage, border, border, border, border, cv::BORDER_WRAP);

		// 指针访问
		const float* src_data = reinterpret_cast<const float*>(sourceMat.data);
		const float* pad_data = reinterpret_cast<const float*>(paddedImage.data);
		int pad_cols = paddedImage.cols;

#pragma omp parallel for reduction(+: overlap_number, vail_number)
		for (int y = 0; y < r_threshold; ++y) {
			for (int x = 0; x < cols; ++x) {
				float centerValue = src_data[y * cols + x];
				if (centerValue < 0.01f || centerValue > 1000.f) {
					continue; // 跳过小于阈值的点
				}
				vail_number++;
				int paddedY = y + border;
				int paddedX = x + border;

				// 手动查找3x3邻域最小差值
				float minVal = std::numeric_limits<float>::max();
				float trueVal = 0;
				for (int dy = -radius; dy <= radius; ++dy) {
					for (int dx = -radius; dx <= radius; ++dx) {
						float neighbor = pad_data[(paddedY + dy) * pad_cols + (paddedX + dx)];
						float diff = neighbor - centerValue;
						float diff_abs = std::abs(diff);
						if (diff_abs < minVal) {
							minVal = diff_abs;
							trueVal = diff;
						}
					}
				}

				float thr = std::max(threshold_, centerValue * tolerance_);
				if (minVal < thr) {
					overlap_number += 1;
					continue;
				}
				if (trueVal < thr) {
					continue;
				}

				difference_binary.at<uchar>(y, x) = 255;
			}
		}

		cv::Mat structureElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
		//cv::erode(difference_binary, difference_binary, structureElement, cv::Point(-1, -1), 1);
		//cv::dilate(difference_binary, difference_binary, structureElement, cv::Point(-1, -1), 1);
		cv::morphologyEx(difference_binary, difference_binary, cv::MORPH_OPEN, structureElement);

		int total_pixels = r_threshold * cols;
		int abnormal_value = cv::countNonZero(difference_binary);
		abnormal_rate_ = static_cast<float>(abnormal_value) / float(vail_number);
		overlap_rate_ = static_cast<float>(overlap_number) / float(vail_number);
		depth_valid_rate_ = static_cast<float>(vail_number) / float(total_pixels);

		if (diff_path.size() > 3)
		{
			cv::imwrite(diff_path, difference_binary);
		}

		if (verbose_) {
			LOG_ERROR("");
			LOG_ERROR(" **************** info ******************");
			LOG_ERROR("vail_number: {}", vail_number);
			LOG_ERROR("overlap_number: {}", overlap_number);
			LOG_ERROR("abnormal_value: {}", abnormal_value);
			LOG_ERROR("abnormal_rate:  {} / {} = {}", abnormal_value, vail_number, abnormal_rate_);
			LOG_ERROR("overlap_rate:  {} / {} = {}", overlap_number, vail_number, overlap_rate_);
			LOG_ERROR("depth_valid_rate:  {} / {} = {}", vail_number ,total_pixels, depth_valid_rate_);
			LOG_ERROR(" -------------- end ----------------");
			LOG_ERROR("");
		}
		return true;
	}
	

	bool
	SpatialPerspectiveFilter::checkSize(cv::Mat const& sourceMat, cv::Mat const& targetMat)
	{
		// 输入类型检查
		if (sourceMat.type() != CV_32FC1 || targetMat.type() != CV_32FC1) {
			std::cerr << "Error: sourceMat and targetMat must be of type CV_32FC1." << std::endl;
			return false;
		}

		if (sourceMat.size() != targetMat.size()) {
			std::cerr << "Error: sourceMat and targetMat must have the same size." << std::endl;
			return false;
		}
		return true;
	}
}
