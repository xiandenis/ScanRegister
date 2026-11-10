#include "panorama_filter.hpp"
#include <logging/logging.hpp>
#define MATH_PI  3.14159265358979323846264338327950288   // pi

namespace Ddx {

	bool 
	PanoramaFilter::difference(cv::Mat const& sourceMat, cv::Mat const& targetMat, std::string const& diff_path)
	{
		if (!checkSize(sourceMat, targetMat)) {
			return false;
		}

		Eigen::Matrix<int, 2, 9> kernel = Eigen::Matrix<int, 2, 9>::Zero();
		kernel.col(0) << -1, -1;	kernel.col(1) << -1, 0;	 kernel.col(2) << -1, 1;
		kernel.col(3) << 0, -1;		kernel.col(4) << 0, 0;	 kernel.col(5) << 0, 1;
		kernel.col(6) << 1, -1;		kernel.col(7) << 1, 0;	 kernel.col(8) << 1, 1;

		int rows = static_cast<int> (sourceMat.rows);
		int cols = static_cast<int> (sourceMat.cols);
		int r_threshold = rows * (1 - 0.12);

		cv::Mat difference_binary(rows, cols, CV_8UC1, cv::Scalar(0));

		float angle = static_cast<float> (MATH_PI / cols);
		float sin_angle_1 = float(-1.0f) * std::sin(angle) * 2.0f;  // 允许最大误差
		float sin_angle_2 = std::sin(angle) * 2.0f * 3.0f;  // 3倍
		float sin_angle_3 = float(-1.0f) * sin_angle_2;
		int total_pixels = r_threshold * cols;


		int overlap_number = 0;
		int vail_number = 0;
		const ushort* src_data = reinterpret_cast<const ushort*>(sourceMat.data);
		const ushort* tgt_data = reinterpret_cast<const ushort*>(targetMat.data);
#ifdef _OPENMP
#pragma omp parallel for reduction(+: overlap_number, vail_number)
#endif
		for (int i = 0; i < total_pixels; ++i)
		{
			int k11x = static_cast<int> (src_data[i]);
			if (k11x == 0)
				continue;
			vail_number++;

			int r = i / cols;
			int c = i % cols;
			int min_value = std::numeric_limits<int>::max();
			int min_value_true = 0;
			for (int k = 0; k < 9; ++k)
			{
				int r_1 = r + kernel(0, k);
				int c_1 = (c + kernel(1, k) + cols) % cols;
				if (r_1 < 0 || r_1 >= rows)
					continue;
				int k12x = static_cast<int> (tgt_data[r_1 * cols + c_1]);
				int k121x = k12x - k11x;
				int k13x = std::abs(k121x);
				if (k13x < min_value) {
					min_value_true = k121x;
					min_value = k13x;
				}
			}

			float k20x = k11x * sin_angle_2;
			int k21x = static_cast<int> (k20x);
			int k22x = std::min(10, k21x);

			if (min_value < k21x) {
				overlap_number++;
				continue;
			}
			if (min_value_true < k21x) {
				continue;
			}

			difference_binary.at<uchar>(r, c) = 255;
		}

		cv::Mat structureElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
		cv::erode(difference_binary, difference_binary, structureElement, cv::Point(-1, -1), 1);
		cv::dilate(difference_binary, difference_binary, structureElement, cv::Point(-1, -1), 1);
		
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
			LOG_ERROR("depth_valid_rate:  {} / {} = {}", vail_number, total_pixels, depth_valid_rate_);
			LOG_ERROR(" -------------- end ----------------");
			LOG_ERROR("");
		}
		return true;
	}


	bool
	PanoramaFilter::checkSize(cv::Mat const& sourceMat, cv::Mat const& targetMat)
	{
		// 输入类型检查
		if (sourceMat.type() != CV_16UC1 || targetMat.type() != CV_16UC1) {
			std::cerr << "Error: sourceMat and targetMat must be of type CV_16UC1." << std::endl;
			return false;
		}

		int s_rows = static_cast<int> (sourceMat.rows);
		int s_cols = static_cast<int> (sourceMat.cols);
		int t_rows = static_cast<int> (targetMat.rows);
		int t_cols = static_cast<int> (targetMat.cols);

		if (s_rows != t_rows || s_cols != t_cols || s_rows <= 0 || s_cols <= 0) {
			std::cerr << "Error: sourceMat and targetMat must have the same size." << std::endl;
			return false;
		}
			
		return true;
	}

}