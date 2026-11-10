#include "resolution_estimate.hpp"
#include "histogram.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Ddx
{
	namespace {

	float estimate_from_histogram(Histogram& histogram, std::vector<float> const& resolution_vec)
	{
		if (histogram.m_freq.empty() || histogram.m_freq.size() != resolution_vec.size())
			return std::nanf("");

		auto maxIt = std::max_element(histogram.m_freq.begin(), histogram.m_freq.end());
		if (maxIt == histogram.m_freq.end() || *maxIt == 0)
			return std::nanf("");

		int maxIndex = static_cast<int>(std::distance(histogram.m_freq.begin(), maxIt));
		return resolution_vec[maxIndex];
	}

	} // namespace

	float
	ResolutionEstimate::estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string const& name)
	{
		if (!cloud || cloud->size() < 10)  return std::nanf("");
		std::vector<float> resolution_vec;
		std::vector<Eigen::Vector2f> range_vec;

		range_vec.push_back(Eigen::Vector2f(0, 15));	resolution_vec.push_back(0.1f); // 取0.1
		range_vec.push_back(Eigen::Vector2f(15, 25));	resolution_vec.push_back(0.2f); // 取0.2
		range_vec.push_back(Eigen::Vector2f(25, 45));	resolution_vec.push_back(0.3f); // 取0.32
		range_vec.push_back(Eigen::Vector2f(45, 65));	resolution_vec.push_back(0.4f); // 取0.45
		range_vec.push_back(Eigen::Vector2f(65, 85));	resolution_vec.push_back(0.6f); // 取0.7
		range_vec.push_back(Eigen::Vector2f(85, 100));	resolution_vec.push_back(0.8f); // 取0.8
		range_vec.push_back(Eigen::Vector2f(100, std::numeric_limits<float>::max())); resolution_vec.push_back(1.0f);

		Histogram histogram;
		histogram.addBinsRanges(range_vec);
		histogram.addCloud(cloud);
		if (name.empty())
			histogram.printf();
		else
			histogram.printf(name);
		return estimate_from_histogram(histogram, resolution_vec);
	}

	float
	ResolutionEstimate::estimate(std::vector<Eigen::Vector3f> const& cloud, std::string const& name)
	{
		if (cloud.size() < 10)  return std::nanf("");
		std::vector<float> resolution_vec;
		std::vector<Eigen::Vector2f> range_vec;
		range_vec.push_back(Eigen::Vector2f(0, 10));	resolution_vec.push_back(0.1f); // 取0.1
		range_vec.push_back(Eigen::Vector2f(10, 20));	resolution_vec.push_back(0.2f); // 取0.2
		range_vec.push_back(Eigen::Vector2f(20, 40));	resolution_vec.push_back(0.3f); // 取0.3
		range_vec.push_back(Eigen::Vector2f(40, 60));	resolution_vec.push_back(0.4f); // 取0.4
		range_vec.push_back(Eigen::Vector2f(60, 80));	resolution_vec.push_back(0.6f); // 取0.6
		range_vec.push_back(Eigen::Vector2f(80, 100));	resolution_vec.push_back(0.8f); // 取0.8
		range_vec.push_back(Eigen::Vector2f(100, std::numeric_limits<float>::max())); resolution_vec.push_back(2.0f); // 取2.0

		Histogram histogram;
		histogram.addBinsRanges(range_vec);
		histogram.addCloud(cloud);
		if (name.empty())
			histogram.printf();
		else
			histogram.printf(name);
		return estimate_from_histogram(histogram, resolution_vec);
	}

}