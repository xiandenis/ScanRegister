#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace Ddx
{
	class Histogram
	{
	public:
		Histogram() : m_bins_number(0), m_samples_number(0), m_each_ceil_size(-1) {};

		void clear();

		void setEachCeilSize(float ceil_size) { m_each_ceil_size = ceil_size; }

		void addBinsRange(Eigen::Vector2f const& range);

		void addBinsRanges(std::vector<Eigen::Vector2f> const& ranges);

		void addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void addCloud(std::vector<Eigen::Vector3f> const& cloud);

		int findBins(float const& depth);

		void printf();

		void printf(std::string const& name);

		static void printf(std::string const& name, Histogram const& h1, Histogram const& h2);
	public:
		size_t m_samples_number;
		size_t m_bins_number;
		std::vector<Eigen::Vector2f> m_bins_range;
		std::vector<size_t> m_freq; // histogram
		float m_each_ceil_size;
	};


}