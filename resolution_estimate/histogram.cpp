#include "histogram.hpp"

#include <algorithm>
#include <cmath>
#include <logging/logging.hpp>

namespace Ddx {

	void
	Histogram::clear()
	{
		m_bins_range.clear();
		m_freq.clear();
		m_bins_number = 0;
		m_samples_number = 0;
	}

	void
	Histogram::addBinsRange(Eigen::Vector2f const& range)
	{
		m_bins_range.push_back(range);
		m_bins_number++;
		m_freq.push_back(0);
	}

	void
	Histogram::addBinsRanges(std::vector<Eigen::Vector2f> const& ranges)
	{
		m_bins_number = ranges.size();
		m_bins_range = ranges;
		m_freq.assign(m_bins_number, 0);
	}

	void
	Histogram::addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		if (!cloud || cloud->empty() || m_bins_number == 0)
			return;

		m_samples_number += cloud->size();
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(cloud->size()); ++i)
		{
			float depth = cloud->points[i].getVector3fMap().norm();
			int b_id = findBins(depth);
			if (b_id < 0) continue;
#ifdef _OPENMP
#pragma omp atomic
#endif
			++m_freq[b_id];
		}
	}

	void
	Histogram::addCloud(std::vector<Eigen::Vector3f> const& cloud)
	{
		if (cloud.empty() || m_bins_number == 0)
			return;

		m_samples_number += cloud.size();
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(cloud.size()); ++i)
		{
			float depth = cloud[i].norm();
			int b_id = findBins(depth);
			if (b_id < 0) continue;
#ifdef _OPENMP
#pragma omp atomic
#endif
			++m_freq[b_id];
		}
	}

	int
	Histogram::findBins(float const& depth)
	{
		if (m_each_ceil_size < 1)
		{
			for (int i = 0; i < m_bins_number; ++i)
			{
				if (m_bins_range[i][0] <= depth && depth < m_bins_range[i][1])
					return i;
			}
		}
		else
		{
			float r1x = depth / m_each_ceil_size;
			return std::min(int(std::ceil(r1x)), int(m_bins_number - 1));
		}

		return -1;
	}

	void
	Histogram::printf()
	{
		if (m_samples_number == 0)
			return;
		LOG_INFO("");
		LOG_INFO("***************** histogram info *****************");
		LOG_INFO(" samples number : {}", m_samples_number);
		LOG_INFO(" bins number : {}", m_bins_number);
		for (int i = 0; i < static_cast<int>(m_bins_number); ++i)
		{
			float rate = (float(m_freq[i]) / float(m_samples_number)) * 100.0f;
			LOG_INFO(" bins [ {} , {} ] = {} ({}%)", m_bins_range[i][0],
				m_bins_range[i][1], m_freq[i], rate);
		}
		LOG_INFO("----------------- histogram end  -----------------");
		LOG_INFO("");
	}

	void
	Histogram::printf(std::string const& name)
	{
		if (m_samples_number == 0)
			return;
		LOG_INFO("");
		LOG_INFO("***************** histogram {} info *****************", name);
		LOG_INFO(" samples number : {}", m_samples_number);
		LOG_INFO(" bins number : {}", m_bins_number);
		for (int i = 0; i < static_cast<int>(m_bins_number); ++i)
		{
			float rate = (float(m_freq[i]) / float(m_samples_number)) * 100.0f;
			LOG_INFO(" bins [ {} , {} ] = {} ({}%)", m_bins_range[i][0],
				m_bins_range[i][1], m_freq[i], rate);
		}
		LOG_INFO("----------------- histogram end  -----------------");
		LOG_INFO("");
	}

	void
	Histogram::printf(std::string const& name, Histogram const& h1, Histogram const& h2)
	{
		if (h1.m_bins_number != h2.m_bins_number) return;
		if (h1.m_samples_number == 0 || h2.m_samples_number == 0) return;

		LOG_INFO("");
		LOG_INFO("***************** histogram {} info *****************", name);
		LOG_INFO(" samples number : {} ------> {}", h1.m_samples_number, h2.m_samples_number);
		LOG_INFO(" bins number : {} ------> {}", h1.m_bins_number, h2.m_bins_number);
		for (int i = 0; i < static_cast<int>(h1.m_bins_number); ++i)
		{
			float rate = (float(h1.m_freq[i]) / float(h1.m_samples_number)) * 100.0f;
			float rate2 = (float(h2.m_freq[i]) / float(h2.m_samples_number)) * 100.0f;
			LOG_INFO(" bins [ {} , {} ] = {} ({}%) -----> {} ({}%)",
				h1.m_bins_range[i][0], h1.m_bins_range[i][1],
				h1.m_freq[i], rate, h2.m_freq[i], rate2);
		}
		LOG_INFO("----------------- histogram end  -----------------");
		LOG_INFO("");
	}
}
