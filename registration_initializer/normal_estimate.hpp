#pragma once

#include <registration_type/pcl_type.hpp>

#include <string>

namespace Ddx
{
	enum class NormalEstimateMethod
	{
		SmallGicpTbb,
		PclOmp
	};

	inline char const* normalEstimateMethodName(NormalEstimateMethod method)
	{
		switch (method) {
		case NormalEstimateMethod::SmallGicpTbb:
			return "small_gicp_tbb";
		case NormalEstimateMethod::PclOmp:
			return "pcl_omp";
		}
		return "small_gicp_tbb";
	}

	inline bool parseNormalEstimateMethod(std::string const& name, NormalEstimateMethod& out)
	{
		if (name == "small_gicp_tbb") {
			out = NormalEstimateMethod::SmallGicpTbb;
			return true;
		}
		if (name == "pcl_omp") {
			out = NormalEstimateMethod::PclOmp;
			return true;
		}
		return false;
	}

	bool estimateNormalsKnn(pcl_type::PointCloudPtr const& cloud, pcl_type::NormalPtr& normal,
	                        NormalEstimateMethod method);
}
