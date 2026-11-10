#include "t_coarse_check.h"
#include <common/defines.hpp>
#include <cmath>


using namespace Ddx;

void TCoarseCheck::initialize()
{
	double tilt_angle_threshold = m_tilt_angle_threshold * MATH_PI / 180.0;
	m_tilt_angle_value = std::cos(tilt_angle_threshold);

	m_compass_angle_value = m_compass_angle_threshold * MATH_PI / 180.0;
}

bool
TCoarseCheck::check(Eigen::Matrix4d const& matrix, int feature_number)
{
	if (feature_number < m_min_feature_number)
	{
		m_process_info = "Too few matched feature points (" + std::to_string(feature_number) + ")";
		return false;
	}

	// maximum station spacing
	Eigen::Vector3d translation = matrix.block<3, 1>(0, 3);
	double distance_1 = translation.norm();
	if (distance_1 > m_max_distance_threshold)
	{
		m_process_info = "Station spacing too large (" + std::to_string(distance_1) + ")";
		return false;
	}

	// minimum station spacing
	if (distance_1 < m_min_distance_threshold)
	{
		m_process_info = "Station spacing too small (" + std::to_string(distance_1) + ")";
		return false;
	}

	// tilt angle
	if (m_tilt_angle_threshold > 0)
	{
		const auto i20x = matrix(2, 2);
		if (i20x < m_tilt_angle_value)
		{
			m_process_info = "Tilt angle out of tolerance (" + std::to_string(MATH_RAD2DEG(std::acos(i20x))) + " deg)";
			return false;
		}
	}

	// altimeter check (reserved)
	if (m_height_threshold > 0)
	{

	}
	// compass check (reserved)
	if (m_compass_angle_threshold > 0)
	{

	}

	return true;
}

