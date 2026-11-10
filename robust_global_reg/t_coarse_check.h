#pragma once

#include <Eigen/Core>

#include <string>

namespace Ddx
{

	/**
	*  Coarse registration quality check
	*/
	class TCoarseCheck
	{
	public:
		TCoarseCheck()
			: m_min_feature_number(10)
			, m_tilt_angle_threshold(15.0)		/* coarse reg: looser than sensor precision */
			, m_height_threshold(-1.0)			/* coarse reg: looser than sensor precision */
			, m_compass_angle_threshold(-1.0)	/* coarse reg: looser than sensor precision */
			, m_min_distance_threshold(-0.01f)
			, m_max_distance_threshold(100.0f)
		{};

		~TCoarseCheck() {};

		// minimum matched feature count
		void setMinimumFeatureNumber(float min_feature_number) { m_min_feature_number = min_feature_number; }

		// tilt angle tolerance (degrees)
		void setTiltAngleThreshold(float angle) { m_tilt_angle_threshold = angle; }

		// height tolerance
		void setHeightThreshold(float height_threshold) { m_height_threshold = height_threshold; }

		// compass angle tolerance (degrees)
		void setCompassAngleThreshold(float angle) { m_compass_angle_threshold = angle; }

		// minimum station spacing
		void setMinDistanceThreshold(double distance) { m_min_distance_threshold = distance; }

		// maximum station spacing
		void setMaxDistanceThreshold(double distance) { m_max_distance_threshold = distance; }


		bool check(Eigen::Matrix4d const& matrix, int feature_number);

		std::string getChechInfo() { return m_process_info; }

		void initialize();
	private:
		float m_min_feature_number;										// minimum matched feature count
		float m_tilt_angle_threshold;									// tilt angle tolerance (degrees)
		float m_height_threshold;										// height tolerance
		float m_compass_angle_threshold;								// compass angle tolerance (degrees)

		double m_min_distance_threshold;									// minimum station spacing
		double m_max_distance_threshold;									// maximum station spacing

		double m_tilt_angle_value;
		double m_compass_angle_value;
		std::string m_process_info;										// check diagnostics message

	};
}