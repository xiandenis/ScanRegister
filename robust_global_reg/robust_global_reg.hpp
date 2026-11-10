#pragma once

#include "robGlob_reg_parameter.hpp"
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace Ddx
{
    class RobustGlobalReg
	{
	public:
		RobustGlobalReg() : m_cost_time(0), m_feature_number(0){}
		RobustGlobalReg(RobGlobRegParameter const& parameter) : m_cost_time(0), m_feature_number(0), m_parameter(parameter){}
		~RobustGlobalReg();
		
		void setParameter(RobGlobRegParameter const& parameter) { m_parameter = parameter; }

        bool setParameterFromFile(std::string const& config_file);

		/**
		 * Coarse global registration on in-memory clouds.
		 *
		 * Consumer owns I/O and frame: apply WorkingFrame (p' = p - O) before call.
		 * When use_compass is true, inputs MUST be station-centered: station origin at (0,0,0).
		 * Module does not load files, infer station origin, or verify the origin.
		 */
		bool run(std::vector<Eigen::Vector3f> const& src,
		         std::vector<Eigen::Vector3f> const& tgt);

        Eigen::Matrix4d getFinalTransformationMatrix() { return m_transformation_matrix; }
        int getFeatureNumber() { return m_feature_number; }

        bool saveResult(bool vaild);
	private:

		void updataCloud(std::vector<Eigen::Vector3f>& cloud, Eigen::Matrix4d const& matrix);

		bool loadResult(std::string const& file_path, Eigen::Matrix4d& matrix);

		bool _run(std::vector<Eigen::Vector3f> const& src_vec, std::vector<Eigen::Vector3f> const& tgt_vec);

		bool _station_lidar_device_run(std::vector<Eigen::Vector3f> const& src_vec,
		                               std::vector<Eigen::Vector3f> const& tgt_vec);
	private:
        RobGlobRegParameter m_parameter;

		Eigen::Matrix4d m_transformation_matrix;
		int m_feature_number;
		double m_cost_time;
	};
}
