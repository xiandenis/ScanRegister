#pragma once

#include "robGlob_reg_parameter.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

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

		bool run();

        Eigen::Matrix4d getFinalTransformationMatrix() { return m_transformation_matrix; }
        int getFeatureNumber() { return m_feature_number; }

        bool saveResult(bool vaild);
	private:

		bool convertCloudToVec(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<Eigen::Vector3f>& e_cloud);

		void updataCloud(std::vector<Eigen::Vector3f>& cloud, Eigen::Matrix4d const& matrix);

		bool loadCloud(std::string const& file, std::vector<Eigen::Vector3f>& e_cloud);


		bool loadResult(std::string const& file_path, Eigen::Matrix4d& matrix);

		bool _run(std::vector<Eigen::Vector3f> const& src_vec, std::vector<Eigen::Vector3f> const& tgt_vec);

		bool _station_lidar_device_run();
	private:
        RobGlobRegParameter m_parameter;

		Eigen::Matrix4d m_transformation_matrix;
		int m_feature_number;
		double m_cost_time;
	};
}