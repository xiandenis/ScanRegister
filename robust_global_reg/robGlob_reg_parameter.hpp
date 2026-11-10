#pragma once

#include <string>
#include <Eigen/Dense>

namespace Ddx
{
    enum
	{
		DEFINEDF = 0,
		INITI_MATRIX = 1,			// initial transform matrix provided
		PRINTF_LOG = 2,				// save log to file
		STATION_LIDAR_DEVICE = 4,	// station LiDAR device mode
	};
	
	struct RobGlobRegParameter
	{
		RobGlobRegParameter()
			: voxel_size(0.3f)
			, close_log(true)
			, use_voxel_sampling(true)
			, use_quatro(false)
			, thr_linearity(0.90f)
			, num_max_corr(5000)
			, normal_r_gain(3.0f)
			, fpfh_r_gain(5.0f)
			, robin_noise_bound_gain(1.0f)
			, solver_noise_bound_gain(0.75f)
			, enable_noise_bound_clamping(true)
			, use_compass(false)
			, search_knn(5)
			, angle_thr(10.f)
			, use_altimeter(false)
			, altitude_thr(2.5f)
			, use_hnsw_knn(true)
			, use_feat_spatial_budget(false)
			, feat_soft_limit(80000)
			, feat_bin_voxel_gain(6.0f)
			, open_iterations(false)
			, min_feature_number(3)
			, max_station_spacing(100.0f)
			, min_station_spacing(-1.0f)
			, number_of_iterations(7)
			, transformation_matrix(Eigen::Matrix4d::Identity())
			, info(0)
		{};

		std::string source_file;
		std::string target_file;
		std::string result_file;
		std::string log_file;
		
		bool close_log;
		float voxel_size;
		bool use_voxel_sampling;
		bool use_quatro;
		float thr_linearity;
		int num_max_corr;
		float normal_r_gain;
		float fpfh_r_gain;
		float robin_noise_bound_gain;
		float solver_noise_bound_gain;
		bool enable_noise_bound_clamping;
		bool use_compass;
		int search_knn;
		float angle_thr;
		bool use_altimeter;
		float altitude_thr;
		/** FPFH knn via hnswlib. Business path: RoughGlobalReg sets from resolution(voxel_size_). */
		bool use_hnsw_knn;
		/** Soft feat_* spatial budget after FPFH. Business path: RoughGlobalReg sets true only when resolution≥0.20 (outdoor). */
		bool use_feat_spatial_budget;
		int feat_soft_limit;
		float feat_bin_voxel_gain;
		bool open_iterations;

		int min_feature_number;
		float max_station_spacing;
		float min_station_spacing; 
		int number_of_iterations;

		Eigen::Matrix4d transformation_matrix;
		int info;

		bool loadConfigFile(std::string const& config_file);
		bool saveConfigFile(std::string const& config_file);
	};
}