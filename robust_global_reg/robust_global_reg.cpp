
#include "robust_global_reg.hpp"
#include "t_coarse_check.h"
#include <common/common.hpp>
#include <logging/logging.hpp>
#include <logging/tic_toc.hpp>
#include <logging/mem_peak.hpp>
#include <point_cloud_io/point_cloud_io.hpp>
#include <point_cloud_io/pcse_reader.hpp>
#include <file_system/file_system.hpp>
#include <robust_matcher/RobustMatcher.hpp>
#if __has_include(<rapidjson/document.h>)
#include <rapidjson/document.h>
#else
#include <json/document.h>
#endif
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace
{

std::string pathStem(std::string const& path)
{
	std::size_t slash = path.find_last_of("/\\");
	std::string base = (slash == std::string::npos) ? path : path.substr(slash + 1);
	std::size_t dot = base.find_last_of('.');
	if (dot != std::string::npos && dot > 0) {
		base.resize(dot);
	}
	return base;
}

/// Prefer parent folder when source is info/<scan>/voxel_*_p.pcd (common RealTimeManager path).
std::string scanNameForMem(std::string const& path)
{
	std::string const stem = pathStem(path);
	if (stem.rfind("voxel_", 0) != 0) {
		return stem;
	}
	std::size_t slash = path.find_last_of("/\\");
	if (slash == std::string::npos || slash == 0) {
		return stem;
	}
	std::string const parent = path.substr(0, slash);
	std::string const parent_stem = pathStem(parent);
	return parent_stem.empty() ? stem : parent_stem;
}

}  // namespace

namespace Ddx
{
    RobustGlobalReg::~RobustGlobalReg()
    {
        LOG_INFO("************* robust global reg [end] *************");
        LOG_INFO("");
    }

    bool
    RobustGlobalReg::setParameterFromFile(std::string const& config_file)
    {
        return m_parameter.loadConfigFile(config_file);
    }

    bool
	RobustGlobalReg::run()
	{
        m_transformation_matrix = Eigen::Matrix4d::Identity();
		auto const path_ok = [](std::string const& path) {
			return !path.empty()
				&& (stlplus::file_exists(path) || point_cloud_io::is_pcse_dataset(path));
		};
		if (!path_ok(m_parameter.source_file))
		{
            LOG_ERROR("source file not exists !");
            return false;
        }

		if (!path_ok(m_parameter.target_file))
		{
            LOG_ERROR("target file not exists !");
            return false;
		}

        if ((m_parameter.info & STATION_LIDAR_DEVICE) == STATION_LIDAR_DEVICE)
		{
			if (!_station_lidar_device_run())
				return false;
		}
		else
		{
			std::vector<Eigen::Vector3f> src_vec;
			if (!loadCloud(m_parameter.source_file, src_vec))
			{
                LOG_ERROR("load cloud fail! ({})", m_parameter.source_file);
                return false;
            }

			std::vector<Eigen::Vector3f> tgt_vec;
			if (!loadCloud(m_parameter.target_file, tgt_vec))
			{
                LOG_ERROR("load cloud fail! ({})", m_parameter.target_file);
				return false;
			}

			if (m_parameter.use_compass) {
				m_parameter.transformation_matrix.block<3, 1>(0, 3) = Eigen::Vector3d::Zero();
			}

			if ((m_parameter.info & INITI_MATRIX) == INITI_MATRIX)
			{
				updataCloud(src_vec, m_parameter.transformation_matrix);
			}

			if (!_run(src_vec, tgt_vec)) {
				return false;
			}
			if ((m_parameter.info & INITI_MATRIX) == INITI_MATRIX) {
				m_transformation_matrix = m_transformation_matrix * m_parameter.transformation_matrix;
			}
		}

        return true;
    }

    bool
	RobustGlobalReg::saveResult(bool vaild)
	{
		std::ofstream outF(m_parameter.result_file, std::ios::out | std::ios::binary);
		if (!outF)
			return false;

		std::string const transformation = Ddx::Common::matrix_to_south_string(m_transformation_matrix);
		outF << "{" << std::endl;
		outF << "\t\"vaild\" : " << (vaild ? "true" : "false") << "," << std::endl;
		outF << "\t\"time\" : " << Ddx::Common::double_to_string(m_cost_time, 6) << "," << std::endl;
		outF << "\t\"feature\" : " << m_feature_number << "," << std::endl;
		outF << "\t\"transformation\" : \"" << transformation << "\"" << std::endl;
		outF << "}" << std::endl;
		outF.close();

		return stlplus::file_exists(m_parameter.result_file);
	}

    bool
	RobustGlobalReg::convertCloudToVec(
		const pcl::PointCloud<pcl::PointXYZ>& cloud,
		std::vector<Eigen::Vector3f>& e_cloud)
	{
		e_cloud.resize(cloud.size());
		for (std::size_t i = 0; i < cloud.size(); ++i)
		{
			e_cloud[i] = Eigen::Vector3f(
				cloud[i].x,
				cloud[i].y,
				cloud[i].z);
		}
		return !e_cloud.empty();
	}

    void
	RobustGlobalReg::updataCloud(std::vector<Eigen::Vector3f>& cloud, Eigen::Matrix4d const& matrix)
	{
		Eigen::Matrix3f R = matrix.block<3, 3>(0, 0).cast<float>();
		Eigen::Vector3f t = matrix.block<3, 1>(0, 3).cast<float>();
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(cloud.size()); ++i)
		{
			cloud[i] = R * cloud[i] + t;
		}
	}

    bool
	RobustGlobalReg::loadCloud(std::string const& file, std::vector<Eigen::Vector3f>& e_cloud)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		if (Ddx::point_cloud_io::load(file, cloud) != 0)
			return false;
		if (cloud.size() < 10)
			return false;
		return convertCloudToVec(cloud, e_cloud);
	}

	bool
	RobustGlobalReg::loadResult(std::string const& file_path, Eigen::Matrix4d& matrix)
	{
		if (!stlplus::file_exists(file_path))
			return false;

		std::ifstream file(file_path);
		std::string const strJson(
			(std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
		rapidjson::Document doc;
		doc.Parse(strJson.c_str());
		if (doc.HasParseError() || !doc.IsObject())
			return false;
		if (!doc.HasMember("transformation"))
			return false;

		matrix = Ddx::Common::south_string_to_matrix(doc["transformation"].GetString());
		return true;
	}

    bool
	RobustGlobalReg::_run(std::vector<Eigen::Vector3f> const& src_vec, std::vector<Eigen::Vector3f> const& tgt_vec)
	{
		TicToc tic_toc(" global reg time : ", false);
		std::string const mem_name = "global_reg " + scanNameForMem(m_parameter.source_file);
		MemPeakScope mem_scope(mem_name);

		robust_matcher::RobustMatcherConfig config(
			m_parameter.voxel_size,
			m_parameter.use_voxel_sampling,
			m_parameter.use_quatro,
			m_parameter.thr_linearity,
			m_parameter.num_max_corr,
			m_parameter.normal_r_gain,
			m_parameter.fpfh_r_gain,
			m_parameter.robin_noise_bound_gain,
			m_parameter.solver_noise_bound_gain,
			m_parameter.enable_noise_bound_clamping
		);
		config.use_compass_ = m_parameter.use_compass;
		config.search_knn_ = m_parameter.search_knn;
		config.angle_thr_ = m_parameter.angle_thr;
		config.use_altimeter = m_parameter.use_altimeter;
		config.altitude_thr = m_parameter.altitude_thr;
		config.use_hnsw_knn_ = m_parameter.use_hnsw_knn;
		if (const char* env = std::getenv("RTR_USE_HNSW_KNN")) {
			if (env[0] == '1' || env[0] == 't' || env[0] == 'T' || env[0] == 'y' || env[0] == 'Y') {
				config.use_hnsw_knn_ = true;
			} else if (env[0] == '0' || env[0] == 'f' || env[0] == 'F' || env[0] == 'n' ||
			           env[0] == 'N') {
				config.use_hnsw_knn_ = false;
			}
		}
		config.use_feat_spatial_budget_ = m_parameter.use_feat_spatial_budget;
		config.feat_soft_limit_         = m_parameter.feat_soft_limit;
		config.feat_bin_voxel_gain_     = m_parameter.feat_bin_voxel_gain;
		if (const char* env = std::getenv("RTR_USE_FEAT_SPATIAL_BUDGET")) {
			if (env[0] == '1' || env[0] == 't' || env[0] == 'T' || env[0] == 'y' || env[0] == 'Y') {
				config.use_feat_spatial_budget_ = true;
			} else if (env[0] == '0' || env[0] == 'f' || env[0] == 'F' || env[0] == 'n' ||
			           env[0] == 'N') {
				config.use_feat_spatial_budget_ = false;
			}
		}
		if (const char* env = std::getenv("RTR_FEAT_SOFT_LIMIT")) {
			try {
				config.feat_soft_limit_ = std::stoi(env);
			} catch (...) {
			}
		}
		if (const char* env = std::getenv("RTR_FEAT_BIN_VOXEL_GAIN")) {
			try {
				config.feat_bin_voxel_gain_ = std::stof(env);
			} catch (...) {
			}
		}
		// Outdoor-only hard gate (same 0.20 threshold as HNSW / RoughGlobalReg).
		// Indoor/garage must not run spatial budget even if JSON/env forced on.
		constexpr float kFeatBudgetMinVoxelSize = 0.20f;
		if (config.use_feat_spatial_budget_ &&
		    m_parameter.voxel_size < kFeatBudgetMinVoxelSize) {
			LOG_INFO("feat_spatial_budget skipped (resolution={:.4f} < {:.2f}, outdoor-only)",
				m_parameter.voxel_size, kFeatBudgetMinVoxelSize);
			config.use_feat_spatial_budget_ = false;
		}

		if (config.use_hnsw_knn_) {
			LOG_INFO("robust_matcher knn backend=HNSW (M={} ef_construction={} ef={})",
				config.hnsw_params_.M, config.hnsw_params_.ef_construction, config.hnsw_params_.ef);
		}
		if (config.use_feat_spatial_budget_) {
			LOG_INFO("feat_spatial_budget on (soft_limit={} bin_gain={:.3f} resolution={:.4f})",
				config.feat_soft_limit_, config.feat_bin_voxel_gain_, m_parameter.voxel_size);
		}

		robust_matcher::RobustMatcher matcher(config);

		const auto solution = matcher.estimate(src_vec, tgt_vec);

		if (matcher.wasFeatSpatialBudgetEnabled()) {
			auto const& sb = matcher.getSrcFeatBudgetResult();
			auto const& tb = matcher.getTgtFeatBudgetResult();
			LOG_INFO("feat_budget src: N={} -> N'={} (bins={}, soft={}, applied={})",
				sb.n_before, sb.n_after, sb.occupied_bins, config.feat_soft_limit_, sb.applied);
			LOG_INFO("feat_budget tgt: N={} -> N'={} (bins={}, soft={}, applied={})",
				tb.n_before, tb.n_after, tb.occupied_bins, config.feat_soft_limit_, tb.applied);
		}

		m_feature_number = static_cast<int>(matcher.getNumFinalInliers());
		{
			auto const matched = matcher.getKeypointsFromInitialMatching();
			std::size_t const n_matched = std::get<0>(matched).size();
			double const t_voxel = matcher.getProcessingTime();
			double const t_extract = matcher.getExtractionTime();
			double const t_match = matcher.getMatchingTime();
			double const t_reject = matcher.getRejectionTime();
			double const t_solve = matcher.getSolverTime();
			double const t_build = matcher.getRobinBuildTreeTime();
			double const t_flann = matcher.getRobinFlannSearchTime();
			double const t_mutual = matcher.getRobinMutualFilterTime();
			LOG_INFO(
				"robust_matcher voxel={:.3f}s extract={:.3f}s match={:.3f}s "
				"(build={:.3f}s flann={:.3f}s mutual={:.3f}s reject={:.3f}s) solve={:.3f}s "
				"| src={} tgt={} feat_src={} feat_tgt={} matched={} inliers={} valid={}",
				t_voxel, t_extract, t_match,
				t_build, t_flann, t_mutual, t_reject, t_solve,
				src_vec.size(), tgt_vec.size(),
				matcher.getRobinNumSourceFeatures(), matcher.getRobinNumTargetFeatures(),
				n_matched, m_feature_number,
				solution.valid ? 1 : 0);
			LOG_INFO(
				"extract_detail (src+tgt) kdtree={:.3f}s normal={:.3f}s setup={:.3f}s "
				"spfh={:.3f}s weight={:.3f}s",
				matcher.getExtractKdtreeTime(), matcher.getExtractNormalTime(),
				matcher.getExtractSetupTime(), matcher.getExtractSpfhTime(),
				matcher.getExtractWeightTime());
		}
		if (!solution.valid)
		{
            LOG_ERROR("registration fail!");
			return false;
		}

		m_transformation_matrix.block<3, 3>(0, 0) = solution.rotation;
		m_transformation_matrix.topRightCorner(3, 1) = solution.translation;
		m_cost_time += tic_toc.toc();
		return true;
	}

    bool
	RobustGlobalReg::_station_lidar_device_run()
	{
		TicToc tic_toc(" global station lidar time : ", false);

		std::vector<Eigen::Vector3f> src_vec;
		if (!loadCloud(m_parameter.source_file, src_vec))
		{
            LOG_ERROR("load cloud fail ({})", m_parameter.source_file);
            return false;
		}
		std::vector<Eigen::Vector3f> tgt_vec;
		if (!loadCloud(m_parameter.target_file, tgt_vec))
		{
            LOG_ERROR("load cloud fail ({})", m_parameter.target_file);
            return false;
        }
		LOG_INFO("station lidar load src={} tgt={} voxel_size={:.4f} max_iters={}",
			src_vec.size(), tgt_vec.size(), m_parameter.voxel_size,
			m_parameter.number_of_iterations);

		float const max_spacing = std::min(m_parameter.max_station_spacing, m_parameter.voxel_size * 200.f);
		Ddx::TCoarseCheck check;
		check.setMaxDistanceThreshold(max_spacing);
		check.setMinDistanceThreshold(m_parameter.min_station_spacing);
		check.setMinimumFeatureNumber(m_parameter.min_feature_number);
		check.initialize();
		for (int i = 0; i < m_parameter.number_of_iterations; ++i)
		{
			LOG_INFO("station lidar coarse iter {}/{}", i + 1, m_parameter.number_of_iterations);
			m_feature_number = 0;
			if (!_run(src_vec, tgt_vec)) {
				LOG_INFO("station lidar coarse iter {} estimate failed", i + 1);
				continue;
			}

			if (check.check(m_transformation_matrix, m_feature_number)) {
				LOG_INFO("station lidar coarse check passed at iter {}", i + 1);
				m_cost_time = tic_toc.toc();
				return true;
			}
			LOG_INFO("station lidar coarse check failed at iter {}: {}",
				i + 1, check.getChechInfo());
		}

		m_cost_time = tic_toc.toc();
		return false;
	}

}
