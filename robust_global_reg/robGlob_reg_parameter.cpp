
#include "robGlob_reg_parameter.hpp"
#include <file_system/file_system.hpp>
#include <common/common.hpp>
#include <logging/logging.hpp>
#if __has_include(<rapidjson/document.h>)
#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#else
#include <json/rapidjson.h>
#include <json/document.h>
#include <json/filereadstream.h>
#include <json/prettywriter.h>
#include <json/stringbuffer.h>
#endif
#include <fstream>
#include <iostream>

namespace Ddx
{
	bool 
	RobGlobRegParameter::loadConfigFile(std::string const& config_file)
	{
		if (!stlplus::file_exists(config_file))
		{
            LOG_ERROR("Config file does not exist ({}), load failed", config_file);
            return false;
		}

		std::ifstream file(config_file);
		std::string strJson((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
		rapidjson::Document doc;
		doc.Parse(strJson.c_str());

		if (doc.HasParseError())
		{
            LOG_ERROR("Failed to parse JSON file, please verify the JSON is valid: {}", config_file);
			return false;
		}

		rapidjson::Value& jsonObj = doc;

		if (jsonObj.HasMember("scan_name")) {
			scan_name = jsonObj["scan_name"].GetString();
		}
		if (jsonObj.HasMember("result_file")) {
			result_file = jsonObj["result_file"].GetString();
		}
		if (jsonObj.HasMember("log_file")) {
			log_file = jsonObj["log_file"].GetString();
		}

		// Optional parameters
		if (jsonObj.HasMember("voxel_size")) {
            LOG_WARN(" has voxel_size field ");
			voxel_size = jsonObj["voxel_size"].GetFloat();
		}
		if (jsonObj.HasMember("use_voxel_sampling")) {
			LOG_WARN(" has use_voxel_sampling field ");
			use_voxel_sampling = jsonObj["use_voxel_sampling"].GetBool();
		}
		if (jsonObj.HasMember("use_quatro")) {
			LOG_WARN(" has use_quatro field ");
			use_quatro = jsonObj["use_quatro"].GetBool();
		}
		if (jsonObj.HasMember("thr_linearity")) {
			LOG_WARN(" has thr_linearity field ");
			thr_linearity = jsonObj["thr_linearity"].GetFloat();
		}
		if (jsonObj.HasMember("num_max_corr")) {
			LOG_WARN(" has num_max_corr field ");
			num_max_corr = jsonObj["num_max_corr"].GetInt();
		}
		if (jsonObj.HasMember("normal_r_gain")) {
			LOG_WARN(" has normal_r_gain field ");
			normal_r_gain = jsonObj["normal_r_gain"].GetFloat();
		}
		if (jsonObj.HasMember("fpfh_r_gain")) {
			LOG_WARN(" has fpfh_r_gain field ");
			fpfh_r_gain = jsonObj["fpfh_r_gain"].GetFloat();
		}
		if (jsonObj.HasMember("robin_noise_bound_gain")) {
			LOG_WARN(" has robin_noise_bound_gain field ");
			robin_noise_bound_gain = jsonObj["robin_noise_bound_gain"].GetFloat();
		}
		if (jsonObj.HasMember("solver_noise_bound_gain")) {
			LOG_WARN(" has solver_noise_bound_gain field ");
			solver_noise_bound_gain = jsonObj["solver_noise_bound_gain"].GetFloat();
		}
		if (jsonObj.HasMember("enable_noise_bound_clamping")) {
			LOG_WARN(" has enable_noise_bound_clamping field ");
			enable_noise_bound_clamping = jsonObj["enable_noise_bound_clamping"].GetBool();
		}
		if (jsonObj.HasMember("use_compass")) {
			LOG_WARN(" has use_compass field ");
			use_compass = jsonObj["use_compass"].GetBool();
		}
		if (jsonObj.HasMember("search_knn")) {
			LOG_WARN(" has search_knn field ");
			search_knn = jsonObj["search_knn"].GetInt();
		}
		if (jsonObj.HasMember("angle_thr")) {
			LOG_WARN(" has angle_thr field ");
			angle_thr = jsonObj["angle_thr"].GetFloat();
		}
		if (jsonObj.HasMember("use_altimeter")) {
			LOG_WARN(" has use_altimeter field ");
			use_altimeter = jsonObj["use_altimeter"].GetBool();
		}
		if (jsonObj.HasMember("altitude_thr")) {
			LOG_WARN(" has altitude_thr field ");
			altitude_thr = jsonObj["altitude_thr"].GetFloat();
		}
		if (jsonObj.HasMember("use_hnsw_knn")) {
			LOG_WARN(" has use_hnsw_knn field ");
			use_hnsw_knn = jsonObj["use_hnsw_knn"].GetBool();
		}
		if (jsonObj.HasMember("use_feat_spatial_budget")) {
			LOG_WARN(" has use_feat_spatial_budget field ");
			use_feat_spatial_budget = jsonObj["use_feat_spatial_budget"].GetBool();
		}
		if (jsonObj.HasMember("feat_soft_limit")) {
			LOG_WARN(" has feat_soft_limit field ");
			feat_soft_limit = jsonObj["feat_soft_limit"].GetInt();
		}
		if (jsonObj.HasMember("feat_bin_voxel_gain")) {
			LOG_WARN(" has feat_bin_voxel_gain field ");
			feat_bin_voxel_gain = jsonObj["feat_bin_voxel_gain"].GetFloat();
		}
		if (jsonObj.HasMember("transformation")) {
			LOG_WARN(" has transformation field ");
			std::string const transformation = jsonObj["transformation"].GetString();
			transformation_matrix = Ddx::Common::south_string_to_matrix(transformation);
		}
		if (jsonObj.HasMember("info")) {
            LOG_WARN(" has info field ");
			info = jsonObj["info"].GetInt();
		}
		if (jsonObj.HasMember("min_feature_number")) {
			LOG_WARN(" has min_feature_number field ");
			min_feature_number = jsonObj["min_feature_number"].GetInt();
		}
		if (jsonObj.HasMember("max_station_spacing")) {
			LOG_WARN(" has max_station_spacing field ");
			max_station_spacing = jsonObj["max_station_spacing"].GetFloat();
		}
		if (jsonObj.HasMember("min_station_spacing")) {
			LOG_WARN(" has min_station_spacing field ");
			min_station_spacing = jsonObj["min_station_spacing"].GetFloat();
		}
		if (jsonObj.HasMember("number_of_iterations")) {
			LOG_WARN(" has number_of_iterations field ");
			number_of_iterations = jsonObj["number_of_iterations"].GetInt();
		}
		if (jsonObj.HasMember("close_log")) {
			LOG_WARN(" has close_log field ");
			close_log = jsonObj["close_log"].GetBool();
		}

		return true;
	}
	
	bool 
	RobGlobRegParameter::saveConfigFile(std::string const& config_file)
	{
		std::ofstream outF(config_file, std::ios::out | std::ios::binary);
		if (!outF)
			return false;

		outF << "{" << std::endl;
		outF << "\t\"scan_name\" : \"" << scan_name << "\"," << std::endl;
		outF << "\t\"result_file\" : \"" << result_file << "\"," << std::endl;
		outF << "\t\"log_file\" : \"" << log_file << "\"," << std::endl;

		outF << "\t\"voxel_size\" : " << voxel_size << "," << std::endl;
		outF << "\t\"close_log\" : " << (close_log ? "true" : "false") << "," << std::endl;
		outF << "\t\"use_voxel_sampling\" : " << (use_voxel_sampling ? "true" : "false") << "," << std::endl;
		outF << "\t\"use_quatro\" : " << (use_quatro ? "true" : "false") << "," << std::endl;
		outF << "\t\"thr_linearity\" : " << thr_linearity << "," << std::endl;
		outF << "\t\"num_max_corr\" : " << num_max_corr << "," << std::endl;
		outF << "\t\"normal_r_gain\" : " << normal_r_gain << "," << std::endl;
		outF << "\t\"fpfh_r_gain\" : " << fpfh_r_gain << "," << std::endl;
		outF << "\t\"robin_noise_bound_gain\" : " << robin_noise_bound_gain << "," << std::endl;
		outF << "\t\"solver_noise_bound_gain\" : " << solver_noise_bound_gain << "," << std::endl;
		outF << "\t\"enable_noise_bound_clamping\" : " << (enable_noise_bound_clamping ? "true" : "false") << "," << std::endl;
		outF << "\t\"use_compass\" : " << (use_compass ? "true" : "false") << "," << std::endl;
		outF << "\t\"search_knn\" : " << search_knn << "," << std::endl;
		outF << "\t\"angle_thr\" : " << angle_thr << "," << std::endl;
		outF << "\t\"use_altimeter\" : " << (use_altimeter ? "true" : "false") << "," << std::endl;
		outF << "\t\"altitude_thr\" : " << altitude_thr << "," << std::endl;
		outF << "\t\"use_hnsw_knn\" : " << (use_hnsw_knn ? "true" : "false") << "," << std::endl;
		outF << "\t\"use_feat_spatial_budget\" : " << (use_feat_spatial_budget ? "true" : "false") << "," << std::endl;
		outF << "\t\"feat_soft_limit\" : " << feat_soft_limit << "," << std::endl;
		outF << "\t\"feat_bin_voxel_gain\" : " << feat_bin_voxel_gain << "," << std::endl;
		outF << "\t\"open_iterations\" : " << (open_iterations ? "true" : "false") << "," << std::endl;
		outF << "\t\"min_feature_number\" : " << min_feature_number << "," << std::endl;
		outF << "\t\"max_station_spacing\" : " << max_station_spacing << "," << std::endl;
		outF << "\t\"min_station_spacing\" : " << min_station_spacing << "," << std::endl;
		outF << "\t\"number_of_iterations\" : " << number_of_iterations << "," << std::endl;
		std::string const transformation = Ddx::Common::matrix_to_south_string(transformation_matrix);
		outF << "\t\"transformation\" : \"" << transformation << "\"," << std::endl;
		outF << "\t\"info\" : " << info << std::endl;
		outF << "}" << std::endl;
		outF.close();
		return stlplus::file_exists(config_file);
	}
}
