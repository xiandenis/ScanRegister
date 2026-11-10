#include "lidar_frame_io.hpp"
#if __has_include(<rapidjson/document.h>)
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#else
#include <json/document.h>
#include <json/prettywriter.h>
#include <json/stringbuffer.h>
#endif
#include <common/common.hpp>
#include <logging/logging.hpp>
#include <fstream>

/*****************************    基础方法        ******************************/
namespace Ddx
{
	class BasisLidarFrameIO
	{
	public:
		typedef rapidjson::Document Json;
		typedef rapidjson::Value Value;
		typedef rapidjson::Document::AllocatorType AllocatorType;

		static bool saveLidarFrame(std::string const& file_p, LidarFrame const& frame);
		static bool loadLidarFrame(std::string const& file_p, LidarFrame& frame);

		static bool saveLidarFrame(std::string const& file_p, std::vector<LidarFrame> const& frame_vec);
		static bool loadLidarFrame(std::string const& file_p, std::vector<LidarFrame>& frame_vec);

		static bool saveLidarFrame(Value& j_io, AllocatorType& allocator, LidarFrame const& frame);
		static bool loadLidarFrame(Value& j_io, LidarFrame& frame);

		static bool saveLidarFrame(Value& j_io, AllocatorType& allocator, std::vector<LidarFrame> const& frame_vec);
		static bool loadLidarFrame(Value& j_io, std::vector<LidarFrame>& frame_vec);

		static bool saveVoxel(Value& j_io, AllocatorType& allocator, SubVoxel const& voxel);
		static bool loadVoxel(Value& j_io, SubVoxel& voxel);

		static bool saveVoxelRoot(Value& j_io, AllocatorType& allocator, VoxelRoot const& root);
		static bool loadVoxelRoot(Value& j_io, VoxelRoot& root);
		static void migrateLegacyRoot(LidarFrame& frame);

		static Value getValue(std::string const& string, AllocatorType& allocator);
	};
	
	bool
	BasisLidarFrameIO::saveLidarFrame(std::string const& file_p, LidarFrame const& frame)
	{
		Json doc;
		doc.SetObject();
		saveLidarFrame(doc, doc.GetAllocator(), frame);

		rapidjson::StringBuffer buffer;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
		writer.SetIndent(' ', 4);
		doc.Accept(writer);

		std::ofstream file(file_p, std::ios::binary | std::ios::out);
		if (!file.is_open()) {
			return false;
		}
		file << buffer.GetString();
		return true;
	}

	bool
	BasisLidarFrameIO::loadLidarFrame(std::string const& file_p, LidarFrame& frame)
	{
		std::ifstream f(file_p, std::ios::binary | std::ios::in);
		if (!f.is_open()) {
			LOG_ERROR(" open file({}) fail", file_p);
			return false;
		}

		std::string strJson((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
		Json doc;
		doc.Parse(strJson.c_str());
		if (doc.HasParseError()) {
			LOG_ERROR("Failed to parse JSON file, please check if JSON is correct! ");
			f.close();
			return false;
		}

		if (!loadLidarFrame(doc, frame)) {
			LOG_ERROR(" parsing error ");
			f.close();
			return false;
		}

		f.close();
		return true;
	}

	bool
	BasisLidarFrameIO::saveLidarFrame(std::string const& file_p, std::vector<LidarFrame> const& frame_vec)
	{
		Json doc;
		doc.SetObject();
		saveLidarFrame(doc, doc.GetAllocator(), frame_vec);

		rapidjson::StringBuffer buffer;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
		writer.SetIndent(' ', 4);
		doc.Accept(writer);

		std::ofstream file(file_p, std::ios::binary | std::ios::out);
		if (!file.is_open()) {
			return false;
		}
		file << buffer.GetString();
		return true;
	}

	bool
	BasisLidarFrameIO::loadLidarFrame(std::string const& file_p, std::vector<LidarFrame>& frame_vec)
	{
		std::ifstream f(file_p, std::ios::binary | std::ios::in);
		if (!f.is_open()) {
			LOG_ERROR(" open file({}) fail", file_p);
			return false;
		}

		std::string strJson((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
		Json doc;
		doc.Parse(strJson.c_str());
		if (doc.HasParseError()) {
			LOG_ERROR("Failed to parse JSON file, please check if JSON is correct!");
			f.close();
			return false;
		}

		if (!loadLidarFrame(doc, frame_vec)) {
			LOG_ERROR(" parsing error ");
			f.close();
			return false;
		}
		f.close();
		return true;
	}

	bool
	BasisLidarFrameIO::saveLidarFrame(Value& j_io, AllocatorType& allocator, LidarFrame const& frame)
	{
		j_io.AddMember("state", frame.state_, allocator);
		j_io.AddMember("name", getValue(frame.name_, allocator), allocator);
		j_io.AddMember("stamp", frame.stamp_, allocator);
		j_io.AddMember("lasFn", getValue(frame.lasFn_, allocator), allocator);
		j_io.AddMember("normalFn", getValue(frame.normalFn_, allocator), allocator);
		j_io.AddMember("info_json", getValue(frame.json_, allocator), allocator);
		j_io.AddMember("info_folder", getValue(frame.info_folder_, allocator), allocator);
		j_io.AddMember("sensor", frame.sensor_, allocator);
		j_io.AddMember("altimeter", frame.altimeter_, allocator);
		j_io.AddMember("angle_resolution", frame.angle_resolution_, allocator);
		j_io.AddMember("gps", getValue(Common::vector3d_to_string(frame.gps_), allocator), allocator);
		j_io.AddMember("global_matrix", getValue(Common::matrix_to_south_string(frame.global_matrix_), allocator), allocator);
		// 站心与 Root/子体素站心系缓存同生命周期；丢站心则 M 无法折回文件系。
		j_io.AddMember("has_station_origin", frame.has_station_origin_, allocator);
		j_io.AddMember(
			"station_origin",
			getValue(Common::vector3d_to_string(frame.station_origin_), allocator),
			allocator);
		j_io.AddMember("point_number", frame.point_number_, allocator);
		j_io.AddMember("scene_scale", frame.scene_scale_, allocator);
		j_io.AddMember("voxel_size", frame.voxel_size_, allocator);
		j_io.AddMember(
			"voxel_scene_class",
			getValue(std::string(voxelSceneClassToString(frame.voxel_scene_class_)), allocator),
			allocator);

		Value root_json(rapidjson::kObjectType);
		saveVoxelRoot(root_json, allocator, frame.root_);
		j_io.AddMember("voxel_root", root_json, allocator);

		Value hobbies(rapidjson::kArrayType);
		for (size_t i = 0; i < frame.subvoxel_vec_.size(); ++i) {
			Value voxel(rapidjson::kObjectType);
			saveVoxel(voxel, allocator, frame.subvoxel_vec_[i]);
			hobbies.PushBack(voxel, allocator);
		}
		j_io.AddMember("sub_voxel", hobbies, allocator);
		return true;
	}

	bool
	BasisLidarFrameIO::loadLidarFrame(Value& j_io, LidarFrame& frame)
	{
		if (!j_io.HasMember("state")) {
			LOG_ERROR(" LidarFrame has not state field");
			return false;
		}
		if (!j_io.HasMember("name")) {
			LOG_ERROR(" LidarFrame has not name field");
			return false;
		}
		if (!j_io.HasMember("stamp")) {
			LOG_ERROR(" LidarFrame has not stamp field");
			return false;
		}
		if (!j_io.HasMember("lasFn")) {
			LOG_ERROR(" LidarFrame has not lasFn field");
			return false;
		}
		if (!j_io.HasMember("normalFn")) {
			LOG_ERROR(" LidarFrame has not normalFn field");
			return false;
		}
		if (!j_io.HasMember("info_json")) {
			LOG_ERROR(" LidarFrame has not info_json field");
			return false;
		}
		if (!j_io.HasMember("info_folder")) {
			LOG_ERROR(" LidarFrame has not info_folder field");
			return false;
		}
		if (!j_io.HasMember("sensor")) {
			LOG_ERROR(" LidarFrame has not sensor field");
			return false;
		}
		if (!j_io.HasMember("altimeter")) {
			LOG_ERROR(" LidarFrame has not altimeter field");
			return false;
		}
		if (!j_io.HasMember("angle_resolution")) {
			LOG_ERROR(" LidarFrame has not angle_resolution field");
			return false;
		}
		if (!j_io.HasMember("gps")) {
			LOG_ERROR(" LidarFrame has not gps field");
			return false;
		}
		if (!j_io.HasMember("global_matrix")) {
			LOG_ERROR(" LidarFrame has not global_matrix field");
			return false;
		}
		if (!j_io.HasMember("point_number")) {
			LOG_ERROR(" LidarFrame has not point_number field");
			return false;
		}
		if (!j_io.HasMember("scene_scale")) {
			LOG_ERROR(" LidarFrame has not scene_scale field");
			return false;
		}
		if (!j_io.HasMember("voxel_size")) {
			LOG_ERROR(" LidarFrame has not voxel_size field");
			return false;
		}
		if (!j_io.HasMember("sub_voxel")) {
			LOG_ERROR(" LidarFrame has not sub_voxel field");
			return false;
		}

		frame.state_ = j_io["state"].GetInt();
		frame.name_ = j_io["name"].GetString();
		frame.stamp_ = j_io["stamp"].GetDouble();
		frame.lasFn_ = j_io["lasFn"].GetString();
		frame.normalFn_ = j_io["normalFn"].GetString();
		frame.json_ = j_io["info_json"].GetString();
		frame.info_folder_ = j_io["info_folder"].GetString();
		frame.sensor_ = j_io["sensor"].GetInt();
		frame.altimeter_ = j_io["altimeter"].GetDouble();
		frame.angle_resolution_ = j_io["angle_resolution"].GetInt();
		frame.gps_ = Common::string_to_vector3d(j_io["gps"].GetString());
		frame.global_matrix_ = Common::south_string_to_matrix(j_io["global_matrix"].GetString());
		// 旧 JSON 缺站心字段 → 未声明（安全默认）；须由用户补声明后再 init。
		if (j_io.HasMember("has_station_origin") && j_io["has_station_origin"].IsBool()) {
			frame.has_station_origin_ = j_io["has_station_origin"].GetBool();
		} else {
			frame.has_station_origin_ = false;
		}
		if (j_io.HasMember("station_origin") && j_io["station_origin"].IsString()) {
			frame.station_origin_ = Common::string_to_vector3d(j_io["station_origin"].GetString());
		} else {
			frame.station_origin_ = Eigen::Vector3d::Zero();
		}
		frame.point_number_ = j_io["point_number"].GetInt();
		frame.scene_scale_ = j_io["scene_scale"].GetDouble();
		frame.voxel_size_ = j_io["voxel_size"].GetDouble();
		if (j_io.HasMember("voxel_scene_class") && j_io["voxel_scene_class"].IsString()) {
			frame.voxel_scene_class_ =
				voxelSceneClassFromString(j_io["voxel_scene_class"].GetString());
		} else {
			frame.voxel_scene_class_ = VoxelSceneClass::Unknown;
		}
		frame.root_ = VoxelRoot();

		Value& voxel_json = j_io["sub_voxel"];
		frame.subvoxel_vec_.clear();
		frame.subvoxel_vec_.reserve(voxel_json.Size());

		for (auto& v : voxel_json.GetArray()) {
			SubVoxel voxel;
			if (!loadVoxel(v, voxel)) {
				return false;
			}
			frame.subvoxel_vec_.push_back(voxel);
		}

		if (j_io.HasMember("voxel_root") && j_io["voxel_root"].IsObject()) {
			if (!loadVoxelRoot(j_io["voxel_root"], frame.root_)) {
				return false;
			}
		}
		else {
			migrateLegacyRoot(frame);
		}

		return true;
	}

	bool
	BasisLidarFrameIO::saveLidarFrame(Value& j_io, AllocatorType& allocator, std::vector<LidarFrame> const& frame_vec)
	{
		Value hobbies(rapidjson::kArrayType);
		for (size_t i = 0; i < frame_vec.size(); ++i)
		{
			Value frame(rapidjson::kObjectType);
			saveLidarFrame(frame, allocator, frame_vec[i]);
			hobbies.PushBack(frame, allocator);
		}
		j_io.AddMember("array", hobbies, allocator);
		return true;
	}

	bool
	BasisLidarFrameIO::loadLidarFrame(Value& j_io, std::vector<LidarFrame>& frame_vec)
	{
		if (!j_io.HasMember("array")) {
			LOG_ERROR(" json has not array field");
			return false;
		}

		Value& hobbies = j_io["array"];
		frame_vec.clear();
		frame_vec.resize(hobbies.Size());
		size_t offset = 0;
		for (auto& v : hobbies.GetArray()) {
			if (!loadLidarFrame(v, frame_vec[offset])) {
				return false;
			}
			++offset;
		}
		return true;
	}

	bool
	BasisLidarFrameIO::saveVoxel(Value& j_io, AllocatorType& allocator, SubVoxel const& voxel)
	{
		j_io.AddMember("name", getValue(voxel.name_, allocator), allocator);
		j_io.AddMember("point_number", voxel.point_number_, allocator);
		j_io.AddMember("voxel_size", voxel.voxel_size_, allocator);
		j_io.AddMember("state", voxel.state_, allocator);
		return true;
	}

	bool
	BasisLidarFrameIO::loadVoxel(Value& j_io, SubVoxel& voxel)
	{
		if (!j_io.HasMember("name")) {
			LOG_ERROR(" voxel has not name field");
			return false;
		}
		if (!j_io.HasMember("point_number")) {
			LOG_ERROR(" voxel has not point_number field");
			return false;
		}
		if (!j_io.HasMember("voxel_size")) {
			LOG_ERROR(" voxel has not voxel_size field");
			return false;
		}
		if (!j_io.HasMember("state")) {
			LOG_ERROR(" voxel has not state field");
			return false;
		}

		voxel.name_ = j_io["name"].GetString();
		voxel.point_number_ = j_io["point_number"].GetInt();
		voxel.voxel_size_ = j_io["voxel_size"].GetDouble();
		voxel.state_ = j_io["state"].GetInt();
		return true;
	}

	bool
	BasisLidarFrameIO::saveVoxelRoot(Value& j_io, AllocatorType& allocator, VoxelRoot const& root)
	{
		j_io.AddMember("name", getValue(root.name_, allocator), allocator);
		j_io.AddMember("cloud_path", getValue(root.cloud_path_, allocator), allocator);
		j_io.AddMember("normal_path", getValue(root.normal_path_, allocator), allocator);
		j_io.AddMember("state", root.state_, allocator);
		j_io.AddMember("point_number", root.point_number_, allocator);
		j_io.AddMember("is_raw_input", root.is_raw_input_, allocator);
		return true;
	}

	bool
	BasisLidarFrameIO::loadVoxelRoot(Value& j_io, VoxelRoot& root)
	{
		if (!j_io.HasMember("cloud_path")) {
			LOG_ERROR(" voxel_root has not cloud_path field");
			return false;
		}
		if (!j_io.HasMember("normal_path")) {
			LOG_ERROR(" voxel_root has not normal_path field");
			return false;
		}
		if (!j_io.HasMember("state")) {
			LOG_ERROR(" voxel_root has not state field");
			return false;
		}
		if (!j_io.HasMember("point_number")) {
			LOG_ERROR(" voxel_root has not point_number field");
			return false;
		}
		if (!j_io.HasMember("is_raw_input")) {
			LOG_ERROR(" voxel_root has not is_raw_input field");
			return false;
		}

		root.cloud_path_ = j_io["cloud_path"].GetString();
		root.normal_path_ = j_io["normal_path"].GetString();
		root.state_ = j_io["state"].GetInt();
		root.point_number_ = j_io["point_number"].GetInt();
		root.is_raw_input_ = j_io["is_raw_input"].GetBool();
		if (j_io.HasMember("name")) {
			root.name_ = j_io["name"].GetString();
		}
		else {
			root.name_ = "Root";
		}
		return true;
	}

	void
	BasisLidarFrameIO::migrateLegacyRoot(LidarFrame& frame)
	{
		for (auto it = frame.subvoxel_vec_.begin(); it != frame.subvoxel_vec_.end(); ) {
			if (it->name_ != "Root") {
				++it;
				continue;
			}
			frame.root_.name_ = it->name_;
			frame.root_.state_ = it->state_;
			frame.root_.point_number_ = it->point_number_;
			frame.root_.is_raw_input_ = false;
			if (frame.root_.cloud_path_.empty()) {
				frame.root_.cloud_path_ = frame.info_folder_ + "/Root_p.pcd";
			}
			if (frame.root_.normal_path_.empty()) {
				frame.root_.normal_path_ = frame.info_folder_ + "/Root_n.pcd";
			}
			it = frame.subvoxel_vec_.erase(it);
		}
	}

	BasisLidarFrameIO::Value
	BasisLidarFrameIO::getValue(std::string const& string, AllocatorType& allocator)
	{
		Value v;
		v.SetString(string.c_str(), allocator);
		return v;
	}
}


namespace Ddx
{
	bool
	LidarFrameIO::saveLidarFrame(std::string const& file_p, LidarFrame const& frame)
	{
		return BasisLidarFrameIO::saveLidarFrame(file_p, frame);
	}

	bool
	LidarFrameIO::loadLidarFrame(std::string const& file_p, LidarFrame& frame)
	{
		return BasisLidarFrameIO::loadLidarFrame(file_p, frame);
	}

	bool
	LidarFrameIO::saveLidarFrame(std::string const& file_p, std::vector<LidarFrame> const& frame_vec)
	{
		return BasisLidarFrameIO::saveLidarFrame(file_p, frame_vec);
	}

	bool
	LidarFrameIO::loadLidarFrame(std::string const& file_p, std::vector<LidarFrame>& frame_vec)
	{
		return BasisLidarFrameIO::loadLidarFrame(file_p, frame_vec);
	}
} // namespace Ddx


