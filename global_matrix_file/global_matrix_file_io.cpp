#include "global_matrix_file_io.hpp"
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

namespace Ddx
{
	class BaseMatrixFileIO
	{
	public:
		typedef rapidjson::Document Json;
		typedef rapidjson::Value Value;
		typedef rapidjson::Document::AllocatorType AllocatorType;

		static bool loading(std::string const& matrix_file_path, std::map<std::string, Eigen::Matrix4d>& site_global_matrix);
		static bool saving(std::string const& matrix_file_path, std::map<std::string, Eigen::Matrix4d> const& site_global_matrix);

		static bool loadMatrix(Value& j_io, std::string& name, Eigen::Matrix4d& transformation);
		static bool saveMatrix(Value& j_io, AllocatorType& allocator, std::string const& name, Eigen::Matrix4d const& transformation);

		static bool loadMatrix(Value& j_io, std::map<std::string, Eigen::Matrix4d>& site_global_matrix);
		static bool saveMatrix(Value& j_io, AllocatorType& allocator, std::map<std::string, Eigen::Matrix4d> const& site_global_matrix);

		static Value getValue(std::string const& string, AllocatorType& allocator);

		static bool writeMatrix(std::string const& m_file, Eigen::Matrix4d const& matrix);
		static bool readMatrix(std::string const& m_file, Eigen::Matrix4d& matrix);

	};

	bool
	BaseMatrixFileIO::loading(std::string const& matrix_file_path, std::map<std::string, Eigen::Matrix4d>& site_global_matrix)
	{
		std::ifstream f(matrix_file_path, std::ios::binary | std::ios::in);
		if (!f.is_open()) {
			LOG_ERROR(" open file({}) fail", matrix_file_path);
			return false;
		}

		std::string strJson((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
		Json doc;
		doc.Parse(strJson.c_str());
		if (doc.HasParseError()) {
			LOG_ERROR("Failed to parse JSON file, please check if JSON is correct! ");
			return false;
		}

		if (!loadMatrix(doc, site_global_matrix)) {
			LOG_ERROR(" parsing error ");
			return false;
		}
		return true;
	}

	bool
	BaseMatrixFileIO::saving(std::string const& matrix_file_path, std::map<std::string, Eigen::Matrix4d> const& site_global_matrix)
	{
		Json doc;
		doc.SetObject();
		saveMatrix(doc, doc.GetAllocator(), site_global_matrix);
		doc.AddMember("version", 1, doc.GetAllocator());

		rapidjson::StringBuffer buffer;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
		writer.SetIndent(' ', 4);
		doc.Accept(writer);

		std::ofstream file(matrix_file_path, std::ios::binary | std::ios::out);
		if (!file.is_open()) {
			LOG_ERROR(" open file({}) fail", matrix_file_path);
			return false;
		}
		file << buffer.GetString();
		return true;
	}

	bool
	BaseMatrixFileIO::loadMatrix(Value& j_io, std::string& name, Eigen::Matrix4d& transformation)
	{
		if (!j_io.IsObject()) {
			LOG_ERROR(" json Transformation element is not object");
			return false;
		}
		if (!j_io.HasMember("name") || !j_io["name"].IsString()) {
			LOG_ERROR(" json has not name field");
			return false;
		}
		if (!j_io.HasMember("rotation") || !j_io["rotation"].IsString()) {
			LOG_ERROR(" json has not rotation field");
			return false;
		}
		if (!j_io.HasMember("translation") || !j_io["translation"].IsString()) {
			LOG_ERROR(" json has not translation field");
			return false;
		}

		name = j_io["name"].GetString();
		Eigen::Matrix3d rotation = Common::string_to_matrix33(j_io["rotation"].GetString());
		Eigen::Vector3d translation = Common::string_to_vector3d(j_io["translation"].GetString());

		transformation = Eigen::Matrix4d::Identity();
		transformation.block<3, 3>(0, 0) = rotation;
		transformation.block<3, 1>(0, 3) = translation;
		return true;
	}

	bool
	BaseMatrixFileIO::saveMatrix(Value& j_io, AllocatorType& allocator, std::string const& name, Eigen::Matrix4d const& transformation)
	{
		Eigen::Matrix3d rotation = transformation.block<3, 3>(0, 0);
		Eigen::Vector3d translation = transformation.block<3, 1>(0, 3);
		j_io.AddMember("name", getValue(name, allocator), allocator);
		j_io.AddMember("rotation", getValue(Common::matrix33_to_string(rotation), allocator), allocator);
		j_io.AddMember("translation", getValue(Common::vector3d_to_string(translation), allocator), allocator);
		return true;
	}

	bool
	BaseMatrixFileIO::loadMatrix(Value& j_io, std::map<std::string, Eigen::Matrix4d>& site_global_matrix)
	{
		if (!j_io.HasMember("Transformation") || !j_io["Transformation"].IsArray()) {
			LOG_ERROR(" json has not Transformation field");
			return false;
		}
		site_global_matrix.clear();
		Value& hobbies = j_io["Transformation"];
		for (auto& v : hobbies.GetArray()) {
			std::string name;
			Eigen::Matrix4d transformation;
			if (!loadMatrix(v, name, transformation)) {
				return false;
			}
			site_global_matrix.emplace(std::make_pair(name, transformation));
		}
		return true;
	}

	bool
	BaseMatrixFileIO::saveMatrix(Value& j_io, AllocatorType& allocator, std::map<std::string, Eigen::Matrix4d> const& site_global_matrix)
	{
		Value hobbies(rapidjson::kArrayType);
		for (auto const& iter : site_global_matrix) {
			Value sub_node(rapidjson::kObjectType);
			saveMatrix(sub_node, allocator, iter.first, iter.second);
			hobbies.PushBack(sub_node, allocator);
		}
		j_io.AddMember("Transformation", hobbies, allocator);
		return true;
	}

	BaseMatrixFileIO::Value
	BaseMatrixFileIO::getValue(std::string const& string, AllocatorType& allocator)
	{
		Value v;
		v.SetString(string.c_str(), static_cast<rapidjson::SizeType>(string.size()), allocator);
		return v;
	}

	bool
	BaseMatrixFileIO::writeMatrix(std::string const& m_file, Eigen::Matrix4d const& matrix)
	{
		Json doc;
		doc.SetObject();
		doc.AddMember("transformation", getValue(Ddx::Common::matrix_to_south_string(matrix), doc.GetAllocator()), doc.GetAllocator());
		rapidjson::StringBuffer buffer;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
		writer.SetIndent(' ', 4);
		doc.Accept(writer);

		std::ofstream file(m_file, std::ios::binary | std::ios::out);
		if (!file.is_open()) {
			LOG_ERROR(" open file({}) fail", m_file);
			return false;
		}
		file << buffer.GetString();
		return true;
	}

	bool
	BaseMatrixFileIO::readMatrix(std::string const& m_file, Eigen::Matrix4d& matrix)
	{
		std::ifstream f(m_file, std::ios::binary | std::ios::in);
		if (!f.is_open()) {
			LOG_ERROR(" open file({}) fail", m_file);
			return false;
		}

		std::string strJson((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
		Json doc;
		doc.Parse(strJson.c_str());
		if (doc.HasParseError()) {
			LOG_ERROR("Failed to parse JSON file, please check if JSON is correct! ");
			return false;
		}

		if (!doc.HasMember("transformation") || !doc["transformation"].IsString()) {
			LOG_ERROR(" json has not transformation field");
			return false;
		}

		matrix = Common::south_string_to_matrix(doc["transformation"].GetString());

		return true;
	}
}

namespace Ddx
{
	bool
	GlobalMatrixFileIO::loading(std::string const& matrix_file_path, std::map<std::string, Eigen::Matrix4d>& site_global_matrix)
	{
		return BaseMatrixFileIO::loading(matrix_file_path, site_global_matrix);
	}

	bool
	GlobalMatrixFileIO::saving(std::string const& matrix_file_path, std::map<std::string, Eigen::Matrix4d> const& site_global_matrix)
	{
		return BaseMatrixFileIO::saving(matrix_file_path, site_global_matrix);
	}
}
