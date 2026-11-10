#include "align_result_io.hpp"
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


/*************************   基础方法     *****************************/
namespace Ddx
{
	class BasisAlignResultIO
	{
	public:
		typedef rapidjson::Document Json;
		typedef rapidjson::Value Value;
		typedef rapidjson::Document::AllocatorType AllocatorType;


		static bool saveAlignResult(std::string const& file_p, AlignResult const& info);
		static bool loadAlignResult(std::string const& file_p, AlignResult& info);

		static bool saveAlignResult(std::string const& file_p, std::vector<AlignResult> const& info_vec);
		static bool loadAlignResult(std::string const& file_p, std::vector<AlignResult>& info_vec);

		static bool saveAlignResult(Value& j_io, AllocatorType& allocator, AlignResult const& info);
		static bool loadAlignResult(Value& j_io, AlignResult& info);

		static bool saveAlignResult(Value& j_io, AllocatorType& allocator, std::vector<AlignResult> const& info_vec);
		static bool loadAlignResult(Value& j_io, std::vector<AlignResult>& info_vec);

		static Value getValue(std::string const& string, AllocatorType& allocator);

	};

	bool
	BasisAlignResultIO::saveAlignResult(std::string const& file_p, AlignResult const& info)
	{
		Json doc;
		doc.SetObject();
		saveAlignResult(doc, doc.GetAllocator(), info);

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
	BasisAlignResultIO::loadAlignResult(std::string const& file_p, AlignResult& info)
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

		if (!loadAlignResult(doc, info)) {
			LOG_ERROR(" parsing error ");
			f.close();
			return false;
		}
		f.close();
		return true;
	}


	bool
	BasisAlignResultIO::saveAlignResult(std::string const& file_p, std::vector<AlignResult> const& info_vec)
	{
		Json doc;
		doc.SetObject();
		saveAlignResult(doc, doc.GetAllocator(), info_vec);

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
	BasisAlignResultIO::loadAlignResult(std::string const& file_p, std::vector<AlignResult>& info_vec)
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
			f.close();
			LOG_ERROR("Failed to parse JSON file, please check if JSON is correct! ");
			return false;
		}

		if (!loadAlignResult(doc, info_vec)) {
			LOG_ERROR(" parsing error ");
			f.close();
			return false;
		}
		f.close();
		return true;
	}

	bool
	BasisAlignResultIO::saveAlignResult(Value& j_io, AllocatorType& allocator, AlignResult const& info)
	{
		j_io.AddMember("state", info.state_, allocator);
		j_io.AddMember("src_name", getValue(info.src_name_, allocator), allocator);
		j_io.AddMember("tgt_name", getValue(info.tgt_name_, allocator), allocator);
		j_io.AddMember("auto_reg", info.auto_reg_, allocator);
		j_io.AddMember("feature_num", info.feature_num_, allocator);
		j_io.AddMember("similarity", info.similarity_, allocator);
		j_io.AddMember("RMS", info.rms_, allocator);
		j_io.AddMember("final_rms", info.final_rms_, allocator);
		j_io.AddMember("weight", info.weight_, allocator);
		j_io.AddMember("matrix", getValue(Ddx::Common::matrix_to_south_string(info.matrix_), allocator), allocator);
		{
			Value info_arr(rapidjson::kArrayType);
			for (int r = 0; r < 6; ++r) {
				for (int c = 0; c < 6; ++c) {
					info_arr.PushBack(info.information_(r, c), allocator);
				}
			}
			j_io.AddMember("information", info_arr, allocator);
		}
		return true;
	}

	bool
	BasisAlignResultIO::loadAlignResult(Value& j_io, AlignResult& info)
	{
		if (!j_io.HasMember("state")) {
			LOG_ERROR(" AlignResult has not state field");
			return false;
		}
		if (!j_io.HasMember("src_name")) {
			LOG_ERROR(" AlignResult has not src_name field");
			return false;
		}
		if (!j_io.HasMember("tgt_name")) {
			LOG_ERROR(" AlignResult has not tgt_name field");
			return false;
		}
		if (!j_io.HasMember("auto_reg")) {
			LOG_ERROR(" AlignResult has not auto_reg field");
			return false;
		}
		if (!j_io.HasMember("feature_num")) {
			LOG_ERROR(" AlignResult has not feature_num field");
			return false;
		}
		if (!j_io.HasMember("similarity")) {
			LOG_ERROR(" AlignResult has not similarity field");
			return false;
		}
		if (!j_io.HasMember("RMS")) {
			LOG_ERROR(" AlignResult has not RMS field");
			return false;
		}
		if (!j_io.HasMember("final_rms")) {
			LOG_ERROR(" AlignResult has not final_rms field");
			return false;
		}
		if (!j_io.HasMember("weight")) {
			LOG_ERROR(" AlignResult has not weight field");
			return false;
		}
		if (!j_io.HasMember("matrix")) {
			LOG_ERROR(" AlignResult has not matrix field");
			return false;
		}

		info.state_ = j_io["state"].GetInt();
		info.src_name_ = j_io["src_name"].GetString();
		info.tgt_name_ = j_io["tgt_name"].GetString();
		info.auto_reg_ = j_io["auto_reg"].GetInt();
		info.feature_num_ = j_io["feature_num"].GetInt();
		info.similarity_ = j_io["similarity"].GetDouble();
		info.rms_ = j_io["RMS"].GetDouble();
		info.final_rms_= j_io["final_rms"].GetDouble();
		info.weight_ = j_io["weight"].GetDouble();
		info.matrix_ = Common::south_string_to_matrix(j_io["matrix"].GetString());
		info.information_ = Eigen::Matrix<double, 6, 6>::Identity();
		if (j_io.HasMember("information") && j_io["information"].IsArray()
			&& j_io["information"].Size() == 36) {
			Value const& arr = j_io["information"];
			for (int r = 0; r < 6; ++r) {
				for (int c = 0; c < 6; ++c) {
					Value const& v = arr[static_cast<rapidjson::SizeType>(r * 6 + c)];
					if (!v.IsNumber()) {
						LOG_ERROR(" AlignResult information entry not numeric");
						return false;
					}
					info.information_(r, c) = v.GetDouble();
				}
			}
		}
		return true;
	}

	bool
	BasisAlignResultIO::saveAlignResult(Value& j_io, AllocatorType& allocator, std::vector<AlignResult> const& info_vec)
	{
		Value hobbies(rapidjson::kArrayType);
		for (size_t i = 0; i < info_vec.size(); ++i)
		{
			Value info(rapidjson::kObjectType);
			saveAlignResult(info, allocator, info_vec[i]);
			hobbies.PushBack(info, allocator);
		}
		j_io.AddMember("array", hobbies, allocator);
		return true;
	}

	bool
	BasisAlignResultIO::loadAlignResult(Value& j_io, std::vector<AlignResult>& info_vec)
	{
		if (!j_io.HasMember("array")) {
			LOG_ERROR(" json has not array field");
			return false;
		}

		Value& hobbies = j_io["array"];
		info_vec.clear();
		info_vec.resize(hobbies.Size());
		size_t offset = 0;
		for (auto& v : hobbies.GetArray()) {
			if (!loadAlignResult(v, info_vec[offset])) {
				return false;
			}
			++offset;
		}
		return true;
	}


	BasisAlignResultIO::Value
	BasisAlignResultIO::getValue(std::string const& string, AllocatorType& allocator)
	{
		Value v;
		v.SetString(string.c_str(), allocator);
		return v;
	}
}

namespace Ddx
{
	bool
	AlignResultIO::saveAlignResult(std::string const& file_p, AlignResult const& info)
	{
		return BasisAlignResultIO::saveAlignResult(file_p, info);
	}

	bool
	AlignResultIO::loadAlignResult(std::string const& file_p, AlignResult& info)
	{
		return BasisAlignResultIO::loadAlignResult(file_p, info);
	}

	bool
	AlignResultIO::saveAlignResult(std::string const& file_p, std::vector<AlignResult> const& info_vec)
	{
		return BasisAlignResultIO::saveAlignResult(file_p, info_vec);
	}

	bool
	AlignResultIO::loadAlignResult(std::string const& file_p, std::vector<AlignResult>& info_vec)
	{
		return BasisAlignResultIO::loadAlignResult(file_p, info_vec);
	}
} // namespace Ddx
