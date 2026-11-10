#include "station_status_io.hpp"
#if __has_include(<rapidjson/document.h>)
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#else
#include <json/document.h>
#include <json/prettywriter.h>
#include <json/stringbuffer.h>
#endif
#include <logging/logging.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <string>

namespace Ddx
{
namespace
{
	using Json = rapidjson::Document;
	using Value = rapidjson::Value;
	using AllocatorType = rapidjson::Document::AllocatorType;

	Value getValue(std::string const& s, AllocatorType& allocator)
	{
		Value v;
		v.SetString(s.c_str(), static_cast<rapidjson::SizeType>(s.size()), allocator);
		return v;
	}

	void addNumOrNull(Value& obj, char const* key, double val, AllocatorType& allocator)
	{
		Value k(key, allocator);
		if (val < 0.0 || !std::isfinite(val)) {
			obj.AddMember(k, Value(rapidjson::kNullType), allocator);
		} else {
			obj.AddMember(k, val, allocator);
		}
	}

	bool saveStation(Value& j_io, AllocatorType& allocator, StationStatus const& st)
	{
		j_io.SetObject();
		j_io.AddMember("name", getValue(st.name_, allocator), allocator);
		j_io.AddMember("state", getValue(st.state_, allocator), allocator);
		j_io.AddMember("phase", getValue(st.phase_, allocator), allocator);
		j_io.AddMember("ref", getValue(st.ref_, allocator), allocator);
		if (!st.phase_started_at_.empty()) {
			j_io.AddMember("phase_started_at", getValue(st.phase_started_at_, allocator), allocator);
		}
		if (!st.message_.empty()) {
			j_io.AddMember("message", getValue(st.message_, allocator), allocator);
		}
		addNumOrNull(j_io, "init_total_s", st.init_total_s_, allocator);
		addNumOrNull(j_io, "reg_total_s", st.reg_total_s_, allocator);
		addNumOrNull(j_io, "rms", st.rms_, allocator);
		addNumOrNull(j_io, "similarity", st.similarity_, allocator);
		return true;
	}

	bool loadStation(Value const& j_io, StationStatus& st)
	{
		st = StationStatus();
		if (!j_io.IsObject()) {
			return false;
		}
		if (j_io.HasMember("name") && j_io["name"].IsString()) {
			st.name_ = j_io["name"].GetString();
		}
		if (j_io.HasMember("state") && j_io["state"].IsString()) {
			st.state_ = j_io["state"].GetString();
		}
		if (j_io.HasMember("phase") && j_io["phase"].IsString()) {
			st.phase_ = j_io["phase"].GetString();
		}
		if (j_io.HasMember("ref") && j_io["ref"].IsString()) {
			st.ref_ = j_io["ref"].GetString();
		}
		if (j_io.HasMember("phase_started_at") && j_io["phase_started_at"].IsString()) {
			st.phase_started_at_ = j_io["phase_started_at"].GetString();
		}
		if (j_io.HasMember("message") && j_io["message"].IsString()) {
			st.message_ = j_io["message"].GetString();
		}
		auto readNum = [&](char const* key, double& out) {
			if (!j_io.HasMember(key) || j_io[key].IsNull()) {
				out = -1.0;
				return;
			}
			if (j_io[key].IsNumber()) {
				out = j_io[key].GetDouble();
			}
		};
		readNum("init_total_s", st.init_total_s_);
		readNum("reg_total_s", st.reg_total_s_);
		readNum("rms", st.rms_);
		readNum("similarity", st.similarity_);
		return !st.name_.empty();
	}

	bool writeFile(std::string const& file_p, Json const& doc)
	{
		rapidjson::StringBuffer buffer;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
		writer.SetIndent(' ', 2);
		doc.Accept(writer);

		std::string const tmp = file_p + ".tmp";
		{
			std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
			if (!out.is_open()) {
				LOG_ERROR("StationStatusIO: open {} fail", tmp);
				return false;
			}
			out.write(buffer.GetString(), static_cast<std::streamsize>(buffer.GetSize()));
		}
#if defined(_WIN32)
		_unlink(file_p.c_str());
#else
		std::remove(file_p.c_str());
#endif
		if (std::rename(tmp.c_str(), file_p.c_str()) != 0) {
			LOG_ERROR("StationStatusIO: rename {} -> {} fail", tmp, file_p);
			return false;
		}
		return true;
	}

} // namespace

RegistrationStatusFile::RegistrationStatusFile()
	: schema_version_(1)
	, current_("")
	, updated_at_("")
{
}

std::string StationStatusIO::nowIso8601()
{
	using clock = std::chrono::system_clock;
	std::time_t const t = clock::to_time_t(clock::now());
	std::tm tm{};
#if defined(_WIN32)
	gmtime_s(&tm, &t);
#else
	gmtime_r(&t, &tm);
#endif
	char buf[32];
	std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
	return std::string(buf);
}

bool StationStatusIO::save(std::string const& file_p, RegistrationStatusFile const& doc)
{
	Json j;
	j.SetObject();
	auto& alloc = j.GetAllocator();
	j.AddMember("schema_version", doc.schema_version_ > 0 ? doc.schema_version_ : 1, alloc);
	j.AddMember("current", getValue(doc.current_, alloc), alloc);
	j.AddMember("updated_at",
		getValue(doc.updated_at_.empty() ? StationStatusIO::nowIso8601() : doc.updated_at_, alloc),
		alloc);

	Value arr(rapidjson::kArrayType);
	for (StationStatus const& st : doc.stations_) {
		Value item(rapidjson::kObjectType);
		saveStation(item, alloc, st);
		arr.PushBack(item, alloc);
	}
	j.AddMember("stations", arr, alloc);
	return writeFile(file_p, j);
}

bool StationStatusIO::load(std::string const& file_p, RegistrationStatusFile& doc)
{
	doc = RegistrationStatusFile();
	std::ifstream in(file_p, std::ios::binary);
	if (!in.is_open()) {
		return false;
	}
	std::string const text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	Json j;
	if (j.Parse(text.c_str()).HasParseError() || !j.IsObject()) {
		LOG_ERROR("StationStatusIO: parse {} fail", file_p);
		return false;
	}
	if (j.HasMember("schema_version") && j["schema_version"].IsInt()) {
		doc.schema_version_ = j["schema_version"].GetInt();
	}
	if (j.HasMember("current") && j["current"].IsString()) {
		doc.current_ = j["current"].GetString();
	}
	if (j.HasMember("updated_at") && j["updated_at"].IsString()) {
		doc.updated_at_ = j["updated_at"].GetString();
	}
	if (j.HasMember("stations") && j["stations"].IsArray()) {
		for (auto const& item : j["stations"].GetArray()) {
			StationStatus st;
			if (loadStation(item, st)) {
				doc.stations_.push_back(std::move(st));
			}
		}
	}
	return true;
}

bool StationStatusIO::upsert(std::string const& file_p, StationStatus const& st, std::string const& current)
{
	if (file_p.empty() || st.name_.empty()) {
		return false;
	}

	RegistrationStatusFile doc;
	load(file_p, doc);	// 文件不存在时保持空文档

	doc.schema_version_ = 1;
	doc.current_ = current;
	doc.updated_at_ = StationStatusIO::nowIso8601();

	bool found = false;
	for (StationStatus& existing : doc.stations_) {
		if (existing.name_ == st.name_) {
			existing = st;
			found = true;
			break;
		}
	}
	if (!found) {
		doc.stations_.push_back(st);
	}
	return save(file_p, doc);
}

} // namespace Ddx
