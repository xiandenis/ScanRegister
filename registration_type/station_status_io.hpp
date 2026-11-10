#pragma once
#include "registration_type.hpp"
#include <string>
#include <vector>

namespace Ddx
{
	/** registration_status.json 整文件内容（web_view_manager 轮询） */
	struct RegistrationStatusFile
	{
		int schema_version_ = 1;
		std::string current_;
		std::string updated_at_;
		std::vector<StationStatus> stations_;

		RegistrationStatusFile();
	};

	class StationStatusIO
	{
	public:
		static std::string nowIso8601();

		static bool save(std::string const& file_p, RegistrationStatusFile const& doc);
		static bool load(std::string const& file_p, RegistrationStatusFile& doc);

		/** 读入已有文件（可无）→ 按站名 upsert → 写回；updated_at 自动刷新 */
		static bool upsert(std::string const& file_p, StationStatus const& st, std::string const& current);
	};
}