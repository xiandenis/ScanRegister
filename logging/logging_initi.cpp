
#include "logging_initi.hpp"
#include "logging.hpp"


namespace Ddx {

	void initiLog(const std::string& log_file_path, const std::string& name, int level)
	{
		LogManager::init(log_file_path, name, spdlog::level::level_enum(level));
	}

	void  resetLog()
	{
		LogManager::reset();
	}

	
	void permanentlyCloseLog()
	{
		LogManager::cleanup();
	}
}