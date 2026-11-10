
#pragma once
#ifdef _MSC_VER //MSVC
#ifndef LOGGING_LOG_EXPORT
#ifndef LOGGING_LOG_API
#define LOGGING_LOG_API __declspec (dllimport)
#endif
#else
#ifndef LOGGING_LOG_API
#define LOGGING_LOG_API __declspec (dllexport)
#endif
#endif
#define SOUTHABI_IMPORT __declspec (dllimport)
#define SOUTHABI_EXPORT __declspec (dllexport)
#define SOUTHABI_API __stdcall
#define SOUTHABI_NATIVE extern "C"
#elif defined __GNUC__ || defined __clang__
#ifdef LOGGING_LOG_EXPORT
#ifndef LOGGING_LOG_API
#define LOGGING_LOG_API __attribute__((visibility("default")))
#endif
#else
#ifndef LOGGING_LOG_API
#define LOGGING_LOG_API
#endif
#endif
#define COARSE_REG_T2_API
#define SOUTHABI_IMPORT
#define SOUTHABI_EXPORT __attribute__ ((visibility ("default")))
#define SOUTHABI_API
#define SOUTHABI_NATIVE extern "C"
#endif

#include <spdlog/spdlog.h>
#include <memory>
#include <string>
#include <mutex>


namespace Ddx {
	class LOGGING_LOG_API LogManager {
	public:
		LogManager(const LogManager&) = delete;
		LogManager& operator=(const LogManager&) = delete;

		/** @brief Initialize a plain file logger (supports re-initialization). */
		static void init(const std::string& log_file_path, const std::string& name,
			spdlog::level::level_enum level = spdlog::level::info,
			bool force_reset = false);

		/** @brief Initialize a daily-rotating file logger (supports re-initialization). */
		static void init_daily(const std::string& log_folder, const std::string& name,
			spdlog::level::level_enum level = spdlog::level::info,
			bool force_reset = false);

		/** @brief Get the logger instance. */
		static std::shared_ptr<spdlog::logger>& get_logger();

		/** @brief Reset the logger (clear the old one; init may be called again). */
		static void reset();

		/** @brief Permanently shut down logging (call before program exit). */
		static void cleanup();

		/** @brief Change log level without re-initialization. */
		static void set_level(spdlog::level::level_enum level);

	private:
		LogManager() = default;
		~LogManager() = default;

		static LogManager& instance();

		// Internal helper: clear the old logger without permanent shutdown.
		static void clear_old_logger_();

		static void reset_spdlog_to_default();

		std::shared_ptr<spdlog::logger> logger_ = nullptr;
		std::mutex mtx_;
		static bool is_permanently_cleaned_;  // true after permanent cleanup at exit
	};

#define LOG_TRACE(...) LogManager::get_logger()->trace(__VA_ARGS__)
#define LOG_DEBUG(...) LogManager::get_logger()->debug(__VA_ARGS__)
#define LOG_INFO(...)  LogManager::get_logger()->info(__VA_ARGS__)
#define LOG_WARN(...)  LogManager::get_logger()->warn(__VA_ARGS__)
#define LOG_ERROR(...) LogManager::get_logger()->error(__VA_ARGS__)
#define LOG_CRITICAL(...) LogManager::get_logger()->critical(__VA_ARGS__)
#define LOG_FLUSH LogManager::get_logger()->flush()
#define LOG_SHUTDOWN LogManager::cleanup()
}
