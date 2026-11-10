#include "logging.hpp"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/daily_file_sink.h>

namespace Ddx {
	bool LogManager::is_permanently_cleaned_ = false;

	LogManager& LogManager::instance() {
		static LogManager inst;
		return inst;
	}

	void LogManager::clear_old_logger_() {
		if (instance().logger_ != nullptr) {
			try {
				instance().logger_->flush();
				spdlog::drop(instance().logger_->name());
				instance().logger_ = nullptr;
				spdlog::set_default_logger(nullptr);
			}
			catch (const spdlog::spdlog_ex& ex) {
				throw std::runtime_error(std::string("clear logger failed: ") + ex.what());
			}
		}
	}

	void LogManager::reset_spdlog_to_default() {
		std::lock_guard<std::mutex> lock(instance().mtx_);

		auto default_console_logger = spdlog::get("console");
		if (!default_console_logger) {
			default_console_logger = spdlog::stdout_color_mt("console");
		}

		spdlog::set_default_logger(default_console_logger);
		default_console_logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] %v");
		spdlog::flush_every(std::chrono::seconds(1));
		spdlog::info("restored spdlog default console logger");
	}

	void LogManager::init(const std::string& log_file_path, const std::string& name,
	                      spdlog::level::level_enum level, bool force_reset) {
		if (log_file_path.empty() || name.empty()) {
			throw std::invalid_argument("log path and name must not be empty");
		}
		if (is_permanently_cleaned_) {
			throw std::runtime_error("logger permanently cleaned up, cannot init");
		}

		std::lock_guard<std::mutex> lock(instance().mtx_);

		if (instance().logger_ != nullptr && force_reset) {
			clear_old_logger_();
		}

		if (!instance().logger_) {
			try {
				auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file_path, true);
				std::string logger_name = "DDX_Reg_" + name;
				instance().logger_ = std::make_shared<spdlog::logger>(logger_name, file_sink);

				instance().logger_->set_level(level);
				instance().logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");
				instance().logger_->flush_on(spdlog::level::warn);
				spdlog::flush_every(std::chrono::seconds(1));
				spdlog::set_default_logger(instance().logger_);

				instance().logger_->info("file logger initialized: {}", log_file_path);
			}
			catch (const spdlog::spdlog_ex& ex) {
				throw std::runtime_error(std::string("logger init failed: ") + ex.what());
			}
		}
		else if (!force_reset) {
			instance().logger_->warn("logger exists, skip init (set force_reset=true to re-init)");
		}
	}

	void LogManager::init_daily(const std::string& log_folder, const std::string& name,
	                            spdlog::level::level_enum level, bool force_reset) {
		if (log_folder.empty() || name.empty()) {
			throw std::invalid_argument("log folder and name must not be empty");
		}
		if (is_permanently_cleaned_) {
			throw std::runtime_error("logger permanently cleaned up, cannot init");
		}

		std::lock_guard<std::mutex> lock(instance().mtx_);

		if (instance().logger_ != nullptr && force_reset) {
			clear_old_logger_();
		}

		if (!instance().logger_) {
			try {
				std::string log_file_base = log_folder + "/manager_" + name;
				auto daily_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(log_file_base, 0, 0);
				instance().logger_ = std::make_shared<spdlog::logger>(name, daily_sink);

				instance().logger_->set_level(level);
				instance().logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");
				instance().logger_->flush_on(spdlog::level::warn);
				spdlog::flush_every(std::chrono::seconds(1));
				spdlog::set_default_logger(instance().logger_);

				instance().logger_->info("daily logger initialized: {}", log_folder);
			}
			catch (const spdlog::spdlog_ex& ex) {
				throw std::runtime_error(std::string("daily logger init failed: ") + ex.what());
			}
		}
		else if (!force_reset) {
			instance().logger_->warn("logger exists, skip init (set force_reset=true to re-init)");
		}
	}

	std::shared_ptr<spdlog::logger>& LogManager::get_logger() {
		std::lock_guard<std::mutex> lock(instance().mtx_);

		if (is_permanently_cleaned_) {
			return instance().logger_;
		}

		if (!instance().logger_) {
			instance().logger_ = spdlog::stdout_color_mt("default_console_logger");
			instance().logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");
			instance().logger_->warn("logger not initialized, using temporary console logger");
			spdlog::flush_every(std::chrono::seconds(1));
		}

		return instance().logger_;
	}

	void LogManager::reset() {
		std::lock_guard<std::mutex> lock(instance().mtx_);
		if (is_permanently_cleaned_) {
			throw std::runtime_error("logger permanently cleaned up, cannot reset");
		}
		clear_old_logger_();
	}

	void LogManager::cleanup() {
		std::lock_guard<std::mutex> lock(instance().mtx_);
		if (is_permanently_cleaned_) {
			return;
		}

		clear_old_logger_();
		spdlog::shutdown();
		is_permanently_cleaned_ = true;
	}

	void LogManager::set_level(spdlog::level::level_enum level) {
		std::lock_guard<std::mutex> lock(instance().mtx_);
		if (instance().logger_) {
			instance().logger_->set_level(level);
			instance().logger_->info("log level set to {}", spdlog::level::to_string_view(level));
		}
		else {
			throw std::runtime_error("logger not initialized, cannot set level");
		}
	}
}
