
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

#include <string>

namespace Ddx {

	/**
	* @brief Initialize a plain file logger.
	* @param log_file_path Full path to the log file.
	* @param name Logger name.
	* @param level spdlog level enum as int (default: 2 = info).
	*/
	void LOGGING_LOG_API initiLog(const std::string& log_file_path, const std::string& name, int level = 2);

	/**
	* @brief Reset the logger (pair with initiLog()).
	*/
	void LOGGING_LOG_API resetLog();

	/**
	* @brief Permanently close file logging (call before main exits).
	*/
	void LOGGING_LOG_API permanentlyCloseLog();

}
