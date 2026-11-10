#pragma once

#include "logging.hpp"

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

/**
 * RAII peak-memory sampler aligned with TicToc scopes.
 *
 * Logs on destruction (when enabled):
 *   "<name> peak_delta : X.X MB  end_delta : Y.Y MB"
 *
 * Disable with env RTR_MEM_PEAK=0 (also accepts f/F/n/N).
 * Default: enabled.
 */
class LOGGING_LOG_API MemPeakScope
{
public:
	explicit MemPeakScope(std::string name,
	                      std::chrono::milliseconds sample_interval = std::chrono::milliseconds(1));

	~MemPeakScope();

	MemPeakScope(MemPeakScope const&) = delete;
	MemPeakScope& operator=(MemPeakScope const&) = delete;

	/// Current process resident / private usage in MB (0 if unavailable).
	static double currentUsageMb();

	static bool enabledFromEnv();

	/// Peak usage minus base since construction (MB). 0 if inactive / disabled.
	double peakDeltaMb() const;

private:
	void samplerLoop();

	std::string name_;
	std::chrono::milliseconds interval_;
	bool active_ = false;
	double base_mb_ = 0.0;
	std::atomic<double> peak_mb_{0.0};
	std::atomic<bool> running_{false};
	std::thread sampler_;
};
