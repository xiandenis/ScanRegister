#include "mem_peak.hpp"

#include <cstdlib>
#include <fstream>
#include <sstream>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#endif

namespace
{

bool envDisabled(char const* value)
{
	if (value == nullptr || value[0] == '\0') {
		return false;
	}
	char const c = value[0];
	return c == '0' || c == 'f' || c == 'F' || c == 'n' || c == 'N';
}

}  // namespace

bool MemPeakScope::enabledFromEnv()
{
	return !envDisabled(std::getenv("RTR_MEM_PEAK"));
}

double MemPeakScope::currentUsageMb()
{
#ifdef _WIN32
	PROCESS_MEMORY_COUNTERS_EX pmc{};
	pmc.cb = sizeof(pmc);
	if (GetProcessMemoryInfo(
	        GetCurrentProcess(),
	        reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&pmc),
	        sizeof(pmc))) {
		return static_cast<double>(pmc.PrivateUsage) / (1024.0 * 1024.0);
	}
	return 0.0;
#else
	std::ifstream status("/proc/self/status");
	if (!status) {
		return 0.0;
	}
	std::string line;
	while (std::getline(status, line)) {
		if (line.compare(0, 6, "VmRSS:") != 0) {
			continue;
		}
		std::istringstream iss(line.substr(6));
		long kb = 0;
		iss >> kb;
		return static_cast<double>(kb) / 1024.0;
	}
	return 0.0;
#endif
}

MemPeakScope::MemPeakScope(std::string name, std::chrono::milliseconds sample_interval)
	: name_(std::move(name))
	, interval_(sample_interval)
{
	if (!enabledFromEnv()) {
		return;
	}
	base_mb_ = currentUsageMb();
	peak_mb_.store(base_mb_, std::memory_order_relaxed);
	active_ = true;
	running_.store(true, std::memory_order_relaxed);
	sampler_ = std::thread([this]() { samplerLoop(); });
}

double MemPeakScope::peakDeltaMb() const
{
	if (!active_) {
		return 0.0;
	}
	double const end_mb = currentUsageMb();
	double const peak = peak_mb_.load(std::memory_order_relaxed);
	double const peak_use = peak > end_mb ? peak : end_mb;
	double const d = peak_use - base_mb_;
	return d > 0.0 ? d : 0.0;
}

MemPeakScope::~MemPeakScope()
{
	if (!active_) {
		return;
	}
	running_.store(false, std::memory_order_relaxed);
	if (sampler_.joinable()) {
		sampler_.join();
	}

	double const end_mb = currentUsageMb();
	double const peak = peak_mb_.load(std::memory_order_relaxed);
	double const peak_use = peak > end_mb ? peak : end_mb;
	double const peak_delta = peak_use - base_mb_;
	double const end_delta = end_mb - base_mb_;

#ifdef _OPENMP
#pragma omp critical
#endif
	{
		Ddx::LOG_INFO("{} peak_delta : {:.1f} MB  end_delta : {:.1f} MB",
		              name_, peak_delta, end_delta);
	}
}

void MemPeakScope::samplerLoop()
{
	while (running_.load(std::memory_order_relaxed)) {
		double const m = currentUsageMb();
		double cur = peak_mb_.load(std::memory_order_relaxed);
		while (m > cur &&
		       !peak_mb_.compare_exchange_weak(cur, m, std::memory_order_relaxed)) {
		}
		std::this_thread::sleep_for(interval_);
	}
}
