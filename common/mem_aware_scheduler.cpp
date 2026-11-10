#include "mem_aware_scheduler.hpp"

#include "logging.hpp"
#include "mem_peak.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
#include <utility>

namespace Ddx {

MemAwareScheduler::MemAwareScheduler(MemAwareSchedulerOptions opt)
	: opt_(std::move(opt))
{
	if (!opt_.usage_mb) {
		opt_.usage_mb = []() { return MemPeakScope::currentUsageMb(); };
	}
	if (!opt_.physical_mb) {
		opt_.physical_mb = []() { return physicalMemoryMb(); };
	}
	if (opt_.max_concurrency < 1) {
		opt_.max_concurrency = 1;
	}
	if (opt_.min_samples_before_parallel < 1) {
		opt_.min_samples_before_parallel = 1;
	}
	bool loaded_disk = false;
	if (!opt_.persist_path.empty()) {
		if (model_.loadFromDisk(opt_.persist_path, opt_.algo_fingerprint)) {
			LOG_INFO("MemAwareScheduler: loaded cost model source=disk n={} path={}",
			         model_.sampleCount(), opt_.persist_path);
			loaded_disk = true;
		} else {
			LOG_WARN("MemAwareScheduler: cost model load skipped ({}) path={}",
			         model_.lastError(), opt_.persist_path);
			model_.reset();
		}
	}
	if (!loaded_disk) {
		if (model_.loadBuiltinPrior()) {
			LOG_INFO("MemAwareScheduler: loaded cost model source=builtin n={}",
			         model_.sampleCount());
		}
	}
}

double MemAwareScheduler::usage() const
{
	double const u = opt_.usage_mb ? opt_.usage_mb() : 0.0;
	return u > 0.0 ? u : 0.0;
}

double MemAwareScheduler::highWater() const
{
	if (opt_.high_water_mb > 0.0) {
		return opt_.high_water_mb;
	}
	double const phys = opt_.physical_mb ? opt_.physical_mb() : 0.0;
	if (phys > 0.0) {
		return phys * 0.82;
	}
	return 8192.0;  // conservative fallback 8 GiB
}

double MemAwareScheduler::estimateFor(uint64_t size) const
{
	return model_.estimate(static_cast<double>(size),
	                       opt_.safety_factor,
	                       opt_.safety_extrapolate_factor,
	                       opt_.min_estimate_mb);
}

void MemAwareScheduler::persistModel()
{
	if (opt_.persist_path.empty()) {
		return;
	}
	if (!model_.saveToDisk(opt_.persist_path, opt_.algo_fingerprint)) {
		LOG_WARN("MemAwareScheduler: cost model save failed ({}) path={}",
		         model_.lastError(), opt_.persist_path);
	}
}

void MemAwareScheduler::run(std::vector<MemAwareTask>& tasks)
{
	admit_reject_ = 0;
	inflight_over_high_ = 0;

	if (tasks.empty()) {
		return;
	}

	std::vector<std::size_t> order(tasks.size());
	for (std::size_t i = 0; i < order.size(); ++i) {
		order[i] = i;
	}
	if (opt_.prefer_heavy_first) {
		std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
			return tasks[a].size > tasks[b].size;
		});
	}

	int const max_conc = opt_.max_concurrency;
	if (max_conc <= 1) {
		for (std::size_t idx : order) {
			auto& t = tasks[idx];
			if (!t.run) {
				continue;
			}
			double const est = estimateFor(t.size);
			double peak = 0.0;
			if (opt_.peak_mb_override) {
				t.run();
				peak = opt_.peak_mb_override(idx, t.size);
			} else {
				MemPeakScope scope(t.name.empty() ? "MemAwareTask" : t.name);
				t.run();
				peak = scope.peakDeltaMb();
			}
			if (peak < 0.0) {
				peak = 0.0;
			}
			if (opt_.enable_learning) {
				if (peak > est) {
					model_.observe(static_cast<double>(t.size), std::max(peak, est * 1.05));
				} else {
					model_.observe(static_cast<double>(t.size), peak);
				}
				persistModel();
			}
			LOG_INFO("MemAwareScheduler: serial '{}' size={} est={:.1f} peak={:.1f} n={} learn={}",
			         t.name, t.size, est, peak, model_.sampleCount(), opt_.enable_learning);
		}
		if (opt_.enable_learning) {
			persistModel();
		}
		return;
	}

	std::mutex mu;
	std::condition_variable cv;
	std::size_t next = 0;
	int active = 0;
	double reserved = 0.0;
	bool stop = false;

	auto worker = [&]() {
		for (;;) {
			std::size_t idx = 0;
			double est = 0.0;
			{
				std::unique_lock<std::mutex> lk(mu);
				for (;;) {
					if (stop && next >= order.size() && active == 0) {
						return;
					}
					if (next >= order.size()) {
						if (active == 0) {
							return;
						}
						cv.wait_for(lk, std::chrono::milliseconds(50));
						continue;
					}

					int effective_max = max_conc;
					if (model_.sampleCount() < opt_.min_samples_before_parallel) {
						effective_max = 1;
					}
					double const u = usage();
					if (!(u > 0.0) && !opt_.peak_mb_override) {
						// Real usage unavailable: stay serial.
						effective_max = 1;
					}
					double const high = highWater();
					if (u >= high && active >= 1) {
						++inflight_over_high_;
						++admit_reject_;
						cv.wait_for(lk, std::chrono::milliseconds(50));
						continue;
					}
					if (active >= effective_max) {
						++admit_reject_;
						cv.wait_for(lk, std::chrono::milliseconds(50));
						continue;
					}

					idx = order[next];
					est = estimateFor(tasks[idx].size);
					double const remaining = high - u - reserved;
					bool const allow_oversize = (active == 0);
					if (!allow_oversize && est > remaining) {
						++admit_reject_;
						cv.wait_for(lk, std::chrono::milliseconds(50));
						continue;
					}

					++next;
					++active;
					reserved += est;
					break;
				}
			}

			auto& t = tasks[idx];
			double peak = 0.0;
			try {
				if (!t.run) {
					peak = 0.0;
				} else if (opt_.peak_mb_override) {
					t.run();
					peak = opt_.peak_mb_override(idx, t.size);
				} else {
					MemPeakScope scope(t.name.empty() ? "MemAwareTask" : t.name);
					t.run();
					peak = scope.peakDeltaMb();
				}
			} catch (std::exception const& ex) {
				LOG_ERROR("MemAwareScheduler: task '{}' threw: {}", t.name, ex.what());
				peak = est;  // assume reserved amount realized
			} catch (...) {
				LOG_ERROR("MemAwareScheduler: task '{}' threw unknown", t.name);
				peak = est;
			}

			if (peak < 0.0) {
				peak = 0.0;
			}

			{
				std::lock_guard<std::mutex> lk(mu);
				// Parallel path: never observe / persist. Process-wide MemPeakScope
				// attributes stacked RSS to each task and poisons the cost model.
				// Learning is serial-only (max_concurrency <= 1 branch above).
				--active;
				reserved -= est;
				if (reserved < 0.0) {
					reserved = 0.0;
				}
				LOG_INFO(
					"MemAwareScheduler: done '{}' size={} est={:.1f} peak={:.1f} "
					"active={} n={} (no-learn parallel)",
					t.name, t.size, est, peak, active, model_.sampleCount());
				cv.notify_all();
			}
		}
	};

	int const workers = max_conc;
	std::vector<std::thread> pool;
	pool.reserve(static_cast<std::size_t>(workers));
	for (int i = 0; i < workers; ++i) {
		pool.emplace_back(worker);
	}
	for (auto& th : pool) {
		th.join();
	}
	LOG_INFO(
		"MemAwareScheduler: finished tasks={} n={} admit_reject≈{} inflight_over_high≈{} "
		"(parallel no-learn)",
		tasks.size(), model_.sampleCount(), admit_reject_, inflight_over_high_);
}

}  // namespace Ddx
