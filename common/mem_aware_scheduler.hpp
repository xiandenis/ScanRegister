#pragma once

#include "mem_cost_model.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace Ddx {

struct MemAwareSchedulerOptions {
	int max_concurrency = 1;
	/// Budget ceiling / RSS safety valve. <=0 → 0.82 * physical.
	double high_water_mb = 0.0;
	double low_water_mb = 0.0;  // reserved for future boost heuristics
	bool prefer_heavy_first = true;
	int min_samples_before_parallel = 4;
	double safety_factor = 1.2;
	double safety_extrapolate_factor = 1.2;
	double min_estimate_mb = 50.0;
	/// Persistent cost-model JSON (cross-project when under data_info). Empty = no disk I/O.
	std::string persist_path;
	/// When false: still load from persist_path if set, but never observe / save (read-only).
	/// Default true preserves historical Session learn+persist behavior for callers that opt in.
	bool enable_learning = true;
	std::string algo_fingerprint;
	/// Injected process usage (MB). Default: MemPeakScope::currentUsageMb.
	std::function<double()> usage_mb;
	/// Injected physical RAM (MB). Default: physicalMemoryMb().
	std::function<double()> physical_mb;
	/// If set, called after each task instead of measuring peak via MemPeakScope.
	/// Signature: (task_index, size) -> peak_delta_mb. Used by unit tests.
	std::function<double(std::size_t, uint64_t)> peak_mb_override;
};

struct MemAwareTask {
	uint64_t size = 0;
	std::function<void()> run;
	std::string name;
};

/// Memory-aware task runner: budget reservation + RSS high-water admission.
/// Cost-model learning/persist only when max_concurrency <= 1 (serial); parallel
/// runs use the loaded model for estimates but never observe (avoids stacked-peak poison).
class MemAwareScheduler {
public:
	explicit MemAwareScheduler(MemAwareSchedulerOptions opt);

	void run(std::vector<MemAwareTask>& tasks);

	MemCostModel const& model() const { return model_; }
	MemCostModel& model() { return model_; }

	int admitRejectCount() const { return admit_reject_; }
	int inflightOverHighCount() const { return inflight_over_high_; }

private:
	double usage() const;
	double highWater() const;
	double estimateFor(uint64_t size) const;
	void persistModel();

	MemAwareSchedulerOptions opt_;
	MemCostModel model_;
	int admit_reject_ = 0;
	int inflight_over_high_ = 0;
};

}  // namespace Ddx
