#pragma once

#include <cstdint>
#include <string>

namespace Ddx {

/// Online univariate regression: peak_mb ≈ a * size + b (size typically file bytes).
/// Persists sufficient statistics to JSON for cross-run / cross-project reuse.
class MemCostModel {
public:
	static constexpr int kSchema = 1;

	struct Fit {
		double a = 0.0;
		double b = 0.0;
		bool valid = false;
	};

	void reset();

	void observe(double size, double peak_mb);

	/// Predict peak MB for size. Applies safety; extra factor when extrapolating.
	double estimate(double size,
	                double safety = 1.2,
	                double safety_extrapolate = 1.2,
	                double min_estimate_mb = 50.0) const;

	Fit fit() const;

	int sampleCount() const { return n_; }
	double sizeMin() const { return size_min_; }
	double sizeMax() const { return size_max_; }
	bool inObservedRange(double size) const;

	bool loadFromDisk(std::string const& path, std::string const& expect_fingerprint = {});
	bool saveToDisk(std::string const& path, std::string const& fingerprint = {}) const;

	/// Deploy cold-start prior from historical serial learn samples (compiled-in).
	/// Prefer disk JSON when present; rebuild constants after major algo path changes.
	bool loadBuiltinPrior();
	static bool hasBuiltinPrior() { return true; }

	std::string const& lastError() const { return last_error_; }

private:
	int n_ = 0;
	double sum_s_ = 0.0;
	double sum_p_ = 0.0;
	double sum_ss_ = 0.0;
	double sum_sp_ = 0.0;
	double size_min_ = 0.0;
	double size_max_ = 0.0;
	mutable std::string last_error_;
};

/// Physical RAM in MB (0 if unavailable).
double physicalMemoryMb();

}  // namespace Ddx
