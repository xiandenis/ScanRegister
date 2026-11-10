#include "mem_cost_model.hpp"

#include "logging.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace Ddx {
namespace {

bool parseDoubleField(std::string const& json, char const* key, double& out)
{
	std::string needle = std::string("\"") + key + "\"";
	auto pos = json.find(needle);
	if (pos == std::string::npos) {
		return false;
	}
	pos = json.find(':', pos + needle.size());
	if (pos == std::string::npos) {
		return false;
	}
	++pos;
	while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\n' || json[pos] == '\r')) {
		++pos;
	}
	try {
		out = std::stod(json.substr(pos));
		return true;
	} catch (...) {
		return false;
	}
}

bool parseIntField(std::string const& json, char const* key, int& out)
{
	double d = 0.0;
	if (!parseDoubleField(json, key, d)) {
		return false;
	}
	out = static_cast<int>(d);
	return true;
}

bool parseStringField(std::string const& json, char const* key, std::string& out)
{
	std::string needle = std::string("\"") + key + "\"";
	auto pos = json.find(needle);
	if (pos == std::string::npos) {
		return false;
	}
	pos = json.find(':', pos + needle.size());
	if (pos == std::string::npos) {
		return false;
	}
	pos = json.find('"', pos + 1);
	if (pos == std::string::npos) {
		return false;
	}
	auto end = json.find('"', pos + 1);
	if (end == std::string::npos) {
		return false;
	}
	out = json.substr(pos + 1, end - pos - 1);
	return true;
}

}  // namespace

double physicalMemoryMb()
{
#ifdef _WIN32
	MEMORYSTATUSEX st{};
	st.dwLength = sizeof(st);
	if (GlobalMemoryStatusEx(&st)) {
		return static_cast<double>(st.ullTotalPhys) / (1024.0 * 1024.0);
	}
	return 0.0;
#else
	long const pages = sysconf(_SC_PHYS_PAGES);
	long const page_size = sysconf(_SC_PAGE_SIZE);
	if (pages > 0 && page_size > 0) {
		return static_cast<double>(pages) * static_cast<double>(page_size) / (1024.0 * 1024.0);
	}
	return 0.0;
#endif
}

void MemCostModel::reset()
{
	n_ = 0;
	sum_s_ = sum_p_ = sum_ss_ = sum_sp_ = 0.0;
	size_min_ = size_max_ = 0.0;
	last_error_.clear();
}

void MemCostModel::observe(double size, double peak_mb)
{
	if (!(size > 0.0) || !(peak_mb >= 0.0) || !std::isfinite(size) || !std::isfinite(peak_mb)) {
		return;
	}
	if (n_ == 0) {
		size_min_ = size_max_ = size;
	} else {
		size_min_ = std::min(size_min_, size);
		size_max_ = std::max(size_max_, size);
	}
	sum_s_ += size;
	sum_p_ += peak_mb;
	sum_ss_ += size * size;
	sum_sp_ += size * peak_mb;
	++n_;
}

bool MemCostModel::inObservedRange(double size) const
{
	if (n_ <= 0) {
		return false;
	}
	return size >= size_min_ && size <= size_max_;
}

MemCostModel::Fit MemCostModel::fit() const
{
	Fit f;
	if (n_ <= 0) {
		return f;
	}
	if (n_ == 1) {
		if (sum_s_ > 0.0) {
			f.a = sum_p_ / sum_s_;
			f.b = 0.0;
			f.valid = true;
		}
		return f;
	}
	double const denom = static_cast<double>(n_) * sum_ss_ - sum_s_ * sum_s_;
	if (std::fabs(denom) < 1e-30) {
		if (sum_s_ > 0.0) {
			f.a = sum_p_ / sum_s_;
			f.b = 0.0;
			f.valid = true;
		}
		return f;
	}
	f.a = (static_cast<double>(n_) * sum_sp_ - sum_s_ * sum_p_) / denom;
	f.b = (sum_p_ - f.a * sum_s_) / static_cast<double>(n_);
	if (!std::isfinite(f.a) || !std::isfinite(f.b)) {
		return Fit{};
	}
	// Peak should not decrease with size for budgeting; clamp negative slope lightly.
	if (f.a < 0.0) {
		f.a = sum_s_ > 0.0 ? (sum_p_ / sum_s_) : 0.0;
		f.b = 0.0;
	}
	f.valid = true;
	return f;
}

double MemCostModel::estimate(double size,
                              double safety,
                              double safety_extrapolate,
                              double min_estimate_mb) const
{
	if (!(size > 0.0)) {
		return std::max(min_estimate_mb, 0.0);
	}
	double saf = safety > 1.0 ? safety : 1.0;
	if (n_ > 0 && !inObservedRange(size) && safety_extrapolate > 1.0) {
		saf *= safety_extrapolate;
	}

	Fit const f = fit();
	double raw = 0.0;
	if (f.valid) {
		raw = f.a * size + f.b;
	} else {
		// No model yet: large enough to usually force serial warm-up under budget.
		raw = std::max(min_estimate_mb * 4.0, 500.0);
	}
	if (raw < 0.0) {
		raw = 0.0;
	}
	return std::max(min_estimate_mb, raw * saf);
}

bool MemCostModel::loadBuiltinPrior()
{
	// Historical serial-learn sufficient stats (schema 1). Used when no disk model.
	last_error_.clear();
	n_ = 42;
	sum_s_ = 37333939676.0;
	sum_p_ = 33450.77734375;
	sum_ss_ = 4.684191660312952e+19;
	sum_sp_ = 37382677721583.953;
	size_min_ = 80605861.0;
	size_max_ = 1579681311.0;
	return true;
}

bool MemCostModel::loadFromDisk(std::string const& path, std::string const& expect_fingerprint)
{
	last_error_.clear();
	std::ifstream in(path);
	if (!in) {
		last_error_ = "open failed: " + path;
		return false;
	}
	std::ostringstream ss;
	ss << in.rdbuf();
	std::string const json = ss.str();

	int schema = 0;
	if (!parseIntField(json, "schema", schema) || schema != kSchema) {
		last_error_ = "schema mismatch";
		reset();
		return false;
	}

	std::string fp;
	if (parseStringField(json, "algo_fingerprint", fp)) {
		if (!expect_fingerprint.empty() && fp != expect_fingerprint) {
			last_error_ = "algo_fingerprint mismatch";
			reset();
			return false;
		}
	}

	int n = 0;
	double sum_s = 0, sum_p = 0, sum_ss = 0, sum_sp = 0, smin = 0, smax = 0;
	if (!parseIntField(json, "n", n) || n < 0) {
		last_error_ = "bad n";
		return false;
	}
	if (n > 0) {
		if (!parseDoubleField(json, "sum_size", sum_s)
		    || !parseDoubleField(json, "sum_peak", sum_p)
		    || !parseDoubleField(json, "sum_size2", sum_ss)
		    || !parseDoubleField(json, "sum_size_peak", sum_sp)
		    || !parseDoubleField(json, "size_min", smin)
		    || !parseDoubleField(json, "size_max", smax)) {
			last_error_ = "missing stats fields";
			return false;
		}
	}

	n_ = n;
	sum_s_ = sum_s;
	sum_p_ = sum_p;
	sum_ss_ = sum_ss;
	sum_sp_ = sum_sp;
	size_min_ = smin;
	size_max_ = smax;
	return true;
}

bool MemCostModel::saveToDisk(std::string const& path, std::string const& fingerprint) const
{
	last_error_.clear();
	std::string const tmp = path + ".tmp";
	std::FILE* f = std::fopen(tmp.c_str(), "wb");
	if (!f) {
		last_error_ = "open tmp failed: " + tmp;
		return false;
	}
	int const n = std::fprintf(
		f,
		"{\n"
		"  \"schema\": %d,\n"
		"  \"n\": %d,\n"
		"  \"sum_size\": %.17g,\n"
		"  \"sum_peak\": %.17g,\n"
		"  \"sum_size2\": %.17g,\n"
		"  \"sum_size_peak\": %.17g,\n"
		"  \"size_min\": %.17g,\n"
		"  \"size_max\": %.17g,\n"
		"  \"algo_fingerprint\": \"%s\"\n"
		"}\n",
		kSchema,
		n_,
		sum_s_,
		sum_p_,
		sum_ss_,
		sum_sp_,
		size_min_,
		size_max_,
		fingerprint.c_str());
	std::fclose(f);
	if (n < 0) {
		last_error_ = "fprintf failed";
		std::remove(tmp.c_str());
		return false;
	}
#ifdef _WIN32
	std::remove(path.c_str());
#endif
	if (std::rename(tmp.c_str(), path.c_str()) != 0) {
		last_error_ = "rename failed: " + path;
		std::remove(tmp.c_str());
		return false;
	}
	return true;
}

}  // namespace Ddx
