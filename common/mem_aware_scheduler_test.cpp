#include "mem_aware_scheduler.hpp"
#include "mem_cost_model.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static int g_fail = 0;

#define CHECK(cond, msg)                                                       \
	do {                                                                       \
		if (!(cond)) {                                                         \
			std::cerr << "FAIL: " << msg << "\n";                              \
			++g_fail;                                                          \
		}                                                                      \
	} while (0)

static void test_regression_interpolate()
{
	Ddx::MemCostModel m;
	m.observe(4e7, 400.0);
	m.observe(5e7, 500.0);
	double const mid = m.estimate(4.5e7, /*safety*/1.0, /*extrapolate*/1.0, /*min*/0.0);
	CHECK(std::fabs(mid - 450.0) < 1.0, "mid interpolate ~450 got " + std::to_string(mid));

	double const outside = m.estimate(6e7, 1.0, 1.2, 0.0);
	CHECK(outside > 600.0, "extrapolate applies extra safety");
}

static void test_persist_roundtrip()
{
	fs::path const dir = fs::temp_directory_path() / "rtr_mem_cost_model_test";
	fs::create_directories(dir);
	fs::path const path = dir / "mem_cost_model.json";

	Ddx::MemCostModel a;
	a.observe(4e7, 400.0);
	a.observe(5e7, 500.0);
	CHECK(a.saveToDisk(path.string(), "fp1"), "save cost model");

	Ddx::MemCostModel b;
	CHECK(b.loadFromDisk(path.string(), "fp1"), "load cost model");
	CHECK(b.sampleCount() == 2, "n==2");
	double const mid = b.estimate(4.5e7, 1.0, 1.0, 0.0);
	CHECK(std::fabs(mid - 450.0) < 1.0, "loaded model interpolates");

	Ddx::MemCostModel c;
	CHECK(!c.loadFromDisk(path.string(), "other"), "fingerprint mismatch rejects");

	fs::remove_all(dir);
}

static void test_budget_and_cross_run_reuse()
{
	std::atomic<double> fake_usage{100.0};

	Ddx::MemAwareSchedulerOptions opt;
	opt.high_water_mb = 2000.0;
	opt.min_samples_before_parallel = 2;
	opt.safety_factor = 1.0;
	opt.safety_extrapolate_factor = 1.0;
	opt.min_estimate_mb = 1.0;
	opt.prefer_heavy_first = true;
	opt.usage_mb = [&]() { return fake_usage.load(); };
	opt.physical_mb = []() { return 16000.0; };
	opt.peak_mb_override = [](std::size_t, uint64_t size) {
		return static_cast<double>(size) / 1.0e5;
	};

	fs::path const dir = fs::temp_directory_path() / "rtr_mem_sched_test";
	fs::create_directories(dir);
	opt.persist_path = (dir / "mem_cost_model.json").string();

	std::vector<Ddx::MemAwareTask> tasks;
	for (uint64_t sz : {40ull * 1000000, 50ull * 1000000, 45ull * 1000000, 30ull * 1000000}) {
		Ddx::MemAwareTask t;
		t.size = sz;
		t.name = std::to_string(sz);
		t.run = []() {};
		tasks.push_back(std::move(t));
	}

	// Serial learn (max_concurrency <= 1) → seed from builtin if no disk, then persist.
	opt.max_concurrency = 1;
	int n_after_serial = 0;
	{
		Ddx::MemAwareScheduler sched(opt);
		int const n0 = sched.model().sampleCount();
		CHECK(n0 >= 4, "builtin or empty seed before serial learn");
		sched.run(tasks);
		n_after_serial = sched.model().sampleCount();
		CHECK(n_after_serial == n0 + 4, "serial learned +4 samples");
	}

	double const mid = [&]() {
		Ddx::MemAwareScheduler loaded(opt);
		return loaded.model().estimate(4.5e7, 1.0, 1.0, 0.0);
	}();
	CHECK(mid > 50.0, "learned model yields finite estimate for 45M");

	// Parallel must NOT observe (n stays).
	opt.max_concurrency = 4;
	{
		Ddx::MemAwareScheduler sched(opt);
		CHECK(sched.model().sampleCount() == n_after_serial, "loaded serial model");
		sched.run(tasks);
		CHECK(sched.model().sampleCount() == n_after_serial, "parallel no-learn keeps n");
	}

	Ddx::MemAwareScheduler sched2(opt);
	CHECK(sched2.model().sampleCount() == n_after_serial, "persist unchanged after parallel");

	fs::remove_all(dir);
}

static void test_builtin_prior_without_disk()
{
	Ddx::MemCostModel m;
	CHECK(Ddx::MemCostModel::hasBuiltinPrior(), "hasBuiltinPrior");
	CHECK(m.loadBuiltinPrior(), "loadBuiltinPrior");
	CHECK(m.sampleCount() == 42, "builtin n==42");
	CHECK(m.fit().valid, "builtin fit valid");
	double const est = m.estimate(4.5e7, 1.0, 1.0, 0.0);
	CHECK(est > 0.0 && std::isfinite(est), "builtin estimate finite");

	// No persist_path → scheduler still seeds builtin.
	Ddx::MemAwareSchedulerOptions opt;
	opt.max_concurrency = 4;
	opt.min_samples_before_parallel = 4;
	opt.enable_learning = false;
	opt.high_water_mb = 2000.0;
	opt.usage_mb = []() { return 100.0; };
	opt.physical_mb = []() { return 16000.0; };
	opt.peak_mb_override = [](std::size_t, uint64_t) { return 10.0; };
	Ddx::MemAwareScheduler sched(opt);
	CHECK(sched.model().sampleCount() == 42, "scheduler seeds builtin when no disk");

	// Disk wins over builtin.
	fs::path const dir = fs::temp_directory_path() / "rtr_mem_builtin_disk";
	fs::create_directories(dir);
	fs::path const path = dir / "mem_cost_model.json";
	Ddx::MemCostModel seed;
	seed.observe(4e7, 400.0);
	seed.observe(5e7, 500.0);
	CHECK(seed.saveToDisk(path.string()), "tiny disk seed");
	opt.persist_path = path.string();
	Ddx::MemAwareScheduler from_disk(opt);
	CHECK(from_disk.model().sampleCount() == 2, "disk overrides builtin");
	fs::remove_all(dir);
}

static void test_read_only_no_learn()
{
	fs::path const dir = fs::temp_directory_path() / "rtr_mem_sched_readonly";
	fs::create_directories(dir);
	fs::path const path = dir / "mem_cost_model.json";

	Ddx::MemCostModel seed;
	seed.observe(4e7, 400.0);
	seed.observe(5e7, 500.0);
	seed.observe(3e7, 300.0);
	seed.observe(6e7, 600.0);
	CHECK(seed.saveToDisk(path.string()), "seed save");

	auto const read_bytes = [&]() {
		std::ifstream in(path, std::ios::binary);
		return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
	};
	std::string const before = read_bytes();
	CHECK(!before.empty(), "seed non-empty");

	Ddx::MemAwareSchedulerOptions opt;
	opt.max_concurrency = 1;
	opt.persist_path = path.string();
	opt.enable_learning = false;
	opt.high_water_mb = 2000.0;
	opt.min_samples_before_parallel = 2;
	opt.min_estimate_mb = 1.0;
	opt.usage_mb = []() { return 100.0; };
	opt.physical_mb = []() { return 16000.0; };
	opt.peak_mb_override = [](std::size_t, uint64_t) { return 999.0; };

	std::vector<Ddx::MemAwareTask> tasks(2);
	tasks[0].size = 40ull * 1000000;
	tasks[0].name = "a";
	tasks[0].run = []() {};
	tasks[1].size = 50ull * 1000000;
	tasks[1].name = "b";
	tasks[1].run = []() {};

	{
		Ddx::MemAwareScheduler sched(opt);
		CHECK(sched.model().sampleCount() == 4, "loaded seed n==4");
		sched.run(tasks);
		CHECK(sched.model().sampleCount() == 4, "read-only keeps n==4");
	}

	std::string const after = read_bytes();
	CHECK(after == before, "read-only does not rewrite cost model file");

	fs::remove_all(dir);
}

int main()
{
	test_regression_interpolate();
	test_persist_roundtrip();
	test_budget_and_cross_run_reuse();
	test_read_only_no_learn();
	test_builtin_prior_without_disk();
	if (g_fail) {
		std::cerr << g_fail << " check(s) failed\n";
		return 1;
	}
	std::cout << "mem_aware_scheduler_test OK\n";
	return 0;
}
