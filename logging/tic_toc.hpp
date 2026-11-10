#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <string>
#include <map>
#include <unordered_map>
#include "logging.hpp"

/**			TimeRecorder
* Can record the number of times an object runs and the duration of each run
*/
class TimeRecorder
{
private:
	TimeRecorder() = default;
	~TimeRecorder() = default;
	TimeRecorder(const TimeRecorder&) = delete;
	TimeRecorder& operator=(const TimeRecorder&) = delete;

public:
	static TimeRecorder& getInstance() {
		static TimeRecorder instance;      // function-local static singleton
		return instance;
	}

	void addObject(std::string const& name, double const& cost_time)
	{
		// Automatic locking/unlocking
		std::lock_guard<std::mutex> lock(mtx);
		if (object_cost_time_map_.find(name) == object_cost_time_map_.end())
		{
			object_cost_time_map_.emplace(std::make_pair(name, std::vector<double>()));
		}
		object_cost_time_map_[name].push_back(cost_time);
	}

	std::vector<double> getObjectCostList(std::string const& name) {
		// Automatic locking/unlocking
		std::lock_guard<std::mutex> lock(mtx);
		if (object_cost_time_map_.find(name) == object_cost_time_map_.end())
		{
			return std::vector<double>();
		}
		return object_cost_time_map_[name];
	}

	void printfObject(std::string const& name)
	{
		// Automatic locking/unlocking
		std::lock_guard<std::mutex> lock(mtx);
		if (object_cost_time_map_.find(name) == object_cost_time_map_.end())
			return;

		printfObject(name, object_cost_time_map_[name]);
	}

	static void printfObject(std::string const& name, std::vector<double> const& cost_time_list)
	{
		double mean_time = 0;
		double min_time = std::numeric_limits<double>::max();
		double max_time = std::numeric_limits<double>::min();

		for (int i = 0; i < cost_time_list.size(); ++i)
		{
			mean_time += cost_time_list[i];
			min_time = std::min(min_time, cost_time_list[i]);
			max_time = std::max(max_time, cost_time_list[i]);
		}
		mean_time /= double(cost_time_list.size());
		Ddx::LOG_INFO(" ============= {} =============", name);
		Ddx::LOG_INFO("\t run number		: {}", cost_time_list.size());
		Ddx::LOG_INFO("\t mean cost time : {}", mean_time);
		Ddx::LOG_INFO("\t min  cost time : {}", min_time);
		Ddx::LOG_INFO("\t max  cost time : {}", max_time);
		Ddx::LOG_INFO("---------------- end ----------------");
		Ddx::LOG_INFO("");
	}
private:
	std::map<std::string, std::vector<double>> object_cost_time_map_;
	std::mutex mtx;
};


class TicToc
{
public:
	TicToc()
		: m_name("")
		, is_deconstruction(false)
		, is_open_recorder(false)
	{
		tic();
	}

	TicToc(std::string const& name, bool cond = true, bool open_recorder = false)
		: m_name(name)
		, is_deconstruction(cond)
		, is_open_recorder(open_recorder)
	{
		tic();
	}

	~TicToc()
	{
		if (is_deconstruction)
		{
#ifdef _OPENMP
#pragma omp critical
#endif
			{
				Ddx::LOG_INFO("{} time : {} ms", m_name, toc());
			}

		}

		if (is_open_recorder)
		{
			TimeRecorder& recorder = TimeRecorder::getInstance();
			recorder.addObject(m_name, toc());
		}
	}

	// Add a new timer, ignore if it exists
	void addStartTimer(std::string const& name, bool cover = false) {
		if (start_map.find(name) == start_map.end())
		{
			start_map.emplace(std::make_pair(name, start_timers.size()));
			start_timers.push_back(std::chrono::system_clock::now());
			start_names.push_back(name);
		}
		else if (cover)
		{
			// If it exists, update the start time
			start_timers[start_map[name]] = std::chrono::system_clock::now();
		}
	}

	// Update End Time
	void addEndTimer(std::string const& name) {
		if (end_map.find(name) == end_map.end())
		{
			end_map.emplace(std::make_pair(name, std::chrono::system_clock::now()));
		}
		else
		{
			end_map[name] = std::chrono::system_clock::now();
		}
	}

	void printf(std::string const& name) {
		if (start_map.find(name) != start_map.end())
		{
			std::chrono::time_point<std::chrono::system_clock> current = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = current - start_timers[start_map[name]];
			double cost_time = elapsed_seconds.count() * 1000.0;
			Ddx::LOG_INFO(" {} cost time : {} ms", name, cost_time);
		}
	}

	// Print the entire process time

	void printfAll() {
		if (start_map.size() <= 0) return;
		Ddx::LOG_INFO("");
		Ddx::LOG_INFO(" =================== {} process =================", m_name);
		std::chrono::time_point<std::chrono::system_clock> current = std::chrono::system_clock::now();
		for (int i = 0; i < start_timers.size(); ++i) {
			const std::string& name = start_names[i];
			if (end_map.find(name) != end_map.end()) {
				current = end_map[name];
				std::chrono::duration<double> elapsed_seconds = current - start_timers[i];
				const double r11x = elapsed_seconds.count() * 1000.0;
				Ddx::LOG_INFO("\t {} cost time : {} ms", name, r11x);
			}
		}
		Ddx::LOG_INFO("\t all cost time : {} ms", toc());
		Ddx::LOG_INFO(" -------------------- end --------------------");
		Ddx::LOG_INFO("");
	}

	double getCostTime(std::string const& name) {
		if (start_map.find(name) != start_map.end()) {
			std::chrono::time_point<std::chrono::system_clock> current = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = current - start_timers[start_map[name]];
			return elapsed_seconds.count() * 1000.0;
		}

		return -1.0;
	}

	void tic() {
		start = std::chrono::system_clock::now();
	}

	double toc() {
		end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		return elapsed_seconds.count() * 1000.0;
	}

private:
	std::chrono::time_point<std::chrono::system_clock> start, end;
	bool is_deconstruction;
	bool is_open_recorder;
	std::string m_name;
	std::vector<std::chrono::time_point<std::chrono::system_clock>> start_timers;
	std::vector<std::string> start_names;
	std::map<std::string, int> start_map;
	std::map<std::string, std::chrono::time_point<std::chrono::system_clock>> end_map;
};
