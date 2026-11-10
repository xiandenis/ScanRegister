#include <overlap_estimation/overlap_estimation.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace fs = std::filesystem;

static double elapsed_ms(std::chrono::steady_clock::time_point const& t0)
{
  using ms = std::chrono::duration<double, std::milli>;
  return ms(std::chrono::steady_clock::now() - t0).count();
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr make_grid_cloud(std::size_t target_count, float spacing)
{
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(target_count);

  const int side = static_cast<int>(std::ceil(std::cbrt(static_cast<double>(target_count))));
  for (int ix = 0; ix < side && cloud->size() < target_count; ++ix)
  {
    for (int iy = 0; iy < side && cloud->size() < target_count; ++iy)
    {
      for (int iz = 0; iz < side && cloud->size() < target_count; ++iz)
      {
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(ix) * spacing;
        pt.y = static_cast<float>(iy) * spacing;
        pt.z = static_cast<float>(iz) * spacing;
        cloud->push_back(pt);
      }
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr make_shifted_copy(
    pcl::PointCloud<pcl::PointXYZ> const& base, float shift_x, float shift_y, float shift_z)
{
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(base.size());
  for (auto const& p : base.points)
  {
    pcl::PointXYZ q = p;
    q.x += shift_x;
    q.y += shift_y;
    q.z += shift_z;
    cloud->push_back(q);
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

struct BenchmarkRow
{
  std::size_t source_points = 0;
  std::size_t target_points = 0;
  double set_target_ms = 0.0;
  double compute_ms = 0.0;
  double compute_region_ms = 0.0;
  float overlap_ratio = 0.0f;
  std::size_t overlap_pairs = 0;
};

static BenchmarkRow run_case(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& source,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& target,
    float voxel_size,
    float distance_thresh,
    int compute_runs)
{
  Ddx::OverlapEstimator estimator(voxel_size, distance_thresh);

  const auto t_set0 = std::chrono::steady_clock::now();
  estimator.SetTargetCloud<pcl::PointXYZ>(target);
  const double set_target_ms = elapsed_ms(t_set0);

  Ddx::OverlapEstimator::Result warmup;
  warmup = estimator.ComputeOverlap<pcl::PointXYZ>(source);

  double compute_total_ms = 0.0;
  Ddx::OverlapEstimator::Result last;
  for (int run = 0; run < compute_runs; ++run)
  {
    const auto t0 = std::chrono::steady_clock::now();
    last = estimator.ComputeOverlap<pcl::PointXYZ>(source);
    compute_total_ms += elapsed_ms(t0);
  }

  Ddx::OverlapEstimator::RegionResult warmup_region;
  warmup_region = estimator.ComputeOverlapRegion<pcl::PointXYZ>(source);

  double compute_region_total_ms = 0.0;
  for (int run = 0; run < compute_runs; ++run)
  {
    const auto t0 = std::chrono::steady_clock::now();
    warmup_region = estimator.ComputeOverlapRegion<pcl::PointXYZ>(source);
    compute_region_total_ms += elapsed_ms(t0);
  }

  BenchmarkRow row;
  row.source_points = source->size();
  row.target_points = target->size();
  row.set_target_ms = set_target_ms;
  row.compute_ms = compute_total_ms / static_cast<double>(compute_runs);
  row.compute_region_ms =
      compute_region_total_ms / static_cast<double>(compute_runs);
  row.overlap_ratio = last.overlap_ratio;
  row.overlap_pairs = last.overlap_idx_src.size();
  (void)warmup;
  (void)warmup_region;
  return row;
}

static void print_header()
{
  std::cout << std::left
            << std::setw(12) << "source_pts"
            << std::setw(12) << "target_pts"
            << std::setw(14) << "set_target_ms"
            << std::setw(14) << "compute_ms"
            << std::setw(14) << "region_ms"
            << std::setw(14) << "src_pts/s"
            << std::setw(12) << "overlap"
            << std::setw(12) << "pairs"
            << "\n";
  std::cout << std::string(104, '-') << "\n";
}

static void print_row(BenchmarkRow const& row)
{
  const double throughput =
      row.compute_ms > 0.0 ? static_cast<double>(row.source_points) / (row.compute_ms / 1000.0) : 0.0;

  std::cout << std::left << std::fixed << std::setprecision(2)
            << std::setw(12) << row.source_points
            << std::setw(12) << row.target_points
            << std::setw(14) << row.set_target_ms
            << std::setw(14) << row.compute_ms
            << std::setw(14) << row.compute_region_ms
            << std::setw(14) << static_cast<std::uint64_t>(throughput)
            << std::setw(12) << row.overlap_ratio
            << std::setw(12) << row.overlap_pairs
            << "\n";
}

static void run_synthetic_suite(float voxel_size, float distance_thresh, int compute_runs)
{
  const std::vector<std::size_t> sizes = {
      10'000,
      50'000,
      100'000,
      250'000,
      500'000,
      1'000'000,
  };

  std::cout << "\n=== synthetic benchmark ===\n";
  std::cout << "voxel_size=" << voxel_size << " m, distance_thresh=" << distance_thresh
            << " m, compute_runs=" << compute_runs << "\n";
  print_header();

  for (auto n : sizes)
  {
    auto target = make_grid_cloud(n, 0.05f);
    auto source = make_shifted_copy(*target, 0.02f, 0.0f, 0.0f);
    print_row(run_case(source, target, voxel_size, distance_thresh, compute_runs));
  }
}

#ifdef _WIN32
#include <windows.h>
#endif

static fs::path to_path(wchar_t const* wpath) { return fs::path(wpath); }

static bool load_pcd(wchar_t const* wpath, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  fs::path fpath = to_path(wpath);
  if (!fs::exists(fpath))
  {
    std::cerr << "PCD not found: " << fpath.string() << "\n";
    return false;
  }

  if (pcl::io::loadPCDFile(fpath.string(), cloud) != 0 || cloud.empty())
  {
    std::cerr << "Failed to load PCD: " << fpath.string() << "\n";
    return false;
  }
  return true;
}

int wmain(int argc, wchar_t* argv[])
{
#ifdef _WIN32
  SetConsoleOutputCP(65001);
#endif

  float voxel_size = 0.5f;
  float distance_thresh = 0.3f;
  int compute_runs = 5;

  std::cout << "=== overlap_estimation benchmark ===\n";
#ifdef _OPENMP
  std::cout << "OpenMP threads: " << omp_get_max_threads() << "\n";
#else
  std::cout << "OpenMP: disabled\n";
#endif

  run_synthetic_suite(voxel_size, distance_thresh, compute_runs);

  if (argc >= 2)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_a;
    pcl::PointCloud<pcl::PointXYZ> cloud_b;
    if (load_pcd(argv[1], cloud_a))
    {
      if (argc >= 3)
      {
        if (!load_pcd(argv[2], cloud_b))
          return 1;
      }
      else
      {
        cloud_b = cloud_a;
      }

      auto src = pcl::PointCloud<pcl::PointXYZ>::ConstPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud_a));
      auto tgt = pcl::PointCloud<pcl::PointXYZ>::ConstPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud_b));

      std::cout << "\n=== real data benchmark ===\n";
      print_header();
      print_row(run_case(src, tgt, voxel_size, distance_thresh, compute_runs));
    }
  }
  else
  {
    std::cout << "\nTip: pass two PCD paths to benchmark real scans:\n";
    std::cout << "  overlap_estimation_benchmark_test.exe source.pcd target.pcd\n";
  }

  return 0;
}
