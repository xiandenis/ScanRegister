#include "robust_global_reg.hpp"

#include <common/common.hpp>
#include <logging/logging.hpp>
#include <logging/tic_toc.hpp>
#include <point_cloud_io/point_cloud_io.hpp>

#include <fast_gicp/gicp/lsq_registration.hpp>
#include <fast_gicp/gicp/impl/lsq_registration_impl.hpp>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/impl/fast_gicp_impl.hpp>

#include <octree/octree_downsample.hpp>
#include <robust_global_reg/pipeline_overlap.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace fs = std::filesystem;

struct StageTiming {
  double load_source_ms = 0.0;
  double load_target_ms = 0.0;
  double coarse_reg_ms = 0.0;
  double coarse_overlap_ms = 0.0;
  double fine_downsample_ms = 0.0;
  double fine_gicp_ms = 0.0;
  double fine_overlap_ms = 0.0;
  double total_ms = 0.0;
};

static fs::path to_path(wchar_t const* wpath)
{
  return fs::path(wpath);
}

static std::string narrow_path(fs::path const& p)
{
  return p.string();
}

static double elapsed_ms(std::chrono::steady_clock::time_point const& t0)
{
  using ms = std::chrono::duration<double, std::milli>;
  return ms(std::chrono::steady_clock::now() - t0).count();
}

static void print_matrix(std::string const& title, Eigen::Matrix4d const& matrix)
{
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "\n=== " << title << " ===\n";
  std::cout << Ddx::Common::eigen44_to_print_string(matrix) << "\n";
  std::cout << "South: " << Ddx::Common::matrix_to_south_string(matrix) << "\n";
}

static bool load_slas_subtract_offset(
    std::string const& path, pcl::PointCloud<pcl::PointXYZ>& cloud, double& load_ms)
{
  const auto t0 = std::chrono::steady_clock::now();
  const int rc = Ddx::point_cloud_io::load_slas(path, cloud, /*subtract_header_offset=*/true);
  load_ms = elapsed_ms(t0);
  return rc == 0 && !cloud.empty();
}

static void cloud_to_vec(pcl::PointCloud<pcl::PointXYZ> const& cloud,
                         std::vector<Eigen::Vector3f>& out)
{
  out.resize(cloud.size());
  for (std::size_t i = 0; i < cloud.size(); ++i)
  {
    out[i] = Eigen::Vector3f(cloud[i].x, cloud[i].y, cloud[i].z);
  }
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr octree_downsample(
    pcl::PointCloud<pcl::PointXYZ> const& cloud, float voxel_size, double& downsample_ms)
{
  const auto t0 = std::chrono::steady_clock::now();
  auto filtered = Ddx::octree::downsampleMedoid(cloud, voxel_size, /*multi_thread=*/true);
  downsample_ms = elapsed_ms(t0);
  return filtered;
}

static bool run_fine_gicp(
    pcl::PointCloud<pcl::PointXYZ> const& source,
    pcl::PointCloud<pcl::PointXYZ> const& target,
    Eigen::Matrix4d const& coarse_guess,
    Eigen::Matrix4d& fine_transform,
    bool& converged,
    double& fitness,
    double& gicp_ms)
{
  fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setNumThreads(
#ifdef _OPENMP
      std::max(1, omp_get_max_threads())
#else
      1
#endif
  );
  gicp.setCorrespondenceRandomness(20);
  gicp.setInputSource(source.makeShared());
  gicp.setInputTarget(target.makeShared());

  pcl::PointCloud<pcl::PointXYZ> aligned;
  const Eigen::Matrix4f init = coarse_guess.cast<float>();

  const auto t0 = std::chrono::steady_clock::now();
  gicp.align(aligned, init);
  gicp_ms = elapsed_ms(t0);

  converged = gicp.hasConverged();
  fitness = gicp.getFitnessScore();
  fine_transform = gicp.getFinalTransformation().cast<double>();
  return converged;
}

static void print_timing_report(StageTiming const& timing, std::size_t src_n, std::size_t tgt_n,
                                std::size_t fine_src_n, std::size_t fine_tgt_n,
                                float coarse_overlap_src, float coarse_overlap_tgt,
                                float fine_overlap_src, float fine_overlap_tgt)
{
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "\n=== timing report (ms) ===\n";
  std::cout << "  load source (" << src_n << " pts)     : " << timing.load_source_ms << "\n";
  std::cout << "  load target (" << tgt_n << " pts)     : " << timing.load_target_ms << "\n";
  std::cout << "  coarse global reg (RobustMatcher) : " << timing.coarse_reg_ms << "\n";
  std::cout << "  coarse region ratio (src/tgt)   : "
            << std::setprecision(4) << coarse_overlap_src << " / "
            << coarse_overlap_tgt << std::setprecision(2)
            << " (" << timing.coarse_overlap_ms << " ms)\n";
  std::cout << "  fine downsample (" << fine_src_n << " / " << fine_tgt_n
            << " pts) : " << timing.fine_downsample_ms << "\n";
  std::cout << "  fine FastGICP                     : " << timing.fine_gicp_ms << "\n";
  std::cout << "  fine region ratio (src/tgt)     : "
            << std::setprecision(4) << fine_overlap_src << " / "
            << fine_overlap_tgt << std::setprecision(2)
            << " (" << timing.fine_overlap_ms << " ms)\n";
  std::cout << "  total                             : " << timing.total_ms << "\n";
}

static bool save_pcd(
    fs::path const& path,
    pcl::PointCloud<pcl::PointXYZ> const& cloud)
{
  std::error_code ec;
  fs::create_directories(path.parent_path(), ec);

  if(cloud.empty())
  {
    std::cerr << "skip empty cloud: " << path.string() << "\n";
    return false;
  }

  if(pcl::io::savePCDFileBinary(path.string(), cloud) != 0)
  {
    std::cerr << "failed to save pcd: " << path.string() << "\n";
    return false;
  }

  std::cout << "saved: " << path.string()
            << " (" << cloud.size() << " pts)\n";
  return true;
}

static bool save_matrix_report(
    fs::path const& path,
    std::string const& title,
    Eigen::Matrix4d const& matrix,
    float overlap_ratio_src,
    float overlap_ratio_tgt)
{
  std::error_code ec;
  fs::create_directories(path.parent_path(), ec);

  std::ofstream out(path);
  if(!out)
  {
    std::cerr << "failed to open: " << path.string() << "\n";
    return false;
  }

  out << std::fixed << std::setprecision(6);
  out << title << "\n";
  out << "overlap_ratio_src: " << std::setprecision(4) << overlap_ratio_src
      << std::setprecision(6) << "\n";
  out << "overlap_ratio_tgt: " << std::setprecision(4) << overlap_ratio_tgt
      << std::setprecision(6) << "\n\n";
  out << Ddx::Common::eigen44_to_print_string(matrix) << "\n\n";
  out << "South: " << Ddx::Common::matrix_to_south_string(matrix) << "\n";
  out.close();

  std::cout << "saved: " << path.string() << "\n";
  return true;
}

static bool save_overlap_clouds(
    fs::path const& output_dir,
    std::string const& stage_prefix,
    pcl::PointCloud<pcl::PointXYZ> const& source,
    pcl::PointCloud<pcl::PointXYZ> const& target,
    Eigen::Matrix4d const& source_to_target,
    float voxel_size,
    float& overlap_ratio_src,
    float& overlap_ratio_tgt)
{
  Ddx::PipelineOverlapContext overlap_ctx(
      voxel_size,
      voxel_size);
  overlap_ctx.setTarget(target.makeShared());

  const auto region_result = overlap_ctx.computeOverlapRegion(
      source,
      source_to_target);

  overlap_ratio_src = region_result.overlap_ratio_src;
  overlap_ratio_tgt = region_result.overlap_ratio_tgt;

  auto source_in_target = overlap_ctx.transformedSource();

  auto region_src =
      Ddx::OverlapEstimator::ExtractCloud<pcl::PointXYZ>(
          source_in_target,
          region_result.region_idx_src);

  auto region_tgt =
      Ddx::OverlapEstimator::ExtractCloud<pcl::PointXYZ>(
          target.makeShared(),
          region_result.region_idx_tgt);

  std::cout << stage_prefix << " region points: src="
            << region_src->size() << " tgt="
            << region_tgt->size() << "\n";

  const bool ok_src = save_pcd(
      output_dir / (stage_prefix + "_region_src.pcd"),
      *region_src);

  const bool ok_tgt = save_pcd(
      output_dir / (stage_prefix + "_region_tgt.pcd"),
      *region_tgt);

  return ok_src && ok_tgt;
}

int wmain(int argc, wchar_t* argv[])
{
  const auto total_t0 = std::chrono::steady_clock::now();

  const fs::path default_source = LR"(D:\架站数据\黄埔军校\SLAS\hp001.slas)";
  const fs::path default_target = LR"(D:\架站数据\黄埔军校\SLAS\hp002.slas)";

  const fs::path default_overlap_output =
      LR"(D:\架站数据\黄埔军校\overlap)";

  const fs::path source_path = (argc >= 2) ? to_path(argv[1]) : default_source;
  const fs::path target_path = (argc >= 3) ? to_path(argv[2]) : default_target;

  const std::string source_file = narrow_path(source_path);
  const std::string target_file = narrow_path(target_path);

  const float voxel_size = (argc >= 4) ? std::stof(narrow_path(to_path(argv[3]))) : 0.3f;
  const fs::path overlap_output_root =
      (argc >= 5) ? to_path(argv[4]) : default_overlap_output;

  const fs::path pair_output_dir =
      overlap_output_root
      / (source_path.stem().string() + "_" + target_path.stem().string());

  std::cout << "=== global reg + FastGICP pipeline test ===\n";
  std::cout << "SLAS load: subtract header offset = true\n";
  std::cout << "voxel_size (coarse + fine downsample): " << voxel_size << "\n";
  std::cout << "overlap output dir: " << narrow_path(pair_output_dir) << "\n";
  std::cout << "source: " << source_file << "\n";
  std::cout << "target: " << target_file << "\n\n";
  std::cout.flush();

  if (!fs::exists(source_file))
  {
    std::cerr << "source file does not exist: " << source_file << "\n";
    return 1;
  }
  if (!fs::exists(target_file))
  {
    std::cerr << "target file does not exist: " << target_file << "\n";
    return 1;
  }

  StageTiming timing;

  pcl::PointCloud<pcl::PointXYZ> source_cloud;
  pcl::PointCloud<pcl::PointXYZ> target_cloud;
  if (!load_slas_subtract_offset(source_file, source_cloud, timing.load_source_ms))
  {
    std::cerr << "failed to load source SLAS\n";
    return 1;
  }
  if (!load_slas_subtract_offset(target_file, target_cloud, timing.load_target_ms))
  {
    std::cerr << "failed to load target SLAS\n";
    return 1;
  }

  Ddx::point_cloud_io::LasHeaderOffset src_offset{};
  Ddx::point_cloud_io::LasHeaderOffset tgt_offset{};
  Ddx::point_cloud_io::read_slas_header_offset(source_file, src_offset);
  Ddx::point_cloud_io::read_slas_header_offset(target_file, tgt_offset);
  std::cout << "source header offset (subtracted): (" << src_offset.x << ", " << src_offset.y
            << ", " << src_offset.z << ")\n";
  std::cout << "target header offset (subtracted): (" << tgt_offset.x << ", " << tgt_offset.y
            << ", " << tgt_offset.z << ")\n\n";

  const fs::path output_dir = fs::temp_directory_path() / "global_fast_gicp_pipeline_test";
  fs::create_directories(output_dir);

  std::vector<Eigen::Vector3f> src_vec;
  std::vector<Eigen::Vector3f> tgt_vec;
  cloud_to_vec(source_cloud, src_vec);
  cloud_to_vec(target_cloud, tgt_vec);

  Ddx::RobGlobRegParameter param;
  param.scan_name = source_path.stem().string();
  param.result_file = narrow_path(output_dir / "coarse_result.json");
  param.log_file = narrow_path(output_dir / "coarse_reg.log");
  param.voxel_size = voxel_size;
  param.close_log = false;

  std::cout << "running coarse global registration (RobustGlobalReg, in-memory)...\n";
  std::cout.flush();

  Ddx::RobustGlobalReg reg(param);
  const auto coarse_t0 = std::chrono::steady_clock::now();
  const bool coarse_ok = reg.run(src_vec, tgt_vec);
  timing.coarse_reg_ms = elapsed_ms(coarse_t0);

  std::cout << "coarse run(): " << (coarse_ok ? "success" : "failed") << "\n";
  std::cout << "coarse feature number: " << reg.getFeatureNumber() << "\n";

  if (!coarse_ok)
  {
    timing.total_ms = elapsed_ms(total_t0);
    print_timing_report(timing, source_cloud.size(), target_cloud.size(), 0, 0, 0.f, 0.f, 0.f, 0.f);
    return 1;
  }

  const Eigen::Matrix4d coarse_T = reg.getFinalTransformationMatrix();
  print_matrix("全局配准矩阵 (source -> target)", coarse_T);
  reg.saveResult(true);

  double coarse_src_ds_ms = 0.0;
  double coarse_tgt_ds_ms = 0.0;
  auto source_coarse = octree_downsample(source_cloud, voxel_size, coarse_src_ds_ms);
  auto target_coarse = octree_downsample(target_cloud, voxel_size, coarse_tgt_ds_ms);

  const auto coarse_overlap_t0 = std::chrono::steady_clock::now();
  float coarse_overlap_src = 0.f;
  float coarse_overlap_tgt = 0.f;
  const bool coarse_overlap_saved = save_overlap_clouds(
      pair_output_dir,
      "coarse",
      *source_coarse,
      *target_coarse,
      coarse_T,
      voxel_size,
      coarse_overlap_src,
      coarse_overlap_tgt);
  timing.coarse_overlap_ms = elapsed_ms(coarse_overlap_t0);

  save_matrix_report(
      pair_output_dir / "coarse_matrix.txt",
      "coarse registration matrix (source -> target)",
      coarse_T,
      coarse_overlap_src,
      coarse_overlap_tgt);

  std::cout << "\ncoarse region ratio (src/tgt, downsampled @ " << voxel_size
            << " m): " << coarse_overlap_src << " / "
            << coarse_overlap_tgt << "\n";
  if(!coarse_overlap_saved)
    std::cerr << "warning: coarse overlap cloud export incomplete\n";

  auto source_fine = octree_downsample(source_cloud, voxel_size, timing.fine_downsample_ms);
  double target_ds_ms = 0.0;
  auto target_fine = octree_downsample(target_cloud, voxel_size, target_ds_ms);
  timing.fine_downsample_ms += target_ds_ms;

  std::cout << "\nrunning fine registration (FastGICP) on downsampled clouds...\n";
  std::cout << "  fine source points: " << source_fine->size() << "\n";
  std::cout << "  fine target points: " << target_fine->size() << "\n";
  std::cout.flush();

  Eigen::Matrix4d fine_T = Eigen::Matrix4d::Identity();
  bool converged = false;
  double fitness = 0.0;
  const bool fine_ok = run_fine_gicp(*source_fine, *target_fine, coarse_T, fine_T, converged,
                                     fitness, timing.fine_gicp_ms);

  std::cout << "fine run(): " << (fine_ok ? "converged" : "not converged") << "\n";
  std::cout << "fine fitness score: " << fitness << "\n";

  print_matrix("精配准矩阵 (source -> target)", fine_T);

  const auto fine_overlap_t0 = std::chrono::steady_clock::now();
  float fine_overlap_src = 0.f;
  float fine_overlap_tgt = 0.f;
  const bool fine_overlap_saved = save_overlap_clouds(
      pair_output_dir,
      "fine",
      *source_fine,
      *target_fine,
      fine_T,
      voxel_size,
      fine_overlap_src,
      fine_overlap_tgt);
  timing.fine_overlap_ms = elapsed_ms(fine_overlap_t0);

  save_matrix_report(
      pair_output_dir / "fine_matrix.txt",
      "fine registration matrix (source -> target)",
      fine_T,
      fine_overlap_src,
      fine_overlap_tgt);

  std::cout << "fine region ratio (src/tgt, downsampled @ " << voxel_size
            << " m): " << fine_overlap_src << " / "
            << fine_overlap_tgt << "\n";
  if(!fine_overlap_saved)
    std::cerr << "warning: fine overlap cloud export incomplete\n";

  std::cout << "\n========== 配准矩阵汇总 ==========\n";
  print_matrix("【1】全局配准矩阵", coarse_T);
  print_matrix("【2】精配准矩阵", fine_T);
  std::cout << "================================\n";

  const Eigen::Matrix4d delta = coarse_T.inverse() * fine_T;
  const double trans_delta = delta.block<3, 1>(0, 3).norm();
  const Eigen::AngleAxisd rot_delta(delta.block<3, 3>(0, 0));
  std::cout << "\nrefinement delta (coarse -> fine):\n";
  std::cout << "  translation norm: " << trans_delta << " m\n";
  std::cout << "  rotation angle  : " << rot_delta.angle() * 180.0 / 3.14159265358979323846 << " deg\n";

  timing.total_ms = elapsed_ms(total_t0);
  print_timing_report(timing, source_cloud.size(), target_cloud.size(), source_fine->size(),
                      target_fine->size(), coarse_overlap_src, coarse_overlap_tgt,
                      fine_overlap_src, fine_overlap_tgt);

  std::cout << "\ncoarse result saved: " << param.result_file << "\n";
  std::cout << "overlap exports dir: " << narrow_path(pair_output_dir) << "\n";
  return fine_ok ? 0 : 2;
}
