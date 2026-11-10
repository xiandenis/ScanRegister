#include "registration_initializer.hpp"

#include <registration_type/lidar_frame_io.hpp>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

static int g_failures = 0;

static void check(bool ok, char const* msg)
{
	if (!ok) {
		++g_failures;
		std::cerr << "FAIL: " << msg << "\n";
	}
}

static bool is_point_cloud_path(fs::path const& p)
{
	auto const ext = p.extension().string();
	return ext == ".slas" || ext == ".las" || ext == ".pcd" || ext == ".ply";
}

static pcl::PointCloud<pcl::PointXYZ> make_sphere_cloud(int n)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.reserve(static_cast<size_t>(n));
	for (int i = 0; i < n; ++i) {
		float const theta = 6.2831853f * static_cast<float>(i % 100) / 100.f;
		float const phi = 3.1415926f * static_cast<float>(i % 50) / 50.f * 0.98f + 0.01f;
		float const r = 10.f + static_cast<float>(i % 7) * 0.01f;
		pcl::PointXYZ p;
		p.x = r * std::sin(phi) * std::cos(theta);
		p.y = r * std::sin(phi) * std::sin(theta);
		p.z = r * std::cos(phi);
		cloud.push_back(p);
	}
	return cloud;
}

static bool run_pipeline(bool use_root, Ddx::LidarFrame const& scan_info)
{
	fs::create_directories(scan_info.info_folder_);

	Ddx::RegistrationInitializer init;
	init.setUseRoot(use_root);

	Ddx::LidarFrame frame;
	frame.name_ = scan_info.name_;
	frame.lasFn_ = scan_info.lasFn_;
	frame.info_folder_ = scan_info.info_folder_;
	frame.has_station_origin_ = scan_info.has_station_origin_;
	frame.station_origin_ = scan_info.station_origin_;

	std::cout << "  --- use_root=" << use_root << " ---\n";
	std::cout << "  name: " << frame.name_ << "\n";
	std::cout << "  lasFn: " << frame.lasFn_ << "\n";
	std::cout << "  info_folder: " << frame.info_folder_ << "\n";
	std::cout << "  has_station_origin=" << frame.has_station_origin_
	          << " station_origin=(" << frame.station_origin_.x() << ", "
	          << frame.station_origin_.y() << ", " << frame.station_origin_.z() << ")\n";

	if (!init.initalState(frame, {3, 10}, true)) {
		std::cerr << "  initalState failed\n";
		return false;
	}

	if (!init.initializd(frame, true)) {
		std::cerr << "  initializd failed\n";
		return false;
	}

	if (!init.check(frame)) {
		std::cerr << "  check failed\n";
		return false;
	}

	check(frame.subvoxel_vec_.size() == 2, "expected two subvoxels");
	check((frame.state_ & Ddx::HAS_CALCU) == Ddx::HAS_CALCU, "frame state should be HAS_CALCU");
	check(frame.voxel_size_ > 0.0, "voxel_size should be positive");
	check(frame.point_number_ >= 10, "point_number should reflect loaded cloud");
	check(frame.has_station_origin_, "station origin declaration should persist");

	for (auto const& voxel : frame.subvoxel_vec_) {
		check((voxel.state_ & Ddx::HAS_CALCU) == Ddx::HAS_CALCU, "subvoxel should be computed");
		check(voxel.point_number_ > 1, "subvoxel point count");
		check(fs::exists(frame.getCloudPath(voxel.name_)), "subvoxel cloud pcd missing");
		check(fs::exists(frame.getNormalPath(voxel.name_)), "subvoxel normal pcd missing");
		std::cout << "  " << voxel.name_ << ": points=" << voxel.point_number_
		          << " cloud=" << frame.getCloudPath(voxel.name_)
		          << "\n";
	}

	if (use_root) {
		check((frame.root_.state_ & Ddx::HAS_CALCU) == Ddx::HAS_CALCU, "root should be computed");
		check(!frame.root_.is_raw_input_, "root should not be raw input when use_root");
		check(fs::exists(frame.getCloudPath("Root")), "Root_p.pcd missing");
		check(frame.root_.point_number_ >= 10, "root point_number should be valid");
		check(frame.root_.point_number_ <= frame.point_number_, "root points should not exceed original");
		std::cout << "  root_filtered_pts=" << frame.root_.point_number_
		          << " original_pts=" << frame.point_number_ << "\n";
	}
	else {
		check(frame.root_.is_raw_input_, "root should be raw input when !use_root");
		check(frame.getCloudPath("Root") == frame.lasFn_, "root cloud path should be lasFn");
	}

	// Station origin must survive LidarFrameIO with the station-frame caches.
	std::string const info_path = frame.getInfoPath();
	Ddx::LidarFrame loaded;
	check(Ddx::LidarFrameIO::loadLidarFrame(info_path, loaded), "reload LidarFrame JSON");
	check(loaded.has_station_origin_, "JSON roundtrip has_station_origin");
	check((loaded.station_origin_ - frame.station_origin_).norm() < 1e-9,
	      "JSON roundtrip station_origin");

	std::cout << "  point_number=" << frame.point_number_
	          << " voxel_size=" << frame.voxel_size_
	          << " root_pts=" << frame.root_.point_number_
	          << " info=" << info_path << "\n";
	return true;
}

static Ddx::LidarFrame make_scan_info_from_slas(fs::path const& slas_path, fs::path const& info_root)
{
	Ddx::LidarFrame frame;
	frame.name_ = slas_path.stem().string();
	frame.lasFn_ = slas_path.string();
	frame.info_folder_ = (info_root / frame.name_).string();
	// Real stand-scan inputs are assumed station-centered in file CRS unless caller overrides.
	frame.has_station_origin_ = true;
	frame.station_origin_ = Eigen::Vector3d::Zero();
	return frame;
}

static bool run_undeclared_station_origin_case(fs::path const& work_dir, fs::path const& las_path)
{
	std::cout << "  --- undeclared station_origin (expect fail) ---\n";
	Ddx::LidarFrame frame;
	frame.name_ = "no_station";
	frame.lasFn_ = las_path.string();
	frame.info_folder_ = (work_dir / "info_undeclared").string();
	frame.has_station_origin_ = false;
	fs::create_directories(frame.info_folder_);

	Ddx::RegistrationInitializer init;
	if (!init.initalState(frame, {3, 10}, true)) {
		std::cerr << "  initalState failed unexpectedly\n";
		return false;
	}
	if (init.initializd(frame, true)) {
		std::cerr << "  initializd should reject undeclared station_origin\n";
		return false;
	}
	std::cout << "  undeclared station_origin rejected as expected\n";
	return true;
}

static int run_synthetic_test(fs::path const& work_dir)
{
	fs::create_directories(work_dir);
	fs::path const las_path = work_dir / "input.pcd";

	pcl::PointCloud<pcl::PointXYZ> const cloud = make_sphere_cloud(5000);
	if (pcl::io::savePCDFileBinary(las_path.string(), cloud) != 0) {
		std::cerr << "failed to write synthetic pcd\n";
		return 1;
	}

	std::cout << "=== registration_initializer synthetic test ===\n";
	std::cout << "input points: " << cloud.size() << "\n";

	Ddx::LidarFrame scan;
	scan.name_ = "test_frame";
	scan.lasFn_ = las_path.string();
	scan.info_folder_ = (work_dir / "info").string();
	scan.has_station_origin_ = true;
	scan.station_origin_ = Eigen::Vector3d::Zero();

	int scenario_failures = 0;
	if (!run_pipeline(true, scan)) {
		++scenario_failures;
	}

	scan.info_folder_ = (work_dir / "info_no_root").string();
	if (!run_pipeline(false, scan)) {
		++scenario_failures;
	}

	if (!run_undeclared_station_origin_case(work_dir, las_path)) {
		++scenario_failures;
	}

	if (scenario_failures == 0 && g_failures == 0) {
		std::cout << "ALL PASSED\n";
		return 0;
	}
	std::cerr << scenario_failures << " scenario(s) failed, " << g_failures << " assertion(s) failed\n";
	return 1;
}

static int run_real_scan_test(fs::path const& slas_path, fs::path const& info_root, bool try_no_root)
{
	if (!fs::exists(slas_path)) {
		std::cerr << "scan file not found: " << slas_path << "\n";
		return 1;
	}

	std::cout << "=== registration_initializer real scan test ===\n";
	std::cout << "slas: " << slas_path << "\n";
	std::cout << "info_root: " << info_root << "\n";

	Ddx::LidarFrame scan = make_scan_info_from_slas(slas_path, info_root);

	int scenario_failures = 0;
	if (!run_pipeline(true, scan)) {
		++scenario_failures;
	}

	if (try_no_root) {
		scan = make_scan_info_from_slas(slas_path, info_root);
		scan.info_folder_ = (fs::path(scan.info_folder_) / "no_root").string();
		if (!run_pipeline(false, scan)) {
			++scenario_failures;
		}
	}

	if (scenario_failures == 0 && g_failures == 0) {
		std::cout << "ALL PASSED\n";
		return 0;
	}
	std::cerr << scenario_failures << " scenario(s) failed, " << g_failures << " assertion(s) failed\n";
	return 1;
}

#ifdef _WIN32
int wmain(int argc, wchar_t* argv[])
#else
int main(int argc, char* argv[])
#endif
{
#ifdef _WIN32
	fs::path const default_slas = L"D:\\架站数据\\黄埔军校\\SLAS\\hp006.slas";
	fs::path const default_info_root = L"D:\\架站数据\\黄埔军校\\info";
#else
	fs::path const default_slas = "/tmp/hp006.slas";
	fs::path const default_info_root = "/tmp/info";
#endif

	if (argc >= 2) {
		fs::path arg1 = argv[1];
		if (is_point_cloud_path(arg1)) {
			fs::path info_root = (argc >= 3) ? fs::path(argv[2]) : default_info_root;
#ifdef _WIN32
			bool try_no_root = (argc >= 4) && (std::wstring(argv[3]) == L"--with-no-root");
#else
			bool try_no_root = (argc >= 4) && (std::string(argv[3]) == "--with-no-root");
#endif
			return run_real_scan_test(arg1, info_root, try_no_root);
		}
		return run_synthetic_test(arg1);
	}

	fs::path const default_synth =
#ifdef _WIN32
		L"build\\tmp_registration_initializer_test";
#else
		"build/tmp_registration_initializer_test";
#endif
	return run_synthetic_test(default_synth);
}
