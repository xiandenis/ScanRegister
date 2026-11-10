#include "rough_global_reg.hpp"

#include <registration_initializer/registration_initializer.hpp>
#include <registration_type/lidar_frame_io.hpp>
#include <logging/logging_initi.hpp>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static Ddx::ScanInfo makeScan(fs::path const& las_path)
{
	Ddx::ScanInfo info;
	info.name_ = las_path.stem().string();
	info.lasFn_ = las_path.string();
	info.sensor_ = Ddx::SENSOR_STAND;
	return info;
}

static Ddx::LidarFrame makeFrame(fs::path const& las_path, fs::path const& info_root)
{
	Ddx::LidarFrame frame;
	frame.name_ = las_path.stem().string();
	frame.lasFn_ = las_path.string();
	frame.info_folder_ = (info_root / frame.name_).string();
	frame.sensor_ = Ddx::SENSOR_STAND;
	// Stand-scan test default: file CRS already station-centered unless overridden.
	frame.has_station_origin_ = true;
	frame.station_origin_ = Eigen::Vector3d::Zero();
	return frame;
}

static bool ensureInitialized(Ddx::LidarFrame& frame, bool force_init)
{
	fs::create_directories(frame.info_folder_);

	std::string const info_path = frame.getInfoPath();
	if (!force_init && fs::exists(info_path) && fs::exists(frame.info_folder_ + "/Root_p.pcd")) {
		if (Ddx::LidarFrameIO::loadLidarFrame(info_path, frame)) {
			std::cout << "  reuse existing info: " << info_path << "\n";
			return true;
		}
	}

	std::cout << "  initializing " << frame.name_
	          << (force_init ? " (force)" : "") << " ...\n";
	Ddx::RegistrationInitializer init;
	init.setUseRoot(true);
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
	return true;
}

static int runScanInfoApi(std::string const& src, std::string const& tgt, float resolution)
{
	std::cout << "=== rough_global_reg ScanInfo API ===\n";
	Ddx::RoughGlobalReg reg;
	if (!reg.run(makeScan(src), makeScan(tgt), resolution)) {
		std::cerr << "rough registration failed\n";
		return 1;
	}
	auto const result = reg.getAlignResult();
	std::cout << "OK " << result.src_name_ << " -> " << result.tgt_name_
	          << " feature_num=" << result.feature_num_ << "\n";
	std::cout << "matrix:\n" << result.matrix_ << "\nALL PASSED\n";
	return 0;
}

#ifdef _WIN32
int wmain(int argc, wchar_t* argv[])
#else
int main(int argc, char* argv[])
#endif
{
	fs::path src_las = "/home/ddx/data/test_las/106_1_zjsq-12-29-106-jg001.las";
	fs::path tgt_las = "/home/ddx/data/test_las/106_1_zjsq-12-29-106-jg002.las";
	fs::path info_root = "/home/ddx/data/info";
	bool force_init = false;
	bool scan_info = false;
	float scan_resolution = 0.4f;

	std::vector<std::string> positional;
	for (int i = 1; i < argc; ++i) {
#ifdef _WIN32
		std::string const arg = fs::path(argv[i]).string();
#else
		std::string const arg = argv[i];
#endif
		if (arg == "--force-init") {
			force_init = true;
		} else if (arg == "--scan-info") {
			scan_info = true;
		} else {
			positional.push_back(arg);
		}
	}

	Ddx::initiLog("logs", "rough_global_reg_test", 1);

	if (scan_info) {
		if (positional.size() < 2 || positional.size() > 3) {
			std::cerr << "usage: rough_global_reg_test --scan-info <src> <tgt> [resolution=0.4]\n";
			return 1;
		}
		if (positional.size() == 3) {
			scan_resolution = std::stof(positional[2]);
		}
		return runScanInfoApi(positional[0], positional[1], scan_resolution);
	}

	if (positional.size() == 3) {
		src_las = positional[0];
		tgt_las = positional[1];
		info_root = positional[2];
	} else if (!positional.empty()) {
		std::cerr << "usage: rough_global_reg_test [src.las tgt.las info_root] [--force-init]\n"
		          << "       rough_global_reg_test --scan-info <src> <tgt> [resolution=0.4]\n";
		return 1;
	}

	std::cout << "=== rough_global_reg LidarFrame API (voxel_3) ===\n";
	std::cout << "src: " << src_las << "\n";
	std::cout << "tgt: " << tgt_las << "\n";
	std::cout << "info_root: " << info_root << "\n";

	if (!fs::exists(src_las) || !fs::exists(tgt_las)) {
		std::cerr << "las file missing\n";
		return 1;
	}
	fs::create_directories(info_root);

	Ddx::LidarFrame source = makeFrame(src_las, info_root);
	Ddx::LidarFrame target = makeFrame(tgt_las, info_root);

	std::cout << "\n[1] registration_initializer\n";
	if (!ensureInitialized(source, force_init) || !ensureInitialized(target, force_init)) {
		return 1;
	}

	float resolution = static_cast<float>((source.voxel_size_ + target.voxel_size_) * 0.5);
	std::cout << "\n[2] rough_global_reg LidarFrame (resolution=" << resolution << ")\n";
	std::cout << "  src.voxel_3=" << source.getCloudPath("voxel_3") << "\n";
	std::cout << "  tgt.voxel_3=" << target.getCloudPath("voxel_3") << "\n";

	Ddx::RoughGlobalReg reg;
	if (!reg.run(source, target, resolution)) {
		std::cerr << "rough registration failed\n";
		return 1;
	}

	auto const result = reg.getAlignResult();
	std::cout << "registration OK (matrix in file/world frame)\n";
	std::cout << "  " << result.src_name_ << " -> " << result.tgt_name_
	          << " feature_num=" << result.feature_num_ << "\n";
	std::cout << "  matrix:\n" << result.matrix_ << "\n";
	std::cout << "ALL PASSED\n";
	return 0;
}
