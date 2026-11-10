#include "robust_global_reg.hpp"

#include <common/common.hpp>
#include <logging/logging.hpp>
#include <point_cloud_io/point_cloud_io.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static fs::path to_path(wchar_t const* wpath)
{
	return fs::path(wpath);
}

static std::string narrow_path(fs::path const& p)
{
	return p.string();
}

static void print_transformation(Eigen::Matrix4d const& matrix)
{
	std::cout << std::fixed << std::setprecision(6);
	std::cout << "\n=== registration matrix (4x4) ===\n";
	std::cout << Ddx::Common::eigen44_to_print_string(matrix) << "\n";
	std::cout << "South format: "
	          << Ddx::Common::matrix_to_south_string(matrix) << "\n";
}

static std::string resolve_source_path(fs::path const& requested)
{
	if (fs::exists(requested))
		return narrow_path(requested);

	const fs::path fallback = LR"(D:\架站数据\黄埔军校\SLAS\hp001.slas)";
	if (fs::exists(fallback))
	{
		std::cerr << "warning: source not found at requested path:\n  "
		          << narrow_path(requested) << "\n";
		std::cerr << "using fallback from hp001.info lasFn:\n  "
		          << narrow_path(fallback) << "\n";
		return narrow_path(fallback);
	}

	return narrow_path(requested);
}

static bool cloud_to_vec(pcl::PointCloud<pcl::PointXYZ> const& cloud,
                         std::vector<Eigen::Vector3f>& out)
{
	out.resize(cloud.size());
	for (std::size_t i = 0; i < cloud.size(); ++i)
	{
		out[i] = Eigen::Vector3f(cloud[i].x, cloud[i].y, cloud[i].z);
	}
	return !out.empty();
}

int wmain(int argc, wchar_t* argv[])
{
	const fs::path default_source =
		LR"(D:\架站数据\黄埔军校\south_reg2\info\hp001\hp001.slas)";
	const fs::path default_target =
		LR"(D:\架站数据\黄埔军校\SLAS\hp002.slas)";

	const fs::path source_path =
		(argc >= 2) ? to_path(argv[1]) : default_source;
	const fs::path target_path =
		(argc >= 3) ? to_path(argv[2]) : default_target;

	const std::string source_file = resolve_source_path(source_path);
	const std::string target_file = narrow_path(target_path);

	std::cout << "=== robust_global_reg test ===\n";
	std::cout << "voxel_size: 0.3\n";
	std::cout << "consumer I/O: point_cloud_io load (caller-managed frame)\n";
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

	Ddx::point_cloud_io::FramedCloud<pcl::PointXYZ> src_framed;
	Ddx::point_cloud_io::FramedCloud<pcl::PointXYZ> tgt_framed;
	if (Ddx::point_cloud_io::load(source_file, src_framed) != 0)
	{
		std::cerr << "failed to load source\n";
		return 1;
	}
	if (Ddx::point_cloud_io::load(target_file, tgt_framed) != 0)
	{
		std::cerr << "failed to load target\n";
		return 1;
	}

	// Job-level shared frame (deferred API): rebase target into source FramedCloud.origin.
	{
		const Eigen::Vector3d delta = tgt_framed.origin - src_framed.origin;
		for (auto& p : tgt_framed.cloud.points)
		{
			p.x = static_cast<float>(static_cast<double>(p.x) + delta.x());
			p.y = static_cast<float>(static_cast<double>(p.y) + delta.y());
			p.z = static_cast<float>(static_cast<double>(p.z) + delta.z());
		}
		tgt_framed.origin = src_framed.origin;
	}
	std::cout << "shared origin (source): (" << src_framed.origin.x() << ", "
	          << src_framed.origin.y() << ", " << src_framed.origin.z() << ")\n";
	std::cout << "src points: " << src_framed.cloud.size()
	          << "  tgt points: " << tgt_framed.cloud.size() << "\n";

	std::vector<Eigen::Vector3f> src_vec;
	std::vector<Eigen::Vector3f> tgt_vec;
	if (!cloud_to_vec(src_framed.cloud, src_vec) || !cloud_to_vec(tgt_framed.cloud, tgt_vec))
	{
		std::cerr << "empty cloud after load\n";
		return 1;
	}

	const fs::path output_dir = fs::temp_directory_path() / "robust_global_reg_test";
	fs::create_directories(output_dir);

	Ddx::RobGlobRegParameter param;
	param.scan_name = fs::path(source_file).stem().string();
	param.result_file = narrow_path(output_dir / "result.json");
	param.log_file = narrow_path(output_dir / "reg.log");
	param.voxel_size = 0.3f;
	param.close_log = false;

	std::cout << "running RobustGlobalReg (in-memory)...\n";
	std::cout.flush();

	Ddx::RobustGlobalReg reg(param);
	const bool ok = reg.run(src_vec, tgt_vec);

	std::cout << "run(): " << (ok ? "success" : "failed") << "\n";
	std::cout << "feature number: " << reg.getFeatureNumber() << "\n";

	if (ok)
	{
		print_transformation(reg.getFinalTransformationMatrix());
		reg.saveResult(true);
		std::cout << "result saved: " << param.result_file << "\n";
	}

	return ok ? 0 : 1;
}
