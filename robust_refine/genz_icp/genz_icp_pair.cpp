#include <genz_icp/genz_icp.hpp>

#include <point_cloud_io/point_cloud_io.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

void PrintUsage(const char* argv0)
{
    std::cerr
        << "Usage:\n  " << argv0
        << " <target> <source> [init.txt]\n"
           "    [--voxel-size 0.05] [--max-distance 1] [--kernel <2/3>]\n"
           "    [--min-range <m>] [--max-range <m>]\n"
           "\n"
           "Target is the map frame. Output T satisfies p_target = T * p_source.\n"
           "Does NOT voxel-downsample the input clouds (indoor / pre-voxelized).\n"
           "--voxel-size only sets VoxelHashMap cell size.\n"
           "init.txt: 16 row-major doubles (4x4). Default identity.\n";
}

bool LoadInit(const std::string& path, Eigen::Matrix4d& M)
{
    std::ifstream in(path);
    if (!in) {
        std::cerr << "genz_icp_pair: cannot open init: " << path << "\n";
        return false;
    }
    M = Eigen::Matrix4d::Identity();
    int count = 0;
    double v = 0.0;
    while (in >> v) {
        if (count >= 16) {
            std::cerr << "genz_icp_pair: init has more than 16 numbers\n";
            return false;
        }
        M(count / 4, count % 4) = v;
        ++count;
    }
    if (count != 16) {
        std::cerr << "genz_icp_pair: init must have exactly 16 doubles, got " << count << "\n";
        return false;
    }
    return true;
}

bool ParseArgs(int argc, char** argv, std::string& target_path, std::string& source_path,
               std::string& init_path, genz_icp::GenZICP::Config& cfg)
{
    cfg = genz_icp::GenZICP::Config::Indoor();
    init_path.clear();

    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* name) -> bool {
            if (i + 1 >= argc) {
                std::cerr << "genz_icp_pair: missing value for " << name << "\n";
                return false;
            }
            return true;
        };
        if (a == "--voxel-size") {
            if (!need("--voxel-size")) return false;
            cfg.voxel_size = std::stod(argv[++i]);
        } else if (a == "--max-distance") {
            if (!need("--max-distance")) return false;
            cfg.max_distance = std::stod(argv[++i]);
        } else if (a == "--kernel") {
            if (!need("--kernel")) return false;
            cfg.kernel = std::stod(argv[++i]);
        } else if (a == "--min-range") {
            if (!need("--min-range")) return false;
            cfg.min_range = std::stod(argv[++i]);
            cfg.use_range_filter = true;
        } else if (a == "--max-range") {
            if (!need("--max-range")) return false;
            cfg.max_range = std::stod(argv[++i]);
            cfg.use_range_filter = true;
        } else if (a.rfind("--", 0) == 0) {
            std::cerr << "genz_icp_pair: unknown flag: " << a << "\n";
            return false;
        } else {
            positional.push_back(a);
        }
    }

    if (positional.size() < 2 || positional.size() > 3) {
        return false;
    }
    target_path = positional[0];
    source_path = positional[1];
    if (positional.size() == 3) {
        init_path = positional[2];
    }
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    std::string target_path;
    std::string source_path;
    std::string init_path;
    genz_icp::GenZICP::Config cfg = genz_icp::GenZICP::Config::Indoor();

    if (!ParseArgs(argc, argv, target_path, source_path, init_path, cfg)) {
        PrintUsage(argv[0]);
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    if (Ddx::point_cloud_io::load(target_path, *target_pcl) != 0) {
        std::cerr << "genz_icp_pair: failed to load target: " << target_path << "\n";
        return 1;
    }
    if (Ddx::point_cloud_io::load(source_path, *source_pcl) != 0) {
        std::cerr << "genz_icp_pair: failed to load source: " << source_path << "\n";
        return 1;
    }
    if (target_pcl->empty() || source_pcl->empty()) {
        std::cerr << "genz_icp_pair: empty cloud after load\n";
        return 1;
    }

    Eigen::Matrix4d T0 = Eigen::Matrix4d::Identity();
    if (!init_path.empty() && !LoadInit(init_path, T0)) {
        return 1;
    }

    std::cout << "genz_icp_pair: map_voxel=" << cfg.voxel_size << " max_distance=" << cfg.max_distance
              << " kernel=" << cfg.kernel << " pts src/tgt=" << source_pcl->size() << "/"
              << target_pcl->size() << " (no cloud downsample)\n"
              << std::flush;

    genz_icp::GenZICP icp(cfg);
    icp.setSourceCloud(source_pcl);
    icp.setTargetCloud(target_pcl);
    if (!icp.align(T0)) {
        std::cerr << "genz_icp_pair: registration failed (non-finite pose or zero correspondences)\n";
        return 2;
    }

    const auto& r = icp.lastResult();
    const Eigen::Matrix4d& M = r.T;
    std::cout.setf(std::ios::fixed);
    std::cout.precision(9);
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            if (col) std::cout << ' ';
            std::cout << M(row, col);
        }
        std::cout << '\n';
    }
    std::cout << "translation_norm " << M.block<3, 1>(0, 3).norm() << '\n';
    std::cout << "planar " << r.planar_count << " non_planar " << r.non_planar_count << '\n';
    return 0;
}
