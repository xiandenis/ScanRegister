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
           "Target is the map frame. Output T satisfies p_target = T * p_source (world).\n"
           "Clouds load as FramedCloud (local); world init is converted to local before align,\n"
           "and the printed T is folded back to world with source/target origins.\n"
           "Does NOT voxel-downsample the input clouds (indoor / pre-voxelized).\n"
           "--voxel-size only sets VoxelHashMap cell size.\n"
           "init.txt: 16 row-major doubles (4x4) in WORLD frame. Default identity.\n";
}

/** M_loc = T(-O_t) * M_world * T(O_s); p_t' ≈ M_loc * p_s'. */
Eigen::Matrix4d WorldToLocalInit(Eigen::Matrix4d const& M_world, Eigen::Vector3d const& O_s,
                                 Eigen::Vector3d const& O_t)
{
    Eigen::Matrix4d T_os = Eigen::Matrix4d::Identity();
    T_os.block<3, 1>(0, 3) = O_s;
    Eigen::Matrix4d T_neg_ot = Eigen::Matrix4d::Identity();
    T_neg_ot.block<3, 1>(0, 3) = -O_t;
    return T_neg_ot * M_world * T_os;
}

/** M_world = T(O_t) * M_loc * T(-O_s). */
Eigen::Matrix4d LocalToWorldPose(Eigen::Matrix4d const& M_loc, Eigen::Vector3d const& O_s,
                                 Eigen::Vector3d const& O_t)
{
    Eigen::Matrix4d T_ot = Eigen::Matrix4d::Identity();
    T_ot.block<3, 1>(0, 3) = O_t;
    Eigen::Matrix4d T_neg_os = Eigen::Matrix4d::Identity();
    T_neg_os.block<3, 1>(0, 3) = -O_s;
    return T_ot * M_loc * T_neg_os;
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

    Ddx::point_cloud_io::FramedCloud<pcl::PointXYZ> target_framed;
    Ddx::point_cloud_io::FramedCloud<pcl::PointXYZ> source_framed;
    if (Ddx::point_cloud_io::load(target_path, target_framed) != 0) {
        std::cerr << "genz_icp_pair: failed to load target: " << target_path << "\n";
        return 1;
    }
    if (Ddx::point_cloud_io::load(source_path, source_framed) != 0) {
        std::cerr << "genz_icp_pair: failed to load source: " << source_path << "\n";
        return 1;
    }
    const Eigen::Vector3d O_s = source_framed.origin;
    const Eigen::Vector3d O_t = target_framed.origin;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcl(
        new pcl::PointCloud<pcl::PointXYZ>(std::move(target_framed.cloud)));
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_pcl(
        new pcl::PointCloud<pcl::PointXYZ>(std::move(source_framed.cloud)));
    if (target_pcl->empty() || source_pcl->empty()) {
        std::cerr << "genz_icp_pair: empty cloud after load\n";
        return 1;
    }

    Eigen::Matrix4d T0_world = Eigen::Matrix4d::Identity();
    if (!init_path.empty() && !LoadInit(init_path, T0_world)) {
        return 1;
    }
    const Eigen::Matrix4d T0_local = WorldToLocalInit(T0_world, O_s, O_t);

    std::cout << "genz_icp_pair: map_voxel=" << cfg.voxel_size << " max_distance=" << cfg.max_distance
              << " kernel=" << cfg.kernel << " pts src/tgt=" << source_pcl->size() << "/"
              << target_pcl->size() << " (no cloud downsample)\n"
              << "O_s=(" << O_s.transpose() << ") O_t=(" << O_t.transpose()
              << ") init converted world→local\n"
              << std::flush;

    genz_icp::GenZICP icp(cfg);
    icp.setSourceCloud(source_pcl);
    icp.setTargetCloud(target_pcl);
    if (!icp.align(T0_local)) {
        std::cerr << "genz_icp_pair: registration failed (non-finite pose or zero correspondences)\n";
        return 2;
    }

    const auto& r = icp.lastResult();
    const Eigen::Matrix4d M = LocalToWorldPose(r.T, O_s, O_t);
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
