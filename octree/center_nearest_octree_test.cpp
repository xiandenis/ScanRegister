#include "center_nearest_octree.hpp"



#include <point_cloud_io/point_cloud_io.hpp>



#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>

#include <pcl/point_types.h>



#include <algorithm>

#include <chrono>

#include <filesystem>

#include <iomanip>

#include <iostream>

#include <string>

#include <vector>



namespace fs = std::filesystem;



using Clock = std::chrono::steady_clock;



static double elapsed_ms(Clock::time_point const& t0, Clock::time_point const& t1)

{

    return std::chrono::duration<double, std::milli>(t1 - t0).count();

}



#ifdef _WIN32

static fs::path to_path(wchar_t const* w)

{

    return fs::path(w);

}



static std::string narrow_path_for_pcl(fs::path const& p)

{

    return p.string();

}

#else

static fs::path to_path(char const* s)

{

    return fs::path(s);

}



static std::string narrow_path_for_pcl(fs::path const& p)

{

    return p.string();

}

#endif



static char const* format_name(Ddx::point_cloud_io::PointCloudFormat fmt)

{

    using F = Ddx::point_cloud_io::PointCloudFormat;

    switch (fmt)

    {

    case F::Las: return "LAS";

    case F::Slas: return "SLAS";

    case F::Ply: return "PLY";

    case F::Pcd: return "PCD";

    default: return "Unknown";

    }

}



static std::vector<fs::path> collect_input_files(fs::path const& input_path)

{

    std::vector<fs::path> files;

    if (fs::is_directory(input_path))

    {

        for (fs::directory_entry const& entry : fs::directory_iterator(input_path))

        {

            if (!entry.is_regular_file())

                continue;

            Ddx::point_cloud_io::PointCloudFormat const fmt =

                Ddx::point_cloud_io::detect_format(narrow_path_for_pcl(entry.path()));

            if (fmt != Ddx::point_cloud_io::PointCloudFormat::Unknown)

                files.push_back(entry.path());

        }

        std::sort(files.begin(), files.end());

    }

    else

    {

        files.push_back(input_path);

    }

    return files;

}



#ifdef _WIN32

int wmain(int argc, wchar_t** argv)

#else

int main(int argc, char** argv)

#endif

{

    fs::path input_path =

        fs::u8path(u8"D:\\架站数据\\氧屋测试数据\\1903\\SLAS");

    fs::path output_path =

        fs::u8path(u8"D:\\架站数据\\氧屋测试数据\\1903\\node_leaf_cloud_all.pcd");

    double min_leaf_size = 0.04;



    if (argc >= 2)

        input_path = to_path(argv[1]);

    if (argc >= 3)

        output_path = to_path(argv[2]);

    if (argc >= 4)

#ifdef _WIN32

        min_leaf_size = std::stod(to_path(argv[3]).string());

#else

        min_leaf_size = std::stod(argv[3]);

#endif



    if (!fs::exists(input_path))

    {

        std::cerr << "Input path not found: " << input_path.u8string() << std::endl;

        return 1;

    }



    std::vector<fs::path> const input_files = collect_input_files(input_path);

    if (input_files.empty())

    {

        std::cerr << "No supported point cloud files found under: "

                  << input_path.u8string() << std::endl;

        return 1;

    }



    Clock::time_point t_total_start = Clock::now();

    double ms_load = 0.0;

    double ms_insert = 0.0;

    double ms_collect = 0.0;

    double ms_export = 0.0;

    double ms_save = 0.0;



    using Sample = Ddx::octree::Sample<double>;

    Ddx::octree::CenterNearestOctree<double> tree(min_leaf_size);

    double const frame_time =

        std::chrono::duration<double>(Clock::now().time_since_epoch()).count();



    std::size_t num_input = 0;

    std::size_t num_accepted = 0;



    std::cout << "Input files:       " << input_files.size() << std::endl;

    std::cout << "min_leaf_size = " << min_leaf_size << std::endl;



    for (fs::path const& file_path : input_files)

    {

        std::string const load_path = narrow_path_for_pcl(file_path);

        Ddx::point_cloud_io::PointCloudFormat const fmt =

            Ddx::point_cloud_io::detect_format(load_path);

        if (fmt == Ddx::point_cloud_io::PointCloudFormat::Unknown)

        {

            std::cerr << "Skipping unsupported file: " << file_path.u8string() << std::endl;

            continue;

        }



        pcl::PointCloud<pcl::PointXYZRGB> cloud;

        {

            Clock::time_point t0 = Clock::now();

            if (Ddx::point_cloud_io::load<pcl::PointXYZRGB>(load_path, cloud) != 0)

            {

                std::cerr << "Failed to load: " << file_path.u8string() << std::endl;

                return 1;

            }

            ms_load += elapsed_ms(t0, Clock::now());

        }



        if (cloud.empty())

        {

            std::cerr << "Empty cloud: " << file_path.u8string() << std::endl;

            continue;

        }



        std::cout << "  [" << format_name(fmt) << "] "

                  << file_path.filename().u8string()

                  << " -> " << cloud.size() << " points";

        if (fmt == Ddx::point_cloud_io::PointCloudFormat::Las ||

            fmt == Ddx::point_cloud_io::PointCloudFormat::Slas)

        {

            std::cout << " (subtract header offset: "

                      << (subtract_las_offset ? "yes" : "no") << ")";

        }

        std::cout << std::endl;



        num_input += cloud.size();

        {

            Clock::time_point t0 = Clock::now();

            for (std::size_t i = 0; i < cloud.size(); ++i)

            {

                pcl::PointXYZRGB const& p = cloud.points[i];

                Sample sample(

                    Eigen::Vector3d(p.x, p.y, p.z),

                    Eigen::Vector3d::Zero(),

                    0.0,

                    p.r, p.g, p.b,

                    frame_time);

                if (tree.insert_sample(sample))

                    ++num_accepted;

            }

            ms_insert += elapsed_ms(t0, Clock::now());

        }

    }



    if (num_input == 0)

    {

        std::cerr << "No points loaded." << std::endl;

        return 1;

    }



    std::vector<Sample> leaf_samples;

    std::vector<double> leaf_distances;

    {

        Clock::time_point t0 = Clock::now();

        tree.collect_leaf_samples(leaf_samples, &leaf_distances);

        ms_collect = elapsed_ms(t0, Clock::now());

    }



    pcl::PointCloud<pcl::PointXYZRGB> out_cloud;

    out_cloud.reserve(leaf_samples.size());

    {

        Clock::time_point t0 = Clock::now();

        for (std::size_t i = 0; i < leaf_samples.size(); ++i)

        {

            Sample const& s = leaf_samples[i];

            pcl::PointXYZRGB p(

                static_cast<float>(s.position.x()),

                static_cast<float>(s.position.y()),

                static_cast<float>(s.position.z()),

                s.color_r, s.color_g, s.color_b);

            out_cloud.push_back(p);

        }

        out_cloud.width = static_cast<std::uint32_t>(out_cloud.size());

        out_cloud.height = 1;

        out_cloud.is_dense = true;

        ms_export = elapsed_ms(t0, Clock::now());

    }



    fs::path work_dir = fs::temp_directory_path() / "center_nearest_octree_work";

    std::error_code ec;

    fs::create_directories(work_dir, ec);

    fs::path work_output = work_dir / "output.pcd";



    {

        Clock::time_point t0 = Clock::now();

        if (pcl::io::savePCDFileBinary(narrow_path_for_pcl(work_output), out_cloud) != 0)

        {

            std::cerr << "Failed to save temp output PCD." << std::endl;

            return 1;

        }

        fs::create_directories(output_path.parent_path(), ec);

        fs::copy_file(work_output, output_path, fs::copy_options::overwrite_existing, ec);

        ms_save = elapsed_ms(t0, Clock::now());

    }

    if (ec)

    {

        std::cerr << "Failed to copy output to: " << output_path.u8string()

                  << " (" << ec.message() << ")" << std::endl;

        return 1;

    }



    double const ms_total = elapsed_ms(t_total_start, Clock::now());

    double const ms_octree = ms_insert + ms_collect;

    std::size_t const num_leaf = leaf_samples.size();

    double const compression = num_input > 0

        ? static_cast<double>(num_input) / static_cast<double>(num_leaf)

        : 0.0;



    std::cout << std::fixed << std::setprecision(2);

    std::cout << "\n--- Result ---\n";

    std::cout << "Input points:      " << num_input << std::endl;

    std::cout << "Accepted points:   " << num_accepted << std::endl;

    std::cout << "Rejected points:   " << (num_input - num_accepted) << std::endl;

    std::cout << "Leaf nodes:        " << num_leaf << std::endl;

    std::cout << "Octree nodes:      " << tree.get_num_nodes() << std::endl;

    std::cout << "Root size:         " << tree.get_root_size() << std::endl;

    std::cout << "Compression ratio: " << compression << " : 1" << std::endl;

    std::cout << "Saved leaf cloud:  " << output_path.u8string() << std::endl;

    std::cout << "(output: pcl::PointXYZRGB, colors from nearest-to-center representatives)\n";



    std::cout << "\n--- Timing (ms) ---\n";

    std::cout << "Load cloud:        " << ms_load << std::endl;

    std::cout << "Insert points:     " << ms_insert << std::endl;

    std::cout << "Collect leaves:    " << ms_collect << std::endl;

    std::cout << "Build output cloud:" << ms_export << std::endl;

    std::cout << "Save PCD:          " << ms_save << std::endl;

    std::cout << "Total:             " << ms_total << std::endl;



    std::cout << std::setprecision(0);

    std::cout << "\n--- Throughput ---\n";

    if (ms_insert > 0.0)

        std::cout << "Insert:            "

                  << (static_cast<double>(num_input) / ms_insert * 1000.0) << " points/s" << std::endl;

    if (ms_octree > 0.0)

        std::cout << "Octree core:       "

                  << (static_cast<double>(num_input) / ms_octree * 1000.0) << " points/s (insert+collect)" << std::endl;

    if (ms_total > 0.0)

        std::cout << "End-to-end:        "

                  << (static_cast<double>(num_input) / ms_total * 1000.0) << " points/s" << std::endl;



    if (num_input > 0 && tree.get_num_nodes() > 0)

    {

        std::cout << "\n--- Structure ---\n";

        std::cout << "Nodes per input:   "

                  << std::setprecision(3)

                  << (static_cast<double>(tree.get_num_nodes()) / static_cast<double>(num_input))

                  << std::endl;

        std::cout << "Avg depth (est.):  "

                  << (std::log2(tree.get_root_size() / min_leaf_size)) << std::endl;

    }



    return 0;

}


