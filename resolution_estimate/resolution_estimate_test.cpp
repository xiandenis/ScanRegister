#include "histogram.hpp"
#include "percentile_resolution_estimate.hpp"
#include "resolution_estimate.hpp"

#include <point_cloud_io/point_cloud_io.hpp>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace fs = std::filesystem;

/** Restore station-centered (file-frame) XYZ: local + FramedCloud.origin. */
static void restore_station_frame(Ddx::point_cloud_io::FramedCloud<pcl::PointXYZ>& framed)
{
	Eigen::Vector3d const& o = framed.origin;
	for (auto& p : framed.cloud.points) {
		p.x = static_cast<float>(static_cast<double>(p.x) + o.x());
		p.y = static_cast<float>(static_cast<double>(p.y) + o.y());
		p.z = static_cast<float>(static_cast<double>(p.z) + o.z());
	}
}

static void print_histogram_detail(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud)
{
	std::vector<float> resolution_vec;
	std::vector<Eigen::Vector2f> range_vec;

	range_vec.push_back(Eigen::Vector2f(0, 15));
	resolution_vec.push_back(0.1f);
	range_vec.push_back(Eigen::Vector2f(15, 25));
	resolution_vec.push_back(0.2f);
	range_vec.push_back(Eigen::Vector2f(25, 45));
	resolution_vec.push_back(0.3f);
	range_vec.push_back(Eigen::Vector2f(45, 65));
	resolution_vec.push_back(0.4f);
	range_vec.push_back(Eigen::Vector2f(65, 85));
	resolution_vec.push_back(0.6f);
	range_vec.push_back(Eigen::Vector2f(85, 100));
	resolution_vec.push_back(0.8f);
	range_vec.push_back(Eigen::Vector2f(100, std::numeric_limits<float>::max()));
	resolution_vec.push_back(1.0f);

	Ddx::Histogram histogram;
	histogram.addBinsRanges(range_vec);
	histogram.addCloud(cloud);

	std::cout << "  points: " << cloud->size() << "\n";
	std::cout << "  depth histogram (radial norm from station origin, meters):\n";
	for (size_t i = 0; i < histogram.m_bins_number; ++i)
	{
		float rate = 100.0f * static_cast<float>(histogram.m_freq[i]) /
		             static_cast<float>(histogram.m_samples_number);
		std::cout << "    [" << histogram.m_bins_range[i][0] << ", "
		          << (histogram.m_bins_range[i][1] == std::numeric_limits<float>::max()
		                  ? std::string("inf")
		                  : std::to_string(histogram.m_bins_range[i][1]))
		          << ") m  freq=" << histogram.m_freq[i] << " (" << std::fixed
		          << std::setprecision(2) << rate << "%)  -> resolution "
		          << resolution_vec[i] << " m\n";
	}
}

static void print_percentile(Ddx::PercentileResolutionResult const& pr, char const* tag)
{
	std::cout << "\n  " << tag << ":\n";
	std::cout << "    r50=" << pr.r50 << " r75=" << pr.r75 << " r90=" << pr.r90
	          << " r_char=" << pr.r_char << "\n";
	std::cout << "    depth_v0=" << pr.resolution_depth
	          << " continuous=" << pr.resolution_continuous
	          << " resolution=" << pr.resolution << " m\n";
	std::cout << "    scene_class=" << Ddx::voxelSceneClassToString(pr.scene_class) << "\n";
	if (pr.curvature_applied) {
		std::cout << "    mode="
		          << (pr.curvature_mode == Ddx::CurvatureModulateMode::kNonplanarExcess
		                  ? "nonplanar_excess"
		                  : "indoor_score")
		          << " p_plane=" << pr.p_plane
		          << " p_np=" << pr.p_nonplanar
		          << " curv_log_std=" << pr.curv_log_std
		          << " indoor_score=" << pr.indoor_score
		          << " factor=" << pr.modulate_factor << "\n";
	} else {
		std::cout << "    curvature: not applied\n";
	}
	if (std::isnan(pr.resolution))
		std::cout << "  (NaN: fewer than 10 valid depths)\n";
}

static pcl::PointCloud<pcl::Normal>::Ptr
load_or_compute_normals(fs::path const& cloud_path,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        fs::path const& normals_arg)
{
	fs::path npath = normals_arg;
	if (npath.empty()) {
		std::string stem = cloud_path.stem().string();
		if (stem.size() >= 2 && stem.compare(stem.size() - 2, 2, "_p") == 0) {
			npath = cloud_path.parent_path() / (stem.substr(0, stem.size() - 2) + "_n.pcd");
		} else {
			npath = cloud_path.parent_path() / (stem + "_n.pcd");
		}
	}

	auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
	if (!npath.empty() && fs::exists(npath)) {
		if (pcl::io::loadPCDFile(npath.string(), *normals) == 0
		    && normals->size() == cloud->size()) {
			std::cout << "  normals: loaded " << npath << " (" << normals->size() << ")\n";
			return normals;
		}
		std::cout << "  normals: load failed or size mismatch, will compute\n";
		normals->clear();
	}

	if (cloud->size() < 10)
		return {};

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(kdtree);
	ne.setKSearch(std::min(15, std::max(3, static_cast<int>(cloud->size()) - 1)));
	ne.setNumberOfThreads(0);
	ne.compute(*normals);
	if (normals->size() != cloud->size())
		return {};
	std::cout << "  normals: computed KNN (" << normals->size() << ")\n";
	return normals;
}

/** 合成：近距平面主导 → indoor_score 高、体素相对 depth_v0 应偏小 */
static int run_synthetic_checks()
{
	std::cout << "\n=== synthetic curvature modulate checks ===\n";
	auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());

	for (int i = 0; i < 200; ++i) {
		float x = 0.05f * static_cast<float>(i % 20);
		float y = 0.05f * static_cast<float>(i / 20);
		cloud->push_back(pcl::PointXYZ(5.0f + x * 0.01f, y, 0.0f));
		pcl::Normal n;
		n.normal_x = 1.0f;
		n.normal_y = 0.0f;
		n.normal_z = 0.0f;
		n.curvature = 0.001f;
		normals->push_back(n);
	}

	Ddx::PercentileResolutionParams params;
	params.quantize = false;
	auto indoor = Ddx::PercentileResolutionEstimate::estimateDetailed(cloud, normals, "synth_indoor", params);
	print_percentile(indoor, "synth indoor-like");
	if (!indoor.curvature_applied || !(indoor.indoor_score > 0.5f)
	    || !(indoor.resolution <= indoor.resolution_depth + 1e-4f)) {
		std::cerr << "FAIL: indoor-like should apply high indoor_score and shrink/keep voxel\n";
		return 1;
	}

	cloud->clear();
	normals->clear();
	for (int i = 0; i < 200; ++i) {
		float t = static_cast<float>(i) / 199.0f;
		float r = 35.0f + 10.0f * t;
		cloud->push_back(pcl::PointXYZ(r, 0.1f * std::sin(t * 20.0f), 0.1f * std::cos(t * 13.0f)));
		pcl::Normal n;
		n.normal_x = 0.0f;
		n.normal_y = 0.0f;
		n.normal_z = 1.0f;
		n.curvature = 0.001f + 0.2f * t;
		normals->push_back(n);
	}
	auto outdoor = Ddx::PercentileResolutionEstimate::estimateDetailed(cloud, normals, "synth_outdoor", params);
	print_percentile(outdoor, "synth outdoor-like (v1 indoor_score)");
	if (!outdoor.curvature_applied
	    || !(outdoor.resolution + 1e-4f >= outdoor.resolution_depth)
	    || !(outdoor.indoor_score < indoor.indoor_score)) {
		std::cerr << "FAIL: outdoor-like should enlarge/keep vs depth_v0 and score lower than indoor\n";
		return 1;
	}

	{
		auto params_v2 = Ddx::PercentileResolutionParams::NonplanarExcess();
		params_v2.quantize = false;

		cloud->clear();
		normals->clear();
		for (int i = 0; i < 200; ++i) {
			cloud->push_back(pcl::PointXYZ(5.0f, 0.01f * static_cast<float>(i), 0.0f));
			pcl::Normal n;
			n.normal_x = 1.0f;
			n.normal_y = 0.0f;
			n.normal_z = 0.0f;
			n.curvature = 0.001f;
			normals->push_back(n);
		}
		auto v2_in = Ddx::PercentileResolutionEstimate::estimateDetailed(
			cloud, normals, "synth_v2_indoor", params_v2);
		print_percentile(v2_in, "synth v2 indoor-plane");
		if (!v2_in.curvature_applied || !(v2_in.modulate_factor < 1.0f - 1e-4f)) {
			std::cerr << "FAIL: v2 indoor-plane should shrink (factor<1)\n";
			return 1;
		}

		cloud->clear();
		normals->clear();
		for (int i = 0; i < 200; ++i) {
			float t = static_cast<float>(i) / 199.0f;
			cloud->push_back(pcl::PointXYZ(20.0f + t, 0.0f, 0.0f));
			pcl::Normal n;
			n.normal_x = 0.0f;
			n.normal_y = 0.0f;
			n.normal_z = 1.0f;
			n.curvature = (i % 20 == 0) ? 0.001f : 0.05f;
			normals->push_back(n);
		}
		auto v2_hi = Ddx::PercentileResolutionEstimate::estimateDetailed(
			cloud, normals, "synth_v2_high_np", params_v2);
		print_percentile(v2_hi, "synth v2 high nonplanar");
		if (!v2_hi.curvature_applied || !(v2_hi.p_nonplanar > 0.65f)
		    || !(v2_hi.modulate_factor > 1.0f + 1e-4f)) {
			std::cerr << "FAIL: v2 high nonplanar should amplify (factor>1)\n";
			return 1;
		}

		cloud->clear();
		normals->clear();
		for (int i = 0; i < 200; ++i) {
			cloud->push_back(pcl::PointXYZ(30.0f, 0.0f, 0.0f));
			pcl::Normal n;
			n.normal_x = 0.0f;
			n.normal_y = 0.0f;
			n.normal_z = 1.0f;
			n.curvature = (i < 80) ? 0.001f : 0.05f;
			normals->push_back(n);
		}
		auto v2_mid = Ddx::PercentileResolutionEstimate::estimateDetailed(
			cloud, normals, "synth_v2_mid", params_v2);
		print_percentile(v2_mid, "synth v2 mid band");
		if (!v2_mid.curvature_applied
		    || std::fabs(v2_mid.modulate_factor - 1.0f) > 1e-3f) {
			std::cerr << "FAIL: v2 mid band should keep factor≈1\n";
			return 1;
		}
	}

	std::cout << "  synthetic checks OK\n";
	return 0;
}

static int run_test(fs::path const& fpath, fs::path const& normals_path)
{
	std::string path = fpath.string();

	std::cout << "=== resolution_estimate test ===\n";
	std::cout << "file: " << path << "\n";

	if (!fs::exists(fpath))
	{
		std::cerr << "file does not exist\n";
		return 1;
	}

	Ddx::point_cloud_io::FramedCloud<pcl::PointXYZ> framed;
	if (Ddx::point_cloud_io::load(path, framed) != 0)
	{
		std::cerr << "failed to load FramedCloud\n";
		return 1;
	}

	std::cout << "  FramedCloud.origin: (" << framed.origin.x() << ", "
	          << framed.origin.y() << ", " << framed.origin.z() << ")\n";
	std::cout << "  local points: " << framed.cloud.size() << "\n";

	// Station-centered LAS: radial depth uses file-frame (站心) coords = local + origin.
	restore_station_frame(framed);
	std::cout << "  restored station-frame (local + origin) for radial depth\n";

	auto cloud_ptr = framed.cloud.makeShared();
	print_histogram_detail(cloud_ptr);

	float resolution = Ddx::ResolutionEstimate::estimate(cloud_ptr, "histogram");
	std::cout << "\n  histogram-mode resolution: " << resolution << " m\n";
	if (std::isnan(resolution))
		std::cout << "  (NaN: fewer than 10 points or empty histogram)\n";

	Ddx::PercentileResolutionResult const pr_depth =
		Ddx::PercentileResolutionEstimate::estimateDetailed(cloud_ptr, "percentile_depth");
	print_percentile(pr_depth, "percentile depth-only");

	// Full-cloud KNN normals on multi-million LAS is too heavy for smoke; require sidecar or small cloud.
	constexpr size_t kComputeNormalsMaxPoints = 500000;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	if (!normals_path.empty() || cloud_ptr->size() <= kComputeNormalsMaxPoints) {
		normals = load_or_compute_normals(fpath, cloud_ptr, normals_path);
	} else {
		std::cout << "\n  skip curvature path (cloud " << cloud_ptr->size()
		          << " > " << kComputeNormalsMaxPoints
		          << "; pass normals PCD as argv[2] to enable)\n";
	}
	if (normals) {
		Ddx::PercentileResolutionResult const pr_curv =
			Ddx::PercentileResolutionEstimate::estimateDetailed(
				cloud_ptr, normals, "percentile_curv");
		print_percentile(pr_curv, "percentile depth+curvature");
	}

	return 0;
}

static fs::path pick_default_cloud()
{
#ifdef _WIN32
	fs::path const dir = L"D:\\data\\huangbei_data\\LAS";
#else
	fs::path const dir = "/data/huangbei_data/LAS";
#endif
	if (!fs::is_directory(dir))
		return {};
	std::vector<fs::path> las;
	for (auto const& e : fs::directory_iterator(dir)) {
		if (!e.is_regular_file())
			continue;
		auto ext = e.path().extension().string();
		for (char& c : ext)
			c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
		if (ext == ".las" || ext == ".slas" || ext == ".pcd" || ext == ".ply")
			las.push_back(e.path());
	}
	if (las.empty())
		return {};
	std::sort(las.begin(), las.end());
	return las.front();
}

#ifdef _WIN32
int wmain(int argc, wchar_t* argv[])
#else
int main(int argc, char* argv[])
#endif
{
#ifdef _WIN32
	if (argc >= 2 && std::wstring(argv[1]) == L"--synthetic")
		return run_synthetic_checks();
	fs::path const fpath = (argc >= 2) ? fs::path(argv[1]) : pick_default_cloud();
	fs::path const npath = (argc >= 3) ? fs::path(argv[2]) : fs::path();
#else
	if (argc >= 2 && std::string(argv[1]) == "--synthetic")
		return run_synthetic_checks();
	fs::path const fpath = (argc >= 2) ? fs::path(argv[1]) : pick_default_cloud();
	fs::path const npath = (argc >= 3) ? fs::path(argv[2]) : fs::path();
#endif

	int rc = 0;
	if (!fpath.empty() && fs::exists(fpath))
		rc = run_test(fpath, npath);
	else if (argc >= 2) {
		std::cerr << "file does not exist: " << fpath << "\n";
		rc = 1;
	} else {
		std::cout << "no cloud file; running synthetic only\n";
	}

	int const synth = run_synthetic_checks();
	return rc != 0 ? rc : synth;
}
