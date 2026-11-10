#include "normal_estimate.hpp"

#include <cmath>
#include <iostream>
#include <string>

static int g_failures = 0;

static void check(bool ok, char const* msg)
{
	if (!ok) {
		++g_failures;
		std::cerr << "FAIL: " << msg << "\n";
	}
}

static Ddx::pcl_type::PointCloudPtr make_xy_plane(int n_side, float z)
{
	auto cloud = Ddx::pcl_type::PointCloudPtr(new Ddx::pcl_type::PointCloud());
	cloud->reserve(static_cast<std::size_t>(n_side * n_side));
	for (int iy = 0; iy < n_side; ++iy) {
		for (int ix = 0; ix < n_side; ++ix) {
			pcl::PointXYZ p;
			p.x = (static_cast<float>(ix) - 0.5f * static_cast<float>(n_side - 1)) * 0.05f;
			p.y = (static_cast<float>(iy) - 0.5f * static_cast<float>(n_side - 1)) * 0.05f;
			p.z = z;
			cloud->push_back(p);
		}
	}
	return cloud;
}

static void check_plane_normals(Ddx::NormalEstimateMethod method)
{
	auto cloud = make_xy_plane(40, 2.0f);
	Ddx::pcl_type::NormalPtr normal;
	check(Ddx::estimateNormalsKnn(cloud, normal, method), "estimateNormalsKnn should succeed");
	check(normal && normal->size() == cloud->size(), "normal size matches cloud");
	if (!normal || normal->size() != cloud->size()) {
		return;
	}

	int bad_orient = 0;
	int bad_align = 0;
	int invalid = 0;
	for (std::size_t i = 0; i < cloud->size(); ++i) {
		auto const& n = normal->points[i];
		if (!std::isfinite(n.normal_x) || !std::isfinite(n.normal_y) || !std::isfinite(n.normal_z)) {
			++invalid;
			continue;
		}
		float const nx = n.normal_x;
		float const ny = n.normal_y;
		float const nz = n.normal_z;
		float const n2 = nx * nx + ny * ny + nz * nz;
		if (n2 < 1e-12f) {
			++invalid;
			continue;
		}
		auto const& p = cloud->points[i];
		float const dot_p = nx * p.x + ny * p.y + nz * p.z;
		if (dot_p > 1e-4f) {
			++bad_orient;
		}
		float const inv = 1.f / std::sqrt(n2);
		float const abs_z = std::fabs(nz * inv);
		if (abs_z < 0.95f) {
			++bad_align;
		}
	}
	check(invalid == 0, "no invalid normals");
	check(bad_orient == 0, "all n·p <= 0");
	check(bad_align == 0, "|nz| ~ 1 after normalize");
	std::cout << "  " << Ddx::normalEstimateMethodName(method)
	          << " invalid=" << invalid << " bad_orient=" << bad_orient
	          << " bad_align=" << bad_align << "\n";
}

int main()
{
	Ddx::NormalEstimateMethod parsed = Ddx::NormalEstimateMethod::PclOmp;
	check(Ddx::parseNormalEstimateMethod("small_gicp_tbb", parsed)
	          && parsed == Ddx::NormalEstimateMethod::SmallGicpTbb,
	      "parse small_gicp_tbb");
	check(Ddx::parseNormalEstimateMethod("pcl_omp", parsed)
	          && parsed == Ddx::NormalEstimateMethod::PclOmp,
	      "parse pcl_omp");
	check(!Ddx::parseNormalEstimateMethod("flann", parsed), "reject unknown");
	check(std::string(Ddx::normalEstimateMethodName(Ddx::NormalEstimateMethod::SmallGicpTbb))
	          == "small_gicp_tbb",
	      "name small_gicp_tbb");

	Ddx::pcl_type::NormalPtr dummy;
	check(!Ddx::estimateNormalsKnn(nullptr, dummy, Ddx::NormalEstimateMethod::PclOmp),
	      "null cloud fails");
	auto empty = Ddx::pcl_type::PointCloudPtr(new Ddx::pcl_type::PointCloud());
	check(!Ddx::estimateNormalsKnn(empty, dummy, Ddx::NormalEstimateMethod::PclOmp),
	      "empty cloud fails");

	std::cout << "=== PclOmp plane ===\n";
	check_plane_normals(Ddx::NormalEstimateMethod::PclOmp);
	std::cout << "=== SmallGicpTbb plane ===\n";
	check_plane_normals(Ddx::NormalEstimateMethod::SmallGicpTbb);

	std::cout << "=== SmallGicpTbb sphere curvature ===\n";
	{
		auto cloud = Ddx::pcl_type::PointCloudPtr(new Ddx::pcl_type::PointCloud());
		int const n_lat = 16;
		int const n_lon = 32;
		cloud->reserve(static_cast<std::size_t>(n_lat * n_lon));
		float const pi = std::acos(-1.f);
		for (int ilat = 0; ilat < n_lat; ++ilat) {
			float const theta = pi * (static_cast<float>(ilat) + 0.5f) / static_cast<float>(n_lat);
			float const st = std::sin(theta);
			float const ct = std::cos(theta);
			for (int ilon = 0; ilon < n_lon; ++ilon) {
				float const phi = 2.f * pi * static_cast<float>(ilon) / static_cast<float>(n_lon);
				pcl::PointXYZ p;
				p.x = st * std::cos(phi);
				p.y = st * std::sin(phi);
				p.z = ct;
				cloud->push_back(p);
			}
		}
		Ddx::pcl_type::NormalPtr normal;
		check(Ddx::estimateNormalsKnn(cloud, normal, Ddx::NormalEstimateMethod::SmallGicpTbb),
		      "sphere estimateNormalsKnn");
		check(normal && normal->size() == cloud->size(), "sphere normal size matches cloud");
		double sum_c = 0.0;
		int n_c = 0;
		if (normal && normal->size() == cloud->size()) {
			for (std::size_t i = 0; i < normal->size(); ++i) {
				float const c = normal->points[i].curvature;
				if (std::isfinite(c)) {
					sum_c += static_cast<double>(c);
					++n_c;
				}
			}
		}
		float const mean_c = (n_c > 0) ? static_cast<float>(sum_c / static_cast<double>(n_c)) : 0.f;
		check(mean_c > 0.f, "SmallGicpTbb sphere mean curvature > 0");
		std::cout << "  small_gicp_tbb sphere mean_curvature=" << mean_c << " n=" << n_c << "\n";
	}

	if (g_failures == 0) {
		std::cout << "ALL PASSED\n";
		return 0;
	}
	std::cerr << g_failures << " assertion(s) failed\n";
	return 1;
}
