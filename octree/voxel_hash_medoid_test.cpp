#include "octree_downsample.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace
{

int g_failures = 0;

void check(bool condition, char const* expr, char const* file, int line)
{
	if (!condition) {
		++g_failures;
		std::cerr << "FAIL " << file << ":" << line << "  " << expr << std::endl;
	}
}

#define CHECK(cond) check((cond), #cond, __FILE__, __LINE__)

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;

CloudPtr makeCloud(std::initializer_list<pcl::PointXYZ> pts)
{
	CloudPtr cloud(new Cloud);
	cloud->points.assign(pts.begin(), pts.end());
	cloud->width = static_cast<std::uint32_t>(cloud->points.size());
	cloud->height = 1;
	cloud->is_dense = false;
	return cloud;
}

void test_rejects_null_empty_and_nonpositive_voxel()
{
	std::vector<int> indices{42};
	CHECK(!Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(nullptr, 0.03f, indices, false));
	CHECK(indices.empty());

	CloudPtr empty(new Cloud);
	indices = {1};
	CHECK(!Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(empty, 0.03f, indices, false));
	CHECK(indices.empty());

	CloudPtr one = makeCloud({pcl::PointXYZ{0.f, 0.f, 0.f}});
	indices = {1};
	CHECK(!Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(one, 0.f, indices, false));
	CHECK(indices.empty());
	CHECK(!Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(one, -0.03f, indices, false));
	CHECK(indices.empty());
}

void test_single_point()
{
	CloudPtr cloud = makeCloud({pcl::PointXYZ{1.2f, -3.4f, 5.6f}});
	std::vector<int> indices;
	CHECK(Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(cloud, 0.03f, indices, false));
	CHECK(indices.size() == 1);
	CHECK(indices[0] == 0);
}

void test_three_points_same_cell_picks_medoid()
{
	// voxel_size=1: all in cell (0,0,0). centroid x=0.5 → nearest is 0.6 (index 1).
	CloudPtr cloud = makeCloud({
		pcl::PointXYZ{0.0f, 0.f, 0.f},
		pcl::PointXYZ{0.6f, 0.f, 0.f},
		pcl::PointXYZ{0.9f, 0.f, 0.f},
	});
	std::vector<int> indices;
	CHECK(Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(cloud, 1.0f, indices, false));
	CHECK(indices.size() == 1);
	CHECK(indices[0] == 1);
}

void test_two_cells()
{
	CloudPtr cloud = makeCloud({
		pcl::PointXYZ{0.1f, 0.f, 0.f},
		pcl::PointXYZ{1.1f, 0.f, 0.f},
	});
	std::vector<int> indices;
	CHECK(Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(cloud, 1.0f, indices, false));
	CHECK(indices.size() == 2);
	CHECK(indices[0] == 0);
	CHECK(indices[1] == 1);
}

void test_negative_floor_splits_cells()
{
	CloudPtr cloud = makeCloud({
		pcl::PointXYZ{-0.1f, 0.f, 0.f},
		pcl::PointXYZ{0.1f, 0.f, 0.f},
	});
	std::vector<int> indices;
	CHECK(Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(cloud, 1.0f, indices, false));
	CHECK(indices.size() == 2);
}

void test_skips_nan_and_inf()
{
	float const nan = std::numeric_limits<float>::quiet_NaN();
	float const inf = std::numeric_limits<float>::infinity();
	CloudPtr cloud = makeCloud({
		pcl::PointXYZ{nan, 0.f, 0.f},
		pcl::PointXYZ{0.f, 0.f, 0.f},
		pcl::PointXYZ{inf, 1.f, 0.f},
	});
	std::vector<int> indices;
	CHECK(Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(cloud, 0.03f, indices, false));
	CHECK(indices.size() == 1);
	CHECK(indices[0] == 1);
}

void test_all_invalid_returns_false()
{
	float const nan = std::numeric_limits<float>::quiet_NaN();
	CloudPtr cloud = makeCloud({pcl::PointXYZ{nan, nan, nan}});
	std::vector<int> indices{7};
	CHECK(!Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(cloud, 0.03f, indices, false));
	CHECK(indices.empty());
}

void test_tie_prefers_smaller_index()
{
	CloudPtr cloud = makeCloud({
		pcl::PointXYZ{0.0f, 0.f, 0.f},
		pcl::PointXYZ{1.0f, 0.f, 0.f},
	});
	std::vector<int> indices;
	CHECK(Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(cloud, 2.0f, indices, false));
	CHECK(indices.size() == 1);
	CHECK(indices[0] == 0);
}

void test_serial_matches_parallel()
{
	CloudPtr cloud(new Cloud);
	for (int i = 0; i < 200; ++i) {
		float const x = static_cast<float>((i % 17) * 0.04);
		float const y = static_cast<float>(((i * 3) % 13) * 0.04);
		float const z = static_cast<float>(((i * 5) % 11) * 0.04);
		cloud->push_back(pcl::PointXYZ{x, y, z});
	}
	std::vector<int> serial;
	std::vector<int> parallel;
	CHECK(Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(cloud, 0.1f, serial, false));
	CHECK(Ddx::octree::downsampleHashMedoidIndices<pcl::PointXYZ>(cloud, 0.1f, parallel, true));
	CHECK(serial == parallel);
	CHECK(!serial.empty());
}

void test_first_keeps_earliest_index_per_cell()
{
	CloudPtr cloud = makeCloud({
		pcl::PointXYZ{0.01f, 0.01f, 0.01f},
		pcl::PointXYZ{0.02f, 0.01f, 0.01f},
		pcl::PointXYZ{1.01f, 0.01f, 0.01f},
	});
	std::vector<int> indices;
	CHECK(Ddx::octree::downsampleHashFirstIndices<pcl::PointXYZ>(cloud, 1.0f, indices));
	CHECK(indices.size() == 2);
	std::sort(indices.begin(), indices.end());
	CHECK(indices[0] == 0);
	CHECK(indices[1] == 2);
}

void test_octree_wrapper_rejects_null_and_empty()
{
	std::vector<int> indices{42};
	CHECK(!Ddx::octree::downsampleOctreeMedoidIndices<pcl::PointXYZ>(nullptr, 0.03f, indices, false));
	CHECK(indices.empty());

	CloudPtr empty(new Cloud);
	indices = {1};
	CHECK(!Ddx::octree::downsampleOctreeMedoidIndices<pcl::PointXYZ>(empty, 0.03f, indices, false));
	CHECK(indices.empty());
}

void test_octree_wrapper_single_point_and_two_cells()
{
	CloudPtr one = makeCloud({pcl::PointXYZ{1.2f, -3.4f, 5.6f}});
	std::vector<int> indices;
	CHECK(Ddx::octree::downsampleOctreeMedoidIndices<pcl::PointXYZ>(one, 0.03f, indices, false));
	CHECK(indices.size() == 1);
	CHECK(indices[0] == 0);

	CloudPtr two = makeCloud({
		pcl::PointXYZ{0.f, 0.f, 0.f},
		pcl::PointXYZ{10.f, 0.f, 0.f},
	});
	indices.clear();
	CHECK(Ddx::octree::downsampleOctreeMedoidIndices<pcl::PointXYZ>(two, 0.03f, indices, true));
	CHECK(indices.size() == 2);
}

} // namespace

int main()
{
	test_rejects_null_empty_and_nonpositive_voxel();
	test_single_point();
	test_three_points_same_cell_picks_medoid();
	test_two_cells();
	test_negative_floor_splits_cells();
	test_skips_nan_and_inf();
	test_all_invalid_returns_false();
	test_tie_prefers_smaller_index();
	test_serial_matches_parallel();
	test_first_keeps_earliest_index_per_cell();
	test_octree_wrapper_rejects_null_and_empty();
	test_octree_wrapper_single_point_and_two_cells();

	if (g_failures == 0) {
		std::cout << "All voxel_hash_medoid tests passed." << std::endl;
		return 0;
	}
	std::cerr << g_failures << " test(s) failed." << std::endl;
	return 1;
}
