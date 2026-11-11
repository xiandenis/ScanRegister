#include <trim_icp/trim_icp_solver.hpp>
#include <trim_icp/overlap_voxel_crop.hpp>
#include <trim_icp/trim_icp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <vector>

static int g_fails = 0;
#define EXPECT(cond)                                                                 \
	do {                                                                             \
		if (!(cond)) {                                                               \
			std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << " " #cond "\n"; \
			++g_fails;                                                               \
		}                                                                            \
	} while (0)

static pcl::PointCloud<pcl::PointXYZ>::Ptr makeBox(float z0, int n_side)
{
	auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < n_side; ++i) {
		for (int j = 0; j < n_side; ++j) {
			for (int k = 0; k < n_side; ++k) {
				pcl::PointXYZ p;
				p.x = 0.1f * static_cast<float>(i);
				p.y = 0.1f * static_cast<float>(j);
				p.z = z0 + 0.1f * static_cast<float>(k);
				cloud->push_back(p);
			}
		}
	}
	return cloud;
}

static pcl::PointCloud<pcl::Normal>::Ptr makeUpNormals(std::size_t n)
{
	auto nrm = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	nrm->resize(n);
	for (auto& nrm_pt : nrm->points) {
		nrm_pt.normal_x = 0.f;
		nrm_pt.normal_y = 0.f;
		nrm_pt.normal_z = 1.f;
	}
	return nrm;
}

static void test_crop_keeps_overlap_only()
{
	auto tgt = makeBox(0.0f, 8);
	auto src = makeBox(0.5f, 8);
	Ddx::OverlapVoxelCropParams p;
	p.voxel_size = 0.2f;
	p.dilation = 1;
	p.min_points = 10;
	std::vector<int> si;
	std::vector<int> ti;
	EXPECT(Ddx::cropMutualOverlapByVoxel(src, tgt, Eigen::Matrix4d::Identity(), p, si, ti));
	EXPECT(static_cast<int>(si.size()) < static_cast<int>(src->size()));
	EXPECT(static_cast<int>(ti.size()) < static_cast<int>(tgt->size()));
	EXPECT(static_cast<int>(si.size()) >= p.min_points);
	EXPECT(static_cast<int>(ti.size()) >= p.min_points);
}

static void test_crop_disjoint_fails()
{
	auto tgt = makeBox(0.0f, 6);
	auto src = makeBox(50.0f, 6);
	Ddx::OverlapVoxelCropParams p;
	p.voxel_size = 0.2f;
	p.dilation = 0;
	p.min_points = 10;
	std::vector<int> si;
	std::vector<int> ti;
	EXPECT(!Ddx::cropMutualOverlapByVoxel(src, tgt, Eigen::Matrix4d::Identity(), p, si, ti));
	EXPECT(si.empty());
	EXPECT(ti.empty());
}

static void test_crop_with_occupancy_stride_still_mutual()
{
	auto tgt = makeBox(0.0f, 8);
	auto src = makeBox(0.5f, 8);
	Ddx::OverlapVoxelCropParams p;
	p.voxel_size = 0.2f;
	p.dilation = 1;
	p.min_points = 10;
	p.occupancy_stride = 3;
	std::vector<int> si;
	std::vector<int> ti;
	EXPECT(Ddx::cropMutualOverlapByVoxel(src, tgt, Eigen::Matrix4d::Identity(), p, si, ti));
	EXPECT(static_cast<int>(si.size()) < static_cast<int>(src->size()));
	EXPECT(static_cast<int>(ti.size()) < static_cast<int>(tgt->size()));
}

static void test_solver_ricp_runs()
{
	auto cloud = makeBox(0.0f, 6);
	Ddx::TrimIcpSolver icp;
	icp.setSourceCloud(cloud);
	icp.setTargetCloud(cloud);
	icp.setMethod(Ddx::registration::BasicICP::Method::RICP);
	EXPECT(icp.align(Eigen::Matrix4d::Identity()));
}

static void test_trim_default_crop_no_nc()
{
	auto tgt = makeBox(0.0f, 8);
	auto src = makeBox(0.5f, 8);
	Ddx::TrimICP icp;
	icp.setSourceCloud(src);
	icp.setTargetCloud(tgt);
	EXPECT(icp.align(Eigen::Matrix4d::Identity()));
}

static void test_trim_nc_without_normals_fails()
{
	auto cloud = makeBox(0.0f, 8);
	Ddx::TrimICP icp;
	icp.setSourceCloud(cloud);
	icp.setTargetCloud(cloud);
	icp.setOverlapCropEnabled(false);
	icp.setNormalConsistencyEnabled(true);
	EXPECT(!icp.align(Eigen::Matrix4d::Identity()));
}

static void test_trim_nc_with_normals_ok()
{
	auto cloud = makeBox(0.0f, 8);
	auto nrm = makeUpNormals(cloud->size());
	Ddx::TrimICP icp;
	icp.setSourceCloud(cloud);
	icp.setTargetCloud(cloud);
	icp.setSourceNormals(nrm);
	icp.setTargetNormals(nrm);
	icp.setOverlapCropEnabled(false);
	icp.setNormalConsistencyEnabled(true);
	EXPECT(icp.align(Eigen::Matrix4d::Identity()));
}

int main()
{
	test_crop_keeps_overlap_only();
	test_crop_disjoint_fails();
	test_crop_with_occupancy_stride_still_mutual();
	test_solver_ricp_runs();
	test_trim_default_crop_no_nc();
	test_trim_nc_without_normals_fails();
	test_trim_nc_with_normals_ok();
	if (g_fails) {
		std::cerr << "trim_icp_overlap_test: " << g_fails << " failures\n";
		return 1;
	}
	std::cout << "trim_icp_overlap_test: ok\n";
	return 0;
}
