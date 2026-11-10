// Smoke / regression for SLAM-homologous BasicICP (no orthogonal NC / scale_max API).
#include "basic_icp.h"

#include <Eigen/Core>
#include <cmath>
#include <iostream>

using Ddx::registration::BasicICP;

namespace {

BasicICP::EigenPointCloud makeBoxCloud(Eigen::Vector3d const& offset)
{
	BasicICP::EigenPointCloud cloud(3, 8);
	int i = 0;
	for (int x = 0; x < 2; ++x) {
		for (int y = 0; y < 2; ++y) {
			for (int z = 0; z < 2; ++z) {
				cloud.col(i++) = offset + Eigen::Vector3d(x, y, z);
			}
		}
	}
	return cloud;
}

BasicICP::EigenPointCloud makeConstantNormals(Eigen::Index n)
{
	BasicICP::EigenPointCloud normals(3, n);
	for (Eigen::Index i = 0; i < n; ++i) {
		normals.col(i) = Eigen::Vector3d(0, 0, 1);
	}
	return normals;
}

bool expectTrue(char const* name, bool ok)
{
	if (!ok) {
		std::cerr << "FAIL: " << name << "\n";
		return false;
	}
	return true;
}

bool expectFalse(char const* name, bool ok)
{
	if (ok) {
		std::cerr << "FAIL: expected false: " << name << "\n";
		return false;
	}
	return true;
}

} // namespace

int main()
{
	int fails = 0;

	{
		BasicICP icp;
		icp.setMethod(BasicICP::Method::RICP);
		icp.setSourceCloud(BasicICP::EigenPointCloud(3, 0));
		icp.setTargetCloud(makeBoxCloud(Eigen::Vector3d::Zero()));
		// SLAM may crash or behave poorly on empty; we only require it does not need NC APIs.
		(void)icp;
	}

	{
		BasicICP icp;
		icp.setMethod(BasicICP::Method::RICP);
		auto src = makeBoxCloud(Eigen::Vector3d(0.05, 0, 0));
		auto tgt = makeBoxCloud(Eigen::Vector3d::Zero());
		icp.setSourceCloud(src);
		icp.setTargetCloud(tgt);
		Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
		init(0, 3) = 0.05;
		if (!expectTrue("RICP happy path", icp.align(init))) {
			++fails;
		}
	}

	{
		BasicICP icp;
		icp.setMethod(BasicICP::Method::RPPL);
		auto src = makeBoxCloud(Eigen::Vector3d(0.02, 0, 0));
		auto tgt = makeBoxCloud(Eigen::Vector3d::Zero());
		auto n = makeConstantNormals(tgt.cols());
		icp.setSourceCloud(src);
		icp.setTargetCloud(tgt);
		icp.setTargetNormal(n);
		Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
		if (!expectTrue("RPPL with target normals", icp.align(init))) {
			++fails;
		}
	}

	if (fails != 0) {
		std::cerr << "basic_icp_guard_test: " << fails << " failed\n";
		return 1;
	}
	std::cout << "basic_icp_guard_test: ok\n";
	return 0;
}
