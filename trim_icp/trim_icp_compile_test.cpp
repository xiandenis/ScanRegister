#include <trim_icp/trim_icp.hpp>
#include <trim_icp/implicit_surface_report.hpp>
#include <trim_icp/overlap_estimator.hpp>

#include <iostream>

int main()
{
	Ddx::TrimICP icp;
	icp.setOverlapCropEnabled(true);
	icp.setOverlapVoxelSize(0.75f);
	icp.setMaxSearchRadius(0.75f);

	Ddx::ImplicitSurfaceReport report;
	(void)report.getFinalRms();

	Ddx::OverlapEstimator overlap;
	(void)overlap.getOverlap();

	std::cout << "trim_icp_compile_test: ok\n";
	return 0;
}
