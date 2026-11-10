#include "working_frame.hpp"

#include <cmath>
#include <iostream>

namespace
{

int g_fails = 0;

void expect(bool cond, char const* msg)
{
	if (!cond)
	{
		std::cerr << "FAIL: " << msg << "\n";
		++g_fails;
	}
}

bool near(double a, double b, double eps = 1e-9)
{
	return std::abs(a - b) <= eps;
}

} // namespace

int main()
{
	using Ddx::WorkingFrame;

	// Magnitudes where float ULP ≫ mm (typical projected CRS).
	Eigen::Vector3d const O(1234567.890123, 7654321.098765, 100.0);
	WorkingFrame const wf = WorkingFrame::fromOrigin(O);

	Eigen::Vector3d const world(1234578.140123, 7654326.598765, 101.0); // local ≈ (10.25, 5.5, 1)
	Eigen::Vector3d const local = wf.toLocal(world);
	expect(near(local.x(), 10.25, 1e-9) && near(local.y(), 5.5, 1e-9) && near(local.z(), 1.0),
	       "toLocal");
	Eigen::Vector3d const back = wf.toWorld(local);
	expect(near(back.x(), world.x()) && near(back.y(), world.y()) && near(back.z(), world.z()),
	       "double roundtrip");

	Eigen::Vector3f const lf = wf.toLocalF(world);
	Eigen::Vector3d const back_f = wf.toWorldFromF(lf);
	expect(near(back_f.x(), world.x(), 1e-3) && near(back_f.y(), world.y(), 1e-3) &&
	           near(back_f.z(), world.z(), 1e-3),
	       "float local roundtrip ~mm");

	// Direct float of absolute large coords loses far more than local float path.
	Eigen::Vector3f const bad(static_cast<float>(world.x()), static_cast<float>(world.y()),
	                          static_cast<float>(world.z()));
	double const err_abs = std::abs(static_cast<double>(bad.x()) - world.x()) +
	                       std::abs(static_cast<double>(bad.y()) - world.y());
	double const err_local = std::abs(back_f.x() - world.x()) + std::abs(back_f.y() - world.y());
	expect(err_abs > 1e-2, "absolute float visibly wrong on large coords");
	expect(err_local < err_abs, "local float beats absolute float on large coords");

	if (g_fails == 0)
		std::cout << "EVIDENCE module=working_frame origin=[" << O.transpose()
		          << "] all_tests_passed\n";
	else
		std::cerr << "working_frame_test failures=" << g_fails << "\n";
	return g_fails == 0 ? 0 : 1;
}
