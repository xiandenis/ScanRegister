#include "e57_scan_geometry.hpp"
#include "defines.hpp"

#include <cmath>
#include <iostream>

namespace {

int g_fail = 0;

void expect(bool cond, char const* msg)
{
	if (!cond) {
		std::cerr << "FAIL: " << msg << "\n";
		++g_fail;
	} else {
		std::cout << "OK: " << msg << "\n";
	}
}

bool near(double a, double b, double tol = 1e-9)
{
	return std::fabs(a - b) <= tol;
}

bool nearVec(Eigen::Vector3d const& a, Eigen::Vector3d const& b, double tol = 1e-9)
{
	return (a - b).norm() <= tol;
}

}  // namespace

int main()
{
	using Ddx::E57ScanGeometry;

	// deg ↔ rad
	expect(near(E57ScanGeometry::deg_to_rad(180.0), MATH_PI), "deg_to_rad(180)=π");
	expect(near(E57ScanGeometry::rad_to_deg(MATH_PI_2), 90.0), "rad_to_deg(π/2)=90");

	// §4.5 对照（度→弧度）
	{
		double th = 0.0;
		double ph = 0.0;
		E57ScanGeometry::encoder_to_e57(E57ScanGeometry::deg_to_rad(0.0),
			E57ScanGeometry::deg_to_rad(0.0), th, ph);
		expect(near(th, 0.0) && near(ph, MATH_PI_2), "enc(0,0)→θ=0 φ=+π/2");

		E57ScanGeometry::encoder_to_e57(E57ScanGeometry::deg_to_rad(0.0),
			E57ScanGeometry::deg_to_rad(90.0), th, ph);
		expect(near(th, 0.0) && near(ph, 0.0), "enc(0,90°)→+X 水平");

		E57ScanGeometry::encoder_to_e57(E57ScanGeometry::deg_to_rad(0.0),
			E57ScanGeometry::deg_to_rad(270.0), th, ph);
		expect(near(th, MATH_PI) && near(ph, 0.0), "enc(0,270°)→-X 水平倒镜");

		E57ScanGeometry::encoder_to_e57(E57ScanGeometry::deg_to_rad(90.0),
			E57ScanGeometry::deg_to_rad(90.0), th, ph);
		expect(near(th, -MATH_PI_2) && near(ph, 0.0), "enc(90°,90°)→-Y 水平");
	}

	// encoder → e57 → normal encoder 往返
	{
		double const cases[][2] = {
			{0.0, 0.0},
			{0.0, MATH_PI_2},
			{MATH_PI_2, MATH_PI_2},
			{MATH_PI, MATH_PI_2},
			{MATH_PI / 3.0, 4.0 * MATH_PI / 3.0},
		};
		for (auto const& c : cases) {
			double th = 0.0;
			double ph = 0.0;
			E57ScanGeometry::encoder_to_e57(c[0], c[1], th, ph);
			double t1 = 0.0;
			double p1 = 0.0;
			E57ScanGeometry::e57_to_encoder_normal(th, ph, t1, p1);
			expect(near(t1, c[0], 1e-8) && near(p1, E57ScanGeometry::wrap_2pi(c[1]), 1e-8),
				"encoder↔e57§5.2 roundtrip");
		}
	}

	// xyz ↔ e57：+X
	{
		Eigen::Vector3d p(2.0, 0.0, 0.0);
		double th = 0.0;
		double ph = 0.0;
		double r = 0.0;
		expect(E57ScanGeometry::xyz_to_e57(p, th, ph, r), "xyz_to_e57 +X");
		expect(near(th, 0.0) && near(ph, 0.0) && near(r, 2.0), "+X → θ=0 φ=0 r=2");
		Eigen::Vector3d q;
		expect(E57ScanGeometry::e57_to_xyz(th, ph, r, q) && nearVec(q, p),
			"e57_to_xyz roundtrip +X");
	}

	// 原点失败
	{
		double th = 0.0;
		double ph = 0.0;
		double r = 0.0;
		expect(!E57ScanGeometry::xyz_to_e57(Eigen::Vector3d::Zero(), th, ph, r),
			"xyz origin → false");
	}

	// 像素：+X → 图像中心附近；反投影在半像素角分辨率内
	{
		constexpr int W = 3600;
		constexpr int H = 1800;
		Eigen::Vector2i uv;
		expect(E57ScanGeometry::e57_to_pixel(0.0, 0.0, W, H, uv), "e57_to_pixel +X");
		expect(uv.x() == W / 2 && uv.y() == H / 2, "+X → (W/2,H/2)");

		double th = 0.0;
		double ph = 0.0;
		expect(E57ScanGeometry::pixel_to_e57(uv, W, H, th, ph), "pixel_to_e57 center");
		// floor(W/2) 的像素中心相对 θ=0 偏半像素；容差取 1 像素角宽
		double const d_az = 2.0 * MATH_PI / static_cast<double>(W);
		double const d_el = MATH_PI / static_cast<double>(H);
		expect(near(th, 0.0, d_az) && near(ph, 0.0, d_el),
			"center pixel → θ,φ within 1 pixel");

		Eigen::Vector3d dir;
		expect(E57ScanGeometry::pixel_to_xyz(uv, W, H, dir), "pixel_to_xyz");
		expect(nearVec(dir, Eigen::Vector3d(1.0, 0.0, 0.0), 5e-3), "center → ≈+X");
	}

	// 码盘 → xyz → 码盘
	{
		double const t1 = E57ScanGeometry::deg_to_rad(90.0);
		double const p1 = E57ScanGeometry::deg_to_rad(90.0);
		Eigen::Vector3d p;
		expect(E57ScanGeometry::encoder_to_xyz(t1, p1, 5.0, p), "encoder_to_xyz");
		expect(nearVec(p, Eigen::Vector3d(0.0, -5.0, 0.0), 1e-9), "enc(90,90)→-Y*5");
		double t1b = 0.0;
		double p1b = 0.0;
		double r = 0.0;
		expect(E57ScanGeometry::xyz_to_encoder_normal(p, t1b, p1b, r),
			"xyz_to_encoder_normal");
		expect(near(t1b, t1) && near(p1b, p1) && near(r, 5.0), "encoder xyz roundtrip");
	}

	// §5.3：同一视线正镜/倒镜不同码盘，再 §4 回到同一 E57
	{
		double const th = -MATH_PI_2;
		double const ph = 0.0;
		double t_f = 0.0;
		double p_f = 0.0;
		double t_m = 0.0;
		double p_m = 0.0;
		E57ScanGeometry::e57_to_encoder_front(th, ph, t_f, p_f);
		E57ScanGeometry::e57_to_encoder_mirror(th, ph, t_m, p_m);
		expect(p_f <= MATH_PI + 1e-12, "front φ₁∈[0,π]");
		expect(p_m > MATH_PI, "mirror φ₁∈(π,2π)");
		double th2 = 0.0;
		double ph2 = 0.0;
		E57ScanGeometry::encoder_to_e57(t_f, p_f, th2, ph2);
		expect(near(th2, th) && near(ph2, ph), "front §5.3→§4");
		E57ScanGeometry::encoder_to_e57(t_m, p_m, th2, ph2);
		expect(near(th2, th) && near(ph2, ph), "mirror §5.3→§4");
	}

	// 组合：encoder → pixel → encoder_normal（水平点；容差按半像素）
	{
		constexpr int W = 360;
		constexpr int H = 180;
		double const t1 = 0.0;
		double const p1 = MATH_PI_2;
		Eigen::Vector2i uv;
		expect(E57ScanGeometry::encoder_to_pixel(t1, p1, W, H, uv), "encoder_to_pixel");
		double t1b = 0.0;
		double p1b = 0.0;
		expect(E57ScanGeometry::pixel_to_encoder_normal(uv, W, H, t1b, p1b),
			"pixel_to_encoder_normal");
		double const d_az = 2.0 * MATH_PI / static_cast<double>(W);
		double const d_el = MATH_PI / static_cast<double>(H);
		expect(near(t1b, t1, d_az) && near(p1b, p1, d_el),
			"encoder↔pixel roundtrip (coarse grid)");
	}

	if (g_fail != 0) {
		std::cerr << g_fail << " failed\n";
		return 1;
	}
	std::cout << "ALL PASSED\n";
	return 0;
}
