#include "e57_scan_geometry.hpp"

#include "defines.hpp"

#include <cmath>
#include <algorithm>

namespace Ddx {
namespace {

bool finite3(Eigen::Vector3d const& p)
{
	return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
}

}  // namespace

double E57ScanGeometry::deg_to_rad(double deg)
{
	return deg * (MATH_PI / 180.0);
}

double E57ScanGeometry::rad_to_deg(double rad)
{
	return rad * (180.0 / MATH_PI);
}

double E57ScanGeometry::wrap_2pi(double rad)
{
	double y = std::fmod(rad, 2.0 * MATH_PI);
	if (y < 0.0) {
		y += 2.0 * MATH_PI;
	}
	return y;
}

double E57ScanGeometry::wrap_pm_pi(double rad)
{
	double y = wrap_2pi(rad);  // [0, 2π)
	if (y > MATH_PI) {
		y -= 2.0 * MATH_PI;
	}
	return y;  // (-π, π]；-X 取 +π
}

void E57ScanGeometry::encoder_to_e57(double theta1, double phi1, double& theta,
	double& phi)
{
	double const phi1w = wrap_2pi(phi1);
	if (phi1w <= MATH_PI) {
		// 正镜 φ₁∈[0,π]
		phi = MATH_PI_2 - phi1w;
		theta = wrap_pm_pi(-theta1);
	} else {
		// 倒镜 φ₁∈(π,2π)
		double const phi_z = 2.0 * MATH_PI - phi1w;
		phi = MATH_PI_2 - phi_z;
		theta = wrap_pm_pi(MATH_PI - theta1);
	}
}

void E57ScanGeometry::e57_to_encoder_normal(double theta, double phi,
	double& theta1, double& phi1)
{
	double const az = wrap_pm_pi(theta);
	if (az > 0.0 && az < MATH_PI) {
		// 倒镜
		theta1 = MATH_PI - az;
		phi1 = wrap_2pi(1.5 * MATH_PI + phi);  // 270° + φ°
	} else if (az >= MATH_PI || az <= -MATH_PI) {
		// ±π → 正镜约定
		theta1 = MATH_PI;
		phi1 = wrap_2pi(MATH_PI_2 - phi);
	} else {
		// (-π, 0] 正镜（含 0）
		theta1 = -az;
		phi1 = wrap_2pi(MATH_PI_2 - phi);
	}
}

void E57ScanGeometry::e57_to_encoder_front(double theta, double phi,
	double& theta1, double& phi1)
{
	double const phi_z = MATH_PI_2 - phi;
	phi1 = wrap_2pi(phi_z);
	theta1 = wrap_2pi(-theta);
}

void E57ScanGeometry::e57_to_encoder_mirror(double theta, double phi,
	double& theta1, double& phi1)
{
	double const phi_z = MATH_PI_2 - phi;
	phi1 = wrap_2pi(2.0 * MATH_PI - phi_z);
	theta1 = wrap_2pi(MATH_PI - theta);
}

bool E57ScanGeometry::e57_to_xyz(double theta, double phi, double r,
	Eigen::Vector3d& p)
{
	if (!(r > 0.0) || !std::isfinite(r) || !std::isfinite(theta) ||
		!std::isfinite(phi)) {
		return false;
	}
	double const cp = std::cos(phi);
	p = Eigen::Vector3d(r * cp * std::cos(theta), r * cp * std::sin(theta),
		r * std::sin(phi));
	return finite3(p);
}

bool E57ScanGeometry::xyz_to_e57(Eigen::Vector3d const& p, double& theta,
	double& phi, double& r)
{
	if (!finite3(p)) {
		return false;
	}
	r = p.norm();
	if (!(r > 0.0) || !std::isfinite(r)) {
		return false;
	}
	theta = std::atan2(p.y(), p.x());
	double const z_hat = std::min(1.0, std::max(-1.0, p.z() / r));
	phi = std::asin(z_hat);
	return true;
}

bool E57ScanGeometry::e57_to_pixel(double theta, double phi, int width,
	int height, Eigen::Vector2i& uv)
{
	if (width <= 0 || height <= 0 || !std::isfinite(theta) || !std::isfinite(phi)) {
		return false;
	}
	double const th = wrap_pm_pi(theta);
	double const X = static_cast<double>(width) * (0.5 - th / (2.0 * MATH_PI));
	double const Y = static_cast<double>(height) * (0.5 - phi / MATH_PI);
	int u = static_cast<int>(std::floor(X));
	int v = static_cast<int>(std::floor(Y));
	u = std::min(std::max(u, 0), width - 1);
	v = std::min(std::max(v, 0), height - 1);
	uv = Eigen::Vector2i(u, v);
	return true;
}

bool E57ScanGeometry::pixel_to_e57(Eigen::Vector2i const& uv, int width,
	int height, double& theta, double& phi)
{
	if (width <= 0 || height <= 0) {
		return false;
	}
	if (uv.x() < 0 || uv.x() >= width || uv.y() < 0 || uv.y() >= height) {
		return false;
	}
	double const X = static_cast<double>(uv.x()) + 0.5;
	double const Y = static_cast<double>(uv.y()) + 0.5;
	theta = wrap_pm_pi(2.0 * MATH_PI * (0.5 - X / static_cast<double>(width)));
	phi = MATH_PI * (0.5 - Y / static_cast<double>(height));
	return true;
}

bool E57ScanGeometry::encoder_to_xyz(double theta1, double phi1, double r,
	Eigen::Vector3d& p)
{
	double theta = 0.0;
	double phi = 0.0;
	encoder_to_e57(theta1, phi1, theta, phi);
	return e57_to_xyz(theta, phi, r, p);
}

bool E57ScanGeometry::xyz_to_encoder_normal(Eigen::Vector3d const& p,
	double& theta1, double& phi1, double& r)
{
	double theta = 0.0;
	double phi = 0.0;
	if (!xyz_to_e57(p, theta, phi, r)) {
		return false;
	}
	e57_to_encoder_normal(theta, phi, theta1, phi1);
	return true;
}

bool E57ScanGeometry::encoder_to_pixel(double theta1, double phi1, int width,
	int height, Eigen::Vector2i& uv)
{
	double theta = 0.0;
	double phi = 0.0;
	encoder_to_e57(theta1, phi1, theta, phi);
	return e57_to_pixel(theta, phi, width, height, uv);
}

bool E57ScanGeometry::pixel_to_encoder_normal(Eigen::Vector2i const& uv,
	int width, int height, double& theta1, double& phi1)
{
	double theta = 0.0;
	double phi = 0.0;
	if (!pixel_to_e57(uv, width, height, theta, phi)) {
		return false;
	}
	e57_to_encoder_normal(theta, phi, theta1, phi1);
	return true;
}

bool E57ScanGeometry::xyz_to_pixel(Eigen::Vector3d const& p, int width,
	int height, Eigen::Vector2i& uv)
{
	double theta = 0.0;
	double phi = 0.0;
	double r = 0.0;
	if (!xyz_to_e57(p, theta, phi, r)) {
		return false;
	}
	return e57_to_pixel(theta, phi, width, height, uv);
}

bool E57ScanGeometry::pixel_to_xyz(Eigen::Vector2i const& uv, int width,
	int height, Eigen::Vector3d& unit_dir)
{
	return pixel_to_xyz(uv, width, height, 1.0, unit_dir);
}

bool E57ScanGeometry::pixel_to_xyz(Eigen::Vector2i const& uv, int width,
	int height, double r, Eigen::Vector3d& p)
{
	double theta = 0.0;
	double phi = 0.0;
	if (!pixel_to_e57(uv, width, height, theta, phi)) {
		return false;
	}
	return e57_to_xyz(theta, phi, r, p);
}

}  // namespace Ddx
