#include "panorama_util.hpp"

#define MATH_PI         3.14159265358979323846264338327950288   // pi
#define MATH_PI_2       1.57079632679489661923132169163975144   // pi/2
#define MATH_PI_4       0.785398163397448309615660845819875721  // pi/4
#define MATH_1_PI       0.318309886183790671537767526745028724  // 1/pi
#define MATH_2_PI       0.636619772367581343075535053490057448  // 2/pi

#define MATH_SQRT2      1.41421356237309504880168872420969808   // sqrt(2)
#define MATH_SQRT3      1.7320508075688772935274463415058723669 // sqrt(3)
#define MATH_1_SQRT_2   0.707106781186547524400844362104849039  // 1/sqrt(2)
#define MATH_2_SQRT_PI  1.12837916709551257389615890312154517   // 2/sqrt(pi)
#define MATH_SQRT_PI    1.7724538509055160272981674833411451828 // sqrt(pi)
#define MATH_SQRT_2PI   2.506628274631000502415765284811045253  // sqrt(2*pi)

namespace Ddx
{
	Eigen::Vector2f
	PanoramaUtil::math_spherical_xy_from_xyz(const Eigen::Vector3f& xyz_)
	{
		const auto r1x = xyz_.normalized();
		const auto r2x = std::atan2(r1x[1], r1x[0]);		// theta	方位角(水平角度)
		const auto r3x = std::acos(r1x[2]);					// phi		仰角（竖直角度）
		return Eigen::Vector2f(r2x, r3x);
	}

	Eigen::Vector3f
	PanoramaUtil::math_xyz_from_spherical_xy(const Eigen::Vector2f& xy_)
	{
		const auto r3x = std::sin(xy_[1]) * std::cos(xy_[0]);
		const auto r4x = std::sin(xy_[1]) * std::sin(xy_[0]);
		const auto r5x = std::cos(xy_[1]);
		const auto r6x = Eigen::Vector3f(r3x, r4x, r5x).normalized();
		return r6x;
	}

	Eigen::Vector2f
	PanoramaUtil::math_texture_xy_from_spherical_xy(const Eigen::Vector2f& xy_)
	{
		return math_texture_xy_from_spherical_xy(xy_[0], xy_[1]);
	}

	Eigen::Vector2f
	PanoramaUtil::math_texture_xy_from_spherical_xy(float const& theta, float const& phi)
	{
		Eigen::Vector2f ret;
		ret[0] = float(0.5) - theta / float(MATH_PI * 2.0);
		ret[1] = phi / float(MATH_PI);
		return ret;
	}

	Eigen::Vector2f
	PanoramaUtil::math_spherical_xy_from_texture_xy(const Eigen::Vector2f& xy_)
	{
		return math_spherical_xy_from_texture_xy(xy_[0], xy_[1]);
	}

	Eigen::Vector2f
	PanoramaUtil::math_spherical_xy_from_texture_xy(float const& x, float const& y)
	{
		const auto r1x = (float(0.5) - x) * float(MATH_PI * 2);
		const auto r2x = y * float(MATH_PI);
		return Eigen::Vector2f(r1x, r2x);
	}

	Eigen::Vector2i
	PanoramaUtil::math_pixel_from_texture_xy(const Eigen::Vector2f& xy_, int width, int height)
	{
		return math_pixel_from_texture_xy(xy_[0], xy_[1], width, height);
	}

	Eigen::Vector2i
	PanoramaUtil::math_pixel_from_texture_xy(float const& x, float const& y, int width, int height)
	{
		// 取固定的范围
		const float r1x = clamp(x * float(width), 0, float(width) - 1e-5);
		const float r2x = clamp(y * float(height), 0, float(height) - 1e-5);

		// 向下取整
		const auto r3x = static_cast<int> (std::floor(r1x));
		const auto r4x = static_cast<int> (std::floor(r2x));

		const auto r5x = std::min(std::max(r3x, 0), width - 1);
		const auto r6x = std::min(std::max(r4x, 0), height - 1);

		return Eigen::Vector2i(r5x, r6x);
	}

	float
	PanoramaUtil::clamp(float x, float min_value, float max_value)
	{
		return std::max(min_value, std::min(x, max_value));
	}

	Eigen::Vector2i
	PanoramaUtil::math_pixel_from_xyz(const Eigen::Vector3f& xyz_, int width, int height)
	{
		// 空间到球面坐标系
		const auto r1x = math_spherical_xy_from_xyz(xyz_);

		// 从球面到 纹理空间
		const auto r2x = math_texture_xy_from_spherical_xy(r1x);

		// 从纹理空间到像素空间
		return math_pixel_from_texture_xy(r2x, width, height);
	}

	Eigen::Vector3f
	PanoramaUtil::math_xyz_from_pixel(const Eigen::Vector2i& xy_, int width, int height)
	{
		// 从像素空间到纹理空间
		Eigen::Vector2f r1x;
		r1x[0] = clamp((xy_[0] + 0.5f) / float(width), 0.f, float(1.0));
		r1x[1] = clamp((xy_[1] + 0.5f) / float(height), 0.f, float(1.0));

		Eigen::Vector2f r2x = math_spherical_xy_from_texture_xy(r1x);

		return math_xyz_from_spherical_xy(r2x);
	}

	Eigen::Vector3f
	PanoramaUtil::math_xyz_from_pixel(const Eigen::Vector2f& xy_, int width, int height)
	{
		// 从像素空间到纹理空间
		Eigen::Vector2f r1x;
		r1x[0] = clamp(xy_[0] / float(width), 0.f, float(1.0));
		r1x[1] = clamp(xy_[1] / float(height), 0.f, float(1.0));

		Eigen::Vector2f r2x = math_spherical_xy_from_texture_xy(r1x);

		return math_xyz_from_spherical_xy(r2x);
	}
}