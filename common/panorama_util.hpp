#pragma once

#include <Eigen/Core>

namespace Ddx
{
	class PanoramaUtil
	{
	public:
		// 从空间（ x , y , z ）到球面坐标 （ theta , phi ）
		static Eigen::Vector2f math_spherical_xy_from_xyz(const Eigen::Vector3f& xyz_);
		// 从球面坐标 （ theta , phi ）到空间（ x , y , z ）
		static Eigen::Vector3f math_xyz_from_spherical_xy(const Eigen::Vector2f& xy_);

		// 从球面坐标（ theta , phi ）到纹理坐标 ( x , y )
		static Eigen::Vector2f math_texture_xy_from_spherical_xy(const Eigen::Vector2f& xy_);
		static Eigen::Vector2f math_texture_xy_from_spherical_xy(float const& theta, float const& phi);

		// 从纹理坐标( x , y )到球面坐标（ theta , phi ）
		static Eigen::Vector2f math_spherical_xy_from_texture_xy(const Eigen::Vector2f& xy_);
		static Eigen::Vector2f math_spherical_xy_from_texture_xy(float const& x, float const& y);

		// 从纹理坐标（x , y ) 到 像素坐标（u , v )
		static Eigen::Vector2i math_pixel_from_texture_xy(const Eigen::Vector2f& xy_, int width, int height);
		static Eigen::Vector2i math_pixel_from_texture_xy(float const& x, float const& y, int width, int height);

		// 从空间直接到全景图像素点
		static Eigen::Vector2i math_pixel_from_xyz(const Eigen::Vector3f& xyz_, int width, int height);

		// 从全景图像素点直接到空间坐标
		static Eigen::Vector3f math_xyz_from_pixel(const Eigen::Vector2i& xy_, int width, int height);
		static Eigen::Vector3f math_xyz_from_pixel(const Eigen::Vector2f& xy_, int width, int height);

		static float clamp(float x, float min_value, float max_value);
	};
}
