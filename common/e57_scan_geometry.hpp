#pragma once

#include <Eigen/Core>

namespace Ddx {

/**
 * 码盘角 ↔ E57 球坐标 ↔ 测站系 XYZ ↔ E57 满球全景像素。
 * 约定：docs/扫描仪数据的定义、E57全景坐标定义.md（§1 / §4 / §5）。
 *
 * 公开角度一律为弧度。设备度制请用 deg_to_rad / rad_to_deg 在边界换算。
 *
 * 天顶/天底（|φ|→π/2）：方位 θ / 码盘 θ₁ / 像素列 u 几何不唯一；本类按公式原样输出，
 * 调用方勿将其当真值（互逆时方位可差一周或任意）。
 */
class E57ScanGeometry {
public:
	static double deg_to_rad(double deg);
	static double rad_to_deg(double rad);

	/** wrap to [0, 2π) */
	static double wrap_2pi(double rad);
	/** wrap to (-π, π] */
	static double wrap_pm_pi(double rad);

	// --- §4 / §5：码盘 ↔ E57（弧度）---
	// θ₁ 竖轴（水平码盘），φ₁ 横轴（垂直码盘）

	static void encoder_to_e57(double theta1, double phi1, double& theta,
		double& phi);

	/** §5.2 正常站扫唯一解：θ₁∈[0,π]，φ₁∈[0,2π) */
	static void e57_to_encoder_normal(double theta, double phi, double& theta1,
		double& phi1);

	/** §5.3 正镜解 */
	static void e57_to_encoder_front(double theta, double phi, double& theta1,
		double& phi1);

	/** §5.3 倒镜解 */
	static void e57_to_encoder_mirror(double theta, double phi, double& theta1,
		double& phi1);

	// --- §1：E57 ↔ XYZ ---

	/** r>0 且有限 → true；写出 p = r·p̂ */
	static bool e57_to_xyz(double theta, double phi, double r, Eigen::Vector3d& p);

	/** ‖p‖>0 且有限 → true；天顶/天底方位按公式原样 */
	static bool xyz_to_e57(Eigen::Vector3d const& p, double& theta, double& phi,
		double& r);

	// --- §1：E57 ↔ 满球像素 (W×H) ---

	static bool e57_to_pixel(double theta, double phi, int width, int height,
		Eigen::Vector2i& uv);

	/** 整数像素中心 (u+1/2, v+1/2) → E57；非法尺寸或越界 → false */
	static bool pixel_to_e57(Eigen::Vector2i const& uv, int width, int height,
		double& theta, double& phi);

	// --- 组合 ---

	static bool encoder_to_xyz(double theta1, double phi1, double r,
		Eigen::Vector3d& p);

	static bool xyz_to_encoder_normal(Eigen::Vector3d const& p, double& theta1,
		double& phi1, double& r);

	static bool encoder_to_pixel(double theta1, double phi1, int width, int height,
		Eigen::Vector2i& uv);

	static bool pixel_to_encoder_normal(Eigen::Vector2i const& uv, int width,
		int height, double& theta1, double& phi1);

	static bool xyz_to_pixel(Eigen::Vector3d const& p, int width, int height,
		Eigen::Vector2i& uv);

	/** 像素 → 单位方向（等价 r=1） */
	static bool pixel_to_xyz(Eigen::Vector2i const& uv, int width, int height,
		Eigen::Vector3d& unit_dir);

	static bool pixel_to_xyz(Eigen::Vector2i const& uv, int width, int height,
		double r, Eigen::Vector3d& p);
};

}  // namespace Ddx
