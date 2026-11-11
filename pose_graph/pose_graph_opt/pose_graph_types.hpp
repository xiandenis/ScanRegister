#pragma once

#include <Eigen/Core>
#include <string>

namespace Ddx {

struct PoseGraphNode {
	std::string name;
	Eigen::Matrix4d T_w{Eigen::Matrix4d::Identity()};
};

struct PoseGraphEdge {
	std::string src;
	std::string tgt;
	Eigen::Matrix4d T_rel{Eigen::Matrix4d::Identity()};
	Eigen::Matrix<double, 6, 6> information{Eigen::Matrix<double, 6, 6>::Identity()};
};

struct PoseGraphSummary {
	int iterations{0};
	double final_cost{0};
	int num_residuals{0};
	bool usable{false};
};

/// Per-edge residual report from current poses vs measured T_rel (PGO consistency).
/// Not a point-cloud RMS; see registration_report for geometric accuracy.
struct PoseGraphEdgeReport {
	std::string src;
	std::string tgt;
	Eigen::Matrix4d T_meas{Eigen::Matrix4d::Identity()}; // measured T_rel (src→tgt)
	Eigen::Matrix4d T_pred{Eigen::Matrix4d::Identity()}; // T_w_tgt^{-1} * T_w_src
	Eigen::Matrix<double, 6, 1> residual{Eigen::Matrix<double, 6, 1>::Zero()}; // [ρ; ϕ]
	double translation_error{0}; // ‖ρ‖ (m)
	double rotation_error{0};    // ‖ϕ‖ (rad)
	double chi2{0};              // eᵀ Ω e
};

} // namespace Ddx
