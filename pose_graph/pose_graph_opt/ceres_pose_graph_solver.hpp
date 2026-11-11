#pragma once

#include "pose_graph_solver.hpp"

namespace Ddx {

/// Ceres backend for batch SE(3) pose-graph optimization.
/// Public header intentionally does not include <ceres/...>.
class CeresPoseGraphSolver : public IPoseGraphSolver {
public:
	bool solve(
		std::vector<PoseGraphNode> const& nodes,
		std::vector<PoseGraphEdge> const& edges,
		std::string const& fixed_name,
		std::map<std::string, Eigen::Matrix4d>& out_poses,
		PoseGraphSummary& summary,
		PoseGraphSolveOptions const& opt) override;
};

} // namespace Ddx
