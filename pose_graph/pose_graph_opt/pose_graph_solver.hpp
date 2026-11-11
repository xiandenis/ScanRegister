#pragma once

#include "pose_graph_types.hpp"

#include <Eigen/Core>
#include <map>
#include <string>
#include <vector>

namespace Ddx {

struct PoseGraphSolveOptions {
	double huber_delta{-1}; // <0 表示无鲁棒核
	int max_num_iterations{50};
};

class IPoseGraphSolver {
public:
	virtual ~IPoseGraphSolver() = default;
	virtual bool solve(
		std::vector<PoseGraphNode> const& nodes,
		std::vector<PoseGraphEdge> const& edges,
		std::string const& fixed_name,
		std::map<std::string, Eigen::Matrix4d>& out_poses,
		PoseGraphSummary& summary,
		PoseGraphSolveOptions const& opt) = 0;
};

} // namespace Ddx
