#pragma once

#include "pose_graph_solver.hpp"
#include "pose_graph_types.hpp"

#include <Eigen/Core>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace Ddx {

/// Facade: build a pose graph, validate connectivity/gauge, then call IPoseGraphSolver.
/// Public header intentionally does not include <ceres/...>.
class PoseGraphOptimizer {
public:
	void clear();

	/// Insert node with world pose initial value. Duplicate name → false.
	bool addNode(std::string const& name, Eigen::Matrix4d const& T_w_init);

	/// Undirected connectivity; endpoints must already exist. Invalid Ω → false.
	bool addEdge(std::string const& src,
		std::string const& tgt,
		Eigen::Matrix4d const& T_rel,
		Eigen::Matrix<double, 6, 6> const& Omega);

	/// Gauge: pin this station. Unset → natural-order smallest name at optimize().
	void setFixedNode(std::string const& name);

	/// Default (if unset before optimize): make_unique<CeresPoseGraphSolver>().
	void setSolver(std::unique_ptr<IPoseGraphSolver> solver);

	void enableHuberLoss(double delta);
	void disableLoss();

	/// Validate + solve. Failure: LOG_ERROR and leave initial poses unchanged.
	bool optimize();

	bool getPose(std::string const& name, Eigen::Matrix4d& T_w) const;
	std::map<std::string, Eigen::Matrix4d> getAllPoses() const;
	PoseGraphSummary summary() const;

	/// Edge residual reports from current poses_ vs each measured T_rel (order = addEdge order).
	/// After successful optimize(): post-PGO consistency; before optimize(): initial poses.
	std::vector<PoseGraphEdgeReport> getEdgeReports() const;

private:
	bool validateGraph(std::string& fixed_out) const;
	bool isConnectedUndirected() const;
	static bool isFiniteMatrix4d(Eigen::Matrix4d const& T);
	static bool isValidInformation(Eigen::Matrix<double, 6, 6> const& Omega);

	std::map<std::string, Eigen::Matrix4d> poses_;
	std::vector<PoseGraphEdge> edges_;
	std::string fixed_name_; // empty → auto gauge
	std::unique_ptr<IPoseGraphSolver> solver_;
	PoseGraphSolveOptions options_;
	PoseGraphSummary summary_;
};

} // namespace Ddx
