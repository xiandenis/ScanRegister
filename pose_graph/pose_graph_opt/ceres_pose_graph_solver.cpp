#include "ceres_pose_graph_solver.hpp"

#include "se3_math.hpp"

#include "logging/logging.hpp"

#include <ceres/ceres.h>
#include <ceres/manifold.h>

#include <memory>
#include <unordered_map>
#include <vector>

namespace Ddx {
namespace {

struct QtBlocks {
	double q[4]{};
	double t[3]{};
};

/// Relative-pose residual: r = L * Log(T_meas^{-1} * T_tgt^{-1} * T_src).
/// Parameter order: q_src[4], t_src[3], q_tgt[4], t_tgt[3].
struct RelativePoseCostFunctor {
	RelativePoseCostFunctor(Eigen::Matrix4d const& T_meas, Eigen::Matrix<double, 6, 6> const& L)
		: T_meas_(T_meas)
		, L_(L)
	{
	}

	bool operator()(double const* const q_src,
		double const* const t_src,
		double const* const q_tgt,
		double const* const t_tgt,
		double* residuals) const
	{
		Eigen::Matrix4d const T_src = qtToMatrix(q_src, t_src);
		Eigen::Matrix4d const T_tgt = qtToMatrix(q_tgt, t_tgt);
		Eigen::Matrix4d const T_pred = T_tgt.inverse() * T_src;
		Eigen::Matrix4d const T_err = T_meas_.inverse() * T_pred;
		Eigen::Matrix<double, 6, 1> const e = se3Log(T_err);
		Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);
		r = L_ * e;
		return true;
	}

	Eigen::Matrix4d T_meas_;
	Eigen::Matrix<double, 6, 6> L_;
};

} // namespace

bool CeresPoseGraphSolver::solve(
	std::vector<PoseGraphNode> const& nodes,
	std::vector<PoseGraphEdge> const& edges,
	std::string const& fixed_name,
	std::map<std::string, Eigen::Matrix4d>& out_poses,
	PoseGraphSummary& summary,
	PoseGraphSolveOptions const& opt)
{
	summary = PoseGraphSummary{};

	if (nodes.empty()) {
		LOG_ERROR("CeresPoseGraphSolver: empty node list");
		return false;
	}

	std::unordered_map<std::string, QtBlocks> params;
	params.reserve(nodes.size());
	for (PoseGraphNode const& n : nodes) {
		if (!n.T_w.allFinite()) {
			LOG_ERROR("CeresPoseGraphSolver: non-finite initial pose for '{}'", n.name);
			return false;
		}
		QtBlocks& qt = params[n.name];
		matrixToQt(n.T_w, qt.q, qt.t);
	}

	if (params.find(fixed_name) == params.end()) {
		LOG_ERROR("CeresPoseGraphSolver: fixed node '{}' not found", fixed_name);
		return false;
	}

	// Pre-factor all Ω; any failure aborts before writing out_poses.
	std::vector<Eigen::Matrix<double, 6, 6>> edge_L;
	edge_L.reserve(edges.size());
	for (PoseGraphEdge const& e : edges) {
		if (params.find(e.src) == params.end() || params.find(e.tgt) == params.end()) {
			LOG_ERROR("CeresPoseGraphSolver: edge '{} -> {}' references missing node", e.src, e.tgt);
			return false;
		}
		if (!e.T_rel.allFinite()) {
			LOG_ERROR("CeresPoseGraphSolver: non-finite T_rel on edge '{} -> {}'", e.src, e.tgt);
			return false;
		}
		Eigen::Matrix<double, 6, 6> L;
		if (!infoCholL(e.information, L)) {
			LOG_ERROR("CeresPoseGraphSolver: infoCholL failed on edge '{} -> {}'", e.src, e.tgt);
			return false;
		}
		edge_L.push_back(L);
	}

	// Shared Huber loss: Problem must not take ownership (would double-delete).
	std::unique_ptr<ceres::LossFunction> loss_owner;
	ceres::LossFunction* loss = nullptr;
	if (opt.huber_delta >= 0.0) {
		loss_owner = std::make_unique<ceres::HuberLoss>(opt.huber_delta);
		loss = loss_owner.get();
	}

	ceres::Problem::Options problem_options;
	problem_options.cost_function_ownership = ceres::TAKE_OWNERSHIP;
	problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problem_options.manifold_ownership = ceres::TAKE_OWNERSHIP;
	ceres::Problem problem(problem_options);

	for (auto& kv : params) {
		QtBlocks& qt = kv.second;
		problem.AddParameterBlock(qt.q, 4);
		problem.AddParameterBlock(qt.t, 3);
		// q layout is (w,x,y,z) from matrixToQt → QuaternionManifold (not EigenQuaternionManifold).
		problem.SetManifold(qt.q, new ceres::QuaternionManifold());
	}

	{
		QtBlocks& fixed = params[fixed_name];
		problem.SetParameterBlockConstant(fixed.q);
		problem.SetParameterBlockConstant(fixed.t);
	}

	for (std::size_t i = 0; i < edges.size(); ++i) {
		PoseGraphEdge const& e = edges[i];
		QtBlocks& src = params[e.src];
		QtBlocks& tgt = params[e.tgt];

		auto* functor = new RelativePoseCostFunctor(e.T_rel, edge_L[i]);
		ceres::CostFunction* cost =
			new ceres::NumericDiffCostFunction<RelativePoseCostFunctor, ceres::CENTRAL, 6, 4, 3, 4, 3>(
				functor);
		problem.AddResidualBlock(cost, loss, src.q, src.t, tgt.q, tgt.t);
	}

	ceres::Solver::Options options;
	options.max_num_iterations = opt.max_num_iterations;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = false;

	ceres::Solver::Summary ceres_summary;
	ceres::Solve(options, &problem, &ceres_summary);

	summary.iterations = static_cast<int>(ceres_summary.iterations.size());
	summary.final_cost = ceres_summary.final_cost;
	summary.num_residuals = ceres_summary.num_residuals;
	summary.usable = ceres_summary.IsSolutionUsable();

	if (!summary.usable) {
		LOG_ERROR("CeresPoseGraphSolver: solution not usable ({})", ceres_summary.message);
		return false;
	}

	out_poses.clear();
	for (auto const& kv : params) {
		out_poses.emplace(kv.first, qtToMatrix(kv.second.q, kv.second.t));
	}
	return true;
}

} // namespace Ddx
