#include "pose_graph_optimizer.hpp"

#include "ceres_pose_graph_solver.hpp"
#include "se3_math.hpp"

#include <common/natural_name.hpp>
#include <logging/logging.hpp>

#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace Ddx {
namespace {

constexpr double kOmegaSymEps = 1e-9;

/// Invert rigid SE(3) without Eigen::Matrix4d::inverse() (NumericDiff leaves stack fragile).
Eigen::Matrix4d invertRigid(Eigen::Matrix4d const& T)
{
	Eigen::Matrix3d const R = T.block<3, 3>(0, 0);
	Eigen::Vector3d const t = T.block<3, 1>(0, 3);
	Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
	Ti.block<3, 3>(0, 0) = R.transpose();
	Ti.block<3, 1>(0, 3) = -R.transpose() * t;
	return Ti;
}

} // namespace

void PoseGraphOptimizer::clear()
{
	poses_.clear();
	edges_.clear();
	fixed_name_.clear();
	solver_.reset();
	options_ = PoseGraphSolveOptions{};
	summary_ = PoseGraphSummary{};
}

bool PoseGraphOptimizer::addNode(std::string const& name, Eigen::Matrix4d const& T_w_init)
{
	if (name.empty()) {
		return false;
	}
	if (poses_.count(name) != 0) {
		return false;
	}
	poses_.emplace(name, T_w_init);
	return true;
}

bool PoseGraphOptimizer::addEdge(std::string const& src,
	std::string const& tgt,
	Eigen::Matrix4d const& T_rel,
	Eigen::Matrix<double, 6, 6> const& Omega)
{
	if (src.empty() || tgt.empty() || src == tgt) {
		return false;
	}
	if (poses_.count(src) == 0 || poses_.count(tgt) == 0) {
		return false;
	}
	if (!isFiniteMatrix4d(T_rel) || !isValidInformation(Omega)) {
		return false;
	}
	PoseGraphEdge e;
	e.src = src;
	e.tgt = tgt;
	e.T_rel = T_rel;
	e.information = Omega;
	edges_.push_back(std::move(e));
	return true;
}

void PoseGraphOptimizer::setFixedNode(std::string const& name)
{
	fixed_name_ = name;
}

void PoseGraphOptimizer::setSolver(std::unique_ptr<IPoseGraphSolver> solver)
{
	solver_ = std::move(solver);
}

void PoseGraphOptimizer::enableHuberLoss(double delta)
{
	options_.huber_delta = delta;
}

void PoseGraphOptimizer::disableLoss()
{
	options_.huber_delta = -1.0;
}

bool PoseGraphOptimizer::isFiniteMatrix4d(Eigen::Matrix4d const& T)
{
	for (int r = 0; r < 4; ++r) {
		for (int c = 0; c < 4; ++c) {
			if (!std::isfinite(T(r, c))) {
				return false;
			}
		}
	}
	return true;
}

bool PoseGraphOptimizer::isValidInformation(Eigen::Matrix<double, 6, 6> const& Omega)
{
	for (int r = 0; r < 6; ++r) {
		for (int c = 0; c < 6; ++c) {
			if (!std::isfinite(Omega(r, c))) {
				return false;
			}
		}
	}
	Eigen::Matrix<double, 6, 6> const asym = Omega - Omega.transpose();
	return asym.norm() <= kOmegaSymEps;
}

bool PoseGraphOptimizer::isConnectedUndirected() const
{
	if (poses_.empty()) {
		return false;
	}
	std::unordered_map<std::string, std::vector<std::string>> adj;
	adj.reserve(poses_.size());
	for (auto const& kv : poses_) {
		adj[kv.first] = {};
	}
	for (auto const& e : edges_) {
		adj[e.src].push_back(e.tgt);
		adj[e.tgt].push_back(e.src);
	}

	std::string const start = poses_.begin()->first;
	std::unordered_set<std::string> seen;
	std::queue<std::string> q;
	q.push(start);
	seen.insert(start);
	while (!q.empty()) {
		std::string const u = q.front();
		q.pop();
		for (std::string const& v : adj[u]) {
			if (seen.insert(v).second) {
				q.push(v);
			}
		}
	}
	return seen.size() == poses_.size();
}

bool PoseGraphOptimizer::validateGraph(std::string& fixed_out) const
{
	if (poses_.empty()) {
		LOG_ERROR("PoseGraphOptimizer: empty graph");
		return false;
	}

	for (auto const& kv : poses_) {
		if (!isFiniteMatrix4d(kv.second)) {
			LOG_ERROR("PoseGraphOptimizer: non-finite initial pose for '{}'", kv.first);
			return false;
		}
	}

	for (auto const& e : edges_) {
		if (poses_.count(e.src) == 0 || poses_.count(e.tgt) == 0) {
			LOG_ERROR("PoseGraphOptimizer: edge endpoint missing ('{}' -> '{}')", e.src, e.tgt);
			return false;
		}
		if (!isFiniteMatrix4d(e.T_rel) || !isValidInformation(e.information)) {
			LOG_ERROR("PoseGraphOptimizer: invalid T_rel or Omega on edge ('{}' -> '{}')", e.src, e.tgt);
			return false;
		}
	}

	if (!isConnectedUndirected()) {
		LOG_ERROR("PoseGraphOptimizer: graph is not a single connected component");
		return false;
	}

	if (!fixed_name_.empty()) {
		if (poses_.count(fixed_name_) == 0) {
			LOG_ERROR("PoseGraphOptimizer: fixed node '{}' not found", fixed_name_);
			return false;
		}
		fixed_out = fixed_name_;
	} else {
		fixed_out = poses_.begin()->first;
		for (auto const& item : poses_) {
			if (naturalNameLess(item.first, fixed_out)) {
				fixed_out = item.first;
			}
		}
	}
	return true;
}

bool PoseGraphOptimizer::optimize()
{
	std::string fixed;
	if (!validateGraph(fixed)) {
		summary_ = PoseGraphSummary{};
		return false;
	}

	// Single station, no edges: trivial success (gauge only).
	if (poses_.size() == 1 && edges_.empty()) {
		summary_.iterations = 0;
		summary_.final_cost = 0.0;
		summary_.num_residuals = 0;
		summary_.usable = true;
		return true;
	}

	if (!solver_) {
		solver_ = std::make_unique<CeresPoseGraphSolver>();
	}

	std::vector<PoseGraphNode> nodes;
	nodes.reserve(poses_.size());
	for (auto const& kv : poses_) {
		PoseGraphNode n;
		n.name = kv.first;
		n.T_w = kv.second;
		nodes.push_back(std::move(n));
	}

	std::map<std::string, Eigen::Matrix4d> out_poses;
	PoseGraphSummary solve_summary;
	bool const ok = solver_->solve(nodes, edges_, fixed, out_poses, solve_summary, options_);
	if (!ok || !solve_summary.usable) {
		LOG_ERROR("PoseGraphOptimizer: solve failed (usable={})", solve_summary.usable);
		summary_ = PoseGraphSummary{};
		// Do not overwrite initial poses_
		return false;
	}

	for (auto const& kv : out_poses) {
		auto it = poses_.find(kv.first);
		if (it != poses_.end()) {
			it->second = kv.second;
		}
	}
	summary_ = solve_summary;
	return true;
}

bool PoseGraphOptimizer::getPose(std::string const& name, Eigen::Matrix4d& T_w) const
{
	auto it = poses_.find(name);
	if (it == poses_.end()) {
		return false;
	}
	T_w = it->second;
	return true;
}

std::map<std::string, Eigen::Matrix4d> PoseGraphOptimizer::getAllPoses() const
{
	return poses_;
}

PoseGraphSummary PoseGraphOptimizer::summary() const
{
	return summary_;
}

std::vector<PoseGraphEdgeReport> PoseGraphOptimizer::getEdgeReports() const
{
	std::vector<PoseGraphEdgeReport> reports;
	reports.reserve(edges_.size());
	for (auto const& e : edges_) {
		auto const it_src = poses_.find(e.src);
		auto const it_tgt = poses_.find(e.tgt);
		if (it_src == poses_.end() || it_tgt == poses_.end()) {
			continue;
		}
		Eigen::Matrix4d const T_pred = invertRigid(it_tgt->second) * it_src->second;
		Eigen::Matrix<double, 6, 1> const residual = se3Log(invertRigid(e.T_rel) * T_pred);

		reports.emplace_back();
		PoseGraphEdgeReport& r = reports.back();
		r.src = e.src;
		r.tgt = e.tgt;
		r.T_meas = e.T_rel;
		r.T_pred = T_pred;
		r.residual = residual;
		r.translation_error = residual.head<3>().norm();
		r.rotation_error = residual.tail<3>().norm();
		r.chi2 = static_cast<double>((residual.transpose() * e.information * residual)(0, 0));
	}
	return reports;
}

} // namespace Ddx
