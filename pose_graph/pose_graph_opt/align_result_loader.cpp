#include "align_result_loader.hpp"

#include <algorithm>

namespace Ddx {

Eigen::Matrix<double, 6, 6> isotropicInformationFromAlign(AlignResult const& e)
{
	// Retained helper (rms/sim/weight → λI). Not called by loadFromAlignResults;
	// active path uses AlignResult::information_ instead.
	double const sim = (std::max)(e.similarity_, 1e-5);
	double const den = (std::max)(e.weight_, 1e-6) * (std::max)(e.rms_, 1e-6);
	double const lambda = sim / den;
	return lambda * Eigen::Matrix<double, 6, 6>::Identity();
}

bool loadFromAlignResults(PoseGraphOptimizer& opt,
	std::vector<std::string> const& names,
	std::map<std::string, Eigen::Matrix4d> const& initial_poses,
	std::vector<AlignResult> const& edges)
{
	opt.clear();

	for (std::string const& name : names) {
		auto const it = initial_poses.find(name);
		if (it == initial_poses.end()) {
			return false;
		}
		if (!opt.addNode(name, it->second)) {
			return false;
		}
	}

	for (AlignResult const& edge : edges) {
		if (!stateIsValid(edge.state_)) {
			continue;
		}

		// Endpoints must already be nodes (same idea as PoseBase skipping unknown frames).
		Eigen::Matrix4d unused;
		if (!opt.getPose(edge.src_name_, unused) || !opt.getPose(edge.tgt_name_, unused)) {
			continue;
		}

		if (!opt.addEdge(edge.src_name_, edge.tgt_name_, edge.matrix_, edge.information_)) {
			return false;
		}
	}

	return true;
}

} // namespace Ddx
