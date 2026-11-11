#pragma once

#include "pose_graph_optimizer.hpp"

#include <registration_type/registration_type.hpp>

#include <Eigen/Core>
#include <map>
#include <string>
#include <vector>

namespace Ddx {

/// Isotropic information from AlignResult scalars (retained, not used by loadFromAlignResults):
/// Ω = λ I_6 with λ = max(sim,1e-5) / (max(weight,1e-6) * max(rms,1e-6)).
Eigen::Matrix<double, 6, 6> isotropicInformationFromAlign(AlignResult const& e);

/// Load nodes + edges into opt (clears first). Keeps only stateIsValid edges (VALID bit).
/// Edge Ω comes from AlignResult::information_ (default I). Returns false on add failure.
bool loadFromAlignResults(PoseGraphOptimizer& opt,
	std::vector<std::string> const& names,
	std::map<std::string, Eigen::Matrix4d> const& initial_poses,
	std::vector<AlignResult> const& edges);

} // namespace Ddx
