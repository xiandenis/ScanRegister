#include "trim_icp_solver.hpp"

namespace Ddx {

bool TrimIcpSolver::align(const Eigen::Matrix4d& initial_guess)
{
	if (!checkCloud()) {
		return false;
	}

	const int n_src = static_cast<int>(m_src_points->size());
	const int n_tgt = static_cast<int>(m_tgt_points->size());

	Eigen::Matrix<double, 3, Eigen::Dynamic> source_ppp(3, n_src);
	Eigen::Matrix<double, 3, Eigen::Dynamic> target_ppp(3, n_tgt);
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < n_src; ++i) {
		source_ppp.col(i).noalias() =
			m_src_points->points[static_cast<std::size_t>(i)].getVector3fMap().cast<double>();
	}
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < n_tgt; ++i) {
		target_ppp.col(i).noalias() =
			m_tgt_points->points[static_cast<std::size_t>(i)].getVector3fMap().cast<double>();
	}

	Ddx::registration::BasicICP icp;
	icp.setSourceCloud(source_ppp);
	icp.setTargetCloud(target_ppp);

	const bool plane_method = m_method == Ddx::registration::BasicICP::Method::PPL
		|| m_method == Ddx::registration::BasicICP::Method::RPPL
		|| m_method == Ddx::registration::BasicICP::Method::SICPPPL;

	if (m_src_normals != nullptr) {
		if (static_cast<int>(m_src_normals->size()) != n_src) {
			return false;
		}
		Eigen::Matrix<double, 3, Eigen::Dynamic> source_n(3, n_src);
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < n_src; ++i) {
			source_n.col(i).noalias() =
				m_src_normals->points[static_cast<std::size_t>(i)].getNormalVector3fMap().cast<double>();
		}
		icp.setSourceNormal(source_n);
	}

	if (plane_method || m_tgt_normals != nullptr) {
		if (m_tgt_normals == nullptr || static_cast<int>(m_tgt_normals->size()) != n_tgt) {
			return false;
		}
		Eigen::Matrix<double, 3, Eigen::Dynamic> target_n(3, n_tgt);
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < n_tgt; ++i) {
			target_n.col(i).noalias() =
				m_tgt_normals->points[static_cast<std::size_t>(i)].getNormalVector3fMap().cast<double>();
		}
		icp.setTargetNormal(target_n);
	}

	icp.setMethod(m_method);
	icp.setProfile(m_profile);

	if (!icp.align(initial_guess)) {
		return false;
	}
	m_transformation = icp.getFinalTransformationMatrix();
	if (m_profile) {
		m_profile_welsch_stages = icp.profileWelschStages();
		m_profile_icp_iters = icp.profileIcpIters();
		m_profile_closest_passes = icp.profileClosestPasses();
		m_profile_aa_rejects = icp.profileAaRejects();
		m_profile_init_ms = icp.profileInitMs();
		m_profile_closest_ms = icp.profileClosestMs();
		m_profile_update_ms = icp.profileUpdateMs();
	} else {
		m_profile_welsch_stages = 0;
		m_profile_icp_iters = 0;
		m_profile_closest_passes = 0;
		m_profile_aa_rejects = 0;
		m_profile_init_ms = 0.0;
		m_profile_closest_ms = 0.0;
		m_profile_update_ms = 0.0;
	}
	return true;
}

bool TrimIcpSolver::alignPPL(const Eigen::Matrix4d& initial_guess)
{
	m_method = Ddx::registration::BasicICP::Method::SICPPPL;
	return align(initial_guess);
}

bool TrimIcpSolver::checkCloud() const
{
	if (m_src_points == nullptr || m_tgt_points == nullptr) {
		return false;
	}
	if (m_src_points->size() < 10 || m_tgt_points->size() < 10) {
		return false;
	}
	return true;
}

} // namespace Ddx
