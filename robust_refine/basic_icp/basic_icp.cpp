#include "basic_icp.h"
#include "ICP.h"
#include "FRICP.h"

using namespace Ddx;
using namespace registration;

namespace {

void clearProfileFields(ICP::Parameters& pars)
{
	pars.profile_welsch_stages = 0;
	pars.profile_icp_iters = 0;
	pars.profile_closest_passes = 0;
	pars.profile_aa_rejects = 0;
	pars.profile_init_ms = 0.0;
	pars.profile_closest_ms = 0.0;
	pars.profile_update_ms = 0.0;
}

}  // namespace

bool BasicICP::align(Eigen::Matrix4d const& gnss)
{
	m_profile_welsch_stages = 0;
	m_profile_icp_iters = 0;
	m_profile_closest_passes = 0;
	m_profile_aa_rejects = 0;
	m_profile_init_ms = 0.0;
	m_profile_closest_ms = 0.0;
	m_profile_update_ms = 0.0;

	// scaling(最大值减去最小值)
	Eigen::Vector3d source_scale, target_scale;
	source_scale.noalias() = m_sourceCloud.rowwise().maxCoeff() - m_sourceCloud.rowwise().minCoeff();
	target_scale.noalias() = m_targetCloud.rowwise().maxCoeff() - m_targetCloud.rowwise().minCoeff();
	float scale = std::max(source_scale.norm(), target_scale.norm());

	// 尺度缩放
	m_sourceCloud /= scale;
	m_targetCloud /= scale;


	// 去中心化
	Eigen::Vector3d source_mean, target_mean;
	source_mean.noalias() = m_sourceCloud.rowwise().sum() / double(m_sourceCloud.cols());
	target_mean.noalias() = m_targetCloud.rowwise().sum() / double(m_targetCloud.cols());
	m_sourceCloud.colwise() -= source_mean;
	m_targetCloud.colwise() -= target_mean;


	// set Sparse-ICP parameters
	ICP::Parameters pars;
	SICP::Parameters spars;
	spars.p = 0.4;
	spars.print_icpn = false;
	pars.profile = m_profile;
	if (m_profile) {
		clearProfileFields(pars);
	}


	// 对矩阵的平移分量就行缩放
	if (true)
	{
		// std::cout << "输入矩阵  .. \n" << gnss << std::endl;
		int dim = 3;
		Eigen::Matrix4d init_trans = gnss;
		init_trans.block(0, dim, dim, 1) /= scale;
		init_trans.block(0, 3, 3, 1) += init_trans.block(0, 0, 3, 3)*source_mean - target_mean;
		pars.use_init = true;
		pars.init_trans = init_trans;
		spars.init_trans = init_trans;
	}

	FRICP<3> fricp;
	switch (m_method)
	{
	case Method::ICP:
	{
		pars.f = ICP::NONE;
		pars.use_AA = false;
		fricp.point_to_point(m_sourceCloud, m_targetCloud, source_mean, target_mean, pars);
		m_transformation_matrix = pars.res_trans;
		break;
	}
	case Method::AA_ICP:
	{
		AAICP::point_to_point_aaicp(m_sourceCloud, m_targetCloud, source_mean, target_mean, pars);
		m_transformation_matrix = pars.res_trans;
		break;
	}
	case Method::FICP:
	{
		pars.f = ICP::NONE;
		pars.use_AA = true;
		fricp.point_to_point(m_sourceCloud, m_targetCloud, source_mean, target_mean, pars);
		m_transformation_matrix = pars.res_trans;
		break;
	}
	case Method::RICP:
	{
		pars.f = ICP::WELSCH;
		pars.use_AA = true;
		fricp.point_to_point(m_sourceCloud, m_targetCloud, source_mean, target_mean, pars);
		m_transformation_matrix = pars.res_trans;
		break;
	}
	case Method::PPL:
	{
		pars.f = ICP::NONE;
		pars.use_AA = false;
		if (m_targetNormal.size() == 0)
		{
			std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
			exit(0);
		}
		fricp.point_to_plane(m_sourceCloud, m_targetCloud, m_sourceNormal, m_targetNormal, source_mean, target_mean, pars);
		m_transformation_matrix = pars.res_trans;
		break;
	}
	case Method::RPPL:
	{
		pars.nu_end_k = 1.0 / 6;
		pars.f = ICP::WELSCH;
		pars.use_AA = true;
		if (m_targetNormal.size() == 0)
		{
			std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
			exit(0);
		}
		fricp.point_to_plane_GN(m_sourceCloud, m_targetCloud, m_sourceNormal, m_targetNormal, source_mean, target_mean, pars);
		m_transformation_matrix = pars.res_trans;
		break;
	}
	case Method::SparseICP:
	{
		SICP::point_to_point(m_sourceCloud, m_targetCloud, source_mean, target_mean, spars);
		m_transformation_matrix = spars.res_trans;
		break;
	}
	case Method::SICPPPL:
	{
		if (m_targetNormal.size() == 0)
		{
			std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
			exit(0);
		}
		SICP::point_to_plane(m_sourceCloud, m_targetCloud, m_targetNormal, source_mean, target_mean, spars);
		m_transformation_matrix = spars.res_trans;
		break;
	}
	default:
		return false;
		break;
	}

	if (m_profile) {
		m_profile_welsch_stages = pars.profile_welsch_stages;
		m_profile_icp_iters = pars.profile_icp_iters;
		m_profile_closest_passes = pars.profile_closest_passes;
		m_profile_aa_rejects = pars.profile_aa_rejects;
		m_profile_init_ms = pars.profile_init_ms;
		m_profile_closest_ms = pars.profile_closest_ms;
		m_profile_update_ms = pars.profile_update_ms;
	}

	m_transformation_matrix.block(0, 3, 3, 1) *= scale;


	// std::cout << "输出矩阵  .. \n" << m_transformation_matrix << std::endl;
// 	std::cout << "输出矩阵 叠加  .. \n" << (m_transformation_matrix *  gnss) << std::endl;

	// m_transformation_matrix = m_transformation_matrix * gnss;
	return true;
}
