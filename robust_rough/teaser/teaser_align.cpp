#include "teaser_align.hpp"
#include "registration.h"

#include <stdexcept>

using namespace teaser;

namespace teaser
{
	inline void getParams(const double noise_bound, TeaserAlign::Method reg_type,
		teaser::RobustRegistrationSolver::Params& params) {
		params.noise_bound = noise_bound;
		params.cbar2 = 1;
		params.estimate_scaling = false;
		params.rotation_max_iterations = 100;
		params.rotation_gnc_factor = 1.4;
		if (reg_type == TeaserAlign::Method::FQUATRO)
		{
			params.rotation_estimation_algorithm =
				teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::QUATRO;
			params.inlier_selection_mode = teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;
		}
		else if (reg_type == TeaserAlign::Method::FGNC_TLS)
		{
			params.rotation_estimation_algorithm =
				teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
			params.inlier_selection_mode = teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_EXACT;
		}
		else
		{
			throw std::invalid_argument("Not implemented!");
		}
		// params.rotation_cost_threshold = 0.0002;
		params.rotation_cost_threshold = 0.005;
	}
}


TeaserAlign::TeaserAlign() : m_transformation_matrix(Eigen::Matrix4d::Identity()), m_method(Method::FGNC_TLS) {};

void TeaserAlign::setSourceCloud(PointCloud const& sourceCloud)
{
	m_source_cloud = Eigen::Map<const PointCloud>(sourceCloud.data(), sourceCloud.rows(), sourceCloud.cols());
}

void TeaserAlign::setTargetCloud(PointCloud const& targeCloud)
{
	m_target_cloud = Eigen::Map<const PointCloud>(targeCloud.data(), targeCloud.rows(), targeCloud.cols());
}

void TeaserAlign::setDSampleRadius(double dSampleRadius)
{
	m_dSampleRadius = dSampleRadius;
}

void TeaserAlign::setMethod(Method type)
{
	m_method = type;
}


bool TeaserAlign::align()
{
	if (m_source_cloud.cols() <= 3 || m_target_cloud.cols() <= 3)
		return false;

	m_transformation_matrix = Eigen::Matrix4d::Identity();

	// Run TEASER++ registration
	// Prepare solver parameters
	teaser::RobustRegistrationSolver::Params params;
	getParams(m_dSampleRadius / 2.0, m_method, params);


	// Solve with TEASER++
	teaser::RobustRegistrationSolver solver(params);
	solver.solve(m_source_cloud, m_target_cloud);


	auto solution = solver.getSolution();
	if (!solution.valid)
		return false;

	m_transformation_matrix = Eigen::Matrix4d::Identity();
	m_transformation_matrix.block(0, 0, 3, 3) = solution.rotation;
	m_transformation_matrix.block(0, 3, 3, 1) = solution.translation;

	return true;
}

Eigen::Matrix4d
TeaserAlign::getFinalTransformationMatrix()
{
	return m_transformation_matrix;
}