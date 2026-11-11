#include "explicit_surface_report.hpp"
#include <Eigen/Dense>
#ifdef _WIN32
#include <windows.h>  // InterlockedIncrement
#endif

namespace Ddx
{
	bool
	ExplicitSurfaceReport::compute()
	{
		if (checkSource() == false)
			return false;
		if (checkTarget() == false)
			return false;
		std::vector<int>  vaild_indices(m_src_points->size(), 1);
		for (int i = 0; i < m_src_points->size(); i++)
		{
			vaild_indices[i] = i;
		}

		return compute(vaild_indices);
	}

	bool
	ExplicitSurfaceReport::compute(std::vector<int> const& vaild_indices)
	{
		if (vaild_indices.size() < 10)
			return false;
		if (checkSource() == false)
			return false;
		if (checkTarget() == false)
			return false;

		int src_num = m_src_points->size();
		m_surface_points.resize(src_num, Eigen::Vector3f::Zero());
		m_surface_points_distance.resize(src_num, -1.f);


		std::vector<int> indices(m_search_neighbors);
		std::vector<float> distances(m_search_neighbors);
		int end_offset = m_search_neighbors - 1;
		float max_distance = m_search_max_radius * m_search_max_radius;

		float rms = 0.f;
		size_t function_number = 0;

#ifdef _OPENMP
#pragma omp parallel for schedule(static) reduction(+: rms) private(indices, distances)
#endif
		for (int i = 0; i < vaild_indices.size(); i++)
		{
			int id = vaild_indices[i];
			Eigen::Vector4f point = m_transformation * m_src_points->points[id].getVector4fMap();
			PointType r11x;
			r11x.x = point[0];
			r11x.y = point[1];
			r11x.z = point[2];

			Eigen::Vector3f sp = point.head<3>();

			m_tgt_tree->nearestKSearch(r11x, m_search_neighbors, indices, distances);
			if (distances[end_offset] > max_distance) continue;
			Eigen::Vector3f normal;
			Eigen::Vector3f center;
			if (_estiPlane(normal, center, m_tgt_points, indices, m_plane_distance)) {

				float d = (sp - center).dot(normal);
				float d2 = d * d;
				rms += d2;
				m_surface_points_distance[id] = d2;
				m_surface_points[id] = sp - d * normal;
#ifdef _WIN32
				InterlockedIncrement(&function_number);
#else
				__sync_fetch_and_add(&function_number, 1);
#endif
			}

		}

		m_inlieNumber = function_number;
		if (function_number < 10)
			return false;

		m_finalRms = rms / float(function_number);
		return true;
	}

	std::vector<Eigen::Vector3f>
	ExplicitSurfaceReport::getPointPairSource(float threshold) const
	{
		float distance_threshold = threshold * threshold;
		std::vector<Eigen::Vector3f> result;
		result.reserve(m_inlieNumber);
		for (int i = 0; i < m_src_points->size(); i++)
		{
			if (m_surface_points_distance[i] > (-0.0001f) && m_surface_points_distance[i] < distance_threshold)
				result.push_back(m_src_points->points[i].getVector3fMap());
		}

		return result;
	}

	std::vector<Eigen::Vector3f>
	ExplicitSurfaceReport::getPointPairTarget(float threshold) const
	{
		float distance_threshold = threshold * threshold;
		std::vector<Eigen::Vector3f> result;
		result.reserve(m_inlieNumber);
		for (int i = 0; i < m_src_points->size(); i++)
		{
			if (m_surface_points_distance[i] > (-0.0001f) && m_surface_points_distance[i] < distance_threshold)
				result.push_back(m_surface_points[i]);
		}

		return result;
	}

	bool 
	ExplicitSurfaceReport::checkSource()
	{
		if (m_src_points == nullptr || m_src_points->size() < 10)
			return false;

		return true;
	}

	bool 
	ExplicitSurfaceReport::checkTarget()
	{
        if (m_tgt_points == nullptr || m_tgt_points->size() < 10) {
            return false;
        }

        if (m_tgt_tree == nullptr)
        {
            m_tgt_tree.reset(new KdTree());
            m_tgt_tree->setInputCloud(m_tgt_points);
        }
        else if (m_tgt_tree->getInputCloud() != m_tgt_points) {
            m_tgt_tree->setInputCloud(m_tgt_points);
        }
        else {}

        return true;
	}
    
	bool
	ExplicitSurfaceReport::_estiPlane(Eigen::Vector3f& normal,
			Eigen::Vector3f& center,
			PointCloud::Ptr src_points,
			std::vector<int> const& indices,
			const double& threshold)
	{
		Eigen::Matrix<double, 3, 3> covariance;
		Eigen::Matrix<double, 9, 1> cumulants;
		cumulants.setZero();

		center = Eigen::Vector3f(0, 0, 0);
		for (size_t i = 0; i < indices.size(); i++)
		{
			const Eigen::Vector3d point = src_points->points[indices[i]].getVector3fMap().cast<double>();
			cumulants(0) += point(0);
			cumulants(1) += point(1);
			cumulants(2) += point(2);
			cumulants(3) += point(0) * point(0);
			cumulants(4) += point(0) * point(1);
			cumulants(5) += point(0) * point(2);
			cumulants(6) += point(1) * point(1);
			cumulants(7) += point(1) * point(2);
			cumulants(8) += point(2) * point(2);

			center += point.cast<float>();
		}

		cumulants /= (double)indices.size();
		covariance(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
		covariance(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
		covariance(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
		covariance(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
		covariance(1, 0) = covariance(0, 1);
		covariance(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
		covariance(2, 0) = covariance(0, 2);
		covariance(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
		covariance(2, 1) = covariance(1, 2);
		center /= (float)indices.size();

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d > solver;
		solver.compute(covariance, Eigen::ComputeEigenvectors);
		Eigen::Vector3d normal_1 = solver.eigenvectors().col(0);
		normal = normal_1.normalized().cast<float>();

		for (int i = 0; i < indices.size(); ++i)
		{
			Eigen::Vector3f pt = src_points->points[indices[i]].getVector3fMap() - center;

			if (std::abs(pt.dot(normal)) > threshold) return false;
		}

		return true;
	}

}