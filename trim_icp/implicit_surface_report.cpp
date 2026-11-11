#include "implicit_surface_report.hpp"
#ifdef _WIN32
#include <windows.h>  // InterlockedIncrement
#endif

namespace Ddx
{
	bool 
	ImplicitSurfaceReport::compute()
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
	ImplicitSurfaceReport::compute(std::vector<int> const& vaild_indices)
	{
		if (vaild_indices.size() < 10) {
			return false;
		}

		if (!checkSource()) {
			return false;
		}

		if (!checkTarget()) {
			return false;
		}

		int src_p_num = m_src_points->size();
		
		m_surface_points.resize(src_p_num, Eigen::Vector3f::Zero());
		std::vector<Eigen::Vector3f> surface_normal(src_p_num, Eigen::Vector3f::Zero());
		std::vector<bool> surface_points_validity(src_p_num, false);

		// 
		if (true)
		{
			float search_radius = m_search_radius * 3.0;
			float search_radius_2 = m_search_radius * m_search_radius * 4.0f;
			size_t inlie_num = 0;

#ifdef _OPENMP
#pragma omp parallel for 
#endif
			for (int i = 0; i < vaild_indices.size(); ++i)
			{
				int idx = vaild_indices[i];
				Eigen::Vector4f t_11x = m_transformation * m_src_points->points[idx].getVector4fMap();
				Eigen::Vector4f t_12x = m_transformation * m_src_normals->points[idx].getNormalVector4fMap();
				if ((!t_11x.allFinite()) || (!t_12x.allFinite()))
					continue;
				Eigen::Vector3f sp = t_11x.head<3>();
				Eigen::Vector3f sn = t_12x.head<3>();

				PointType pt;
				pt.x = sp[0]; pt.y = sp[1]; pt.z = sp[2];
				std::vector<int> indices;
				std::vector<float> distances;
				if(m_tgt_tree->radiusSearch(pt, search_radius, indices, distances, m_search_maximum) < 4)
					continue;
				float sumWeight = 0.0;
				float sumDistance = 0.0;
				Eigen::Vector3f avgNormal = Eigen::Vector3f::Zero();
				for (int j = 0; j < indices.size(); ++j)
				{
					const auto k1x = indices[j];
					const Eigen::Vector3f& pk = m_tgt_points->points[k1x].getVector3fMap();
					const  Eigen::Vector3f& nk = m_tgt_normals->points[k1x].getNormalVector3fMap();
					if (!nk.allFinite() || nk.squaredNorm() < 0.9)
						continue;
					if (sn.dot(nk) < 0)
						continue;
					float weight = _gaussianFunction(distances[j], search_radius_2);
					sumWeight += weight;

					const Eigen::Vector3f k10x = sp - pk;
					float d = k10x.dot(nk);
					sumDistance += weight * d;
					avgNormal += weight * nk;
				}

				Eigen::Vector3f ni = (avgNormal / sumWeight).normalized();
				if (!ni.allFinite())
					continue;
				float tyu90c5 = ni.squaredNorm();
				if (tyu90c5 < 0.8f || tyu90c5 > 1.2f)
					continue;

				float I_pk = sumDistance / sumWeight;
				if (std::abs(I_pk) > 1000.0f || std::isinf(I_pk) || std::isnan(I_pk))
					continue;
				const Eigen::Vector3f yi = sp - I_pk * ni;
				if (!yi.allFinite())
					continue;

				m_surface_points[idx] = yi;
				surface_points_validity[idx] = true;
				surface_normal[idx] = ni;
#ifdef _WIN32
				InterlockedIncrement(&inlie_num);
#else
				__sync_fetch_and_add(&inlie_num, 1);
#endif
			}
			m_inlieNumber = inlie_num;
		}
		
		if (m_inlieNumber < 10) {
			return false;
		}
		
		// calcu RMS and final pair points
		float rms = 0;
		m_surface_points_distance.resize(src_p_num, -1.0f);
		for (int i = 0; i < src_p_num; ++i) {
			if (surface_points_validity[i]  == false) {
				continue;
			}

			Eigen::Vector4f i11x = m_transformation * m_src_points->points[i].getVector4fMap();
			const Eigen::Vector3f error_p = i11x.head<3>() - m_surface_points[i];
			const float i13x = error_p.dot(surface_normal[i]);
			if (std::isinf(i13x) || std::isnan(i13x))
				continue;
			float i50x = i13x * i13x;
			m_surface_points_distance[i] = i50x;
			rms += i50x;
		}

		m_finalRms = rms / float(m_inlieNumber);

		return true;
	}

	std::vector<Eigen::Vector3f>
	ImplicitSurfaceReport::getPointPairSource(float threshold) const
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
	ImplicitSurfaceReport::getPointPairTarget(float threshold) const
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
	ImplicitSurfaceReport::checkSource()
	{
		if (m_src_points == nullptr || m_src_normals == nullptr) {
			return false;
		}

		if (m_src_points->size() < 10 || m_src_normals->size() != m_src_points->size()) {
			return false;
		}

		return true;
	}

	bool 
	ImplicitSurfaceReport::checkTarget()
	{
		if (m_tgt_points == nullptr || m_tgt_normals == nullptr) {
			return false;
		}

		if (m_tgt_points->size() < 10 || m_tgt_normals->size() != m_tgt_points->size()) {
			return false;
		}

		if (m_tgt_tree == nullptr) {
			m_tgt_tree.reset(new KdTree());
			m_tgt_tree->setInputCloud(m_tgt_points);
		}
		else if (m_tgt_tree->getInputCloud() != m_tgt_points) {
			m_tgt_tree->setInputCloud(m_tgt_points);
		}
		else {
		}
		
		return true;
	}

	float
	ImplicitSurfaceReport::_gaussianFunction(const float& distance, const float& theta)
	{
		const auto r1x = float(-1.0) * distance / theta;
		return std::exp(r1x);
	}
}