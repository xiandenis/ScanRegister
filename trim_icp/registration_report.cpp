#include "registration_report.hpp"


namespace Ddx
{
    bool 
    RegistrationReport::compute()
    {
        if (checkSoucre() == false) 
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
    RegistrationReport::compute(std::vector<int> const& vaild_indices)
    {
        if (vaild_indices.size() < 10)
            return false;
        if (checkSoucre() == false)
            return false;
        if (checkTarget() == false)
            return false;

        std::vector<int> indices(1);
        std::vector<float> distace(1);

        int function_number = 0;
        float rms = 0;

        m_tgt_indices.resize(m_src_points->size(), -1);
        m_distances.resize(m_src_points->size(), -1);


#ifdef _OPENMP
#pragma omp parallel for schedule(static) reduction(+: function_number, rms)
#endif
        for (int i = 0; i < vaild_indices.size(); i++)
        {
            int id = vaild_indices[i];
            Eigen::Vector4f point = m_transformation * m_src_points->points[id].getVector4fMap();
            PointType r11x;
            r11x.x = point[0];
            r11x.y = point[1];
            r11x.z = point[2];
            m_tgt_tree->nearestKSearch(r11x, 1, indices, distace);

            if (distace[0] < m_max_search_radius)
                continue;
            m_tgt_indices[id] = indices[0];
            m_distances[id] = distace[0];
            function_number++;
            rms += distace[0];
        }

        if (function_number < 10)
            return false;

        m_finalRms = rms / float(function_number);
        m_inlieNumber = function_number;
        return true;
    }

    std::vector<Eigen::Vector3f>
    RegistrationReport::getPointPairSource(float threshold) const
    {
        float distance_threshold = threshold * threshold;
        std::vector<Eigen::Vector3f> result;
        result.reserve(m_inlieNumber);
        for (int i = 0; i < m_src_points->size(); i++)
        {
            if (m_tgt_indices[i] > 0 && m_distances[i] < distance_threshold)
                result.push_back(m_src_points->points[i].getVector3fMap());
        }

        return result;
    }

    std::vector<Eigen::Vector3f>
    RegistrationReport::getPointPairTarget(float threshold) const
    {
        float distance_threshold = threshold * threshold;
        std::vector<Eigen::Vector3f> result;
        result.reserve(m_inlieNumber);
        for (int i = 0; i < m_src_points->size(); i++)
        {
            if (m_tgt_indices[i] > 0 && m_distances[i] < distance_threshold)
                result.push_back(m_tgt_points->points[m_tgt_indices[i]].getVector3fMap());
        }

        return result;
    }
    
    bool 
    RegistrationReport::checkSoucre()
    {
        if (m_src_points == nullptr || m_src_points->size() < 10) 
            return false;

        return true;
    }

    bool 
    RegistrationReport::checkTarget()
    {
        if (m_tgt_points == nullptr || m_tgt_points->size() < 10) {
            return false;
        }

        if (m_tgt_tree == nullptr)
        {
            m_tgt_tree.reset(new kdTree());
            m_tgt_tree->setInputCloud(m_tgt_points);
        }
        else if (m_tgt_tree->getInputCloud() != m_tgt_points) {
            m_tgt_tree->setInputCloud(m_tgt_points);
        }
        else {}

        return true;
    }
}
