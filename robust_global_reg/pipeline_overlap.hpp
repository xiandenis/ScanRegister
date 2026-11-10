#pragma once

#include <overlap_estimation/overlap_estimation.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <cstdint>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace Ddx
{

inline void transformPointCloud(
    pcl::PointCloud<pcl::PointXYZ> const& cloud,
    Eigen::Matrix4d const& transform,
    pcl::PointCloud<pcl::PointXYZ>& out)
{
    out.resize(cloud.size());
    out.width = static_cast<std::uint32_t>(cloud.size());
    out.height = 1;
    out.is_dense = true;

    if(cloud.empty())
        return;

    const Eigen::Matrix4f T =
        transform.cast<float>();

#ifdef _OPENMP
    const int num_threads =
        omp_get_max_threads();
#else
    const int num_threads = 1;
#endif

    if(num_threads > 1 &&
       cloud.size() >= OverlapEstimator::kParallelMinPoints)
    {
#pragma omp parallel for schedule(static)
        for(int i = 0;
            i < static_cast<int>(cloud.size());
            ++i)
        {
            const auto& p = cloud.points[
                static_cast<size_t>(i)];

            const Eigen::Vector4f v(
                p.x,
                p.y,
                p.z,
                1.f);

            const Eigen::Vector4f q = T * v;

            out.points[static_cast<size_t>(i)] =
                pcl::PointXYZ(
                    q.x(),
                    q.y(),
                    q.z());
        }
        return;
    }

    for(size_t i = 0; i < cloud.size(); ++i)
    {
        const auto& p = cloud.points[i];

        const Eigen::Vector4f v(
            p.x,
            p.y,
            p.z,
            1.f);

        const Eigen::Vector4f q = T * v;

        out.points[i] =
            pcl::PointXYZ(
                q.x(),
                q.y(),
                q.z());
    }
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
    pcl::PointCloud<pcl::PointXYZ> const& cloud,
    Eigen::Matrix4d const& transform)
{
    auto out =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>());

    transformPointCloud(
        cloud,
        transform,
        *out);

    return out;
}

class PipelineOverlapContext
{
public:

    PipelineOverlapContext(
        float voxel_size,
        float distance_thresh)
        : estimator_(
            voxel_size,
            distance_thresh)
        , transform_buffer_(
            new pcl::PointCloud<pcl::PointXYZ>())
    {
    }

    void setTarget(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& target)
    {
        if(target == cached_target_)
            return;

        cached_target_ = target;

        if(target && !target->empty())
            estimator_.SetTargetCloud<pcl::PointXYZ>(target);
    }

    void clearTarget()
    {
        cached_target_.reset();
        estimator_.SetTargetCloud<pcl::PointXYZ>(nullptr);
    }

    float computeOverlapRatio(
        pcl::PointCloud<pcl::PointXYZ> const& source,
        Eigen::Matrix4d const& source_to_target)
    {
        if(source.empty() ||
           !cached_target_ ||
           cached_target_->empty())
            return 0.f;

        transformPointCloud(
            source,
            source_to_target,
            *transform_buffer_);

        return estimator_.ComputeOverlapRatio<pcl::PointXYZ>(
            transform_buffer_);
    }

    OverlapEstimator::Result computeOverlap(
        pcl::PointCloud<pcl::PointXYZ> const& source,
        Eigen::Matrix4d const& source_to_target)
    {
        OverlapEstimator::Result result;

        if(source.empty() ||
           !cached_target_ ||
           cached_target_->empty())
            return result;

        transformPointCloud(
            source,
            source_to_target,
            *transform_buffer_);

        return estimator_.ComputeOverlap<pcl::PointXYZ>(
            transform_buffer_);
    }

    OverlapEstimator::RegionResult computeOverlapRegion(
        pcl::PointCloud<pcl::PointXYZ> const& source,
        Eigen::Matrix4d const& source_to_target)
    {
        OverlapEstimator::RegionResult result;

        if(source.empty() ||
           !cached_target_ ||
           cached_target_->empty())
            return result;

        transformPointCloud(
            source,
            source_to_target,
            *transform_buffer_);

        return estimator_.ComputeOverlapRegion<pcl::PointXYZ>(
            transform_buffer_);
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr
    transformedSource() const
    {
        return transform_buffer_;
    }

    OverlapEstimator const& estimator() const
    {
        return estimator_;
    }

private:

    OverlapEstimator estimator_;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cached_target_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_buffer_;
};

inline float computePipelineOverlapRatio(
    pcl::PointCloud<pcl::PointXYZ> const& source,
    pcl::PointCloud<pcl::PointXYZ> const& target,
    Eigen::Matrix4d const& source_to_target,
    float voxel_size,
    float distance_thresh)
{
    if(source.empty() || target.empty())
        return 0.f;

    PipelineOverlapContext ctx(
        voxel_size,
        distance_thresh);

    ctx.setTarget(target.makeShared());

    return ctx.computeOverlapRatio(
        source,
        source_to_target);
}

inline OverlapEstimator::Result computePipelineOverlap(
    pcl::PointCloud<pcl::PointXYZ> const& source,
    pcl::PointCloud<pcl::PointXYZ> const& target,
    Eigen::Matrix4d const& source_to_target,
    float voxel_size,
    float distance_thresh)
{
    OverlapEstimator::Result result;

    if(source.empty() || target.empty())
        return result;

    PipelineOverlapContext ctx(
        voxel_size,
        distance_thresh);

    ctx.setTarget(target.makeShared());

    return ctx.computeOverlap(
        source,
        source_to_target);
}

inline OverlapEstimator::RegionResult computePipelineOverlapRegion(
    pcl::PointCloud<pcl::PointXYZ> const& source,
    pcl::PointCloud<pcl::PointXYZ> const& target,
    Eigen::Matrix4d const& source_to_target,
    float voxel_size,
    float distance_thresh)
{
    OverlapEstimator::RegionResult result;

    if(source.empty() || target.empty())
        return result;

    PipelineOverlapContext ctx(
        voxel_size,
        distance_thresh);

    ctx.setTarget(target.makeShared());

    return ctx.computeOverlapRegion(
        source,
        source_to_target);
}

}  // namespace Ddx
