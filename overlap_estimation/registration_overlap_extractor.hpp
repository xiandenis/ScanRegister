#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <vector>
#include <cmath>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace Ddx
{

/// 基于初始位姿、半径邻域搜索与法向一致性，为配准提取 source/target 重叠子集。
/// guess 须为刚体变换 SE(3)：点 p' = R p + t，法向 n' = R n。
/// 算法参考 legacy ICP 重叠筛选：将一侧点变换到对侧坐标系后 radiusSearch，
/// 并要求邻域内法向同向比例不低于阈值。
class RegistrationOverlapExtractor
{
public:

    static constexpr int kParallelMinPoints = 1000;

    struct Params
    {
        float radius = 0.3f;
        int max_neighbors = 10;
        float normal_consistency_thresh = 0.2f;
    };

    struct Result
    {
        std::vector<int> source_indices;
        std::vector<int> target_indices;

        float overlap_ratio_src = 0.f;
        float overlap_ratio_tgt = 0.f;
    };

    RegistrationOverlapExtractor()
        : RegistrationOverlapExtractor(Params())
    {
    }

    explicit RegistrationOverlapExtractor(Params params)
        : params_(params)
    {
        params_.radius =
            std::max(params_.radius, 0.f);
        params_.max_neighbors =
            std::max(params_.max_neighbors, 1);
        params_.normal_consistency_thresh =
            std::max(
                0.f,
                std::min(
                    params_.normal_consistency_thresh,
                    1.f));
    }

    Params params() const { return params_; }

    void SetSourceTree(
        pcl::search::KdTree<pcl::PointXYZ>::ConstPtr tree)
    {
        external_source_tree_ = tree;
        owned_source_tree_.reset();
    }

    void SetTargetTree(
        pcl::search::KdTree<pcl::PointXYZ>::ConstPtr tree)
    {
        external_target_tree_ = tree;
        owned_target_tree_.reset();
    }

    void ClearSourceTree()
    {
        external_source_tree_.reset();
        rebuildOwnedSourceTree();
    }

    void ClearTargetTree()
    {
        external_target_tree_.reset();
        rebuildOwnedTargetTree();
    }

    void SetSourceCloud(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        pcl::PointCloud<pcl::Normal>::ConstPtr normals)
    {
        const bool cloud_unchanged =
            source_cloud_ &&
            cloud &&
            source_cloud_.get() == cloud.get();

        source_cloud_ = cloud;
        source_normals_ = normals;

        if(cloud_unchanged && (owned_source_tree_ || external_source_tree_))
            return;

        if(!external_source_tree_)
            rebuildOwnedSourceTree();
    }

    void SetTargetCloud(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        pcl::PointCloud<pcl::Normal>::ConstPtr normals)
    {
        const bool cloud_unchanged =
            target_cloud_ &&
            cloud &&
            target_cloud_.get() == cloud.get();

        target_cloud_ = cloud;
        target_normals_ = normals;

        if(cloud_unchanged && (owned_target_tree_ || external_target_tree_))
            return;

        if(!external_target_tree_)
            rebuildOwnedTargetTree();
    }

    /// @param source_to_target_guess 刚体位姿 SE(3)，将 source 变换到 target 坐标系。
    Result Extract(
        Eigen::Matrix4f const& source_to_target_guess) const
    {
        return Extract(
            toRigidPose(source_to_target_guess));
    }

    /// @param source_to_target_guess 刚体位姿 SE(3)（float）。
    Result Extract(
        Eigen::Isometry3f const& source_to_target_guess) const
    {
        Result result;

        if(!isReady())
            return result;

        const Eigen::Isometry3f target_to_source =
            source_to_target_guess.inverse();

        const int source_size =
            static_cast<int>(source_cloud_->size());
        const int target_size =
            static_cast<int>(target_cloud_->size());

        const bool sides_serial =
            source_size >= kParallelMinPoints ||
            target_size >= kParallelMinPoints;

        if(sides_serial)
        {
            extractSide(
                *source_cloud_,
                *source_normals_,
                *targetTree(),
                *target_normals_,
                source_to_target_guess,
                source_size,
                result.source_indices,
                result.overlap_ratio_src);

            extractSide(
                *target_cloud_,
                *target_normals_,
                *sourceTree(),
                *source_normals_,
                target_to_source,
                target_size,
                result.target_indices,
                result.overlap_ratio_tgt);
        }
        else
        {
#ifdef _OPENMP
#pragma omp parallel sections
            {
#pragma omp section
                extractSide(
                    *source_cloud_,
                    *source_normals_,
                    *targetTree(),
                    *target_normals_,
                    source_to_target_guess,
                    source_size,
                    result.source_indices,
                    result.overlap_ratio_src);

#pragma omp section
                extractSide(
                    *target_cloud_,
                    *target_normals_,
                    *sourceTree(),
                    *source_normals_,
                    target_to_source,
                    target_size,
                    result.target_indices,
                    result.overlap_ratio_tgt);
            }
#else
            extractSide(
                *source_cloud_,
                *source_normals_,
                *targetTree(),
                *target_normals_,
                source_to_target_guess,
                source_size,
                result.source_indices,
                result.overlap_ratio_src);

            extractSide(
                *target_cloud_,
                *target_normals_,
                *sourceTree(),
                *source_normals_,
                target_to_source,
                target_size,
                result.target_indices,
                result.overlap_ratio_tgt);
#endif
        }

        return result;
    }

    /// @param source_to_target_guess 刚体位姿 SE(3)（double），内部转为 float。
    Result Extract(
        Eigen::Matrix4d const& source_to_target_guess) const
    {
        return Extract(
            toRigidPose(
                source_to_target_guess.cast<float>()));
    }

    /// @param source_to_target_guess 刚体位姿 SE(3)（double），内部转为 float。
    Result Extract(
        Eigen::Isometry3d const& source_to_target_guess) const
    {
        Eigen::Isometry3f guess =
            Eigen::Isometry3f::Identity();

        guess.linear() =
            source_to_target_guess.linear()
            .cast<float>();

        guess.translation() =
            source_to_target_guess.translation()
            .cast<float>();

        return Extract(guess);
    }

    template<typename PointT>
    static typename pcl::PointCloud<PointT>::Ptr
    ExtractCloud(
        typename pcl::PointCloud<PointT>::ConstPtr const& cloud,
        std::vector<int> const& indices)
    {
        auto out =
            typename pcl::PointCloud<PointT>::Ptr(
                new pcl::PointCloud<PointT>());

        out->reserve(indices.size());

        for(int idx : indices)
        {
            if(idx < 0 ||
               idx >= static_cast<int>(cloud->size()))
                continue;

            out->push_back(
                cloud->points[
                    static_cast<size_t>(idx)]);
        }

        out->width = out->size();
        out->height = 1;
        out->is_dense = true;

        return out;
    }

private:

    static Eigen::Isometry3f toRigidPose(
        Eigen::Matrix4f const& transform)
    {
        Eigen::Isometry3f pose =
            Eigen::Isometry3f::Identity();

        pose.linear() =
            transform.block<3, 3>(0, 0);
        pose.translation() =
            transform.block<3, 1>(0, 3);

        return pose;
    }

    bool isReady() const
    {
        if(!source_cloud_ ||
           !source_normals_ ||
           !target_cloud_ ||
           !target_normals_ ||
           source_cloud_->empty() ||
           target_cloud_->empty() ||
           source_cloud_->size() !=
               source_normals_->size() ||
           target_cloud_->size() !=
               target_normals_->size())
        {
            return false;
        }

        if(!sourceTree() || !targetTree())
            return false;

        if(external_source_tree_ &&
           !treeMatchesCloud(
               external_source_tree_,
               source_cloud_))
        {
            return false;
        }

        if(external_target_tree_ &&
           !treeMatchesCloud(
               external_target_tree_,
               target_cloud_))
        {
            return false;
        }

        return true;
    }

    static bool treeMatchesCloud(
        pcl::search::KdTree<pcl::PointXYZ>::ConstPtr const& tree,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud)
    {
        if(!tree || !cloud)
            return false;

        const auto input = tree->getInputCloud();
        return input && input.get() == cloud.get();
    }

    pcl::search::KdTree<pcl::PointXYZ> const*
    sourceTree() const
    {
        if(external_source_tree_)
            return external_source_tree_.get();

        return owned_source_tree_.get();
    }

    pcl::search::KdTree<pcl::PointXYZ> const*
    targetTree() const
    {
        if(external_target_tree_)
            return external_target_tree_.get();

        return owned_target_tree_.get();
    }

    void rebuildOwnedSourceTree()
    {
        owned_source_tree_.reset();

        if(!source_cloud_ || source_cloud_->empty())
            return;

        owned_source_tree_ =
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(
                new pcl::search::KdTree<pcl::PointXYZ>());

        owned_source_tree_->setInputCloud(source_cloud_);
    }

    void rebuildOwnedTargetTree()
    {
        owned_target_tree_.reset();

        if(!target_cloud_ || target_cloud_->empty())
            return;

        owned_target_tree_ =
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(
                new pcl::search::KdTree<pcl::PointXYZ>());

        owned_target_tree_->setInputCloud(target_cloud_);
    }

    static bool passesNormalConsistency(
        Eigen::Vector3f const& transformed_normal,
        std::vector<int> const& neighbor_indices,
        pcl::PointCloud<pcl::Normal> const& neighbor_normals,
        float normal_consistency_thresh)
    {
        const int neighbor_count =
            static_cast<int>(neighbor_indices.size());

        if(neighbor_count <= 0)
            return false;

        if(transformed_normal.squaredNorm() <= 1e-9f)
            return false;

        const int min_hits =
            static_cast<int>(
                std::ceil(
                    normal_consistency_thresh
                    * static_cast<float>(neighbor_count)));

        int directional_consistency = 0;
        int processed = 0;

        for(int idx : neighbor_indices)
        {
            ++processed;

            const auto& neighbor =
                neighbor_normals.points[
                    static_cast<size_t>(idx)];

            if(!std::isfinite(neighbor.normal_x) ||
               !std::isfinite(neighbor.normal_y) ||
               !std::isfinite(neighbor.normal_z))
            {
                continue;
            }

            const float dot =
                transformed_normal.x() * neighbor.normal_x
                + transformed_normal.y() * neighbor.normal_y
                + transformed_normal.z() * neighbor.normal_z;

            if(dot > 0.f)
                ++directional_consistency;

            if(directional_consistency >= min_hits)
                return true;

            const int remaining =
                neighbor_count - processed;

            if(directional_consistency + remaining < min_hits)
                return false;
        }

        return directional_consistency >= min_hits;
    }

    void extractSide(
        pcl::PointCloud<pcl::PointXYZ> const& cloud,
        pcl::PointCloud<pcl::Normal> const& normals,
        pcl::search::KdTree<pcl::PointXYZ> const& other_tree,
        pcl::PointCloud<pcl::Normal> const& other_normals,
        Eigen::Isometry3f const& cloud_to_other,
        int cloud_size,
        std::vector<int>& out_indices,
        float& out_overlap_ratio) const
    {
        out_indices.clear();
        out_overlap_ratio = 0.f;

        if(cloud_size <= 0)
            return;

        const Eigen::Matrix3f& rotation =
            cloud_to_other.linear();
        const Eigen::Vector3f& translation =
            cloud_to_other.translation();

#ifdef _OPENMP
        const int num_threads = omp_get_max_threads();
#else
        const int num_threads = 1;
#endif

        const bool use_parallel =
            num_threads > 1 &&
            cloud_size >= kParallelMinPoints;

        std::vector<std::vector<int>>
            local_indices(
                use_parallel
                    ? static_cast<size_t>(num_threads)
                    : 1u);

        if(use_parallel)
        {
#pragma omp parallel
            {
#ifdef _OPENMP
                const int tid = omp_get_thread_num();
#else
                const int tid = 0;
#endif

                auto& local = local_indices[
                    static_cast<size_t>(tid)];

                local.reserve(
                    static_cast<size_t>(
                        cloud_size / num_threads + 1));

                std::vector<int> neighbor_indices;
                std::vector<float> neighbor_distances;
                neighbor_indices.reserve(16);
                neighbor_distances.reserve(16);

#pragma omp for schedule(static)
                for(int i = 0; i < cloud_size; ++i)
                {
                    collectOverlapIndex(
                        cloud,
                        normals,
                        other_tree,
                        other_normals,
                        rotation,
                        translation,
                        i,
                        local,
                        neighbor_indices,
                        neighbor_distances);
                }
            }
        }
        else
        {
            auto& local = local_indices.front();
            local.reserve(static_cast<size_t>(cloud_size));

            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_distances;
            neighbor_indices.reserve(16);
            neighbor_distances.reserve(16);

            for(int i = 0; i < cloud_size; ++i)
            {
                collectOverlapIndex(
                    cloud,
                    normals,
                    other_tree,
                    other_normals,
                    rotation,
                    translation,
                    i,
                    local,
                    neighbor_indices,
                    neighbor_distances);
            }
        }

        size_t total = 0;

        for(auto const& local : local_indices)
            total += local.size();

        out_indices.resize(total);

        size_t offset = 0;

        for(auto const& local : local_indices)
        {
            if(local.empty())
                continue;

            std::copy(
                local.begin(),
                local.end(),
                out_indices.begin()
                    + static_cast<std::ptrdiff_t>(offset));

            offset += local.size();
        }

        // schedule(static) 下各线程负责连续 i 段，按 tid 拼接后索引已升序，无需 sort。

        if(out_indices.empty())
            return;

        out_overlap_ratio =
            static_cast<float>(out_indices.size())
            / static_cast<float>(cloud_size);
    }

    void collectOverlapIndex(
        pcl::PointCloud<pcl::PointXYZ> const& cloud,
        pcl::PointCloud<pcl::Normal> const& normals,
        pcl::search::KdTree<pcl::PointXYZ> const& other_tree,
        pcl::PointCloud<pcl::Normal> const& other_normals,
        Eigen::Matrix3f const& rotation,
        Eigen::Vector3f const& translation,
        int i,
        std::vector<int>& local,
        std::vector<int>& neighbor_indices,
        std::vector<float>& neighbor_distances) const
    {
        const Eigen::Vector3f point(
            cloud.points[static_cast<size_t>(i)].x,
            cloud.points[static_cast<size_t>(i)].y,
            cloud.points[static_cast<size_t>(i)].z);

        if(!point.allFinite())
            return;

        const auto& normal_pt =
            normals.points[static_cast<size_t>(i)];

        Eigen::Vector3f source_normal(
            normal_pt.normal_x,
            normal_pt.normal_y,
            normal_pt.normal_z);

        if(!source_normal.allFinite())
            return;

        const Eigen::Vector3f transformed_point =
            rotation * point + translation;

        if(!transformed_point.allFinite())
            return;

        const Eigen::Vector3f transformed_normal =
            rotation * source_normal;

        pcl::PointXYZ query;
        query.x = transformed_point.x();
        query.y = transformed_point.y();
        query.z = transformed_point.z();

        neighbor_indices.clear();
        neighbor_distances.clear();

        if(other_tree.radiusSearch(
               query,
               params_.radius,
               neighbor_indices,
               neighbor_distances,
               params_.max_neighbors) < 1)
        {
            return;
        }

        if(!passesNormalConsistency(
               transformed_normal,
               neighbor_indices,
               other_normals,
               params_.normal_consistency_thresh))
        {
            return;
        }

        local.push_back(i);
    }

    Params params_;

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr source_cloud_;
    pcl::PointCloud<pcl::Normal>::ConstPtr source_normals_;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud_;
    pcl::PointCloud<pcl::Normal>::ConstPtr target_normals_;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr owned_source_tree_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr owned_target_tree_;
    pcl::search::KdTree<pcl::PointXYZ>::ConstPtr external_source_tree_;
    pcl::search::KdTree<pcl::PointXYZ>::ConstPtr external_target_tree_;
};

}  // namespace Ddx
