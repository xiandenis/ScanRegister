#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace Ddx{

class OverlapEstimator
{
public:

    static constexpr size_t kParallelMinPoints = 100000;

    struct Result
    {
        float overlap_ratio = 0.0f;

        std::vector<int> overlap_idx_src;
        std::vector<int> overlap_idx_tgt;
    };

    struct RegionResult
    {
        float overlap_ratio_src = 0.0f;
        float overlap_ratio_tgt = 0.0f;

        std::vector<int> region_idx_src;
        std::vector<int> region_idx_tgt;
    };

private:

    struct VoxelKey
    {
        int x;
        int y;
        int z;

        bool operator==(
            const VoxelKey& other) const
        {
            return x == other.x &&
                   y == other.y &&
                   z == other.z;
        }
    };

    struct VoxelOffset
    {
        int dx;
        int dy;
        int dz;
    };

    struct CachedPoint
    {
        float x;
        float y;
        float z;
    };

    struct VoxelHash
    {
        size_t operator()(
            const VoxelKey& k) const
        {
            return
                ((uint64_t)k.x * 73856093u) ^
                ((uint64_t)k.y * 19349669u) ^
                ((uint64_t)k.z * 83492791u);
        }
    };

    using VoxelMap = std::unordered_map<
        VoxelKey,
        std::vector<int>,
        VoxelHash>;

public:

    explicit OverlapEstimator(
        float voxel_size = 0.5f,
        float distance_thresh = 0.3f)
        : voxel_size_(std::max(voxel_size, 1e-6f))
        , distance_thresh_(std::max(distance_thresh, 0.f))
        , distance_thresh2_(
            distance_thresh_ * distance_thresh_)
        , inv_voxel_size_(1.f / voxel_size_)
        , neighbor_radius_(
            std::max(
                1,
                static_cast<int>(
                    std::ceil(
                        distance_thresh_ / voxel_size_))))
    {
        BuildNeighborOffsets();
    }

    template<typename PointT>
    void SetTargetCloud(
        const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
        Clear();

        if(!cloud || cloud->empty())
            return;

        target_size_ = cloud->size();
        target_points_.resize(target_size_);

#ifdef _OPENMP
        const int num_threads =
            omp_get_max_threads();
#else
        const int num_threads = 1;
#endif

        if(num_threads > 1 &&
           target_size_ >= kParallelMinPoints)
        {
            std::vector<VoxelMap>
                local_maps(
                    static_cast<size_t>(num_threads));

#pragma omp parallel
            {
#ifdef _OPENMP
                const int tid =
                    omp_get_thread_num();
#else
                const int tid = 0;
#endif

#pragma omp for schedule(static)
                for(int i = 0;
                    i < (int)target_size_;
                    ++i)
                {
                    const auto& pt =
                        cloud->points[i];

                    target_points_[
                        static_cast<size_t>(i)] = {
                        pt.x,
                        pt.y,
                        pt.z
                    };

                    auto key =
                        PointToVoxel(pt);

                    local_maps[
                        static_cast<size_t>(tid)][key]
                        .push_back(i);
                }
            }

            MergeLocalVoxelMaps(
                voxel_map_,
                local_maps);
            return;
        }

        voxel_map_.reserve(
            std::min(
                target_size_,
                static_cast<size_t>(1024)));

        for(size_t i = 0; i < target_size_; ++i)
        {
            const auto& pt = cloud->points[i];

            target_points_[i] = {
                pt.x,
                pt.y,
                pt.z
            };

            auto key =
                PointToVoxel(pt);

            voxel_map_[key].push_back(
                static_cast<int>(i));
        }
    }

    template<typename PointT>
    Result ComputeOverlap(
        const typename pcl::PointCloud<PointT>::ConstPtr& source,
        bool collect_target_indices = true) const
    {
        Result result;

        if(!source || source->empty())
            return result;

        if(target_points_.empty() || voxel_map_.empty())
            return result;

#ifdef _OPENMP
        int num_threads =
            omp_get_max_threads();
#else
        int num_threads = 1;
#endif

        std::vector<std::vector<int>>
            local_src(num_threads);

        std::vector<std::vector<int>>
            local_tgt(
                collect_target_indices
                    ? static_cast<size_t>(num_threads)
                    : 0);

#pragma omp parallel
        {
#ifdef _OPENMP
            int tid =
                omp_get_thread_num();
#else
            int tid = 0;
#endif

            const size_t per_thread =
                (source->size() + num_threads - 1)
                / static_cast<size_t>(num_threads);

            local_src[tid].reserve(per_thread);
            if(collect_target_indices)
                local_tgt[tid].reserve(per_thread);

#pragma omp for schedule(static)
            for(int i = 0;
                i < (int)source->size();
                ++i)
            {
                int idx_tgt = -1;

                if(!FindOverlapForSourcePoint(
                       source->points[i],
                       collect_target_indices
                           ? &idx_tgt
                           : nullptr))
                    continue;

                local_src[tid].push_back(i);
                if(collect_target_indices)
                    local_tgt[tid].push_back(idx_tgt);
            }
        }

        size_t overlap_num = 0;

        for(auto& v : local_src)
            overlap_num += v.size();

        result.overlap_idx_src.reserve(
            overlap_num);

        if(collect_target_indices)
            result.overlap_idx_tgt.reserve(
                overlap_num);

        for(size_t i=0;
            i<local_src.size();
            ++i)
        {
            result.overlap_idx_src.insert(
                result.overlap_idx_src.end(),
                local_src[i].begin(),
                local_src[i].end());

            if(collect_target_indices)
            {
                result.overlap_idx_tgt.insert(
                    result.overlap_idx_tgt.end(),
                    local_tgt[i].begin(),
                    local_tgt[i].end());
            }
        }

        result.overlap_ratio =
            static_cast<float>(overlap_num)
            /
            static_cast<float>(source->size());

        return result;
    }

    template<typename PointT>
    float ComputeOverlapRatio(
        const typename pcl::PointCloud<PointT>::ConstPtr& source) const
    {
        if(!source || source->empty())
            return 0.f;

        if(target_points_.empty() || voxel_map_.empty())
            return 0.f;

#ifdef _OPENMP
        int num_threads =
            omp_get_max_threads();
#else
        int num_threads = 1;
#endif

        std::vector<size_t>
            local_counts(
                static_cast<size_t>(num_threads),
                0);

#pragma omp parallel
        {
#ifdef _OPENMP
            int tid =
                omp_get_thread_num();
#else
            int tid = 0;
#endif

#pragma omp for schedule(static)
            for(int i = 0;
                i < (int)source->size();
                ++i)
            {
                if(FindOverlapForSourcePoint(
                       source->points[i]))
                {
                    ++local_counts[
                        static_cast<size_t>(tid)];
                }
            }
        }

        size_t overlap_num = 0;

        for(auto count : local_counts)
            overlap_num += count;

        return static_cast<float>(overlap_num)
            / static_cast<float>(source->size());
    }

    template<typename PointT>
    Result ComputeOverlap(
        const typename pcl::PointCloud<PointT>::ConstPtr& source,
        const typename pcl::PointCloud<PointT>::ConstPtr& target,
        bool collect_target_indices = true) const
    {
        if(!target || target->empty())
            return Result{};

        if(target->size() != target_size_)
            return Result{};

        return ComputeOverlap<PointT>(
            source,
            collect_target_indices);
    }

    template<typename PointT>
    float ComputeOverlapRatio(
        const typename pcl::PointCloud<PointT>::ConstPtr& source,
        const typename pcl::PointCloud<PointT>::ConstPtr& target) const
    {
        if(!target || target->empty())
            return 0.f;

        if(target->size() != target_size_)
            return 0.f;

        return ComputeOverlapRatio<PointT>(source);
    }

    template<typename PointT>
    RegionResult ComputeOverlapRegion(
        const typename pcl::PointCloud<PointT>::ConstPtr& source) const
    {
        RegionResult result;

        if(!source || source->empty())
            return result;

        if(target_points_.empty() || voxel_map_.empty())
            return result;

        std::vector<CachedPoint>
            source_points;

        VoxelMap source_voxel_map;

        BuildSourceIndex<PointT>(
            source,
            source_points,
            source_voxel_map);

#ifdef _OPENMP
        int num_threads =
            omp_get_max_threads();
#else
        int num_threads = 1;
#endif

        std::vector<std::vector<int>>
            local_src(num_threads);

        std::vector<std::vector<int>>
            local_tgt(num_threads);

#pragma omp parallel
        {
#ifdef _OPENMP
            int tid =
                omp_get_thread_num();
#else
            int tid = 0;
#endif

            const size_t src_per_thread =
                (source->size() + num_threads - 1)
                / static_cast<size_t>(num_threads);

            const size_t tgt_per_thread =
                (target_size_ + num_threads - 1)
                / static_cast<size_t>(num_threads);

            local_src[tid].reserve(src_per_thread);
            local_tgt[tid].reserve(tgt_per_thread);

#pragma omp for schedule(static)
            for(int i = 0;
                i < (int)source->size();
                ++i)
            {
                if(FindOverlapForSourcePoint(
                       source->points[i]))
                {
                    local_src[tid].push_back(i);
                }
            }

#pragma omp for schedule(static)
            for(int i = 0;
                i < (int)target_size_;
                ++i)
            {
                if(FindOverlapForTargetPoint(
                       target_points_[
                           static_cast<size_t>(i)],
                       source_points,
                       source_voxel_map))
                {
                    local_tgt[tid].push_back(i);
                }
            }
        }

        size_t region_src_num = 0;
        size_t region_tgt_num = 0;

        for(auto& v : local_src)
            region_src_num += v.size();

        for(auto& v : local_tgt)
            region_tgt_num += v.size();

        result.region_idx_src.reserve(
            region_src_num);

        result.region_idx_tgt.reserve(
            region_tgt_num);

        for(size_t i = 0;
            i < local_src.size();
            ++i)
        {
            result.region_idx_src.insert(
                result.region_idx_src.end(),
                local_src[i].begin(),
                local_src[i].end());

            result.region_idx_tgt.insert(
                result.region_idx_tgt.end(),
                local_tgt[i].begin(),
                local_tgt[i].end());
        }

        result.overlap_ratio_src =
            static_cast<float>(region_src_num)
            / static_cast<float>(source->size());

        result.overlap_ratio_tgt =
            static_cast<float>(region_tgt_num)
            / static_cast<float>(target_size_);

        return result;
    }

    template<typename PointT>
    RegionResult ComputeOverlapRegion(
        const typename pcl::PointCloud<PointT>::ConstPtr& source,
        const typename pcl::PointCloud<PointT>::ConstPtr& target) const
    {
        if(!target || target->empty())
            return RegionResult{};

        if(target->size() != target_size_)
            return RegionResult{};

        return ComputeOverlapRegion<PointT>(source);
    }

    template<typename PointT>
    static typename pcl::PointCloud<PointT>::Ptr
    ExtractCloud(
        const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
        const std::vector<int>& indices)
    {
        auto out =
            typename pcl::PointCloud<PointT>::Ptr(
                new pcl::PointCloud<PointT>());

        out->reserve(indices.size());

        for(auto idx : indices)
        {
            out->push_back(
                cloud->points[idx]);
        }

        out->width = out->size();
        out->height = 1;
        out->is_dense = true;

        return out;
    }

    /// 在体素邻域内收集 query 的最多 k 个最近 target 点（按距离平方，小顶堆维护）。
    /// 复用 SetTargetCloud 已建索引，无需另建 KdTree。返回实际找到的点数。
    template<typename PointT>
    int CollectKNearest(
        PointT const& query,
        int k,
        std::vector<int>& out_indices,
        std::vector<float>& out_dist2) const
    {
        out_indices.clear();
        out_dist2.clear();

        if(k <= 0 || target_points_.empty() || voxel_map_.empty())
            return 0;

        // max-heap by dist2：堆顶为当前最远（最差）候选
        std::vector<std::pair<float, int>> best;
        best.reserve(static_cast<size_t>(k));

        auto const center = PointToVoxel(query);

        for(auto const& off : neighbor_offsets_)
        {
            VoxelKey const key{
                center.x + off.dx,
                center.y + off.dy,
                center.z + off.dz};

            auto const iter = voxel_map_.find(key);
            if(iter == voxel_map_.end())
                continue;

            for(int idx_tgt : iter->second)
            {
                auto const& pt =
                    target_points_[static_cast<size_t>(idx_tgt)];

                float const ddx = query.x - pt.x;
                float const ddy = query.y - pt.y;
                float const ddz = query.z - pt.z;
                float const dist2 =
                    ddx * ddx + ddy * ddy + ddz * ddz;

                if(static_cast<int>(best.size()) < k)
                {
                    best.emplace_back(dist2, idx_tgt);
                    std::push_heap(
                        best.begin(),
                        best.end(),
                        [](auto const& a, auto const& b)
                        {
                            return a.first < b.first;
                        });
                }
                else if(dist2 < best.front().first)
                {
                    std::pop_heap(
                        best.begin(),
                        best.end(),
                        [](auto const& a, auto const& b)
                        {
                            return a.first < b.first;
                        });
                    best.back() = {dist2, idx_tgt};
                    std::push_heap(
                        best.begin(),
                        best.end(),
                        [](auto const& a, auto const& b)
                        {
                            return a.first < b.first;
                        });
                }
            }
        }

        std::sort(
            best.begin(),
            best.end(),
            [](auto const& a, auto const& b)
            {
                return a.first < b.first;
            });

        out_indices.reserve(best.size());
        out_dist2.reserve(best.size());
        for(auto const& e : best)
        {
            out_indices.push_back(e.second);
            out_dist2.push_back(e.first);
        }

        return static_cast<int>(best.size());
    }

    /// 在体素邻域内、距离阈值内收集候选，再取其中最近的最多 k 个。
    /// 候选上限避免共线退化，同时仍可早停，稠密场景远快于精确全邻域 KNN。
    template<typename PointT>
    int CollectKWithinDistance(
        PointT const& query,
        int k,
        std::vector<int>& out_indices,
        std::vector<float>& out_dist2) const
    {
        out_indices.clear();
        out_dist2.clear();

        if(k <= 0 || target_points_.empty() || voxel_map_.empty())
            return 0;

        int const candidate_limit =
            std::max(k * 8, 32);

        std::vector<std::pair<float, int>> candidates;
        candidates.reserve(static_cast<size_t>(candidate_limit));

        auto const center = PointToVoxel(query);

        for(auto const& off : neighbor_offsets_)
        {
            VoxelKey const key{
                center.x + off.dx,
                center.y + off.dy,
                center.z + off.dz};

            auto const iter = voxel_map_.find(key);
            if(iter == voxel_map_.end())
                continue;

            for(int idx_tgt : iter->second)
            {
                auto const& pt =
                    target_points_[static_cast<size_t>(idx_tgt)];

                float const ddx = query.x - pt.x;
                float const ddy = query.y - pt.y;
                float const ddz = query.z - pt.z;
                float const dist2 =
                    ddx * ddx + ddy * ddy + ddz * ddz;

                if(dist2 >= distance_thresh2_)
                    continue;

                candidates.emplace_back(dist2, idx_tgt);
                if(static_cast<int>(candidates.size()) >= candidate_limit)
                    goto finish_candidates;
            }
        }

    finish_candidates:
        if(candidates.empty())
            return 0;

        int const take =
            std::min(k, static_cast<int>(candidates.size()));
        std::partial_sort(
            candidates.begin(),
            candidates.begin() + take,
            candidates.end(),
            [](auto const& a, auto const& b)
            {
                return a.first < b.first;
            });

        out_indices.reserve(static_cast<size_t>(take));
        out_dist2.reserve(static_cast<size_t>(take));
        for(int i = 0; i < take; ++i)
        {
            out_indices.push_back(candidates[static_cast<size_t>(i)].second);
            out_dist2.push_back(candidates[static_cast<size_t>(i)].first);
        }

        return take;
    }

private:

    template<typename PointT>
    bool FindOverlapForSourcePoint(
        PointT const& ps,
        int* out_idx_tgt = nullptr) const
    {
        auto center =
            PointToVoxel(ps);

        for(const auto& off :
            neighbor_offsets_)
        {
            VoxelKey key{
                center.x + off.dx,
                center.y + off.dy,
                center.z + off.dz
            };

            auto iter =
                voxel_map_.find(key);

            if(iter ==
               voxel_map_.end())
                continue;

            for(auto idx_tgt :
                iter->second)
            {
                const auto& pt =
                    target_points_[
                        static_cast<size_t>(
                            idx_tgt)];

                float ddx =
                    ps.x - pt.x;

                float ddy =
                    ps.y - pt.y;

                float ddz =
                    ps.z - pt.z;

                float dist2 =
                    ddx*ddx +
                    ddy*ddy +
                    ddz*ddz;

                if(dist2 <
                   distance_thresh2_)
                {
                    if(out_idx_tgt)
                        *out_idx_tgt = idx_tgt;

                    return true;
                }
            }
        }

        return false;
    }

    bool FindOverlapForTargetPoint(
        CachedPoint const& pt_tgt,
        std::vector<CachedPoint> const&
            source_points,
        VoxelMap const& source_voxel_map) const
    {
        VoxelKey center{
            static_cast<int>(
                std::floor(
                    pt_tgt.x * inv_voxel_size_)),
            static_cast<int>(
                std::floor(
                    pt_tgt.y * inv_voxel_size_)),
            static_cast<int>(
                std::floor(
                    pt_tgt.z * inv_voxel_size_))
        };

        for(const auto& off :
            neighbor_offsets_)
        {
            VoxelKey key{
                center.x + off.dx,
                center.y + off.dy,
                center.z + off.dz
            };

            auto iter =
                source_voxel_map.find(key);

            if(iter ==
               source_voxel_map.end())
                continue;

            for(auto idx_src :
                iter->second)
            {
                const auto& ps =
                    source_points[
                        static_cast<size_t>(
                            idx_src)];

                float ddx =
                    ps.x - pt_tgt.x;

                float ddy =
                    ps.y - pt_tgt.y;

                float ddz =
                    ps.z - pt_tgt.z;

                float dist2 =
                    ddx*ddx +
                    ddy*ddy +
                    ddz*ddz;

                if(dist2 <
                   distance_thresh2_)
                    return true;
            }
        }

        return false;
    }

    template<typename PointT>
    void BuildSourceIndex(
        const typename pcl::PointCloud<PointT>::ConstPtr& source,
        std::vector<CachedPoint>& source_points,
        VoxelMap& source_voxel_map) const
    {
        const size_t source_size = source->size();
        source_points.resize(source_size);
        source_voxel_map.clear();

#ifdef _OPENMP
        const int num_threads =
            omp_get_max_threads();
#else
        const int num_threads = 1;
#endif

        if(num_threads > 1 &&
           source_size >= kParallelMinPoints)
        {
            std::vector<VoxelMap>
                local_maps(
                    static_cast<size_t>(num_threads));

#pragma omp parallel
            {
#ifdef _OPENMP
                const int tid =
                    omp_get_thread_num();
#else
                const int tid = 0;
#endif

#pragma omp for schedule(static)
                for(int i = 0;
                    i < (int)source_size;
                    ++i)
                {
                    const auto& pt =
                        source->points[
                            static_cast<size_t>(i)];

                    source_points[
                        static_cast<size_t>(i)] = {
                        pt.x,
                        pt.y,
                        pt.z
                    };

                    auto key =
                        PointToVoxel(pt);

                    local_maps[
                        static_cast<size_t>(tid)][key]
                        .push_back(i);
                }
            }

            MergeLocalVoxelMaps(
                source_voxel_map,
                local_maps);
            return;
        }

        source_voxel_map.reserve(
            std::min(
                source_size,
                static_cast<size_t>(1024)));

        for(size_t i = 0;
            i < source_size;
            ++i)
        {
            const auto& pt =
                source->points[i];

            source_points[i] = {
                pt.x,
                pt.y,
                pt.z
            };

            auto key =
                PointToVoxel(pt);

            source_voxel_map[key].push_back(
                static_cast<int>(i));
        }
    }

    template<typename PointT>
    VoxelKey PointToVoxel(
        const PointT& pt) const
    {
        return {
            static_cast<int>(
                std::floor(
                    pt.x * inv_voxel_size_)),
            static_cast<int>(
                std::floor(
                    pt.y * inv_voxel_size_)),
            static_cast<int>(
                std::floor(
                    pt.z * inv_voxel_size_))
        };
    }

    static int ChebyshevDistance(
        VoxelOffset const& off)
    {
        return std::max({
            std::abs(off.dx),
            std::abs(off.dy),
            std::abs(off.dz)});
    }

    void BuildNeighborOffsets()
    {
        neighbor_offsets_.clear();

        const int r = neighbor_radius_;
        const size_t count =
            static_cast<size_t>(2 * r + 1)
            * static_cast<size_t>(2 * r + 1)
            * static_cast<size_t>(2 * r + 1);

        neighbor_offsets_.reserve(count);

        for(int dx = -r; dx <= r; ++dx)
        {
            for(int dy = -r; dy <= r; ++dy)
            {
                for(int dz = -r; dz <= r; ++dz)
                {
                    neighbor_offsets_.push_back(
                        {dx, dy, dz});
                }
            }
        }

        std::sort(
            neighbor_offsets_.begin(),
            neighbor_offsets_.end(),
            [](VoxelOffset const& a, VoxelOffset const& b)
            {
                const int ca =
                    ChebyshevDistance(a);
                const int cb =
                    ChebyshevDistance(b);

                if(ca != cb)
                    return ca < cb;

                if(a.dx != b.dx)
                    return a.dx < b.dx;

                if(a.dy != b.dy)
                    return a.dy < b.dy;

                return a.dz < b.dz;
            });
    }

    void Clear()
    {
        voxel_map_.clear();
        target_points_.clear();
        target_size_ = 0;
    }

    static void MergeLocalVoxelMaps(
        VoxelMap& dest,
        std::vector<VoxelMap> const& local_maps)
    {
        size_t bucket_hint = 0;

        for(auto const& local : local_maps)
            bucket_hint += local.size();

        dest.reserve(bucket_hint);

        for(auto const& local : local_maps)
        {
            for(auto const& entry : local)
            {
                auto& indices =
                    dest[entry.first];

                indices.insert(
                    indices.end(),
                    entry.second.begin(),
                    entry.second.end());
            }
        }
    }

private:

    float voxel_size_;
    float inv_voxel_size_;
    float distance_thresh_;
    float distance_thresh2_;

    int neighbor_radius_;

    size_t target_size_ = 0;

    std::vector<CachedPoint> target_points_;

    std::vector<VoxelOffset> neighbor_offsets_;

    VoxelMap voxel_map_;
};
}
