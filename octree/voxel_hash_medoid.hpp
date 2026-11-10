#pragma once

#include <pcl/point_cloud.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include <tbb/parallel_sort.h>

namespace Ddx
{
	namespace octree
	{
		namespace detail
		{
			struct VoxelRec
			{
				std::int32_t ix = 0;
				std::int32_t iy = 0;
				std::int32_t iz = 0;
				int idx = 0;
			};

			inline bool voxelRecLess(VoxelRec const& a, VoxelRec const& b)
			{
				if (a.ix != b.ix)
					return a.ix < b.ix;
				if (a.iy != b.iy)
					return a.iy < b.iy;
				if (a.iz != b.iz)
					return a.iz < b.iz;
				return a.idx < b.idx;
			}

			inline bool sameVoxel(VoxelRec const& a, VoxelRec const& b)
			{
				return a.ix == b.ix && a.iy == b.iy && a.iz == b.iz;
			}

			inline std::int32_t voxelAxis(float v, double inv_size)
			{
				return static_cast<std::int32_t>(std::floor(static_cast<double>(v) * inv_size));
			}

			inline bool finiteXyz(float x, float y, float z)
			{
				return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
			}
		}

		/// Origin-aligned integer voxel grid; one true medoid (nearest to cell centroid) per occupied cell.
		template <typename PointT>
		bool downsampleHashMedoidIndices(typename pcl::PointCloud<PointT>::ConstPtr const& cloud,
		                                 float voxel_size, std::vector<int>& indices,
		                                 bool multi_thread = true)
		{
			indices.clear();
			if (!cloud || cloud->empty() || !(voxel_size > 0.f) || !std::isfinite(voxel_size))
				return false;

			double const inv_size = 1.0 / static_cast<double>(voxel_size);
			std::vector<detail::VoxelRec> recs;
			recs.reserve(cloud->size());
			for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
				PointT const& p = cloud->points[static_cast<std::size_t>(i)];
				if (!detail::finiteXyz(p.x, p.y, p.z))
					continue;
				detail::VoxelRec rec;
				rec.ix = detail::voxelAxis(p.x, inv_size);
				rec.iy = detail::voxelAxis(p.y, inv_size);
				rec.iz = detail::voxelAxis(p.z, inv_size);
				rec.idx = i;
				recs.push_back(rec);
			}
			if (recs.empty())
				return false;

			if (multi_thread)
				tbb::parallel_sort(recs.begin(), recs.end(), detail::voxelRecLess);
			else
				std::sort(recs.begin(), recs.end(), detail::voxelRecLess);

			indices.reserve(recs.size());
			std::size_t i = 0;
			while (i < recs.size()) {
				std::size_t j = i + 1;
				while (j < recs.size() && detail::sameVoxel(recs[i], recs[j]))
					++j;

				double sx = 0.0;
				double sy = 0.0;
				double sz = 0.0;
				std::size_t const n = j - i;
				for (std::size_t k = i; k < j; ++k) {
					PointT const& p = cloud->points[static_cast<std::size_t>(recs[k].idx)];
					sx += static_cast<double>(p.x);
					sy += static_cast<double>(p.y);
					sz += static_cast<double>(p.z);
				}
				double const inv_n = 1.0 / static_cast<double>(n);
				float const cx = static_cast<float>(sx * inv_n);
				float const cy = static_cast<float>(sy * inv_n);
				float const cz = static_cast<float>(sz * inv_n);

				float best_d = std::numeric_limits<float>::max();
				int best_idx = recs[i].idx;
				for (std::size_t k = i; k < j; ++k) {
					int const idx = recs[k].idx;
					PointT const& p = cloud->points[static_cast<std::size_t>(idx)];
					float const dx = p.x - cx;
					float const dy = p.y - cy;
					float const dz = p.z - cz;
					float const d = dx * dx + dy * dy + dz * dz;
					if (d < best_d || (d == best_d && idx < best_idx)) {
						best_d = d;
						best_idx = idx;
					}
				}
				indices.push_back(best_idx);
				i = j;
			}

			std::sort(indices.begin(), indices.end());
			return true;
		}

		/// Origin-aligned integer voxel grid; keep the first finite point that lands in each cell.
		/// Single-pass hash map — faster than medoid (no sort / no per-cell search); good for ICP solver thinning.
		template <typename PointT>
		bool downsampleHashFirstIndices(typename pcl::PointCloud<PointT>::ConstPtr const& cloud,
		                                float voxel_size, std::vector<int>& indices)
		{
			indices.clear();
			if (!cloud || cloud->empty() || !(voxel_size > 0.f) || !std::isfinite(voxel_size))
				return false;

			struct Key {
				std::int32_t ix = 0;
				std::int32_t iy = 0;
				std::int32_t iz = 0;
				bool operator==(Key const& o) const
				{
					return ix == o.ix && iy == o.iy && iz == o.iz;
				}
			};
			struct KeyHash {
				std::size_t operator()(Key const& k) const noexcept
				{
					std::size_t h = static_cast<std::size_t>(static_cast<std::uint32_t>(k.ix));
					h ^= static_cast<std::size_t>(static_cast<std::uint32_t>(k.iy)) + 0x9e3779b9u
					     + (h << 6) + (h >> 2);
					h ^= static_cast<std::size_t>(static_cast<std::uint32_t>(k.iz)) + 0x9e3779b9u
					     + (h << 6) + (h >> 2);
					return h;
				}
			};

			double const inv_size = 1.0 / static_cast<double>(voxel_size);
			std::unordered_map<Key, int, KeyHash> first;
			first.reserve(cloud->size() / 4 + 1);
			for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
				PointT const& p = cloud->points[static_cast<std::size_t>(i)];
				if (!detail::finiteXyz(p.x, p.y, p.z))
					continue;
				Key const k{detail::voxelAxis(p.x, inv_size), detail::voxelAxis(p.y, inv_size),
				            detail::voxelAxis(p.z, inv_size)};
				first.emplace(k, i);
			}
			if (first.empty())
				return false;

			indices.reserve(first.size());
			for (auto const& kv : first)
				indices.push_back(kv.second);
			// Order irrelevant for extract-by-index; skip sort to keep the fast path cheap.
			return true;
		}
	}
}
