#include "overlap_voxel_crop.hpp"

#include <cmath>
#include <cstdint>
#include <unordered_set>

namespace Ddx {
namespace {

struct VoxelKey {
	int x = 0;
	int y = 0;
	int z = 0;

	bool operator==(const VoxelKey& o) const
	{
		return x == o.x && y == o.y && z == o.z;
	}
};

struct VoxelKeyHash {
	std::size_t operator()(const VoxelKey& k) const noexcept
	{
		std::size_t h = static_cast<std::size_t>(static_cast<uint32_t>(k.x));
		h ^= static_cast<std::size_t>(static_cast<uint32_t>(k.y)) + 0x9e3779b9u + (h << 6) + (h >> 2);
		h ^= static_cast<std::size_t>(static_cast<uint32_t>(k.z)) + 0x9e3779b9u + (h << 6) + (h >> 2);
		return h;
	}
};

using VoxelSet = std::unordered_set<VoxelKey, VoxelKeyHash>;

inline VoxelKey toVoxel(double x, double y, double z, double inv_vs)
{
	VoxelKey k;
	k.x = static_cast<int>(std::floor(x * inv_vs));
	k.y = static_cast<int>(std::floor(y * inv_vs));
	k.z = static_cast<int>(std::floor(z * inv_vs));
	return k;
}

/// True if any occupied voxel lies within Chebyshev distance `dilation` of `key`.
inline bool hitsDilatedOccupancy(const VoxelSet& occupied, const VoxelKey& key, int dilation)
{
	if (dilation <= 0) {
		return occupied.find(key) != occupied.end();
	}
	for (int dx = -dilation; dx <= dilation; ++dx) {
		for (int dy = -dilation; dy <= dilation; ++dy) {
			for (int dz = -dilation; dz <= dilation; ++dz) {
				if (occupied.find(VoxelKey{key.x + dx, key.y + dy, key.z + dz}) != occupied.end()) {
					return true;
				}
			}
		}
	}
	return false;
}

VoxelSet buildOccupancy(
	const pcl::PointCloud<pcl::PointXYZ>& cloud,
	const Eigen::Matrix4d* T_opt,
	double inv_vs,
	int stride)
{
	VoxelSet occupied;
	const int n = static_cast<int>(cloud.size());
	const int step = stride < 1 ? 1 : stride;
	occupied.reserve(static_cast<std::size_t>(n / step + 1));
	for (int i = 0; i < n; i += step) {
		const auto& pt = cloud.points[static_cast<std::size_t>(i)];
		Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
		if (T_opt) {
			p = (*T_opt) * p;
		}
		if (!p.head<3>().allFinite()) {
			continue;
		}
		occupied.insert(toVoxel(p.x(), p.y(), p.z(), inv_vs));
	}
	return occupied;
}

} // namespace

bool cropMutualOverlapByVoxel(
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& src,
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& tgt,
	const Eigen::Matrix4d& T_src_to_tgt,
	const OverlapVoxelCropParams& params,
	std::vector<int>& src_indices_out,
	std::vector<int>& tgt_indices_out)
{
	src_indices_out.clear();
	tgt_indices_out.clear();

	if (!src || !tgt) {
		return false;
	}
	if (params.voxel_size <= 0.f || params.dilation < 0 || params.min_points < 1) {
		return false;
	}
	if (params.occupancy_stride < 1) {
		return false;
	}
	if (src->empty() || tgt->empty()) {
		return false;
	}

	const double inv_vs = 1.0 / static_cast<double>(params.voxel_size);
	const int dil = params.dilation;
	const int stride = params.occupancy_stride;

	const VoxelSet tgt_occ = buildOccupancy(*tgt, nullptr, inv_vs, stride);

	const int n_src = static_cast<int>(src->size());
	std::vector<char> src_keep(static_cast<std::size_t>(n_src), 0);
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
	for (int i = 0; i < n_src; ++i) {
		const auto& pt = src->points[static_cast<std::size_t>(i)];
		Eigen::Vector4d p = T_src_to_tgt * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
		if (!p.head<3>().allFinite()) {
			continue;
		}
		if (hitsDilatedOccupancy(tgt_occ, toVoxel(p.x(), p.y(), p.z(), inv_vs), dil)) {
			src_keep[static_cast<std::size_t>(i)] = 1;
		}
	}

	src_indices_out.reserve(static_cast<std::size_t>(n_src));
	for (int i = 0; i < n_src; ++i) {
		if (src_keep[static_cast<std::size_t>(i)]) {
			src_indices_out.push_back(i);
		}
	}

	if (static_cast<int>(src_indices_out.size()) < params.min_points) {
		src_indices_out.clear();
		return false;
	}

	VoxelSet src_occ;
	src_occ.reserve(src_indices_out.size() / static_cast<std::size_t>(stride) + 1);
	for (std::size_t k = 0; k < src_indices_out.size(); k += static_cast<std::size_t>(stride)) {
		const int idx = src_indices_out[k];
		const auto& pt = src->points[static_cast<std::size_t>(idx)];
		Eigen::Vector4d p = T_src_to_tgt * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
		if (!p.head<3>().allFinite()) {
			continue;
		}
		src_occ.insert(toVoxel(p.x(), p.y(), p.z(), inv_vs));
	}

	const int n_tgt = static_cast<int>(tgt->size());
	std::vector<char> tgt_keep(static_cast<std::size_t>(n_tgt), 0);
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
	for (int i = 0; i < n_tgt; ++i) {
		const auto& pt = tgt->points[static_cast<std::size_t>(i)];
		if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
			continue;
		}
		if (hitsDilatedOccupancy(src_occ, toVoxel(pt.x, pt.y, pt.z, inv_vs), dil)) {
			tgt_keep[static_cast<std::size_t>(i)] = 1;
		}
	}

	tgt_indices_out.reserve(static_cast<std::size_t>(n_tgt));
	for (int i = 0; i < n_tgt; ++i) {
		if (tgt_keep[static_cast<std::size_t>(i)]) {
			tgt_indices_out.push_back(i);
		}
	}

	if (static_cast<int>(tgt_indices_out.size()) < params.min_points) {
		src_indices_out.clear();
		tgt_indices_out.clear();
		return false;
	}

	return true;
}

} // namespace Ddx
