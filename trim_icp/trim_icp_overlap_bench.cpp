// Micro-benchmark: three mutual-overlap crop strategies.
// A) Original: bidirectional nearestKSearch within radius (geometry only)
// B) Previous: hash voxel + materialized 26-neighbor dilation
// C) Current: hash voxel + query-time Chebyshev dilation (+ optional occupancy stride)

#include <trim_icp/overlap_voxel_crop.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

using Clock = std::chrono::steady_clock;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;

namespace {

struct VoxelKey {
	int x = 0;
	int y = 0;
	int z = 0;
	bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
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

CloudPtr makePartialOverlapClouds(int n_side, float z_shift, CloudPtr& tgt_out)
{
	auto src = CloudPtr(new Cloud);
	tgt_out = CloudPtr(new Cloud);
	const std::size_t n = static_cast<std::size_t>(n_side) * static_cast<std::size_t>(n_side)
		* static_cast<std::size_t>(n_side);
	src->reserve(n);
	tgt_out->reserve(n);
	for (int i = 0; i < n_side; ++i) {
		for (int j = 0; j < n_side; ++j) {
			for (int k = 0; k < n_side; ++k) {
				pcl::PointXYZ p;
				p.x = 0.05f * static_cast<float>(i);
				p.y = 0.05f * static_cast<float>(j);
				p.z = 0.05f * static_cast<float>(k);
				tgt_out->push_back(p);
				p.z += z_shift;
				src->push_back(p);
			}
		}
	}
	return src;
}

inline VoxelKey toVoxel(double x, double y, double z, double inv_vs)
{
	return VoxelKey{
		static_cast<int>(std::floor(x * inv_vs)),
		static_cast<int>(std::floor(y * inv_vs)),
		static_cast<int>(std::floor(z * inv_vs))};
}

void dilateMaterialize(VoxelSet& occupied, int dilation)
{
	if (dilation <= 0 || occupied.empty()) {
		return;
	}
	VoxelSet current = occupied;
	for (int d = 0; d < dilation; ++d) {
		VoxelSet expanded = current;
		for (const VoxelKey& k : current) {
			for (int dx = -1; dx <= 1; ++dx) {
				for (int dy = -1; dy <= 1; ++dy) {
					for (int dz = -1; dz <= 1; ++dz) {
						if (dx == 0 && dy == 0 && dz == 0) {
							continue;
						}
						expanded.insert(VoxelKey{k.x + dx, k.y + dy, k.z + dz});
					}
				}
			}
		}
		current.swap(expanded);
	}
	occupied.swap(current);
}

bool cropKnnMutual(
	const CloudPtr& src,
	const CloudPtr& tgt,
	const Eigen::Matrix4d& T,
	float radius,
	int min_points,
	std::vector<int>& src_out,
	std::vector<int>& tgt_out)
{
	src_out.clear();
	tgt_out.clear();
	const float r2 = radius * radius;

	pcl::search::KdTree<pcl::PointXYZ> tgt_tree;
	tgt_tree.setInputCloud(tgt);
	std::vector<int> nn_idx(1);
	std::vector<float> nn_dist(1);

	src_out.reserve(src->size());
	for (int i = 0; i < static_cast<int>(src->size()); ++i) {
		const auto& pt = src->points[static_cast<std::size_t>(i)];
		Eigen::Vector4d p = T * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
		if (!p.head<3>().allFinite()) {
			continue;
		}
		pcl::PointXYZ q;
		q.x = static_cast<float>(p.x());
		q.y = static_cast<float>(p.y());
		q.z = static_cast<float>(p.z());
		if (tgt_tree.nearestKSearch(q, 1, nn_idx, nn_dist) < 1) {
			continue;
		}
		if (nn_dist[0] <= r2) {
			src_out.push_back(i);
		}
	}
	if (static_cast<int>(src_out.size()) < min_points) {
		src_out.clear();
		return false;
	}

	auto src_xf = CloudPtr(new Cloud);
	src_xf->reserve(src_out.size());
	for (int idx : src_out) {
		const auto& pt = src->points[static_cast<std::size_t>(idx)];
		Eigen::Vector4d p = T * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
		pcl::PointXYZ q;
		q.x = static_cast<float>(p.x());
		q.y = static_cast<float>(p.y());
		q.z = static_cast<float>(p.z());
		src_xf->push_back(q);
	}
	pcl::search::KdTree<pcl::PointXYZ> src_tree;
	src_tree.setInputCloud(src_xf);

	tgt_out.reserve(tgt->size());
	for (int i = 0; i < static_cast<int>(tgt->size()); ++i) {
		const auto& pt = tgt->points[static_cast<std::size_t>(i)];
		if (src_tree.nearestKSearch(pt, 1, nn_idx, nn_dist) < 1) {
			continue;
		}
		if (nn_dist[0] <= r2) {
			tgt_out.push_back(i);
		}
	}
	if (static_cast<int>(tgt_out.size()) < min_points) {
		src_out.clear();
		tgt_out.clear();
		return false;
	}
	return true;
}

bool cropMaterialize(
	const CloudPtr& src,
	const CloudPtr& tgt,
	const Eigen::Matrix4d& T,
	float voxel_size,
	int dilation,
	int min_points,
	std::vector<int>& src_out,
	std::vector<int>& tgt_out,
	std::size_t* peak_voxels_out)
{
	src_out.clear();
	tgt_out.clear();
	const double inv_vs = 1.0 / static_cast<double>(voxel_size);

	VoxelSet tgt_occ;
	tgt_occ.reserve(tgt->size());
	for (const auto& pt : tgt->points) {
		tgt_occ.insert(toVoxel(pt.x, pt.y, pt.z, inv_vs));
	}
	dilateMaterialize(tgt_occ, dilation);
	std::size_t peak = tgt_occ.size();

	src_out.reserve(src->size());
	for (int i = 0; i < static_cast<int>(src->size()); ++i) {
		const auto& pt = src->points[static_cast<std::size_t>(i)];
		Eigen::Vector4d p = T * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
		if (!p.head<3>().allFinite()) {
			continue;
		}
		if (tgt_occ.find(toVoxel(p.x(), p.y(), p.z(), inv_vs)) != tgt_occ.end()) {
			src_out.push_back(i);
		}
	}
	if (static_cast<int>(src_out.size()) < min_points) {
		src_out.clear();
		return false;
	}

	VoxelSet src_occ;
	src_occ.reserve(src_out.size());
	for (int idx : src_out) {
		const auto& pt = src->points[static_cast<std::size_t>(idx)];
		Eigen::Vector4d p = T * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
		src_occ.insert(toVoxel(p.x(), p.y(), p.z(), inv_vs));
	}
	dilateMaterialize(src_occ, dilation);
	peak = (std::max)(peak, src_occ.size());

	tgt_out.reserve(tgt->size());
	for (int i = 0; i < static_cast<int>(tgt->size()); ++i) {
		const auto& pt = tgt->points[static_cast<std::size_t>(i)];
		if (src_occ.find(toVoxel(pt.x, pt.y, pt.z, inv_vs)) != src_occ.end()) {
			tgt_out.push_back(i);
		}
	}
	if (static_cast<int>(tgt_out.size()) < min_points) {
		src_out.clear();
		tgt_out.clear();
		return false;
	}
	if (peak_voxels_out) {
		*peak_voxels_out = peak;
	}
	return true;
}

struct RunResult {
	std::string name;
	double ms = 0.0;
	std::size_t src_kept = 0;
	std::size_t tgt_kept = 0;
	std::size_t peak_voxels = 0;
	bool ok = false;
};

template <typename Fn>
RunResult timeIt(const std::string& name, int warmup, int rounds, Fn&& fn)
{
	for (int i = 0; i < warmup; ++i) {
		fn();
	}
	RunResult r;
	r.name = name;
	const auto t0 = Clock::now();
	for (int i = 0; i < rounds; ++i) {
		r.ok = fn();
	}
	const auto t1 = Clock::now();
	r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / static_cast<double>(rounds);
	return r;
}

} // namespace

int main(int argc, char** argv)
{
	int n_side = 80; // 80^3 = 512k points each side
	int rounds = 3;
	int warmup = 1;
	float voxel = 0.2f;
	int dilation = 1;
	float radius = 0.2f;
	int stride = 1;
	if (argc > 1) {
		n_side = std::atoi(argv[1]);
	}
	if (argc > 2) {
		rounds = std::atoi(argv[2]);
	}
	if (argc > 3) {
		stride = std::atoi(argv[3]);
	}

	CloudPtr tgt;
	CloudPtr src = makePartialOverlapClouds(n_side, 0.8f, tgt);
	const Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	const int min_pts = 10;

	std::cout << "trim_icp_overlap_bench\n"
			  << "  N_side=" << n_side << "  Ns=Nt=" << src->size()
			  << "  rounds=" << rounds << "  voxel=" << voxel
			  << "  dilation=" << dilation << "  radius=" << radius
			  << "  stride(current)=" << stride << "\n\n";

	std::vector<int> si;
	std::vector<int> ti;
	std::size_t peak_vox = 0;

	auto r_knn = timeIt("A_original_knn", warmup, rounds, [&]() {
		const bool ok = cropKnnMutual(src, tgt, T, radius, min_pts, si, ti);
		return ok;
	});
	r_knn.src_kept = si.size();
	r_knn.tgt_kept = ti.size();

	auto r_mat = timeIt("B_prev_materialize", warmup, rounds, [&]() {
		peak_vox = 0;
		const bool ok = cropMaterialize(src, tgt, T, voxel, dilation, min_pts, si, ti, &peak_vox);
		return ok;
	});
	r_mat.src_kept = si.size();
	r_mat.tgt_kept = ti.size();
	r_mat.peak_voxels = peak_vox;

	Ddx::OverlapVoxelCropParams cur;
	cur.voxel_size = voxel;
	cur.dilation = dilation;
	cur.min_points = min_pts;
	cur.occupancy_stride = stride;
	auto r_cur = timeIt("C_current_query_dilate", warmup, rounds, [&]() {
		return Ddx::cropMutualOverlapByVoxel(src, tgt, T, cur, si, ti);
	});
	r_cur.src_kept = si.size();
	r_cur.tgt_kept = ti.size();

	// Estimate current occupancy size (raw, no dilate) for memory proxy
	{
		const double inv_vs = 1.0 / static_cast<double>(voxel);
		VoxelSet occ;
		for (int i = 0; i < static_cast<int>(tgt->size()); i += stride) {
			const auto& pt = tgt->points[static_cast<std::size_t>(i)];
			occ.insert(toVoxel(pt.x, pt.y, pt.z, inv_vs));
		}
		r_cur.peak_voxels = occ.size();
	}

	auto printRow = [](const RunResult& r) {
		std::cout << std::left << std::setw(26) << r.name
				  << " ok=" << (r.ok ? 1 : 0)
				  << "  ms=" << std::fixed << std::setprecision(2) << std::setw(10) << r.ms
				  << "  src_kept=" << r.src_kept
				  << "  tgt_kept=" << r.tgt_kept
				  << "  peak_voxels~=" << r.peak_voxels
				  << "\n";
	};

	printRow(r_knn);
	printRow(r_mat);
	printRow(r_cur);

	if (r_knn.ms > 0.0) {
		std::cout << "\nSpeedup vs A_original_knn:\n"
				  << "  B: " << (r_knn.ms / r_mat.ms) << "x\n"
				  << "  C: " << (r_knn.ms / r_cur.ms) << "x\n";
	}
	if (r_mat.ms > 0.0) {
		std::cout << "Speedup vs B_prev_materialize:\n"
				  << "  C: " << (r_mat.ms / r_cur.ms) << "x\n";
	}
	if (r_mat.peak_voxels > 0 && r_cur.peak_voxels > 0) {
		std::cout << "Voxel-set size ratio (C/B raw proxy): "
				  << (static_cast<double>(r_cur.peak_voxels) / static_cast<double>(r_mat.peak_voxels))
				  << "\n";
	}

	return (r_knn.ok && r_mat.ok && r_cur.ok) ? 0 : 1;
}
