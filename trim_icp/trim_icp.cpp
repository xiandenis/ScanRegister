#include "trim_icp.hpp"

#include "trim_icp_solver.hpp"
#include "overlap_voxel_crop.hpp"
#include "knn_normal_overlap_crop.hpp"

#include <octree/octree_downsample.hpp>

#include <chrono>
#include <cmath>
#include <utility>

namespace Ddx {
namespace {

using Clock = std::chrono::steady_clock;

double msSince(Clock::time_point t0)
{
	return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

template <typename CloudT>
typename CloudT::Ptr extractByIndices(const typename CloudT::Ptr& in, const std::vector<int>& indices)
{
	auto out = typename CloudT::Ptr(new CloudT);
	out->reserve(indices.size());
	for (int idx : indices) {
		out->push_back(in->points[static_cast<std::size_t>(idx)]);
	}
	return out;
}

} // namespace

bool TrimICP::align(const Eigen::Matrix4d& initial_guess)
{
	m_last_timing = TrimIcpPhaseTiming{};
	auto const t_all = Clock::now();

	if (!checkCloud()) {
		m_last_timing.total_ms = msSince(t_all);
		return false;
	}
	if (m_nc_enabled && !checkNormal()) {
		m_last_timing.total_ms = msSince(t_all);
		return false;
	}

	m_last_timing.src_in = static_cast<int>(m_src_points->size());
	m_last_timing.tgt_in = static_cast<int>(m_tgt_points->size());

	PointCloud::Ptr src_use = m_src_points;
	PointCloud::Ptr tgt_use = m_tgt_points;
	NormalCloud::Ptr src_n_use = m_src_normals;
	NormalCloud::Ptr tgt_n_use = m_tgt_normals;

	PointCloud::Ptr src_cropped;
	PointCloud::Ptr tgt_cropped;
	NormalCloud::Ptr src_n_cropped;
	NormalCloud::Ptr tgt_n_cropped;

	if (m_overlap_crop_enabled) {
		OverlapVoxelCropParams params;
		params.voxel_size = m_overlap_voxel_size;
		params.dilation = m_overlap_dilation;
		params.occupancy_stride = m_overlap_occupancy_stride;
		params.min_points = 10;

		std::vector<int> src_indices;
		std::vector<int> tgt_indices;
		auto const t0 = Clock::now();
		if (!cropMutualOverlapByVoxel(
				m_src_points, m_tgt_points, initial_guess, params, src_indices, tgt_indices)) {
			m_last_timing.crop_ms = msSince(t0);
			m_last_timing.total_ms = msSince(t_all);
			return false;
		}

		src_cropped = extractByIndices<PointCloud>(m_src_points, src_indices);
		tgt_cropped = extractByIndices<PointCloud>(m_tgt_points, tgt_indices);
		src_use = src_cropped;
		tgt_use = tgt_cropped;

		if (m_src_normals) {
			src_n_cropped = extractByIndices<NormalCloud>(m_src_normals, src_indices);
			src_n_use = src_n_cropped;
		}
		if (m_tgt_normals) {
			tgt_n_cropped = extractByIndices<NormalCloud>(m_tgt_normals, tgt_indices);
			tgt_n_use = tgt_n_cropped;
		}
		m_last_timing.crop_ms = msSince(t0);
	}
	m_last_timing.src_after_crop = static_cast<int>(src_use->size());
	m_last_timing.tgt_after_crop = static_cast<int>(tgt_use->size());

	// Optional fast voxel thinning after crop, before NC trees/KNN (cheaper NC on thinner clouds).
	m_last_timing.src_after_ds = m_last_timing.src_after_crop;
	m_last_timing.tgt_after_ds = m_last_timing.tgt_after_crop;
	if (m_solver_ds_enabled && m_solver_ds_voxel > 0.f && std::isfinite(m_solver_ds_voxel)) {
		auto const t0 = Clock::now();
		std::vector<int> src_ds;
		std::vector<int> tgt_ds;
		if (!Ddx::octree::downsampleFirstIndices<PointType>(src_use, m_solver_ds_voxel, src_ds)
			|| !Ddx::octree::downsampleFirstIndices<PointType>(tgt_use, m_solver_ds_voxel, tgt_ds)
			|| src_ds.size() < 10 || tgt_ds.size() < 10) {
			m_last_timing.downsample_ms = msSince(t0);
			m_last_timing.total_ms = msSince(t_all);
			return false;
		}
		PointCloud::Ptr src_ds_cloud = extractByIndices<PointCloud>(src_use, src_ds);
		PointCloud::Ptr tgt_ds_cloud = extractByIndices<PointCloud>(tgt_use, tgt_ds);
		if (src_n_use) {
			if (src_n_use->size() != src_use->size()) {
				m_last_timing.downsample_ms = msSince(t0);
				m_last_timing.total_ms = msSince(t_all);
				return false;
			}
			src_n_use = extractByIndices<NormalCloud>(src_n_use, src_ds);
		}
		if (tgt_n_use) {
			if (tgt_n_use->size() != tgt_use->size()) {
				m_last_timing.downsample_ms = msSince(t0);
				m_last_timing.total_ms = msSince(t_all);
				return false;
			}
			tgt_n_use = extractByIndices<NormalCloud>(tgt_n_use, tgt_ds);
		}
		src_use = src_ds_cloud;
		tgt_use = tgt_ds_cloud;
		m_last_timing.downsample_ms = msSince(t0);
		m_last_timing.src_after_ds = static_cast<int>(src_use->size());
		m_last_timing.tgt_after_ds = static_cast<int>(tgt_use->size());
	}

	// South-style normal prefilter on the current working subset (after crop / optional ds).
	if (m_nc_enabled) {
		if (!src_n_use || !tgt_n_use
			|| src_n_use->size() != src_use->size()
			|| tgt_n_use->size() != tgt_use->size()) {
			m_last_timing.total_ms = msSince(t_all);
			return false;
		}
		{
			auto const t0 = Clock::now();
			if (!ensureTreesFor(src_use, tgt_use)) {
				m_last_timing.trees_ms = msSince(t0);
				m_last_timing.total_ms = msSince(t_all);
				return false;
			}
			m_last_timing.trees_ms = msSince(t0);
		}

		KnnNormalOverlapCropParams knn;
		knn.max_search_radius = m_overlap_voxel_size;
		knn.min_points = 10;
		knn.dir_thr = static_cast<float>(m_nc_thr);

		std::vector<int> src_keep;
		std::vector<int> tgt_keep;
		{
			auto const t0 = Clock::now();
			if (!cropMutualOverlapByKnnNormal(
					src_use,
					src_n_use,
					m_src_tree,
					tgt_use,
					tgt_n_use,
					m_tgt_tree,
					initial_guess,
					knn,
					src_keep,
					tgt_keep)) {
				m_last_timing.nc_knn_ms = msSince(t0);
				m_last_timing.total_ms = msSince(t_all);
				return false;
			}

			src_cropped = extractByIndices<PointCloud>(src_use, src_keep);
			tgt_cropped = extractByIndices<PointCloud>(tgt_use, tgt_keep);
			src_n_cropped = extractByIndices<NormalCloud>(src_n_use, src_keep);
			tgt_n_cropped = extractByIndices<NormalCloud>(tgt_n_use, tgt_keep);
			src_use = src_cropped;
			tgt_use = tgt_cropped;
			src_n_use = src_n_cropped;
			tgt_n_use = tgt_n_cropped;
			m_last_timing.nc_knn_ms = msSince(t0);
		}
	}
	m_last_timing.src_after_nc = static_cast<int>(src_use->size());
	m_last_timing.tgt_after_nc = static_cast<int>(tgt_use->size());

	// Role swap only at TrimIcpSolver: denser side fixed, thinner moves; API T stays src→tgt.
	// Swap Ptr handles only (no point-cloud deep copy).
	bool const swapped =
		m_auto_role_swap && src_use->size() > tgt_use->size();
	m_last_timing.auto_swapped = swapped;
	Eigen::Matrix4d solver_guess = initial_guess;
	if (swapped) {
		std::swap(src_use, tgt_use);
		std::swap(src_n_use, tgt_n_use);
		solver_guess = initial_guess.inverse();
	}

	TrimIcpSolver solver;
	solver.setSourceCloud(src_use);
	solver.setTargetCloud(tgt_use);
	if (src_n_use) {
		solver.setSourceNormals(src_n_use);
	}
	if (tgt_n_use) {
		solver.setTargetNormals(tgt_n_use);
	}
	solver.setMethod(m_method);
	solver.setProfile(m_profile);

	{
		auto const t0 = Clock::now();
		if (!solver.align(solver_guess)) {
			m_last_timing.solver_ms = msSince(t0);
			m_last_timing.total_ms = msSince(t_all);
			return false;
		}
		m_last_timing.solver_ms = msSince(t0);
	}

	m_transformation = solver.getFinalTransformation();
	if (swapped) {
		m_transformation = m_transformation.inverse();
	}
	m_last_timing.profile = m_profile;
	if (m_profile) {
		m_last_timing.solver_welsch_stages = solver.profileWelschStages();
		m_last_timing.solver_icp_iters = solver.profileIcpIters();
		m_last_timing.solver_closest_passes = solver.profileClosestPasses();
		m_last_timing.solver_aa_rejects = solver.profileAaRejects();
		m_last_timing.solver_init_ms = solver.profileInitMs();
		m_last_timing.solver_closest_ms = solver.profileClosestMs();
		m_last_timing.solver_update_ms = solver.profileUpdateMs();
	}
	m_last_timing.ok = true;
	m_last_timing.total_ms = msSince(t_all);
	return true;
}

bool TrimICP::checkCloud() const
{
	if (m_src_points == nullptr || m_tgt_points == nullptr) {
		return false;
	}
	if (m_src_points->size() < 10 || m_tgt_points->size() < 10) {
		return false;
	}
	return true;
}

bool TrimICP::checkNormal() const
{
	if (m_src_normals == nullptr || m_tgt_normals == nullptr) {
		return false;
	}
	if (m_src_normals->size() != m_src_points->size() || m_tgt_normals->size() != m_tgt_points->size()) {
		return false;
	}
	return true;
}

bool TrimICP::ensureTreesFor(const PointCloud::Ptr& src, const PointCloud::Ptr& tgt)
{
	if (!src || !tgt) {
		return false;
	}
	if (m_src_tree == nullptr || m_src_tree->getInputCloud() != src) {
		m_src_tree.reset(new KdTree());
		m_src_tree->setInputCloud(src);
	}
	if (m_tgt_tree == nullptr || m_tgt_tree->getInputCloud() != tgt) {
		m_tgt_tree.reset(new KdTree());
		m_tgt_tree->setInputCloud(tgt);
	}
	return true;
}

} // namespace Ddx
