#pragma once
#include <Eigen/Dense>
#include <basic_icp/basic_icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace Ddx {

/// 单次 align 分账（毫秒 + 点数）；失败阶段之后的字段保持 0。
/// solver 内 profile_* 仅在 setProfile(true) 时有意义，默认全 0。
struct TrimIcpPhaseTiming {
	double crop_ms = 0.0;
	double trees_ms = 0.0;
	double nc_knn_ms = 0.0;
	double downsample_ms = 0.0;
	double solver_ms = 0.0;
	double total_ms = 0.0;
	int src_in = 0;
	int tgt_in = 0;
	int src_after_crop = 0;
	int tgt_after_crop = 0;
	int src_after_nc = 0;
	int tgt_after_nc = 0;
	int src_after_ds = 0;
	int tgt_after_ds = 0;
	bool ok = false;
	/// True if TrimICP swapped roles for the internal solve (API T still src→tgt).
	bool auto_swapped = false;
	bool profile = false;
	int solver_welsch_stages = 0;
	int solver_icp_iters = 0;
	int solver_closest_passes = 0;
	int solver_aa_rejects = 0;
	double solver_init_ms = 0.0;
	double solver_closest_ms = 0.0;
	double solver_update_ms = 0.0;
};

/**
 * TrimICP — 上层编排：可选哈希体素互裁 → 可选 south 式法向预筛 → TrimIcpSolver → SLAM basic_icp。
 *
 * ## 输入只有点（无可用法向）时怎么用
 * - Method 用**点到点**：`RICP`（默认）/ `ICP` / `FICP` / `AA_ICP` / `SparseICP`。
 * - **不要**开法向预筛：`setNormalConsistencyEnabled(false)`（默认已是 false）。
 * - **不要**设 `PPL` / `RPPL` / `SICPPPL`（点到面，缺目标法向会失败）。
 * - 重叠裁剪与法向无关，默认可开；全量也可 `setOverlapCropEnabled(false)`。
 *
 * ## 何时需要法向
 * 1. **点到面 Method**：`PPL` / `RPPL` / `SICPPPL` → 至少需要**目标法向**。
 * 2. **法向预筛（NC 开关改义）**：`setNormalConsistencyEnabled(true)`
 *    → **源+目标法向都必须有**；在体素互裁（若开启）**之后**做双向 KNN + `dir_thr` 点积过滤（south 同源）。
 *    缺法向时 `align` 直接 `false`。默认阈值 `dir_thr=-0.1`。
 * 3. 法向质量：需「可用且朝向一致」的法向；本类不估计、不翻转。
 *
 * ## 推荐组合（简表）
 * | 场景           | Method     | 预筛 | 法向     |
 * |----------------|------------|------|----------|
 * | 仅 XYZ 点云    | RICP       | 关   | 不需要   |
 * | 有可靠双边法向 | RICP/RPPL  | 可开 | 源+目标  |
 * | 点到面精修     | RPPL/SICPPPL | 可选 | 按上表 |
 *
 * 重叠裁剪参数：`voxel_size≈R`（可用 `setMaxSearchRadius`）、`dilation≥1`（默认 1）。
 * 预筛半径复用 `overlap_voxel_size` / `setMaxSearchRadius`。
 *
 * ## 求解前下采样（默认开 · 0.05 m）
 * `setSolverDownsampleEnabled` / `setSolverDownsampleVoxelSize`：
 * 顺序为 **crop → ds → NC → solver**（先抽稀再 NC）；哈希体素**首点**；法向随索引同步。
 *
 * ## 源–目标自动对调（默认开）
 * 仅在调用 `TrimIcpSolver` 前：工作子集 `|src| > |tgt|` 则对调求解初值/结果再求逆；
 * crop/NC/ds 仍按入口方向。对外始终 \(T_{s\to t}\)。
 */
class TrimICP
{
private:
	typedef pcl::PointXYZ PointType;
	typedef pcl::Normal NormalType;
	typedef pcl::PointCloud<PointType> PointCloud;
	typedef pcl::search::KdTree<PointType> KdTree;
	typedef pcl::PointCloud<NormalType> NormalCloud;

public:
	TrimICP()
		: m_transformation(Eigen::Matrix4d::Identity())
		, m_src_points(nullptr)
		, m_src_normals(nullptr)
		, m_src_tree(nullptr)
		, m_tgt_points(nullptr)
		, m_tgt_normals(nullptr)
		, m_tgt_tree(nullptr)
		, m_overlap_crop_enabled(true)
		, m_overlap_voxel_size(0.75f)
		, m_overlap_dilation(1)
		, m_overlap_occupancy_stride(1)
		, m_nc_enabled(false)
		, m_nc_thr(-0.1)
		, m_solver_ds_enabled(true)
		, m_solver_ds_voxel(0.05f)
		, m_auto_role_swap(true)
		, m_method(Ddx::registration::BasicICP::Method::RICP)
	{
	}

	void setSourceCloud(const PointCloud::Ptr& src_points) { m_src_points = src_points; }
	/// 可选；点到面或开启法向预筛时需要。
	void setSourceNormals(const NormalCloud::Ptr& src_normals) { m_src_normals = src_normals; }
	void setSourceTree(const KdTree::Ptr& src_tree) { m_src_tree = src_tree; }
	void setTargetCloud(const PointCloud::Ptr& tgt_points) { m_tgt_points = tgt_points; }
	/// 可选；点到面需要目标法向；开启预筛时源+目标都要。
	void setTargetNormals(const NormalCloud::Ptr& tgt_normals) { m_tgt_normals = tgt_normals; }
	void setTargetTree(const KdTree::Ptr& tgt_tree) { m_tgt_tree = tgt_tree; }

	void setOverlapCropEnabled(bool enabled) { m_overlap_crop_enabled = enabled; }
	void setOverlapVoxelSize(float voxel_size) { m_overlap_voxel_size = voxel_size; }
	void setOverlapDilation(int dilation) { m_overlap_dilation = dilation; }
	/// Build occupancy from every N-th point (N>=1); filter still uses all points.
	void setOverlapOccupancyStride(int stride) { m_overlap_occupancy_stride = stride; }

	/// Maps to overlap voxel size / 预筛搜索半径。
	void setMaxSearchRadius(float max_search_radius) { m_overlap_voxel_size = max_search_radius; }

	/// 默认 false。为 true 时：体素后（若开）做 south 式 KNN+dir_thr 预筛；必须已设源+目标法向。
	void setNormalConsistencyEnabled(bool enabled) { m_nc_enabled = enabled; }
	/// dir_thr：保留当 n·n >= thr（默认 -0.1，south 同源）。
	void setNormalConsistencyThreshold(double thr) { m_nc_thr = thr; }

	/// 默认 true。开启后在求解前对工作子集做哈希体素首点下采样。
	void setSolverDownsampleEnabled(bool enabled) { m_solver_ds_enabled = enabled; }
	/// 求解下采样体素边长（米）；默认 0.05。仅当 enabled 且 >0 时生效。
	void setSolverDownsampleVoxelSize(float voxel_size) { m_solver_ds_voxel = voxel_size; }

	/// 默认 true。|src|>|tgt| 时内部对调角色，出口仍返回 caller 的 \(T_{s\to t}\)。
	void setAutoRoleSwapEnabled(bool enabled) { m_auto_role_swap = enabled; }

	/// 默认 RICP（点到点，仅需 XYZ）。点到面 Method 见类注释。
	void setMethod(Ddx::registration::BasicICP::Method method) { m_method = method; }
	/// 默认 false：FRICP 内不计时/不计次；产品路径请保持关闭。
	void setProfile(bool enabled) { m_profile = enabled; }
	bool profile() const { return m_profile; }

	bool align(const Eigen::Matrix4d& initial_guess = Eigen::Matrix4d::Identity());

	Eigen::Matrix4d getFinalTransformation() const { return m_transformation; }
	TrimIcpPhaseTiming const& lastTiming() const { return m_last_timing; }

private:
	bool checkCloud() const;
	bool checkNormal() const;
	/// Ensure KdTrees match the given working clouds (for NC prefilter).
	bool ensureTreesFor(const PointCloud::Ptr& src, const PointCloud::Ptr& tgt);

	PointCloud::Ptr m_src_points;
	NormalCloud::Ptr m_src_normals;
	KdTree::Ptr m_src_tree;
	PointCloud::Ptr m_tgt_points;
	NormalCloud::Ptr m_tgt_normals;
	KdTree::Ptr m_tgt_tree;

	Eigen::Matrix4d m_transformation;

	bool m_overlap_crop_enabled;
	float m_overlap_voxel_size;
	int m_overlap_dilation;
	int m_overlap_occupancy_stride;

	bool m_nc_enabled;
	double m_nc_thr;
	bool m_solver_ds_enabled;
	float m_solver_ds_voxel;
	bool m_auto_role_swap;
	Ddx::registration::BasicICP::Method m_method;
	bool m_profile = false;
	TrimIcpPhaseTiming m_last_timing;
};

} // namespace Ddx
