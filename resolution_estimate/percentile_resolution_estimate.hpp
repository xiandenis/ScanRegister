#pragma once

#include <registration_type/registration_type.hpp>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <string>
#include <vector>

namespace Ddx
{
	/** 曲率调制策略；默认 kIndoorScore 保持旧行为。 */
	enum class CurvatureModulateMode
	{
		/** v1：s=w*p_plane+(1-w)*(1-σ̃)，v=v0*(1+α*(1-2s)) */
		kIndoorScore = 0,
		/**
		 * v2：非平面超额放大 + 高平面室内抑制。
		 * p_np>p0 → 放大；p_plane≥indoor_thr → 缩小；中间带 m=1（不动深度）。
		 */
		kNonplanarExcess = 1,
	};

	/** 分位数场景尺度 → 粗配准体素：深度主尺度 + 曲率有界室内外调制。 */
	struct PercentileResolutionParams
	{
		float weight_p50 = 0.5f;
		float weight_p75 = 0.3f;
		float weight_p90 = 0.2f;
		/** v0 = clamp(scale_k * r_char, v_min, v_max)；默认约 12 m → 0.15 m */
		float scale_k = 0.0125f;
		float v_min = 0.1f;
		float v_max = 1.0f;
		/** 深度截断（米），抑制极远噪点拉高 P90 */
		float depth_cap = 120.0f;
		/** 大云（如 Root）最多参与估计的深度样本数；0 表示全量 */
		size_t max_samples = 200000;
		/** true：量化到 {0.1,0.2,0.3,0.4,0.6,0.8,1.0}；false：返回连续值 */
		bool quantize = true;

		/** 用法向曲率分布对 v0 做有界调制；无有效法向时自动退回纯深度 */
		bool use_curvature_modulate = true;
		/** κ < thr 计为平面点 */
		float curvature_plane_thr = 0.01f;
		/** log(κ+ε) 标准差归一化参考；σ̃ = clamp(σ / σ_ref, 0, 1)（仅 kIndoorScore） */
		float curvature_log_std_ref = 1.5f;
		/** kIndoorScore 调制幅度：v = v0 * (1 + α*(1-2s))，α∈[0,1] */
		float curvature_modulate_alpha = 0.25f;
		/** kIndoorScore：室内倾向 s = w*p_plane + (1-w)*(1-σ̃) */
		float curvature_plane_weight = 0.6f;

		/** 曲率调制模式；默认旧版 kIndoorScore，不改变既有调用语义 */
		CurvatureModulateMode curvature_modulate_mode = CurvatureModulateMode::kIndoorScore;
		/** kNonplanarExcess：非平面占比超过此阈值才放大（默认 0.65） */
		float nonplanar_excess_p0 = 0.65f;
		/** kNonplanarExcess：超额放大幅度 */
		float nonplanar_excess_alpha = 0.60f;
		/** kNonplanarExcess：超额映射指数 ≥1 */
		float nonplanar_excess_gamma = 1.0f;
		/** kNonplanarExcess：平面占比 ≥ 此值走室内缩小 */
		float nonplanar_indoor_plane_thr = 0.60f;
		/** kNonplanarExcess：室内缩小幅度 */
		float nonplanar_indoor_alpha = 0.25f;

		/** 工厂：v2 非平面超额默认参数（不改动默认构造的 v1 语义） */
		static PercentileResolutionParams NonplanarExcess()
		{
			PercentileResolutionParams p;
			p.curvature_modulate_mode = CurvatureModulateMode::kNonplanarExcess;
			return p;
		}
	};

	struct PercentileResolutionResult
	{
		float r50 = std::numeric_limits<float>::quiet_NaN();
		float r75 = std::numeric_limits<float>::quiet_NaN();
		float r90 = std::numeric_limits<float>::quiet_NaN();
		float r_char = std::numeric_limits<float>::quiet_NaN();
		/** 深度通道体素（曲率调制前） */
		float resolution_depth = std::numeric_limits<float>::quiet_NaN();
		float resolution_continuous = std::numeric_limits<float>::quiet_NaN();
		/** 最终推荐体素（米）；quantize 时为档位值 */
		float resolution = std::numeric_limits<float>::quiet_NaN();

		float p_plane = std::numeric_limits<float>::quiet_NaN();
		float p_nonplanar = std::numeric_limits<float>::quiet_NaN();
		float curv_log_std = std::numeric_limits<float>::quiet_NaN();
		/** 室内倾向 [0,1]；越大越偏室内（体素往小调）；v2 下为派生示意量 */
		float indoor_score = std::numeric_limits<float>::quiet_NaN();
		/** 施加的乘性因子 m（v = v0 * m） */
		float modulate_factor = std::numeric_limits<float>::quiet_NaN();
		bool curvature_applied = false;
		CurvatureModulateMode curvature_mode = CurvatureModulateMode::kIndoorScore;
		/** v2 曲率分支场景提示；未调制时为 Unknown */
		VoxelSceneClass scene_class = VoxelSceneClass::Unknown;
	};

	class PercentileResolutionEstimate
	{
	public:
		static float estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		                      std::string const& name = "",
		                      PercentileResolutionParams const& params = {});

		static float estimate(std::vector<Eigen::Vector3f> const& cloud,
		                      std::string const& name = "",
		                      PercentileResolutionParams const& params = {});

		static float estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		                      pcl::PointCloud<pcl::Normal>::Ptr normals,
		                      std::string const& name = "",
		                      PercentileResolutionParams const& params = {});

		static PercentileResolutionResult estimateDetailed(
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			std::string const& name = "",
			PercentileResolutionParams const& params = {});

		static PercentileResolutionResult estimateDetailed(
			std::vector<Eigen::Vector3f> const& cloud,
			std::string const& name = "",
			PercentileResolutionParams const& params = {});

		static PercentileResolutionResult estimateDetailed(
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			pcl::PointCloud<pcl::Normal>::Ptr normals,
			std::string const& name = "",
			PercentileResolutionParams const& params = {});
	};

} // namespace Ddx
