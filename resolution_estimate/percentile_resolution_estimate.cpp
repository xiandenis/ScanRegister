#include "percentile_resolution_estimate.hpp"

#include <logging/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>

namespace Ddx
{
	char const* voxelSceneClassToString(VoxelSceneClass c)
	{
		switch (c) {
		case VoxelSceneClass::IndoorShrink:
			return "indoor_shrink";
		case VoxelSceneClass::OutdoorExcess:
			return "outdoor_excess";
		case VoxelSceneClass::Neutral:
			return "neutral";
		case VoxelSceneClass::Unknown:
		default:
			return "unknown";
		}
	}

	VoxelSceneClass voxelSceneClassFromString(std::string const& s)
	{
		if (s == "indoor_shrink")
			return VoxelSceneClass::IndoorShrink;
		if (s == "outdoor_excess")
			return VoxelSceneClass::OutdoorExcess;
		if (s == "neutral")
			return VoxelSceneClass::Neutral;
		return VoxelSceneClass::Unknown;
	}

	namespace {

	float percentile_sorted(std::vector<float> const& sorted, float p)
	{
		if (sorted.empty())
			return std::numeric_limits<float>::quiet_NaN();
		p = std::clamp(p, 0.0f, 1.0f);
		float const idx = p * static_cast<float>(sorted.size() - 1);
		size_t const i0 = static_cast<size_t>(idx);
		size_t const i1 = std::min(i0 + 1, sorted.size() - 1);
		float const t = idx - static_cast<float>(i0);
		return sorted[i0] * (1.0f - t) + sorted[i1] * t;
	}

	float quantize_resolution(float v)
	{
		static float const kSteps[] = {0.1f, 0.2f, 0.3f, 0.4f, 0.6f, 0.8f, 1.0f};
		float best = kSteps[0];
		float best_abs = std::fabs(v - best);
		for (float s : kSteps) {
			float const d = std::fabs(v - s);
			if (d < best_abs) {
				best_abs = d;
				best = s;
			}
		}
		return best;
	}

	void normalize_weights(PercentileResolutionParams& p)
	{
		float sum = p.weight_p50 + p.weight_p75 + p.weight_p90;
		if (!(sum > 0.0f) || !std::isfinite(sum)) {
			p.weight_p50 = 0.5f;
			p.weight_p75 = 0.3f;
			p.weight_p90 = 0.2f;
			return;
		}
		p.weight_p50 /= sum;
		p.weight_p75 /= sum;
		p.weight_p90 /= sum;
	}

	size_t sample_stride(size_t n, size_t max_samples)
	{
		if (n == 0)
			return 1;
		if (max_samples > 0 && n > max_samples)
			return (n + max_samples - 1) / max_samples;
		return 1;
	}

	void fill_curvature_stats(PercentileResolutionResult& out,
	                          std::vector<float> const& curvatures,
	                          float plane_thr)
	{
		float const thr = (plane_thr > 0.0f && std::isfinite(plane_thr)) ? plane_thr : 0.01f;
		float const eps = 1e-6f;
		size_t plane_count = 0;
		std::vector<float> log_vals;
		log_vals.reserve(curvatures.size());

		for (float kappa : curvatures) {
			float k = kappa;
			if (!std::isfinite(k) || k <= 0.0f)
				k = 0.0f;
			if (k < thr)
				++plane_count;
			log_vals.push_back(std::log(k + eps));
		}

		out.p_plane = static_cast<float>(plane_count) / static_cast<float>(curvatures.size());
		out.p_nonplanar = 1.0f - out.p_plane;

		float mean_log = std::accumulate(log_vals.begin(), log_vals.end(), 0.0f)
		                 / static_cast<float>(log_vals.size());
		float var = 0.0f;
		for (float lv : log_vals) {
			float const d = lv - mean_log;
			var += d * d;
		}
		var /= static_cast<float>(log_vals.size());
		out.curv_log_std = std::sqrt(var);
	}

	void apply_factor(PercentileResolutionResult& out,
	                  PercentileResolutionParams const& params,
	                  float factor)
	{
		if (!std::isfinite(factor) || !(factor > 0.0f))
			return;
		float v = out.resolution_depth * factor;
		if (!std::isfinite(v) || v <= 0.0f)
			return;
		out.modulate_factor = factor;
		out.resolution_continuous = std::clamp(v, params.v_min, params.v_max);
		out.resolution = params.quantize ? quantize_resolution(out.resolution_continuous)
		                                : out.resolution_continuous;
		out.curvature_applied = true;
	}

	/** v1：indoor_score 对称调制 */
	void apply_curvature_modulate_indoor_score(PercentileResolutionResult& out,
	                                           PercentileResolutionParams const& params)
	{
		float const sigma_ref =
			(params.curvature_log_std_ref > 0.0f && std::isfinite(params.curvature_log_std_ref))
				? params.curvature_log_std_ref
				: 1.5f;
		float const sigma_tilde = std::clamp(out.curv_log_std / sigma_ref, 0.0f, 1.0f);

		float w = params.curvature_plane_weight;
		if (!std::isfinite(w))
			w = 0.6f;
		w = std::clamp(w, 0.0f, 1.0f);
		out.indoor_score = std::clamp(w * out.p_plane + (1.0f - w) * (1.0f - sigma_tilde), 0.0f, 1.0f);

		float alpha = params.curvature_modulate_alpha;
		if (!std::isfinite(alpha) || alpha < 0.0f)
			alpha = 0.0f;
		alpha = std::min(alpha, 1.0f);

		float const factor = 1.0f + alpha * (1.0f - 2.0f * out.indoor_score);
		if (factor > 1.0f + 1e-6f)
			out.scene_class = VoxelSceneClass::OutdoorExcess;
		else if (factor < 1.0f - 1e-6f)
			out.scene_class = VoxelSceneClass::IndoorShrink;
		else
			out.scene_class = VoxelSceneClass::Neutral;
		apply_factor(out, params, factor);
	}

	/** v2：非平面超额放大；高平面室内缩小；中间带保持 v0 */
	void apply_curvature_modulate_nonplanar_excess(PercentileResolutionResult& out,
	                                               PercentileResolutionParams const& params)
	{
		float p0 = params.nonplanar_excess_p0;
		if (!std::isfinite(p0))
			p0 = 0.70f;
		p0 = std::clamp(p0, 0.05f, 0.95f);

		float indoor_thr = params.nonplanar_indoor_plane_thr;
		if (!std::isfinite(indoor_thr))
			indoor_thr = 0.60f;
		indoor_thr = std::clamp(indoor_thr, 0.05f, 0.95f);

		float gamma = params.nonplanar_excess_gamma;
		if (!std::isfinite(gamma) || gamma < 1.0f)
			gamma = 1.0f;

		float alpha_out = params.nonplanar_excess_alpha;
		if (!std::isfinite(alpha_out) || alpha_out < 0.0f)
			alpha_out = 0.0f;
		alpha_out = std::min(alpha_out, 1.0f);

		float alpha_in = params.nonplanar_indoor_alpha;
		if (!std::isfinite(alpha_in) || alpha_in < 0.0f)
			alpha_in = 0.0f;
		alpha_in = std::min(alpha_in, 1.0f);

		out.indoor_score = out.p_plane; // v2 示意：平面占比即室内倾向

		float factor = 1.0f;
		if (out.p_plane >= indoor_thr) {
			float const denom = std::max(1.0f - indoor_thr, 1e-3f);
			float t = std::clamp((out.p_plane - indoor_thr) / denom, 0.0f, 1.0f);
			t = std::pow(t, gamma);
			factor = 1.0f - alpha_in * t;
			out.scene_class = VoxelSceneClass::IndoorShrink;
		} else if (out.p_nonplanar > p0) {
			float const denom = std::max(1.0f - p0, 1e-3f);
			float t = std::clamp((out.p_nonplanar - p0) / denom, 0.0f, 1.0f);
			t = std::pow(t, gamma);
			factor = 1.0f + alpha_out * t;
			out.scene_class = VoxelSceneClass::OutdoorExcess;
		} else {
			// 中间带 factor=1，保持深度 v0
			out.scene_class = VoxelSceneClass::Neutral;
		}

		apply_factor(out, params, factor);
	}

	void apply_curvature_modulate(PercentileResolutionResult& out,
	                              std::vector<float> const& curvatures,
	                              PercentileResolutionParams const& params)
	{
		if (!params.use_curvature_modulate || curvatures.size() < 10)
			return;
		if (!std::isfinite(out.resolution_depth) || !(out.resolution_depth > 0.0f))
			return;

		fill_curvature_stats(out, curvatures, params.curvature_plane_thr);
		out.curvature_mode = params.curvature_modulate_mode;

		if (params.curvature_modulate_mode == CurvatureModulateMode::kNonplanarExcess)
			apply_curvature_modulate_nonplanar_excess(out, params);
		else
			apply_curvature_modulate_indoor_score(out, params);
	}

	void log_result(PercentileResolutionResult const& out,
	                std::string const& name,
	                PercentileResolutionParams const& params,
	                size_t n_samples,
	                float depth_cap)
	{
		LOG_INFO("");
		LOG_INFO("***************** percentile resolution {} *****************",
		         name.empty() ? std::string("estimate") : name);
		LOG_INFO(" samples : {}  depth_cap : {}", n_samples, depth_cap);
		LOG_INFO(" r50={:.3f}  r75={:.3f}  r90={:.3f}  r_char={:.3f}",
		         out.r50, out.r75, out.r90, out.r_char);
		LOG_INFO(" depth_v0={:.4f}  continuous={:.4f}  resolution={:.4f}  (k={}, quantize={})",
		         out.resolution_depth, out.resolution_continuous, out.resolution, params.scale_k,
		         params.quantize ? 1 : 0);
		if (out.curvature_applied) {
			char const* mode =
				(out.curvature_mode == CurvatureModulateMode::kNonplanarExcess)
					? "nonplanar_excess"
					: "indoor_score";
			LOG_INFO(
				" curvature[{}]: p_plane={:.3f}  p_np={:.3f}  log_std={:.3f}  "
				"indoor_score={:.3f}  factor={:.4f}",
				mode, out.p_plane, out.p_nonplanar, out.curv_log_std, out.indoor_score,
				out.modulate_factor);
		} else {
			LOG_INFO(" curvature: not applied");
		}
		LOG_INFO(" scene_class={}", voxelSceneClassToString(out.scene_class));
		LOG_INFO("----------------- percentile resolution end -----------------");
		LOG_INFO("");
	}

	PercentileResolutionResult estimate_from_depths(std::vector<float>& depths,
	                                                std::vector<float> const* curvatures,
	                                                std::string const& name,
	                                                PercentileResolutionParams params)
	{
		PercentileResolutionResult out;
		if (depths.size() < 10)
			return out;

		normalize_weights(params);
		float const cap = (params.depth_cap > 0.0f && std::isfinite(params.depth_cap))
			                  ? params.depth_cap
			                  : std::numeric_limits<float>::max();
		for (float& d : depths) {
			if (d > cap)
				d = cap;
		}

		std::sort(depths.begin(), depths.end());
		out.r50 = percentile_sorted(depths, 0.50f);
		out.r75 = percentile_sorted(depths, 0.75f);
		out.r90 = percentile_sorted(depths, 0.90f);
		out.r_char = params.weight_p50 * out.r50
		           + params.weight_p75 * out.r75
		           + params.weight_p90 * out.r90;

		float v = params.scale_k * out.r_char;
		if (!std::isfinite(v) || v <= 0.0f)
			return PercentileResolutionResult{};

		out.resolution_depth = std::clamp(v, params.v_min, params.v_max);
		out.resolution_continuous = out.resolution_depth;
		out.resolution = params.quantize ? quantize_resolution(out.resolution_continuous)
		                                : out.resolution_continuous;

		if (curvatures != nullptr)
			apply_curvature_modulate(out, *curvatures, params);

		log_result(out, name, params, depths.size(), cap);
		return out;
	}

	template <typename GetDepth>
	void collect_depths(size_t n, size_t max_samples, GetDepth&& get_depth,
	                    std::vector<float>& depths)
	{
		depths.clear();
		if (n == 0)
			return;

		size_t const stride = sample_stride(n, max_samples);
		depths.reserve((n + stride - 1) / stride);
		for (size_t i = 0; i < n; i += stride) {
			float const d = get_depth(i);
			if (std::isfinite(d) && d > 0.0f)
				depths.push_back(d);
		}
	}

	template <typename GetDepth, typename GetCurvature>
	void collect_depths_and_curvatures(size_t n, size_t max_samples,
	                                   GetDepth&& get_depth,
	                                   GetCurvature&& get_curvature,
	                                   std::vector<float>& depths,
	                                   std::vector<float>& curvatures)
	{
		depths.clear();
		curvatures.clear();
		if (n == 0)
			return;

		size_t const stride = sample_stride(n, max_samples);
		size_t const approx = (n + stride - 1) / stride;
		depths.reserve(approx);
		curvatures.reserve(approx);
		for (size_t i = 0; i < n; i += stride) {
			float const d = get_depth(i);
			if (!std::isfinite(d) || !(d > 0.0f))
				continue;
			depths.push_back(d);
			curvatures.push_back(get_curvature(i));
		}
	}

	} // namespace

	float
	PercentileResolutionEstimate::estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	                                       std::string const& name,
	                                       PercentileResolutionParams const& params)
	{
		return estimateDetailed(cloud, name, params).resolution;
	}

	float
	PercentileResolutionEstimate::estimate(std::vector<Eigen::Vector3f> const& cloud,
	                                       std::string const& name,
	                                       PercentileResolutionParams const& params)
	{
		return estimateDetailed(cloud, name, params).resolution;
	}

	float
	PercentileResolutionEstimate::estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	                                       pcl::PointCloud<pcl::Normal>::Ptr normals,
	                                       std::string const& name,
	                                       PercentileResolutionParams const& params)
	{
		return estimateDetailed(cloud, normals, name, params).resolution;
	}

	PercentileResolutionResult
	PercentileResolutionEstimate::estimateDetailed(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		std::string const& name,
		PercentileResolutionParams const& params)
	{
		if (!cloud || cloud->size() < 10)
			return {};

		std::vector<float> depths;
		collect_depths(cloud->size(), params.max_samples,
		               [&](size_t i) { return cloud->points[i].getVector3fMap().norm(); },
		               depths);
		return estimate_from_depths(depths, nullptr, name, params);
	}

	PercentileResolutionResult
	PercentileResolutionEstimate::estimateDetailed(
		std::vector<Eigen::Vector3f> const& cloud,
		std::string const& name,
		PercentileResolutionParams const& params)
	{
		if (cloud.size() < 10)
			return {};

		std::vector<float> depths;
		collect_depths(cloud.size(), params.max_samples,
		               [&](size_t i) { return cloud[i].norm(); },
		               depths);
		return estimate_from_depths(depths, nullptr, name, params);
	}

	PercentileResolutionResult
	PercentileResolutionEstimate::estimateDetailed(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::Normal>::Ptr normals,
		std::string const& name,
		PercentileResolutionParams const& params)
	{
		if (!cloud || cloud->size() < 10)
			return {};

		bool const normals_ok = normals
			&& !normals->empty()
			&& normals->size() == cloud->size()
			&& params.use_curvature_modulate;

		if (!normals_ok)
			return estimateDetailed(cloud, name, params);

		std::vector<float> depths;
		std::vector<float> curvatures;
		collect_depths_and_curvatures(
			cloud->size(), params.max_samples,
			[&](size_t i) { return cloud->points[i].getVector3fMap().norm(); },
			[&](size_t i) { return normals->points[i].curvature; },
			depths, curvatures);
		return estimate_from_depths(depths, &curvatures, name, params);
	}

} // namespace Ddx
