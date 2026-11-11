#pragma once
#include <Eigen/Dense>
#include <basic_icp/basic_icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Ddx {

/**
 * TrimIcpSolver — 底层求解（假定输入已是工作子集；不做重叠裁剪 / 法向预筛）。
 *
 * 仅点云：Method=`RICP`（默认）等点到点，无需法向。
 * 点到面 Method：需要目标法向（及按内核要求的源法向）。
 * BasicICP 为 SLAM 同源：无正交 NC / max_iter / eps setter。
 */
class TrimIcpSolver
{
private:
	typedef pcl::PointXYZ PointType;
	typedef pcl::Normal NormalType;
	typedef pcl::PointCloud<PointType> PointCloud;
	typedef pcl::PointCloud<NormalType> NormalCloud;

public:
	TrimIcpSolver()
		: m_src_points(nullptr)
		, m_src_normals(nullptr)
		, m_tgt_points(nullptr)
		, m_tgt_normals(nullptr)
		, m_transformation(Eigen::Matrix4d::Identity())
		, m_method(Ddx::registration::BasicICP::Method::RICP)
	{
	}

	void setSourceCloud(const PointCloud::Ptr& src_points) { m_src_points = src_points; }
	void setSourceNormals(const NormalCloud::Ptr& src_normals) { m_src_normals = src_normals; }
	void setTargetCloud(const PointCloud::Ptr& tgt_points) { m_tgt_points = tgt_points; }
	void setTargetNormals(const NormalCloud::Ptr& tgt_normals) { m_tgt_normals = tgt_normals; }

	void setMethod(Ddx::registration::BasicICP::Method method) { m_method = method; }
	void setProfile(bool enabled) { m_profile = enabled; }
	bool profile() const { return m_profile; }

	bool align(const Eigen::Matrix4d& initial_guess = Eigen::Matrix4d::Identity());

	/// Convenience: Method::SICPPPL then align (requires target normals).
	bool alignPPL(const Eigen::Matrix4d& initial_guess = Eigen::Matrix4d::Identity());

	Eigen::Matrix4d getFinalTransformation() const { return m_transformation; }

	int profileWelschStages() const { return m_profile_welsch_stages; }
	int profileIcpIters() const { return m_profile_icp_iters; }
	int profileClosestPasses() const { return m_profile_closest_passes; }
	int profileAaRejects() const { return m_profile_aa_rejects; }
	double profileInitMs() const { return m_profile_init_ms; }
	double profileClosestMs() const { return m_profile_closest_ms; }
	double profileUpdateMs() const { return m_profile_update_ms; }

private:
	bool checkCloud() const;

	PointCloud::Ptr m_src_points;
	NormalCloud::Ptr m_src_normals;
	PointCloud::Ptr m_tgt_points;
	NormalCloud::Ptr m_tgt_normals;
	Eigen::Matrix4d m_transformation;

	Ddx::registration::BasicICP::Method m_method;
	bool m_profile = false;
	int m_profile_welsch_stages = 0;
	int m_profile_icp_iters = 0;
	int m_profile_closest_passes = 0;
	int m_profile_aa_rejects = 0;
	double m_profile_init_ms = 0.0;
	double m_profile_closest_ms = 0.0;
	double m_profile_update_ms = 0.0;
};

} // namespace Ddx
