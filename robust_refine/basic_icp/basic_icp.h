#pragma once
#include <Eigen/Core>


namespace Ddx
{
	namespace registration
	{
		class BasicICP
		{
		public:
			typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EigenNormaCloud;
			typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EigenPointCloud;
	
			enum class Method 
			{
				ICP = 1,			// 传统的点到点icp
				AA_ICP,				// 通过安德鲁算法加速的点到点icp
				FICP,				// Fast ICP
				RICP,				// Robust ICP
				PPL,				// 面到面icp
				RPPL,				// Robust ICP point-to-plane
				SparseICP,			// Sparse ICP
				SICPPPL				// Sparse ICP point-to-plane
			};
		public:
			BasicICP() : m_method(Method::RICP), m_transformation_matrix(Eigen::Matrix4d::Identity()) {}

			void setSourceCloud(EigenPointCloud const& cloud) { m_sourceCloud = cloud; }
			void setSourceNormal(EigenPointCloud const& cloud) { m_sourceNormal = cloud;}

			void setTargetCloud(EigenPointCloud const& cloud) { m_targetCloud = cloud;}
			void setTargetNormal(EigenPointCloud const& cloud) { m_targetNormal = cloud; }

			void setMethod(Method method) { m_method = method; }
			/// 默认 false：不采集 FRICP 内分账（热路径零探测开销）。
			void setProfile(bool enabled) { m_profile = enabled; }
			bool profile() const { return m_profile; }

			bool align(Eigen::Matrix4d const& gnss);

			Eigen::Matrix4d getFinalTransformationMatrix() { return m_transformation_matrix; }

			int profileWelschStages() const { return m_profile_welsch_stages; }
			int profileIcpIters() const { return m_profile_icp_iters; }
			int profileClosestPasses() const { return m_profile_closest_passes; }
			int profileAaRejects() const { return m_profile_aa_rejects; }
			double profileInitMs() const { return m_profile_init_ms; }
			double profileClosestMs() const { return m_profile_closest_ms; }
			double profileUpdateMs() const { return m_profile_update_ms; }

		private:
			// 点云
			EigenPointCloud m_sourceCloud;
			EigenNormaCloud m_sourceNormal;

			EigenPointCloud m_targetCloud;
			EigenNormaCloud m_targetNormal;

			Method m_method;
			bool m_profile = false;
			int m_profile_welsch_stages = 0;
			int m_profile_icp_iters = 0;
			int m_profile_closest_passes = 0;
			int m_profile_aa_rejects = 0;
			double m_profile_init_ms = 0.0;
			double m_profile_closest_ms = 0.0;
			double m_profile_update_ms = 0.0;

			Eigen::Matrix4d m_transformation_matrix;
		};
	}

}



