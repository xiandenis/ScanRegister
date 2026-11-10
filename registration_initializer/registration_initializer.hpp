#pragma once

#include <registration_type/registration_type.hpp>
#include <registration_type/pcl_type.hpp>
#include "normal_estimate.hpp"

namespace Ddx {


	class RegistrationInitializer
	{
	public:
		RegistrationInitializer()
		: m_resolutionX(5000)
		, m_resolutionY(2500)
		, m_use_root(true)
		, m_root_filtered_point_number_(0)
		, m_normal_estimate_method(NormalEstimateMethod::SmallGicpTbb){};

		~RegistrationInitializer() {};
        void setUseRoot(bool use_root) { m_use_root = use_root; }
		void setNormalEstimateMethod(NormalEstimateMethod method)
		{
			m_normal_estimate_method = method;
		}
		NormalEstimateMethod getNormalEstimateMethod() const
		{
			return m_normal_estimate_method;
		}

		bool initalState(LidarFrame &frame, int resolution = 10, bool enforcement = false);
		bool initalState(LidarFrame &frame, std::vector<int> const& resolution_vec, bool enforcement = false);

		bool initializd(LidarFrame& frame, bool mul_thread = true);

		bool check(LidarFrame const& frame);
	private:
		bool _initalVoxel(int resolution, LidarFrame &frame);

		/* 这是一个固定分辨率的下采样 */
		bool _initalRoot(LidarFrame &frame);

		/* 计算子体素（Medoid 下采样 + 法向）。voxel_3 法向预留给高精度 ICP，勿为加速而跳过。 */
		bool _cacalVoxel(LidarFrame& frame, int sub_voxel_index);

		/* 计算根体素 */
		bool _calculateRoot(LidarFrame& frame, bool mul_thread = true);

		bool _calculateNormal(pcl_type::PointCloudPtr cloud, pcl_type::NormalPtr& normal);
		
		bool _resolution(LidarFrame& frame);

		bool _checkVoxel(LidarFrame const& frame, SubVoxel const& voxel);

		bool _checkRoot(LidarFrame const& frame);
	private:
		pcl_type::PointCloudPtr m_last_cloud;	// 上个子体素的点云（用于下采样）旧版的m_raw_cloud
		pcl_type::NormalPtr m_last_normal;	// 上个体素的法向量（用于下采样）  旧版的m_raw_normal
		int m_root_filtered_point_number_;		// Root 角度深度筛选后的点数（供 _resolution 压缩比）

		int m_resolutionX;//= 5120;		// 分辨率12
		int m_resolutionY;// = 2560;	// 分辨率12

        /* 是否使用根体素，用于大数据输入时的角度深度筛选以节约内存。
         * m_use_root=true：写 Root_p.pcd；root_.normal_path_ 仅预留，当前不写 Root_n.pcd。
         * m_use_root=false：cloud_path_/normal_path_ 指向 lasFn_/normalFn_（原输入）。
        */
        bool m_use_root;
		NormalEstimateMethod m_normal_estimate_method;
	};
}
