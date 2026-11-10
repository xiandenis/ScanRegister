#include "registration_initializer.hpp"
#include "angle_downsample.hpp"
#include <file_system/file_system.hpp>
#include <common/common.hpp>
#include <common/panorama_util.hpp>
#include <depth_map_tool/depth_map_tool.hpp>
#include <point_cloud_io/point_cloud_io.hpp>
#include <point_cloud_io/pcse_reader.hpp>
#include <resolution_estimate/percentile_resolution_estimate.hpp>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <registration_type/lidar_frame_io.hpp>
#include <octree/octree_downsample.hpp>
#include <logging/logging.hpp>
#include <logging/tic_toc.hpp>
#include <logging/mem_peak.hpp>

#include <filesystem>
#include <system_error>


namespace Ddx
{
using pcl_type::PointCloud;
using pcl_type::PointCloudPtr;
using pcl_type::Normal;
using pcl_type::NormalPtr;
	bool
	RegistrationInitializer::initalState(LidarFrame &frame, int resolution, bool enforcement)
	{

        if (enforcement == false) {
            if (Ddx::LidarFrameIO::loadLidarFrame(frame.getInfoPath(), frame) == true) {
                return true;
            }
        }

		// PCSE 是数据集目录，不是普通点云文件；file_exists 对目录为 false。
		if (!stlplus::file_exists(frame.lasFn_)
			&& !point_cloud_io::is_pcse_dataset(frame.lasFn_)) {
			LOG_ERROR(" lasFn({}) not exists ", frame.lasFn_);
			return false;
		}

		{
			std::error_code ec;
			std::filesystem::create_directories(frame.info_folder_, ec);
		}
		frame.subvoxel_vec_.clear();
        frame.root_ = VoxelRoot();
        _initalRoot(frame);

		_initalVoxel(resolution, frame);
       
		frame.state_ = Ddx::INITIAL;
		Ddx::LidarFrameIO::saveLidarFrame(frame.getInfoPath(), frame);

		return true;
	}

	bool 
	RegistrationInitializer::initalState(LidarFrame &frame, std::vector<int> const& resolution_vec, bool enforcement)
	{
		if (enforcement == false) {
			if (Ddx::LidarFrameIO::loadLidarFrame(frame.getInfoPath(), frame) == true) {
				return true;
			}
		}

		if (!stlplus::file_exists(frame.lasFn_)
			&& !point_cloud_io::is_pcse_dataset(frame.lasFn_)) {
			LOG_ERROR(" lasFn({}) not exists ", frame.lasFn_);
			return false;
		}
		{
			std::error_code ec;
			std::filesystem::create_directories(frame.info_folder_, ec);
		}
		frame.subvoxel_vec_.clear();
		frame.root_ = VoxelRoot();
		_initalRoot(frame);

		for (int i = 0; i < static_cast<int>(resolution_vec.size()); ++i) {
			_initalVoxel(resolution_vec[i], frame);
		}

		frame.state_ = Ddx::INITIAL;
		Ddx::LidarFrameIO::saveLidarFrame(frame.getInfoPath(), frame);
		return true;
	}

	bool
	RegistrationInitializer::initializd(LidarFrame& frame, bool mul_thread)
	{
		std::string const scope_init = "initializd " + frame.name_;
		TicToc time(scope_init);
		MemPeakScope mem_init(scope_init);
		m_last_cloud.reset(new PointCloud());
		m_last_normal.reset(new Normal());
		m_root_filtered_point_number_ = 0;

		if (frame.subvoxel_vec_.empty()) {
			LOG_WARN(" {} subvoxel_vec_ is empty, call initalState first! ", frame.name_);
			return false;
		}

		// 站心必须用户声明；禁止从文件猜测。折系后 Root/子体素缓存均为站心系。
		if (!frame.has_station_origin_) {
			LOG_ERROR(
				" {} has_station_origin_ is false; declare station_origin_ in file CRS before init ",
				frame.name_);
			return false;
		}

		{
			std::error_code ec;
			std::filesystem::create_directories(frame.info_folder_, ec);
		}

		{
			std::string const scope_load = "load " + frame.name_;
			TicToc t_load(scope_load);
			MemPeakScope mem_load(scope_load);
			Ddx::point_cloud_io::FramedCloud<pcl_type::PointType> framed;
			if (Ddx::point_cloud_io::load(frame.lasFn_, framed) != 0 || framed.cloud.size() < 10) {
				LOG_WARN(" {} load point cloud fail! ", frame.name_);
				return false;
			}
			// p_station = local + FramedCloud.origin - station_origin_ (file CRS).
			Eigen::Vector3d const& o = framed.origin;
			Eigen::Vector3d const& s = frame.station_origin_;
			for (auto& p : framed.cloud.points) {
				p.x = static_cast<float>(static_cast<double>(p.x) + o.x() - s.x());
				p.y = static_cast<float>(static_cast<double>(p.y) + o.y() - s.y());
				p.z = static_cast<float>(static_cast<double>(p.z) + o.z() - s.z());
			}
			*m_last_cloud = std::move(framed.cloud);
		}
		frame.point_number_ = static_cast<int>(m_last_cloud->size());

		if (_calculateRoot(frame, mul_thread) == false) {
            LOG_WARN(" {} calculateRoot failed! ", frame.name_);
            return false;
        }

		for (int i = 0; i < static_cast<int>(frame.subvoxel_vec_.size()); ++i) {
			if (_cacalVoxel(frame, i) == false) {
				LOG_WARN(" {} cacalVoxel({}) failed! ", frame.name_, i);
				return false;
			}
		}

		{
			std::string const scope_res = "resolution " + frame.name_;
			TicToc t_res(scope_res);
			MemPeakScope mem_res(scope_res);
			if (_resolution(frame) == false) {
				LOG_WARN("{} resolution estimate fail!", frame.name_);
			}
		}

		frame.state_ = Ddx::HAS_CALCU;
		LidarFrameIO::saveLidarFrame(frame.getInfoPath(), frame);
		return true;
	}

	bool
	RegistrationInitializer::check(LidarFrame const& frame)
	{
		if (frame.state_ == Ddx::INITIAL)
			return false;

		if (frame.point_number_ <= 10)
			return false;

		if (frame.hasRoot()) {
			if (_checkRoot(frame) == false)
				return false;
		}

		for (int i = 0; i < static_cast<int>(frame.subvoxel_vec_.size()); ++i) {
			if (_checkVoxel(frame, frame.subvoxel_vec_[i]) == false)
				return false;
		}

		return true;
	}

	bool
	RegistrationInitializer::_initalVoxel(int resolution, LidarFrame &frame)
	{
		SubVoxel Voxel;
		Voxel.name_ = Ddx::Common::getVoxelName(resolution);
		Voxel.point_number_ = 0;
		Voxel.voxel_size_ = resolution / double(100);
		Voxel.state_ = Ddx::INITIAL;
		frame.subvoxel_vec_.push_back(Voxel);
		return true;
	}

	bool
	RegistrationInitializer::_initalRoot(LidarFrame &frame)
	{
        frame.root_= VoxelRoot();

        if (m_use_root) {
            frame.root_.cloud_path_ = frame.info_folder_ + "/Root_p.pcd";
            frame.root_.normal_path_ = frame.info_folder_ + "/Root_n.pcd"; // 预留，当前不写出
            frame.root_.point_number_ = 0;
            frame.root_.is_raw_input_ = false;
        }
        else {
            frame.root_.cloud_path_ = frame.lasFn_;
            frame.root_.normal_path_ = frame.normalFn_;
            frame.root_.point_number_ = frame.point_number_;
            frame.root_.is_raw_input_ = true; 
        }
		
		return true;
	}

	bool
	RegistrationInitializer::_cacalVoxel(LidarFrame& frame, int sub_voxel_index)
	{
		if (sub_voxel_index < 0 || sub_voxel_index >= static_cast<int>(frame.subvoxel_vec_.size())) {
			LOG_ERROR(" {} sub voxel index({}) out of range! ", frame.name_, sub_voxel_index);
			return false;
		}

		if (m_last_cloud == nullptr || m_last_cloud->size() < 10) {
			LOG_WARN(" {} last cloud too few for sub voxel {}! ", frame.name_, sub_voxel_index);
			return false;
		}

		SubVoxel& voxel = frame.subvoxel_vec_[sub_voxel_index];
		std::string const scope_voxel = "voxel " + frame.name_ + " " + voxel.name_;
		TicToc t_voxel(scope_voxel);
		MemPeakScope mem_voxel(scope_voxel);

		std::vector<int> ds_indices;
		{
			std::string const scope_medoid = "medoid " + frame.name_ + " " + voxel.name_;
			TicToc t_medoid(scope_medoid);
			MemPeakScope mem_medoid(scope_medoid);
			if (!Ddx::octree::downsampleMedoidIndices<pcl_type::PointType>(m_last_cloud, static_cast<float>(voxel.voxel_size_),
			                                          ds_indices, true))
			{
				return false;
			}
		}

		PointCloudPtr cloud(new PointCloud());
		pcl::copyPointCloud(*m_last_cloud, ds_indices, *cloud);
		if (cloud->size() < 2) {
			LOG_WARN(" {} sub voxel {} downsampled cloud too few! ", frame.name_, sub_voxel_index);
			return false;
		}
		voxel.point_number_ = static_cast<int>(cloud->size());

		std::string cloud_path = frame.getCloudPath(voxel.name_);
		if (pcl::io::savePCDFileBinary(cloud_path, *cloud) != 0) {
			return false;
		}
		if (!stlplus::file_exists(cloud_path)) {
			return false;
		}

		// voxel_3 法向落盘留给后续高精度 ICP；当前粗/精配准只读 *_p.pcd。
		// 若跳过本级估向（仅链式拷贝到 voxel_10）：黄陂约 0.40–0.52 s/站、5 站合计 ~2.3 s
		// （占 voxel_3 整段 ~27%、registerScan ~8.5%）；佛山岭南约 0.13–0.39 s/站、合计 ~1.3 s。
		NormalPtr normal(new Normal());
		if (m_last_normal != nullptr && !m_last_normal->empty()
			&& m_last_normal->size() == m_last_cloud->size()) {
			pcl::copyPointCloud(*m_last_normal, ds_indices, *normal);
		}
		if (normal->empty() || normal->size() != cloud->size()) {
			std::string const scope_normal = "normal " + frame.name_ + " " + voxel.name_;
			TicToc t_normal(scope_normal);
			MemPeakScope mem_normal(scope_normal);
			if (_calculateNormal(cloud, normal) == false) {
				return false;
			}
		}

		std::string normal_path = frame.getNormalPath(voxel.name_);
		if (pcl::io::savePCDFileBinary(normal_path, *normal) != 0) {
			return false;
		}
		if (!stlplus::file_exists(normal_path)) {
			return false;
		}

		m_last_cloud = cloud;
		m_last_normal = normal;
		voxel.state_ |= Ddx::HAS_CALCU;
		return true;
	}

	bool 
	RegistrationInitializer::_calculateRoot(LidarFrame& frame, bool mul_thread)
	{
		if (!m_use_root) {
			frame.root_.state_ = Ddx::HAS_CALCU;
			frame.root_.cloud_path_ = frame.lasFn_;
			frame.root_.normal_path_ = frame.normalFn_;
			frame.root_.point_number_ = frame.point_number_;
			frame.root_.is_raw_input_ = true;
			m_root_filtered_point_number_ = frame.point_number_;
			return true;
		}

		PointCloudPtr filtered(new PointCloud());
		{
			std::string const scope_filter = "root_filter " + frame.name_;
			TicToc t_filter(scope_filter);
			MemPeakScope mem_filter(scope_filter);
			if (mul_thread) {
				if (AngleDownsample::downsampling(m_last_cloud, filtered, m_resolutionX, m_resolutionY) == false) {
					return false;
				}
			}
			else {
				cv::Mat depth_map = Ddx::DepthMapTool::depth_create(m_last_cloud, m_resolutionX, m_resolutionY);
				if (depth_map.empty()) {
					return false;
				}
				const float* src_data = reinterpret_cast<const float*>(depth_map.data);
				std::vector<int> indices;
				indices.reserve(m_last_cloud->size());
				for (int i = 0; i < static_cast<int>(m_last_cloud->size()); ++i) {
					Eigen::Vector3f p = m_last_cloud->points[i].getVector3fMap();
					Eigen::Vector2i pixel = Ddx::PanoramaUtil::math_pixel_from_xyz(p, m_resolutionX, m_resolutionY);
					if (pixel[0] < 0 || pixel[0] >= m_resolutionX || pixel[1] < 0 || pixel[1] >= m_resolutionY) {
						continue;
					}
					float depth = src_data[pixel[1] * m_resolutionX + pixel[0]];
					float depth_2 = p.norm();
					if (std::abs(depth - depth_2) < 0.0001f) {
						indices.push_back(i);
					}
				}
				if (indices.empty()) {
					return false;
				}
				pcl::copyPointCloud(*m_last_cloud, indices, *filtered);
			}
		}

		if (filtered->size() < 10) {
			LOG_WARN(" {} root filtered cloud too few! ", frame.name_);
			return false;
		}

		m_last_cloud = filtered;
		m_root_filtered_point_number_ = static_cast<int>(m_last_cloud->size());

		std::string cloud_path = frame.getCloudPath("Root");
		{
			std::string const scope_save = "root_save " + frame.name_;
			TicToc t_save(scope_save);
			MemPeakScope mem_save(scope_save);
			if (pcl::io::savePCDFileBinaryCompressed(cloud_path, *m_last_cloud) != 0) {
				return false;
			}
		}
		if (!stlplus::file_exists(cloud_path)) {
			return false;
		}

		frame.root_.cloud_path_ = cloud_path;
		frame.root_.state_ = Ddx::HAS_CALCU;
		frame.root_.point_number_ = static_cast<int>(m_last_cloud->size());
		frame.root_.is_raw_input_ = false;
		return true;
	}




	bool 
	RegistrationInitializer::_calculateNormal(PointCloudPtr cloud, NormalPtr& normal)
	{
		LOG_INFO("normal_estimate_method={}",
		         normalEstimateMethodName(m_normal_estimate_method));
		return estimateNormalsKnn(cloud, normal, m_normal_estimate_method);
	}

	bool 
	RegistrationInitializer::_resolution(LidarFrame& frame)
	{
		if (m_last_cloud == nullptr || m_last_cloud->size() < 10) {
			return false;
		}

		// 压缩比用原始点数 / Root 筛选后点数；use_root 时 root_.point_number_ 等于筛选后点数，不能作分子。
		if ((frame.root_.state_ & Ddx::HAS_CALCU) == Ddx::HAS_CALCU) {
			const int filtered_size = m_root_filtered_point_number_;
			if (filtered_size > 0 && frame.point_number_ > filtered_size) {
				const int times = frame.point_number_ / filtered_size;

				if (times >= 100) {
					frame.voxel_size_ = 0.1f;
					frame.voxel_scene_class_ = VoxelSceneClass::Unknown;
					return true;
				}
				if (times >= 50) {
					frame.voxel_size_ = 0.2f;
					frame.voxel_scene_class_ = VoxelSceneClass::Unknown;
					return true;
				}
			}
		}

		// 末级子体素：深度分位数 + v2 非平面超额调制（旧 kIndoorScore 接口仍可用）。
		PercentileResolutionParams params = PercentileResolutionParams::NonplanarExcess();
		params.quantize = false;
		PercentileResolutionResult const detailed =
			PercentileResolutionEstimate::estimateDetailed(
				m_last_cloud, m_last_normal, frame.name_, params);
		if (std::isnan(detailed.resolution)) {
			return false;
		}
		frame.voxel_size_ = std::max(0.15f, detailed.resolution);
		frame.voxel_scene_class_ = detailed.scene_class;
		return true;
	}

	bool
	RegistrationInitializer::_checkVoxel(LidarFrame const& frame, SubVoxel const& voxel)
	{
		if ((voxel.state_ & Ddx::HAS_CALCU) != Ddx::HAS_CALCU)
			return false;

		if (voxel.point_number_ <= 1)
			return false;

		if (stlplus::file_exists(frame.getCloudPath(voxel.name_)) == false)
			return false;

		if (stlplus::file_exists(frame.getNormalPath(voxel.name_)) == false)
			return false;

		return true;
	}

	bool
	RegistrationInitializer::_checkRoot(LidarFrame const& frame)
	{
		if ((frame.root_.state_ & Ddx::HAS_CALCU) != Ddx::HAS_CALCU)
			return false;

		if (frame.root_.point_number_ <= 1)
			return false;

		if (stlplus::file_exists(frame.getCloudPath("Root")) == false)
			return false;

		return true;
	}
}

