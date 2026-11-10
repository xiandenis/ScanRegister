#include "registration_type.hpp"

namespace Ddx {

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

ScanInfo::ScanInfo()
	: name_("")
	, stamp_(0)
	, lasFn_("")
	, normalFn_("")
	, json_("")
	, cache_in_source_(false)
	, sensor_(SENSOR_STAND | SENSOR_INCLINOMETER)
	, altimeter_(0)
	, gps_(Eigen::Vector3d::Zero())
	, angle_resolution_(0)
	, global_matrix_(Eigen::Matrix4d::Identity())
{
}

VoxelRoot::VoxelRoot()
	: name_("Root")
	, cloud_path_("")
	, normal_path_("")
	, state_(INITIAL)
	, point_number_(0)
	, is_raw_input_(false)
{
}

SubVoxel::SubVoxel()
	: state_(INITIAL)
	, name_("")
	, point_number_(0)
	, voxel_size_(0)
{
}

LidarFrame::LidarFrame()
	: ScanInfo()
	, state_(INITIAL)
	, point_number_(0)
	, scene_scale_(-1)
	, voxel_size_(0.3)
	, voxel_scene_class_(VoxelSceneClass::Unknown)
	, info_folder_("")
	, root_()
{
}

LidarFrame::LidarFrame(ScanInfo const& scan_info)
	: ScanInfo(scan_info)
	, state_(INITIAL)
	, point_number_(0)
	, scene_scale_(-1)
	, voxel_size_(0.3)
	, voxel_scene_class_(VoxelSceneClass::Unknown)
	, info_folder_("")
	, root_()
{
}

bool LidarFrame::hasRoot() const
{
	return (root_.state_ & HAS_CALCU) == HAS_CALCU
		|| !root_.cloud_path_.empty();
}

void LidarFrame::setScanInfo(ScanInfo const& info)
{
	name_ = info.name_;
	stamp_ = info.stamp_;
	lasFn_ = info.lasFn_;
	normalFn_ = info.normalFn_;
	json_ = info.json_;
	sensor_ = info.sensor_;
	altimeter_ = info.altimeter_;
	gps_ = info.gps_;
	angle_resolution_ = info.angle_resolution_;
	global_matrix_ = info.global_matrix_;
}

std::string LidarFrame::getInfoPath() const
{
	if (json_.size() <= 1) {
		return info_folder_ + "/source.info";
	}
	return json_;
}

std::string LidarFrame::getCloudPath(std::string const& name) const
{
	if (name == "Raw") {
		return lasFn_;
	}
	if (name == "Root") {
		if (!root_.cloud_path_.empty()) {
			return root_.cloud_path_;
		}
		if (root_.is_raw_input_) {
			return lasFn_;
		}
		return info_folder_ + "/Root_p.pcd";
	}
	for (size_t i = 0; i < subvoxel_vec_.size(); ++i) {
		if (subvoxel_vec_[i].name_ == name) {
			return info_folder_ + "/" + subvoxel_vec_[i].name_ + "_p.pcd";
		}
	}
	return "";
}

std::string LidarFrame::getNormalPath(std::string const& name) const
{
	if (name == "Raw") {
		return normalFn_;
	}
	if (name == "Root") {
		if (!root_.normal_path_.empty()) {
			return root_.normal_path_;
		}
		if (root_.is_raw_input_) {
			return normalFn_;
		}
		return info_folder_ + "/Root_n.pcd";
	}
	for (size_t i = 0; i < subvoxel_vec_.size(); ++i) {
		if (subvoxel_vec_[i].name_ == name) {
			return info_folder_ + "/" + subvoxel_vec_[i].name_ + "_n.pcd";
		}
	}
	return "";
}

int LidarFrame::findSubVoxelIndex(std::string const& node_name) const
{
	for (size_t i = 0; i < subvoxel_vec_.size(); ++i) {
		if (subvoxel_vec_[i].name_ == node_name) {
			return static_cast<int>(i);
		}
	}
	return -1;
}

SubVoxel LidarFrame::getSubVoxel(std::string const& node_name) const
{
	for (const auto& subvoxel : subvoxel_vec_) {
		if (subvoxel.name_ == node_name) {
			return subvoxel;
		}
	}
	return SubVoxel();
}

AlignResult::AlignResult()
	: state_(INITIAL)
	, src_name_("")
	, tgt_name_("")
	, matrix_(Eigen::Matrix4d::Identity())
	, similarity_(0.0)
	, rms_(-1.0)
	, final_rms_(-1.0)
	, weight_(1.0)
	, feature_num_(0)
	, auto_reg_(0)
	, information_(Eigen::Matrix<double, 6, 6>::Identity())
{
}

void AlignResult::reset()
{
	state_ = INITIAL;
	matrix_ = Eigen::Matrix4d::Identity();
	similarity_ = 0.0;
	rms_ = -1.0;
	final_rms_ = -1.0;
	weight_ = 1.0;
	feature_num_ = 0;
	information_ = Eigen::Matrix<double, 6, 6>::Identity();
}

void AlignResult::resetState()
{
	state_ = INITIAL;
}

void AlignResult::resetAll()
{
	reset();
	auto_reg_ = 0;
}

StationStatus::StationStatus()
	: name_("")
	, state_("")
	, phase_("")
	, ref_("")
	, phase_started_at_("")
	, message_("")
	, init_total_s_(-1.0)
	, reg_total_s_(-1.0)
	, rms_(-1.0)
	, similarity_(-1.0)
{
}

} // namespace Ddx
