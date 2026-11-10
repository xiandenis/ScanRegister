#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace Ddx {

// 传感器使用情况
typedef int SensorInfo;
enum
{
	OTHER_SENSOR = 0,			// 其他传感器/其它设备数据/以及跨源数据/地图（没有法向量数据）
	SENSOR_STAND = 1,			// 架站
	SENSOR_INCLINOMETER = 2,	// 倾角器
	SENSOR_ALTIMETER = 4,		// 高度计
	SENSOR_COMPASS = 8,			// 罗盘
	SENSOR_GPS = 16,			// GPS
	SENSOR_INS = 32,			// 惯导
};

// 扫描信息
struct ScanInfo
{
	std::string name_;				// 站名
	double stamp_;					// 时间戳 [s]；架站可用于分组
	std::string lasFn_;				// 原始点云路径
	std::string normalFn_;			// 法线文件路径（可空）
	std::string json_;				// 额外 JSON（可空）
	/// 为 true 且 lasFn_ 是文件夹时，初始化缓存放在该文件夹下。默认 false，缓存放在外部 data_info。
	bool cache_in_source_ = false;
	SensorInfo sensor_;				// 传感器位标志（见 SensorInfo）
	double altimeter_;				// 高度计读数 [m]（相对/绝对依设备约定）
	Eigen::Vector3d gps_;			// GPS / 地理坐标（设备约定；非本库配准坐标系）
	int angle_resolution_;			// 角分辨率编码；0=无；SLS 常见 3/6/12/24
	Eigen::Matrix4d global_matrix_;	// 世界←测站 4×4；平移单位 [m]

	ScanInfo();
};

// 过程记录状态
typedef int StateType;
enum
{
	INITIAL = 0,		// 初始状态
	VALID = 1,			// 有效
	HAS_CALCU = 2,		// 已经计算
	INVALID = 4,		// 无效状态
	PROCESSING = 8,		// 处理过程中
	MANUAL = 128,		// 手动
	WINDOWS = 256,		// win 平板
};

/// StateType 为位标志，可 OR 组合（如 VALID|HAS_CALCU）。禁止用 == VALID 判断「是否有效」。
inline bool stateHas(StateType s, StateType flag) { return (s & flag) == flag; }
inline bool stateIsValid(StateType s) { return stateHas(s, VALID); }
inline bool stateIsProcessing(StateType s) { return stateHas(s, PROCESSING); }
inline bool stateIsInitial(StateType s) { return s == INITIAL; }

struct VoxelRoot
{
	std::string name_;			// 名称（仅作标记，不参与路径拼接）
	std::string cloud_path_;	// 点云完整路径（非空时原样使用，不参与拼接）
	std::string normal_path_;	// 法向量完整路径（预留扩展；当前 Root 流程不写 Root_n.pcd）
	StateType state_;			// 状态
	int point_number_;			// 点数
	bool is_raw_input_;			// 是否原输入

	VoxelRoot();
};

struct SubVoxel
{
	StateType state_;		// 状态
	std::string name_;		// 名称（如 voxel_3）
	int point_number_;		// 点数
	double voxel_size_;		// 体素边长 [m]

	SubVoxel();
};

/** 体素估计曲率调制场景提示（仅记录，不参与 HNSW/feat_budget 分流）。 */
enum class VoxelSceneClass
{
	Unknown = 0,		// 未走曲率调制 / 压缩比早退等
	IndoorShrink,		// v2：高平面 → 室内缩小
	OutdoorExcess,		// v2：非平面超额 → 放大
	Neutral,			// v2：中间带，factor=1
};

char const* voxelSceneClassToString(VoxelSceneClass c);
VoxelSceneClass voxelSceneClassFromString(std::string const& s);

struct LidarFrame : public ScanInfo
{
	StateType state_;						// 状态
	int point_number_;						// 点数
	double scene_scale_;					// 场景尺度因子 [1]（矿山等；1=不缩放）
	double voxel_size_;						// 主配准体素边长 [m]
	/** 体素估计曲率调制场景提示，非业务分流 */
	VoxelSceneClass voxel_scene_class_;
	/// 初始化缓存目录。由 StationInitialize 按 lasFn_ 写入，不来自 ScanInfo。
	std::string info_folder_;
	VoxelRoot root_;						// Root 体素（角度深度筛选层，与子体素分离）
	std::vector<SubVoxel> subvoxel_vec_;	// 子体素

	LidarFrame();
	explicit LidarFrame(ScanInfo const& scan_info);

	bool hasRoot() const;
	void setScanInfo(ScanInfo const& info);
	std::string getInfoPath() const;
	std::string getCloudPath(std::string const& name) const;
	std::string getNormalPath(std::string const& name) const;
	int findSubVoxelIndex(std::string const& node_name) const;
	SubVoxel getSubVoxel(std::string const& node_name) const;
};

/// 配准边结果。
///
/// **量纲约定（现网，2026-10 起强制）：**
/// - `matrix_`：刚体变换 source→target；平移 [m]，旋转无量纲。
/// - `rms_` / `final_rms_`：**已开方的 RMSE** [m]，即 √mean(d²)
///  （由 FineRegistration / RegistrationReportEstimator 填写）。
///   **禁止**再写入 mean(d²) / 0.1² 一类「未开方」旧值。
///   south 旧码曾用未开方尺度（门限 0.01 ≈ 0.1²）；本仓库门控已按米制 RMSE。
/// - `similarity_`：重叠率等相似度 [0,1]，无量纲。
/// - `weight_`：边权，越小越好；Fine 默认 `weight_ = rms_ / max(similarity_,ε)`，单位随 rms_ 为 [m]。
/// - `information_`：SE(3) 信息矩阵 Ω，布局 [t|r]；平移块量纲约 [1/m²]，旋转块 [1/rad²]。
struct AlignResult
{
	StateType state_;			// 状态
	std::string src_name_;
	std::string tgt_name_;
	Eigen::Matrix4d matrix_;	// source→target；平移 [m]
	double similarity_;			// [0,1]，通常为重叠率
	double rms_;				// RMSE [m]（已开方）
	double final_rms_;			// 最终 RMSE [m]（已开方；常与 rms_ 相同）
	double weight_;				// 越小越好；Fine 路径单位随 rms_ [m]
	int feature_num_;			// 匹配/内点特征数 [1]
	int auto_reg_;				// 自动配准路径位标志（1=Regular, 2=FPFH 等）
	/// 相对位姿边信息矩阵 Ω（6×6，布局 [t|r]）；默认 I，供 pose_graph_opt 使用
	Eigen::Matrix<double, 6, 6> information_;

	AlignResult();

	void reset();
	void resetState();
	void resetAll();
};

struct StationGraphEdge
{
	std::string src_name_;
	std::string tgt_name_;
	double rms_ = 0.0;			// RMSE [m]（已开方），与 AlignResult::rms_ 同约定
	double similarity_ = 0.0;	// [0,1]
};

struct StationGraphVertex
{
	std::string name_;
	/// 世界←测站全局矩阵；无条目时 has_global_matrix_=false，matrix 为 I
	bool has_global_matrix_ = false;
	Eigen::Matrix4d global_matrix_ = Eigen::Matrix4d::Identity();
};

struct StationGraph
{
	std::vector<StationGraphVertex> vertices_;
	std::vector<StationGraphEdge> edges_;
};

/** web_view_manager 站级进度（phase + init/reg 总秒）；数值字段 <0 表示尚未结束 / 无效 */
struct StationStatus
{
	std::string name_;
	std::string state_;				// queued / running / done / failed
	std::string phase_;				// queued / init / reg / done / failed
	std::string ref_;
	std::string phase_started_at_;
	std::string message_;
	double init_total_s_;		// init 耗时 [s]；<0 无效
	double reg_total_s_;		// 配准耗时 [s]；<0 无效
	double rms_;				// RMSE [m]（已开方）；与 AlignResult 同约定
	double similarity_;			// [0,1]

	StationStatus();
};

} // namespace Ddx
