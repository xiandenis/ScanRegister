#include "rough_global_reg.hpp"

#include <logging/logging.hpp>
#include <robust_global_reg/robust_global_reg.hpp>
#include <robust_global_reg/robGlob_reg_parameter.hpp>
#include <file_system/file_system.hpp>
#include <point_cloud_io/point_cloud_io.hpp>
#include <point_cloud_io/pcse_reader.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace Ddx
{
namespace
{
    Eigen::Matrix4d worldToLocalInit(Eigen::Matrix4d const& M_world, Eigen::Vector3d const& O_s,
                                     Eigen::Vector3d const& O_t)
    {
        Eigen::Matrix4d T_os = Eigen::Matrix4d::Identity();
        T_os.block<3, 1>(0, 3) = O_s;
        Eigen::Matrix4d T_neg_ot = Eigen::Matrix4d::Identity();
        T_neg_ot.block<3, 1>(0, 3) = -O_t;
        return T_neg_ot * M_world * T_os;
    }

    Eigen::Matrix4d localToWorldPose(Eigen::Matrix4d const& M_loc, Eigen::Vector3d const& O_s,
                                     Eigen::Vector3d const& O_t)
    {
        Eigen::Matrix4d T_ot = Eigen::Matrix4d::Identity();
        T_ot.block<3, 1>(0, 3) = O_t;
        Eigen::Matrix4d T_neg_os = Eigen::Matrix4d::Identity();
        T_neg_os.block<3, 1>(0, 3) = -O_s;
        return T_ot * M_loc * T_neg_os;
    }

    bool pathOk(std::string const& path)
    {
        return !path.empty()
            && (stlplus::file_exists(path) || point_cloud_io::is_pcse_dataset(path));
    }

    bool cloudToVec(pcl::PointCloud<pcl::PointXYZ> const& cloud, std::vector<Eigen::Vector3f>& out)
    {
        out.clear();
        out.reserve(cloud.size());
        for (auto const& p : cloud.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                continue;
            }
            out.emplace_back(p.x, p.y, p.z);
        }
        return !out.empty();
    }

    /** p_station = p_local + O_framed - O_station; framed.origin becomes station_origin. */
    void convertFramedToStation(point_cloud_io::FramedCloud<pcl::PointXYZ>& framed,
                                Eigen::Vector3d const& station_origin)
    {
        Eigen::Vector3d const delta = framed.origin - station_origin;
        for (auto& p : framed.cloud.points) {
            p.x = static_cast<float>(static_cast<double>(p.x) + delta.x());
            p.y = static_cast<float>(static_cast<double>(p.y) + delta.y());
            p.z = static_cast<float>(static_cast<double>(p.z) + delta.z());
        }
        framed.origin = station_origin;
    }

    /** Rebase target FramedCloud into source origin (job frame A). */
    void rebaseToOrigin(point_cloud_io::FramedCloud<pcl::PointXYZ>& framed,
                        Eigen::Vector3d const& job_origin)
    {
        Eigen::Vector3d const delta = framed.origin - job_origin;
        if (delta.squaredNorm() < 1e-24) {
            framed.origin = job_origin;
            return;
        }
        for (auto& p : framed.cloud.points) {
            p.x = static_cast<float>(static_cast<double>(p.x) + delta.x());
            p.y = static_cast<float>(static_cast<double>(p.y) + delta.y());
            p.z = static_cast<float>(static_cast<double>(p.z) + delta.z());
        }
        framed.origin = job_origin;
    }

    bool loadStationPcdToVec(std::string const& path, std::string const& name,
                             std::vector<Eigen::Vector3f>& out)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        if (pcl::io::loadPCDFile(path, cloud) < 0) {
            LOG_ERROR("failed to load station-centered PCD ({}) path={}", name, path);
            return false;
        }
        if (!cloudToVec(cloud, out)) {
            LOG_ERROR("empty station-centered PCD ({})", name);
            return false;
        }
        return true;
    }
} // namespace

    int RoughGlobalReg::resolveIterationCount(float resolution, int iter_number)
    {
        if (iter_number > 0) {
            return iter_number;
        }
        if (resolution < 0.166f) {
            return 7;
        }
        if (resolution < 0.222f) {
            return 2;
        }
        return 1;
    }

    bool RoughGlobalReg::canUseCompass(ScanInfo const& source, ScanInfo const& target)
    {
        return (source.sensor_ & SENSOR_COMPASS) == SENSOR_COMPASS
            && (target.sensor_ & SENSOR_COMPASS) == SENSOR_COMPASS
            && source.has_station_origin_
            && target.has_station_origin_;
    }

    bool RoughGlobalReg::resolveUseHnswKnn(float resolution)
    {
        constexpr float kHnswMinVoxelSize = 0.20f;
        return resolution >= kHnswMinVoxelSize;
    }

    bool RoughGlobalReg::resolveUseFeatSpatialBudget(float resolution)
    {
        constexpr float kFeatBudgetMinVoxelSize = 0.20f;
        return resolution >= kFeatBudgetMinVoxelSize;
    }

    bool RoughGlobalReg::run(ScanInfo const& source, ScanInfo const& target, float resolution,
                             Eigen::Matrix4d const& guess, int /*iter_number*/, bool use_compass)
    {
        m_alignResult.reset();
        m_alignResult.src_name_ = source.name_;
        m_alignResult.tgt_name_ = target.name_;
        return runFromScanInfo(source, target, resolution, -1, use_compass, &guess);
    }

    bool RoughGlobalReg::run(ScanInfo const& source, ScanInfo const& target, float resolution,
                             int iter_number)
    {
        m_alignResult.reset();
        m_alignResult.src_name_ = source.name_;
        m_alignResult.tgt_name_ = target.name_;
        return runFromScanInfo(source, target, resolution, iter_number,
                               canUseCompass(source, target), nullptr);
    }

    bool RoughGlobalReg::run(LidarFrame const& source, LidarFrame const& target, float resolution,
                             Eigen::Matrix4d const& guess, int /*iter_number*/, bool use_compass)
    {
        m_alignResult.reset();
        m_alignResult.src_name_ = source.name_;
        m_alignResult.tgt_name_ = target.name_;
        return runFromLidarFrame(source, target, resolution, -1, use_compass, &guess);
    }

    bool RoughGlobalReg::run(LidarFrame const& source, LidarFrame const& target, float resolution,
                             int iter_number)
    {
        m_alignResult.reset();
        m_alignResult.src_name_ = source.name_;
        m_alignResult.tgt_name_ = target.name_;
        return runFromLidarFrame(source, target, resolution, iter_number,
                                 canUseCompass(source, target), nullptr);
    }

    bool RoughGlobalReg::runFromScanInfo(ScanInfo const& source, ScanInfo const& target,
                                         float resolution, int iter_number, bool use_compass,
                                         Eigen::Matrix4d const* initial_guess_world)
    {
        if (!pathOk(source.lasFn_)) {
            LOG_ERROR("source lasFn_ is empty or not exists ({})", source.name_);
            m_alignResult.state_ = INVALID;
            return false;
        }
        if (!pathOk(target.lasFn_)) {
            LOG_ERROR("target lasFn_ is empty or not exists ({})", target.name_);
            m_alignResult.state_ = INVALID;
            return false;
        }

        if (use_compass) {
            if (!source.has_station_origin_ || !target.has_station_origin_) {
                LOG_ERROR(
                    "use_compass requires station-centered ScanInfo "
                    "(has_station_origin_ on both); refused ({} / {})",
                    source.name_, target.name_);
                m_alignResult.state_ = INVALID;
                return false;
            }
        }

        Eigen::Vector3d O_s = Eigen::Vector3d::Zero();
        Eigen::Vector3d O_t = Eigen::Vector3d::Zero();
        std::vector<Eigen::Vector3f> src_vec;
        std::vector<Eigen::Vector3f> tgt_vec;

        {
            point_cloud_io::FramedCloud<pcl::PointXYZ> src_framed;
            if (point_cloud_io::load(source.lasFn_, src_framed) != 0) {
                LOG_ERROR("failed to load source FramedCloud ({})", source.name_);
                m_alignResult.state_ = INVALID;
                return false;
            }
            if (use_compass) {
                convertFramedToStation(src_framed, source.station_origin_);
                O_s = source.station_origin_;
            } else {
                O_s = src_framed.origin;
            }
            if (!cloudToVec(src_framed.cloud, src_vec)) {
                LOG_ERROR("empty source cloud after load/localize ({})", source.name_);
                m_alignResult.state_ = INVALID;
                return false;
            }
        }

        {
            point_cloud_io::FramedCloud<pcl::PointXYZ> tgt_framed;
            if (point_cloud_io::load(target.lasFn_, tgt_framed) != 0) {
                LOG_ERROR("failed to load target FramedCloud ({})", target.name_);
                m_alignResult.state_ = INVALID;
                return false;
            }
            if (use_compass) {
                convertFramedToStation(tgt_framed, target.station_origin_);
                O_t = target.station_origin_;
            } else {
                O_t = O_s;
                rebaseToOrigin(tgt_framed, O_s);
            }
            if (!cloudToVec(tgt_framed.cloud, tgt_vec)) {
                LOG_ERROR("empty target cloud after load/localize ({})", target.name_);
                m_alignResult.state_ = INVALID;
                return false;
            }
        }

        return runKernel(source.name_, target.name_, src_vec, tgt_vec, O_s, O_t, resolution,
                         iter_number, use_compass, initial_guess_world);
    }

    bool RoughGlobalReg::runFromLidarFrame(LidarFrame const& source, LidarFrame const& target,
                                           float resolution, int iter_number, bool use_compass,
                                           Eigen::Matrix4d const* initial_guess_world)
    {
        // voxel_3 caches are station-centered; fold-back needs declared station origins.
        if (!source.has_station_origin_ || !target.has_station_origin_) {
            LOG_ERROR(
                "LidarFrame rough reg requires has_station_origin_ on both ({} / {})",
                source.name_, target.name_);
            m_alignResult.state_ = INVALID;
            return false;
        }

        std::string const src_path = source.getCloudPath("voxel_3");
        std::string const tgt_path = target.getCloudPath("voxel_3");
        if (!pathOk(src_path)) {
            LOG_ERROR("source voxel_3 missing ({}) path={}", source.name_, src_path);
            m_alignResult.state_ = INVALID;
            return false;
        }
        if (!pathOk(tgt_path)) {
            LOG_ERROR("target voxel_3 missing ({}) path={}", target.name_, tgt_path);
            m_alignResult.state_ = INVALID;
            return false;
        }

        // Do NOT FramedCloud-localize station caches (would steal a false IO origin).
        Eigen::Vector3d const O_s = source.station_origin_;
        Eigen::Vector3d const O_t = target.station_origin_;
        std::vector<Eigen::Vector3f> src_vec;
        std::vector<Eigen::Vector3f> tgt_vec;

        if (!loadStationPcdToVec(src_path, source.name_, src_vec)) {
            m_alignResult.state_ = INVALID;
            return false;
        }
        if (!loadStationPcdToVec(tgt_path, target.name_, tgt_vec)) {
            m_alignResult.state_ = INVALID;
            return false;
        }

        return runKernel(source.name_, target.name_, src_vec, tgt_vec, O_s, O_t, resolution,
                         iter_number, use_compass, initial_guess_world);
    }

    bool RoughGlobalReg::runKernel(std::string const& src_name, std::string const& tgt_name,
                                   std::vector<Eigen::Vector3f>& src_vec,
                                   std::vector<Eigen::Vector3f>& tgt_vec,
                                   Eigen::Vector3d const& O_s, Eigen::Vector3d const& O_t,
                                   float resolution, int iter_number, bool use_compass,
                                   Eigen::Matrix4d const* initial_guess_world)
    {
        RobGlobRegParameter parameter;
        parameter.scan_name = src_name;
        parameter.voxel_size = resolution;
        parameter.info = DEFINEDF;

        if (initial_guess_world != nullptr) {
            parameter.transformation_matrix =
                worldToLocalInit(*initial_guess_world, O_s, O_t);
            parameter.info |= INITI_MATRIX;
        } else {
            parameter.info |= STATION_LIDAR_DEVICE;
            parameter.number_of_iterations = resolveIterationCount(resolution, iter_number);
        }

        if (use_compass) {
            parameter.use_compass = true;
        }

        parameter.use_hnsw_knn = resolveUseHnswKnn(resolution);
        parameter.use_feat_spatial_budget = resolveUseFeatSpatialBudget(resolution);
        LOG_INFO(
            "rough_global_reg knn={} feat_budget={} compass={} (resolution={:.4f}, threshold=0.20)",
            parameter.use_hnsw_knn ? "HNSW" : "FLANN",
            parameter.use_feat_spatial_budget ? "ON" : "OFF", use_compass ? "ON" : "OFF",
            resolution);

        RobustGlobalReg reg;
        reg.setParameter(parameter);
        if (!reg.run(src_vec, tgt_vec)) {
            LOG_ERROR("registration fail! ({} -> {})", src_name, tgt_name);
            m_alignResult.state_ = INVALID;
            return false;
        }

        int const feature_number = reg.getFeatureNumber();
        Eigen::Matrix4d const M_loc = reg.getFinalTransformationMatrix();
        m_alignResult.matrix_ = localToWorldPose(M_loc, O_s, O_t);
        m_alignResult.feature_num_ = std::max(20, feature_number);
        m_alignResult.similarity_ = 0.25;
        m_alignResult.rms_ = 0.2;
        m_alignResult.final_rms_ = m_alignResult.rms_;
        m_alignResult.weight_ = 1.0;
        m_alignResult.state_ = VALID | HAS_CALCU;
        return true;
    }
}
