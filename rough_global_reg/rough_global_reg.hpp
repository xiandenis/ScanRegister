#pragma once

#include <registration_type/registration_type.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace Ddx
{
    /**
     * Business facade over RobustGlobalReg.
     *
     * - ScanInfo: load lasFn_ as FramedCloud (IO localize); optional station fold for compass.
     * - LidarFrame: **SENSOR_STAND-only** entry (架站且站心已知). Requires RegistrationInitializer
     *   (HAS_CALCU + voxel_3). Loads station-centered voxel_3 cache (no FramedCloud re-localize);
     *   pose fold uses station_origin_ (file CRS). Non-SENSOR_STAND → refuse.
     *
     * Compass: only on station-centered data (SENSOR_STAND + SENSOR_COMPASS).
     * guess / AlignResult.matrix_ are file/world frame.
     */
    class RoughGlobalReg
    {
    public:
        // --- ScanInfo (raw lasFn_ via FramedCloud) ---
        bool run(ScanInfo const& source, ScanInfo const& target, float resolution,
                 Eigen::Matrix4d const& guess, int iter_number = -1, bool use_compass = false);
        bool run(ScanInfo const& source, ScanInfo const& target, float resolution,
                 int iter_number = -1);

        // --- LidarFrame: SENSOR_STAND-only（非架站/未知站心禁止）+ 已初始化 (HAS_CALCU, voxel_3) ---
        bool run(LidarFrame const& source, LidarFrame const& target, float resolution,
                 Eigen::Matrix4d const& guess, int iter_number = -1, bool use_compass = false);
        bool run(LidarFrame const& source, LidarFrame const& target, float resolution,
                 int iter_number = -1);

        AlignResult getAlignResult() const { return m_alignResult; }

    private:
        static int resolveIterationCount(float resolution, int iter_number);
        /** SENSOR_COMPASS on both AND SENSOR_STAND on both. */
        static bool canUseCompass(ScanInfo const& source, ScanInfo const& target);
        /** SENSOR_STAND + initialized frame with usable voxel_3 cache. */
        static bool isReadyLidarFrame(LidarFrame const& frame, std::string* reason = nullptr);
        static bool resolveUseHnswKnn(float resolution);
        static bool resolveUseFeatSpatialBudget(float resolution);

        bool runFromScanInfo(ScanInfo const& source, ScanInfo const& target, float resolution,
                             int iter_number, bool use_compass,
                             Eigen::Matrix4d const* initial_guess_world);
        bool runFromLidarFrame(LidarFrame const& source, LidarFrame const& target, float resolution,
                               int iter_number, bool use_compass,
                               Eigen::Matrix4d const* initial_guess_world);
        bool runKernel(std::string const& src_name, std::string const& tgt_name,
                       std::vector<Eigen::Vector3f>& src_vec, std::vector<Eigen::Vector3f>& tgt_vec,
                       Eigen::Vector3d const& O_s, Eigen::Vector3d const& O_t, float resolution,
                       int iter_number, bool use_compass,
                       Eigen::Matrix4d const* initial_guess_world);

        AlignResult m_alignResult;
    };
}
