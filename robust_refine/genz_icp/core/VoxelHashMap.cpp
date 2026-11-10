// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "VoxelHashMap.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

// This parameters are not intended to be changed, therefore we do not expose it
namespace {
struct ResultTuple { 
    ResultTuple(std::size_t n) {
        source.reserve(n);
        target.reserve(n);
    }
    std::vector<Eigen::Vector3d> source;
    std::vector<Eigen::Vector3d> target;
};

static const std::array<Eigen::Vector3i, 27> voxel_shifts{
    Eigen::Vector3i(0, 0, 0),   Eigen::Vector3i(1, 0, 0),   Eigen::Vector3i(-1, 0, 0),
    Eigen::Vector3i(0, 1, 0),   Eigen::Vector3i(0, -1, 0),  Eigen::Vector3i(0, 0, 1),
    Eigen::Vector3i(0, 0, -1),  Eigen::Vector3i(1, 1, 0),   Eigen::Vector3i(1, -1, 0),
    Eigen::Vector3i(-1, 1, 0),  Eigen::Vector3i(-1, -1, 0), Eigen::Vector3i(1, 0, 1),
    Eigen::Vector3i(1, 0, -1),  Eigen::Vector3i(-1, 0, 1),  Eigen::Vector3i(-1, 0, -1),
    Eigen::Vector3i(0, 1, 1),   Eigen::Vector3i(0, 1, -1),  Eigen::Vector3i(0, -1, 1),
    Eigen::Vector3i(0, -1, -1), Eigen::Vector3i(1, 1, 1),   Eigen::Vector3i(1, 1, -1),
    Eigen::Vector3i(1, -1, 1),  Eigen::Vector3i(1, -1, -1), Eigen::Vector3i(-1, 1, 1),
    Eigen::Vector3i(-1, 1, -1), Eigen::Vector3i(-1, -1, 1), Eigen::Vector3i(-1, -1, -1)
};

static const size_t min_neighbors_for_normal_estimation = 5; 
}  // namespace

namespace genz_icp {

std::tuple<Eigen::Vector3d, size_t, Eigen::Matrix3d, double> VoxelHashMap::GetClosestNeighbor(
    const Eigen::Vector3d &query) const {
    Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

    double closest_squared_distance = std::numeric_limits<double>::max();
    size_t n_neighbors = 0;
    
    const auto kx = static_cast<int>(std::floor(query.x() / voxel_size_));
    const auto ky = static_cast<int>(std::floor(query.y() / voxel_size_));
    const auto kz = static_cast<int>(std::floor(query.z() / voxel_size_));

    std::for_each(voxel_shifts.cbegin(), voxel_shifts.cend(), [&](const auto &voxel_shift) {
        Voxel voxel(kx+voxel_shift.x(), ky+voxel_shift.y(), kz+voxel_shift.z());
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            const auto &points = search->second.points;
            std::for_each(points.cbegin(), points.cend(), [&](const auto &neighbor){
                double squared_distance = (neighbor - query).squaredNorm();
                if (squared_distance < closest_squared_distance){
                    closest_neighbor = neighbor;
                    closest_squared_distance = squared_distance;
                }
                centroid += neighbor;
                covariance += neighbor*neighbor.transpose();
                n_neighbors++;
            }
        );
        }
    });

    if (n_neighbors >= min_neighbors_for_normal_estimation){
        centroid /= static_cast<double>(n_neighbors);
        covariance /= static_cast<double>(n_neighbors);
        covariance -= centroid*centroid.transpose();
    }
    double closest_distance = (n_neighbors > 0) ? std::sqrt(closest_squared_distance) : std::numeric_limits<double>::max();
    return std::make_tuple(closest_neighbor, n_neighbors, covariance, closest_distance);
}


std::pair<bool, Eigen::Vector3d> VoxelHashMap::DeterminePlanarity(
    const Eigen::Matrix3d &covariance) const{
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    // Compute the normal as the eigenvector of the smallest eigenvalue
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance, Eigen::ComputeEigenvectors);
    normal = solver.eigenvectors().col(0);
    normal = normal.normalized();

    // Planarity check
    const auto &eigenvalues = solver.eigenvalues();
    double lambda3 = eigenvalues[0];
    double lambda2 = eigenvalues[1];
    double lambda1 = eigenvalues[2];

    const double eig_sum = lambda1 + lambda2 + lambda3;
    const bool is_planar =
        (eig_sum > 0.0) && ((lambda3 / eig_sum) < planarity_threshold_);

    return {is_planar, normal};
}

VoxelHashMap::Vector3dVectorTuple7 VoxelHashMap::GetCorrespondences(
    const Vector3dVector &points, double max_correspondance_distance) const {

    // NOTE: Do NOT use ResultTuple(points.size()) as the parallel_reduce identity.
    // TBB clones the identity for every split task; reserving O(N) per task OOM/corrupts
    // on ~1e6+ clouds (Qingyuan voxel_3 @ map_voxel<=0.05 → STATUS_STACK_BUFFER_OVERRUN).
    struct ResultTuple {
        Vector3dVector source;
        Vector3dVector target;
        Vector3dVector normals;
        Vector3dVector non_planar_source;
        Vector3dVector non_planar_target;
        size_t planar_count = 0;
        size_t non_planar_count = 0;

        ResultTuple() = default;

        void reserve_for_range(std::size_t n) {
            source.reserve(source.size() + n);
            target.reserve(target.size() + n);
            normals.reserve(normals.size() + n);
            non_planar_source.reserve(non_planar_source.size() + n);
            non_planar_target.reserve(non_planar_target.size() + n);
        }

        ResultTuple &append_moved(ResultTuple &&other) {
            source.insert(source.end(), std::make_move_iterator(other.source.begin()),
                          std::make_move_iterator(other.source.end()));
            target.insert(target.end(), std::make_move_iterator(other.target.begin()),
                          std::make_move_iterator(other.target.end()));
            normals.insert(normals.end(), std::make_move_iterator(other.normals.begin()),
                           std::make_move_iterator(other.normals.end()));
            non_planar_source.insert(non_planar_source.end(),
                                     std::make_move_iterator(other.non_planar_source.begin()),
                                     std::make_move_iterator(other.non_planar_source.end()));
            non_planar_target.insert(non_planar_target.end(),
                                     std::make_move_iterator(other.non_planar_target.begin()),
                                     std::make_move_iterator(other.non_planar_target.end()));
            planar_count += other.planar_count;
            non_planar_count += other.non_planar_count;
            return *this;
        }
    };

    auto compute = [&](const tbb::blocked_range<size_t> &r, ResultTuple result) -> ResultTuple {
        result.reserve_for_range(r.size());
        for (size_t i = r.begin(); i != r.end(); ++i) {
            const Eigen::Vector3d &point = points[i];

            const auto &[closest_neighbor, n_neighbors, covariance, closest_distance] =
                GetClosestNeighbor(point);
            if (closest_distance > max_correspondance_distance) continue;

            if (n_neighbors >= min_neighbors_for_normal_estimation) {
                const auto &[is_planar, normal] = DeterminePlanarity(covariance);
                if (is_planar) {
                    result.source.emplace_back(point);
                    result.target.emplace_back(closest_neighbor);
                    result.normals.emplace_back(normal);
                    result.planar_count++;
                } else {
                    result.non_planar_source.emplace_back(point);
                    result.non_planar_target.emplace_back(closest_neighbor);
                    result.non_planar_count++;
                }
            } else {
                result.non_planar_source.emplace_back(point);
                result.non_planar_target.emplace_back(closest_neighbor);
                result.non_planar_count++;
            }
        }
        return result;
    };

    const auto &[source, target, normals, non_planar_source, non_planar_target, planar_count,
                 non_planar_count] =
        tbb::parallel_reduce(
            tbb::blocked_range<size_t>(0, points.size()), ResultTuple{}, compute,
            [](ResultTuple a, ResultTuple b) { return std::move(a.append_moved(std::move(b))); });

    return std::make_tuple(source, target, normals, non_planar_source, non_planar_target,
                           planar_count, non_planar_count);
}

std::vector<Eigen::Vector3d> VoxelHashMap::Pointcloud() const {
    std::vector<Eigen::Vector3d> points;
    points.reserve(max_points_per_voxel_ * map_.size());
    for (const auto &[voxel, voxel_block] : map_) {
        (void)voxel;
        for (const auto &point : voxel_block.points) {
            points.emplace_back(point);
        }
    }
    return points;
}

void VoxelHashMap::Update(const Vector3dVector &points, const Eigen::Vector3d &origin) {
    AddPoints(points);
    RemovePointsFarFromLocation(origin);
}

void VoxelHashMap::Update(const Vector3dVector &points, const Sophus::SE3d &pose) {
    Vector3dVector points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    const Eigen::Vector3d &origin = pose.translation();
    Update(points_transformed, origin);
}

void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points) {
    map_.reserve(map_.size() + points.size());
    for (const auto &point : points) {
        const Eigen::Vector3d scaled = point / voxel_size_;
        const Voxel voxel(static_cast<int>(std::floor(scaled.x())),
                          static_cast<int>(std::floor(scaled.y())),
                          static_cast<int>(std::floor(scaled.z())));
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            search.value().AddPoint(point);
        } else {
            map_.insert({voxel, VoxelBlock(point, max_points_per_voxel_)});
        }
    }
}

void VoxelHashMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    const auto max_distance2 = map_cleanup_radius_ * map_cleanup_radius_;
    for (auto it = map_.begin(); it != map_.end();) {
        if ((it->second.points.front() - origin).squaredNorm() > (max_distance2)) {
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
}
}  // namespace genz_icp
