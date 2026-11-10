// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <Eigen/Core>
#include "kdtree.hpp"
#include "nanoflann_tbb.hpp"
#include <matcher_points/traits.hpp>

namespace robust_matcher {

/// @brief Unsafe KdTree with multi-thread tree construction with TBB backend.
/// @note  This class only parallelizes the tree construction. The search is still single-threaded
/// as in the normal KdTree.
template <class PointCloud>
using UnsafeKdTreeTBB = UnsafeKdTreeGeneric<PointCloud, robust_matcher::KDTreeSingleIndexAdaptorTBB>;

/// @brief KdTree with multi-thread tree construction with TBB backend.
/// @note  This class only parallelizes the tree construction. The search is still single-threaded
/// as in the normal KdTree.
template <class PointCloud>
using KdTreeTBB = KdTreeGeneric<PointCloud, robust_matcher::KDTreeSingleIndexAdaptorTBB>;

}  // namespace robust_matcher
