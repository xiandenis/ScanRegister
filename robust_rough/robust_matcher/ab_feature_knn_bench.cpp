/**
 * Microbench: FLANN vs hnswlib knn on synthetic 33D descriptors.
 * Usage: ab_feature_knn_bench [N] [ef] [ef_construction] [M]
 *   N               = points / queries (default 40000)
 *   ef              = hnsw search ef (default 32)
 *   ef_construction = hnsw build ef (default 100)
 *   M               = hnsw M (default 16)
 */
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <flann/flann.hpp>
#include <hnswlib/hnswlib.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace {

constexpr int kDim = 33;

using Feature = std::vector<Eigen::VectorXf>;
using FlannTree = flann::Index<flann::L2<float>>;

std::vector<float> packFeatures(const Feature& feats) {
  const size_t n = feats.size();
  std::vector<float> out(n * kDim);
  for (size_t i = 0; i < n; ++i) {
    for (int d = 0; d < kDim; ++d) {
      out[i * kDim + d] = feats[i](d);
    }
  }
  return out;
}

Feature makeFeatures(size_t n, std::mt19937& rng) {
  std::normal_distribution<float> dist(0.f, 1.f);
  Feature feats(n);
  for (size_t i = 0; i < n; ++i) {
    feats[i] = Eigen::VectorXf(kDim);
    for (int d = 0; d < kDim; ++d) {
      feats[i](d) = dist(rng);
    }
    feats[i].normalize();
  }
  return feats;
}

double secondsSince(std::chrono::high_resolution_clock::time_point t0) {
  auto t1 = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
}

}  // namespace

int main(int argc, char** argv) {
  size_t n           = 40000;
  int ef             = 32;
  int ef_construction = 100;
  int M              = 16;
  if (argc >= 2) n = static_cast<size_t>(std::stoul(argv[1]));
  if (argc >= 3) ef = std::stoi(argv[2]);
  if (argc >= 4) ef_construction = std::stoi(argv[3]);
  if (argc >= 5) M = std::stoi(argv[4]);

  std::mt19937 rng(42);
  Feature data = makeFeatures(n, rng);
  Feature queries = data;  // self-query for recall check

  std::vector<float> data_mat = packFeatures(data);
  std::vector<float> query_mat = packFeatures(queries);

  // ---- FLANN ----
  auto t0 = std::chrono::high_resolution_clock::now();
  flann::Matrix<float> dataset(&data_mat[0], static_cast<int>(n), kDim);
  FlannTree flann_tree(dataset, flann::KDTreeSingleIndexParams(15));
  flann_tree.buildIndex();
  const double flann_build = secondsSince(t0);

  std::vector<std::array<int, 2>> flann_idx(n);
  std::vector<std::array<float, 2>> flann_dist(n);
  t0 = std::chrono::high_resolution_clock::now();
  tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](tbb::blocked_range<size_t> r) {
    for (size_t i = r.begin(); i < r.end(); ++i) {
      flann::Matrix<float> q(&query_mat[i * kDim], 1, kDim);
      flann::Matrix<int> indices(&flann_idx[i][0], 1, 2);
      flann::Matrix<float> dists(&flann_dist[i][0], 1, 2);
      auto params = flann::SearchParams(128);
      params.cores = 1;
      flann_tree.knnSearch(q, indices, dists, 2, params);
    }
  });
  const double flann_query = secondsSince(t0);

  // ---- HNSW (parallel addPoint; mirrors ROBINMatchingHnsw) ----
  t0 = std::chrono::high_resolution_clock::now();
  hnswlib::L2Space space(kDim);
  hnswlib::HierarchicalNSW<float> hnsw(&space, n, M, ef_construction);
  if (n > 0) {
    hnsw.addPoint(data_mat.data(), 0);
  }
  if (n > 1) {
    tbb::parallel_for(size_t{1}, n, [&](size_t i) {
      hnsw.addPoint(data_mat.data() + i * kDim, i);
    });
  }
  hnsw.setEf(static_cast<size_t>(ef));
  const double hnsw_build = secondsSince(t0);

  std::vector<std::array<int, 2>> hnsw_idx(n);
  std::vector<std::array<float, 2>> hnsw_dist(n);
  t0 = std::chrono::high_resolution_clock::now();
  tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](tbb::blocked_range<size_t> r) {
    for (size_t i = r.begin(); i < r.end(); ++i) {
      auto res = hnsw.searchKnnCloserFirst(query_mat.data() + i * kDim, 2);
      for (size_t k = 0; k < 2; ++k) {
        if (k < res.size()) {
          hnsw_dist[i][k] = res[k].first;
          hnsw_idx[i][k]  = static_cast<int>(res[k].second);
        } else {
          hnsw_dist[i][k] = 1e30f;
          hnsw_idx[i][k]  = -1;
        }
      }
    }
  });
  const double hnsw_query = secondsSince(t0);

  size_t top1 = 0;
  size_t top2 = 0;
  for (size_t i = 0; i < n; ++i) {
    if (hnsw_idx[i][0] == flann_idx[i][0]) ++top1;
    // Top-2 set overlap (order-insensitive)
    const bool a0 = (hnsw_idx[i][0] == flann_idx[i][0] || hnsw_idx[i][0] == flann_idx[i][1]);
    const bool a1 = (hnsw_idx[i][1] == flann_idx[i][0] || hnsw_idx[i][1] == flann_idx[i][1]);
    if (a0 && a1) ++top2;
  }

  const double flann_total = flann_build + flann_query;
  const double hnsw_total  = hnsw_build + hnsw_query;
  const double improve =
      flann_total > 0.0 ? (flann_total - hnsw_total) / flann_total * 100.0 : 0.0;

  std::cout << "N=" << n << " dim=" << kDim << " ef=" << ef << " M=" << M
            << " ef_construction=" << ef_construction << "\n";
  std::cout << "FLANN  build=" << flann_build << "s query=" << flann_query
            << "s total=" << flann_total << "s\n";
  std::cout << "HNSW   build=" << hnsw_build << "s query=" << hnsw_query
            << "s total=" << hnsw_total << "s\n";
  std::cout << "improve(total)=" << improve << "%  (positive => HNSW faster)\n";
  std::cout << "Top-1 agreement vs FLANN: " << (100.0 * top1 / n) << "%\n";
  std::cout << "Top-2 set agreement vs FLANN: " << (100.0 * top2 / n) << "%\n";
  return 0;
}
