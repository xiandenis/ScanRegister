#include "ROBINMatchingHnsw.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <tbb/parallel_for.h>
#include <tbb/parallel_invoke.h>

namespace robust_matcher {

ROBINMatchingHnsw::ROBINMatchingHnsw(const float noise_bound,
                                     const int num_max_corr,
                                     const float tuple_scale,
                                     HnswParams hnsw_params) {
  noise_bound_      = noise_bound;
  num_max_corr_     = num_max_corr;
  tuple_test_ratio_ = tuple_scale;
  hnsw_params_      = hnsw_params;
}

std::vector<std::pair<int, int>> ROBINMatchingHnsw::establishCorrespondences(
    std::vector<Eigen::Vector3f>& source_points,
    std::vector<Eigen::Vector3f>& target_points,
    Feature& source_features,
    Feature& target_features,
    std::string robin_mode,
    float tuple_scale,
    bool use_ratio_test) {
  pointcloud_.clear();
  features_.clear();
  corres_cross_checked_.clear();
  corres_.clear();

  pointcloud_.emplace_back(source_points);
  pointcloud_.emplace_back(target_points);
  features_.emplace_back(source_features);
  features_.emplace_back(target_features);

  setStatuses();
  match(robin_mode, tuple_scale, use_ratio_test);
  return corres_;
}

std::vector<std::pair<int, int>> ROBINMatchingHnsw::establishCorrespondencesStation(
    std::vector<Eigen::Vector3f>& source_points,
    std::vector<Eigen::Vector3f>& target_points,
    Feature& source_features,
    Feature& target_features,
    std::string robin_mode,
    float tuple_scale,
    float angle_thr,
    float altitude_thr,
    int search_knn) {
  pointcloud_.clear();
  features_.clear();
  corres_cross_checked_.clear();
  corres_.clear();

  pointcloud_.emplace_back(source_points);
  pointcloud_.emplace_back(target_points);
  features_.emplace_back(source_features);
  features_.emplace_back(target_features);

  setStatuses();
  match_compass(robin_mode, tuple_scale, angle_thr, altitude_thr, search_knn);
  return corres_;
}

void ROBINMatchingHnsw::buildHnsw(const Feature& data, HnswIndex* tree) const {
  if (data.empty()) {
    throw std::runtime_error("ROBINMatchingHnsw::buildHnsw: empty feature set");
  }
  tree->rows = data.size();
  tree->dim  = static_cast<size_t>(data[0].size());
  tree->data.resize(tree->rows * tree->dim);

  // Pack + insert are both parallel; hnswlib addPoint is internally synchronized.
  tbb::parallel_for(size_t{0}, tree->rows, [&](size_t i) {
    if (static_cast<size_t>(data[i].size()) != tree->dim) {
      throw std::runtime_error("ROBINMatchingHnsw::buildHnsw: inconsistent feature dim");
    }
    float* row = tree->data.data() + i * tree->dim;
    for (size_t j = 0; j < tree->dim; ++j) {
      row[j] = data[i](static_cast<Eigen::Index>(j));
    }
  });

  tree->space = std::make_unique<hnswlib::L2Space>(tree->dim);
  tree->index = std::make_unique<hnswlib::HierarchicalNSW<float>>(
      tree->space.get(),
      tree->rows,
      hnsw_params_.M,
      hnsw_params_.ef_construction);

  // Seed entry point single-threaded, then insert the rest in parallel (official MT pattern).
  if (tree->rows > 0) {
    tree->index->addPoint(tree->data.data(), 0);
  }
  if (tree->rows > 1) {
    tbb::parallel_for(size_t{1}, tree->rows, [&](size_t i) {
      tree->index->addPoint(tree->data.data() + i * tree->dim, i);
    });
  }
  tree->index->setEf(static_cast<size_t>(hnsw_params_.ef));
}

void ROBINMatchingHnsw::searchHnsw(const HnswIndex& tree,
                                   const Eigen::VectorXf& input,
                                   std::array<int, 2>& indices,
                                   std::array<float, 2>& dists,
                                   int nn) const {
  if (!tree.index || tree.dim == 0) {
    throw std::runtime_error("ROBINMatchingHnsw::searchHnsw: empty index");
  }
  if (static_cast<size_t>(input.size()) != tree.dim) {
    throw std::runtime_error("ROBINMatchingHnsw::searchHnsw: query dim mismatch");
  }

  // Contiguous query buffer (avoid VectorXf indirection in distance loop).
  alignas(32) float query_stack[64];
  float* query_ptr = query_stack;
  std::vector<float> query_heap;
  if (tree.dim > 64) {
    query_heap.resize(tree.dim);
    query_ptr = query_heap.data();
  }
  for (size_t i = 0; i < tree.dim; ++i) {
    query_ptr[i] = input(static_cast<Eigen::Index>(i));
  }

  auto result = tree.index->searchKnnCloserFirst(query_ptr, static_cast<size_t>(nn));
  const size_t got = result.size();
  for (int k = 0; k < nn; ++k) {
    if (static_cast<size_t>(k) < got) {
      dists[k]   = result[static_cast<size_t>(k)].first;
      indices[k] = static_cast<int>(result[static_cast<size_t>(k)].second);
    } else {
      dists[k]   = std::numeric_limits<float>::max();
      indices[k] = -1;
    }
  }
}

void ROBINMatchingHnsw::searchHnswAll(const HnswIndex& tree,
                                      const Feature& inputs,
                                      std::vector<int>& indices,
                                      std::vector<float>& dists,
                                      int nn) const {
  if (!tree.index || tree.dim == 0) {
    throw std::runtime_error("ROBINMatchingHnsw::searchHnswAll: empty index");
  }
  if (inputs.empty()) {
    indices.clear();
    dists.clear();
    return;
  }
  if (nn < 1) {
    throw std::runtime_error("ROBINMatchingHnsw::searchHnswAll: nn < 1");
  }

  const size_t n = inputs.size();
  indices.assign(n * static_cast<size_t>(nn), -1);
  dists.assign(n * static_cast<size_t>(nn), std::numeric_limits<float>::max());

  tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](tbb::blocked_range<size_t> r) {
    alignas(32) float query_stack[64];
    std::vector<float> query_heap;
    float* query_ptr = query_stack;
    if (tree.dim > 64) {
      query_heap.resize(tree.dim);
      query_ptr = query_heap.data();
    }
    for (size_t i = r.begin(); i < r.end(); ++i) {
      if (static_cast<size_t>(inputs[i].size()) != tree.dim) {
        throw std::runtime_error("ROBINMatchingHnsw::searchHnswAll: query dim mismatch");
      }
      for (size_t d = 0; d < tree.dim; ++d) {
        query_ptr[d] = inputs[i](static_cast<Eigen::Index>(d));
      }
      auto result = tree.index->searchKnnCloserFirst(query_ptr, static_cast<size_t>(nn));
      const size_t got = result.size();
      for (int k = 0; k < nn; ++k) {
        const size_t flat = i * static_cast<size_t>(nn) + static_cast<size_t>(k);
        if (static_cast<size_t>(k) < got) {
          dists[flat]   = result[static_cast<size_t>(k)].first;
          indices[flat] = static_cast<int>(result[static_cast<size_t>(k)].second);
        } else {
          dists[flat]   = std::numeric_limits<float>::max();
          indices[flat] = -1;
        }
      }
    }
  });
}

void ROBINMatchingHnsw::match(const std::string& robin_mode, float tuple_scale, bool use_ratio_test) {
  build_tree_time_    = -1.0;
  flann_search_time_  = -1.0;
  mutual_filter_time_ = -1.0;
  rejection_time_     = -1.0;

  auto t0 = std::chrono::high_resolution_clock::now();
  HnswIndex feature_tree_i;
  HnswIndex feature_tree_j;
  // Build both descriptor indexes concurrently (largest remaining wall-clock cost).
  tbb::parallel_invoke([&] { buildHnsw(features_[fi_], &feature_tree_i); },
                       [&] { buildHnsw(features_[fj_], &feature_tree_j); });
  auto t1 = std::chrono::high_resolution_clock::now();
  build_tree_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();

  int num_candidates = use_ratio_test ? 2 : 1;
  std::vector<std::array<int, 2>> corres_K(nPtj_, {0, 0});
  std::vector<std::array<int, 2>> corres_K2(nPti_, {0, 0});
  std::vector<std::array<float, 2>> dis_j(nPtj_, {0.0, 0.0});
  std::vector<std::array<float, 2>> dis_i(nPti_, {0.0, 0.0});

  std::vector<int> i_to_j_multi(nPti_, -1);
  std::vector<int> j_to_i_multi(nPtj_, -1);
  corres_cross_checked_.clear();

  std::vector<std::tuple<int, int, float>> matched_pairs;

  auto t2 = std::chrono::high_resolution_clock::now();
  tbb::parallel_for(tbb::blocked_range<size_t>(0, nPtj_), [&](tbb::blocked_range<size_t> r) {
    for (size_t j = r.begin(); j < r.end(); ++j) {
      searchHnsw(feature_tree_i, features_[fj_][j], corres_K[j], dis_j[j], num_candidates);
      bool is_over_ratio = use_ratio_test ? dis_j[j][0] > thr_ratio_test_ * dis_j[j][1] : false;
      if (dis_j[j][0] > sqr_thr_dist_ || is_over_ratio) {
        continue;
      }
      if (corres_K[j][0] >= 0 && corres_K[j][0] < nPti_) {
        if (i_to_j_multi[corres_K[j][0]] == -1) {
          searchHnsw(feature_tree_j,
                     features_[fi_][corres_K[j][0]],
                     corres_K2[corres_K[j][0]],
                     dis_i[corres_K[j][0]],
                     1);
          i_to_j_multi[corres_K[j][0]] = corres_K2[corres_K[j][0]][0];
        }
        j_to_i_multi[j] = corres_K[j][0];
      }
    }
  });
  auto t3 = std::chrono::high_resolution_clock::now();
  flann_search_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();

  matched_pairs.reserve(nPti_);
  for (ssize_t j = 0; j < nPtj_; j++) {
    int ji = j_to_i_multi[j];
    if (ji != -1 && j == i_to_j_multi[ji]) {
      float ratio = use_ratio_test ? dis_j[j][0] / dis_j[j][1] : 0.0f;
      matched_pairs.emplace_back(ji, static_cast<int>(j), ratio);
    }
  }

  if (matched_pairs.size() > num_max_corr_) {
    if (use_ratio_test) {
      std::sort(matched_pairs.begin(), matched_pairs.end(), [](const auto& a, const auto& b) {
        return std::get<2>(a) < std::get<2>(b);
      });
      matched_pairs.resize(num_max_corr_);
    } else {
      std::random_device rd;
      std::mt19937 gen(rd());
      for (size_t i = 0; i < num_max_corr_; ++i) {
        size_t j = i + gen() % (matched_pairs.size() - i);
        std::swap(matched_pairs[i], matched_pairs[j]);
      }
      matched_pairs.resize(num_max_corr_);
    }
  }

  for (const auto& corres : matched_pairs) {
    corres_cross_checked_.emplace_back(std::get<0>(corres), std::get<1>(corres));
  }
  auto t4 = std::chrono::high_resolution_clock::now();
  mutual_filter_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count();

  corres_.clear();
  auto t_rejection_init = std::chrono::high_resolution_clock::now();
  if (robin_mode == "None") {
    runTupleTest(corres_cross_checked_, corres_, tuple_scale);
  } else if (robin_mode == "max_core" || robin_mode == "max_clique") {
    applyOutlierPruning(corres_cross_checked_, corres_, robin_mode);
  } else {
    throw std::invalid_argument("Wrong ROBIN mode has come.");
  }
  auto t_rejection_end = std::chrono::high_resolution_clock::now();
  rejection_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_rejection_end - t_rejection_init)
          .count();
}

void ROBINMatchingHnsw::match_compass(const std::string& robin_mode,
                                      float tuple_scale,
                                      float angle_thr,
                                      float altitude_thr,
                                      int search_knn) {
  build_tree_time_    = -1.0;
  flann_search_time_  = -1.0;
  mutual_filter_time_ = -1.0;
  rejection_time_     = -1.0;

  auto t0 = std::chrono::high_resolution_clock::now();
  HnswIndex feature_tree_i;
  HnswIndex feature_tree_j;
  tbb::parallel_invoke([&] { buildHnsw(features_[fi_], &feature_tree_i); },
                       [&] { buildHnsw(features_[fj_], &feature_tree_j); });
  // Station path queries k=search_knn; ensure ef is at least k.
  const size_t ef_query = std::max(static_cast<size_t>(hnsw_params_.ef), static_cast<size_t>(search_knn));
  feature_tree_i.index->setEf(ef_query);
  feature_tree_j.index->setEf(ef_query);
  auto t1 = std::chrono::high_resolution_clock::now();
  build_tree_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();

  std::vector<int> corres_K;
  std::vector<float> dis_j;
  std::vector<int> corres_K2;
  std::vector<float> dis_i;

  auto t2 = std::chrono::high_resolution_clock::now();
  searchHnswAll(feature_tree_j, features_[fi_], corres_K, dis_j, search_knn);
  searchHnswAll(feature_tree_i, features_[fj_], corres_K2, dis_i, search_knn);
  auto t3 = std::chrono::high_resolution_clock::now();
  flann_search_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();

  std::vector<int> s_azimuth_angle = cacluAzimuthAngle(pointcloud_[fi_]);
  std::vector<int> t_azimuth_angle = cacluAzimuthAngle(pointcloud_[fj_]);

  if (nPtj_ > nPti_) {
    std::vector<bool> selection(corres_K.size(), false);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, selection.size()), [&](tbb::blocked_range<size_t> r) {
      for (size_t j = r.begin(); j < r.end(); ++j) {
        int r1x   = static_cast<int>(j) / search_knn;
        int t_id  = corres_K[j];
        int start = t_id * search_knn;
        int end   = start + search_knn;
        if (!findIndexExists(r1x, start, end, corres_K2)) continue;
        if (!compassError(s_azimuth_angle[r1x], t_azimuth_angle[t_id], angle_thr)) continue;
        if (!altitudeError(pointcloud_[fi_][r1x][2], pointcloud_[fj_][t_id][2], altitude_thr)) {
          continue;
        }
        selection[j] = true;
      }
    });

    corres_cross_checked_.clear();
    for (int i = 0; i < nPti_; ++i) {
      int start = i * search_knn;
      int end   = start + search_knn;
      int t_id  = cacluBestMatch(start, end, selection, dis_j);
      if (t_id < 0) continue;
      corres_cross_checked_.emplace_back(i, corres_K[t_id]);
    }
  } else {
    std::vector<bool> selection(corres_K2.size(), false);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, selection.size()), [&](tbb::blocked_range<size_t> r) {
      for (size_t j = r.begin(); j < r.end(); ++j) {
        int r1x   = static_cast<int>(j) / search_knn;
        int t_id  = corres_K2[j];
        int start = t_id * search_knn;
        int end   = start + search_knn;
        if (!findIndexExists(r1x, start, end, corres_K)) continue;
        if (!compassError(t_azimuth_angle[r1x], s_azimuth_angle[t_id], angle_thr)) continue;
        if (!altitudeError(pointcloud_[fj_][r1x][2], pointcloud_[fi_][t_id][2], altitude_thr)) {
          continue;
        }
        selection[j] = true;
      }
    });

    corres_cross_checked_.clear();
    for (int i = 0; i < nPtj_; ++i) {
      int start = i * search_knn;
      int end   = start + search_knn;
      int t_id  = cacluBestMatch(start, end, selection, dis_i);
      if (t_id < 0) continue;
      corres_cross_checked_.emplace_back(corres_K2[t_id], i);
    }
  }

  if (corres_cross_checked_.size() > num_max_corr_) {
    std::random_device rd;
    std::mt19937 gen(rd());
    for (size_t i = 0; i < num_max_corr_; ++i) {
      size_t j = i + gen() % (corres_cross_checked_.size() - i);
      std::swap(corres_cross_checked_[i], corres_cross_checked_[j]);
    }
    corres_cross_checked_.resize(num_max_corr_);
  }
  auto t4 = std::chrono::high_resolution_clock::now();
  mutual_filter_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count();

  corres_.clear();
  auto t_rejection_init = std::chrono::high_resolution_clock::now();
  if (robin_mode == "None") {
    runTupleTest(corres_cross_checked_, corres_, tuple_scale);
  } else if (robin_mode == "max_core" || robin_mode == "max_clique") {
    applyOutlierPruning(corres_cross_checked_, corres_, robin_mode);
  } else {
    throw std::invalid_argument("Wrong ROBIN mode has come.");
  }
  auto t_rejection_end = std::chrono::high_resolution_clock::now();
  rejection_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_rejection_end - t_rejection_init)
          .count();
}

std::vector<int> ROBINMatchingHnsw::cacluAzimuthAngle(std::vector<Eigen::Vector3f> const& cloud) {
  std::vector<int> ret(cloud.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud.size()), [&](tbb::blocked_range<size_t> r) {
    for (size_t j = r.begin(); j < r.end(); ++j) {
      const auto r2x = std::atan2(cloud[j][1], cloud[j][0]);
      const auto r3x = float(0.5) - r2x / float(M_PI * 2.0);
      ret[j]         = static_cast<int>(std::floor(r3x * 36000.0));
    }
  });
  return ret;
}

int ROBINMatchingHnsw::cacluBestMatch(const int& start,
                                      const int& end,
                                      std::vector<bool> const& selector,
                                      std::vector<float> const& /*distances*/) {
  for (int i = start; i < end; ++i) {
    if (selector[static_cast<size_t>(i)]) {
      return i;
    }
  }
  return -1;
}

bool ROBINMatchingHnsw::compassError(const int& s_c, const int& t_c, const float& c_thr) {
  const auto r11x = std::abs(s_c - t_c);
  const auto r12x = (s_c + 18000) % 36000;
  const auto r13x = (t_c + 18000) % 36000;
  const auto r14x = std::abs(r12x - r13x);

  float error = std::min(r11x, r14x) * 0.01f;
  return error <= c_thr;
}

bool ROBINMatchingHnsw::altitudeError(const float& s_c, const float& t_c, const float& c_thr) {
  if (c_thr <= 0.00001f || std::abs(s_c - t_c) < c_thr) return true;
  return false;
}

bool ROBINMatchingHnsw::findIndexExists(const int& s_i,
                                        const int& start,
                                        const int& end,
                                        std::vector<int> const& indices) {
  for (int i = start; i < end; ++i) {
    if (s_i == indices[static_cast<size_t>(i)]) {
      return true;
    }
  }
  return false;
}

void ROBINMatchingHnsw::setStatuses() {
  fi_ = 0;
  fj_ = 1;
  swapped_ = false;

  if (pointcloud_[fj_].size() > pointcloud_[fi_].size()) {
    size_t temp = fi_;
    fi_         = fj_;
    fj_         = temp;
    swapped_    = true;
  }

  nPti_ = static_cast<ssize_t>(pointcloud_[fi_].size());
  nPtj_ = static_cast<ssize_t>(pointcloud_[fj_].size());
}

void ROBINMatchingHnsw::runTupleTest(const std::vector<std::pair<int, int>>& corres,
                                     std::vector<std::pair<int, int>>& corres_out,
                                     const float /*tuple_scale*/) {
  if (!corres.empty()) {
    size_t rand0, rand1, rand2;
    size_t idi0, idi1, idi2;
    size_t idj0, idj1, idj2;
    size_t ncorr           = corres.size();
    size_t number_of_trial = ncorr * 100;
    std::vector<std::pair<int, int>> corres_tuple;
    corres_tuple.reserve(ncorr);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> distribution(0, ncorr - 1);

    std::vector<bool> is_already_included(ncorr, false);
    corres_out.clear();

    for (size_t i = 0; i < number_of_trial; i++) {
      rand0 = distribution(gen);
      rand1 = distribution(gen);
      rand2 = distribution(gen);

      idi0 = corres[rand0].first;
      idj0 = corres[rand0].second;
      idi1 = corres[rand1].first;
      idj1 = corres[rand1].second;
      idi2 = corres[rand2].first;
      idj2 = corres[rand2].second;

      if (!isValidIndex(idi0, nPti_) || !isValidIndex(idi1, nPti_) || !isValidIndex(idi2, nPti_) ||
          !isValidIndex(idj0, nPtj_) || !isValidIndex(idj1, nPtj_) || !isValidIndex(idj2, nPtj_)) {
        continue;
      }

      const Eigen::Vector3f& pti0 = pointcloud_[fi_][idi0];
      const Eigen::Vector3f& pti1 = pointcloud_[fi_][idi1];
      const Eigen::Vector3f& pti2 = pointcloud_[fi_][idi2];
      float li0 = (pti0 - pti1).norm();
      float li1 = (pti1 - pti2).norm();
      float li2 = (pti2 - pti0).norm();

      const Eigen::Vector3f& ptj0 = pointcloud_[fj_][idj0];
      const Eigen::Vector3f& ptj1 = pointcloud_[fj_][idj1];
      const Eigen::Vector3f& ptj2 = pointcloud_[fj_][idj2];
      float lj0 = (ptj0 - ptj1).norm();
      float lj1 = (ptj1 - ptj2).norm();
      float lj2 = (ptj2 - ptj0).norm();

      static float thr = noise_bound_;
      if (li0 - thr > lj0 || lj0 > li0 + thr) continue;
      if (li1 - thr > lj1 || lj1 > li1 + thr) continue;
      if (li2 - thr > lj2 || lj2 > li2 + thr) continue;

      if (!is_already_included[rand0]) {
        corres_tuple.emplace_back(idi0, idj0);
        is_already_included[rand0] = true;
      }
      if (!is_already_included[rand1]) {
        corres_tuple.emplace_back(idi1, idj1);
        is_already_included[rand1] = true;
      }
      if (!is_already_included[rand2]) {
        corres_tuple.emplace_back(idi2, idj2);
        is_already_included[rand2] = true;
      }
    }
    corres_out.clear();
    for (size_t i = 0; i < corres_tuple.size(); ++i) {
      if (swapped_) {
        corres_out.emplace_back(corres_tuple[i].second, corres_tuple[i].first);
      } else {
        corres_out.emplace_back(corres_tuple[i].first, corres_tuple[i].second);
      }
    }
  } else {
    throw std::invalid_argument("Wrong tuple constraint has come.");
  }
}

void ROBINMatchingHnsw::applyOutlierPruning(const std::vector<std::pair<int, int>>& corres,
                                            std::vector<std::pair<int, int>>& corres_out,
                                            const std::string& robin_mode) {
  if (!corres.empty()) {
    size_t ncorr = corres.size();
    corres_out.clear();

    Eigen::Matrix<double, 3, Eigen::Dynamic> src_robin(3, ncorr);
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_robin(3, ncorr);

#pragma omp parallel for
    for (size_t i = 0; i < ncorr; ++i) {
      src_robin.col(i) = pointcloud_[fi_][corres[i].first].cast<double>();
      tgt_robin.col(i) = pointcloud_[fj_][corres[i].second].cast<double>();
    }

    robin::IGraph* inlier_graph = robin::Make3dRegInvGraph(src_robin, tgt_robin, noise_bound_);
    const auto& filtered_indices = [&]() {
      if (robin_mode == "max_core") {
        return robin::FindInlierStructure(inlier_graph, robin::InlierGraphStructure::MAX_CORE);
      } else if (robin_mode == "max_clique") {
        return robin::FindInlierStructure(inlier_graph, robin::InlierGraphStructure::MAX_CLIQUE);
      } else {
        throw std::runtime_error("Something's wrong!");
      }
    }();

    for (size_t i = 0; i < filtered_indices.size(); ++i) {
      const auto& corres_pair = corres[filtered_indices[i]];
      if (swapped_) {
        corres_out.emplace_back(corres_pair.second, corres_pair.first);
      } else {
        corres_out.emplace_back(corres_pair.first, corres_pair.second);
      }
    }

    num_init_corr_   = ncorr;
    num_pruned_corr_ = filtered_indices.size();
  } else {
    throw std::invalid_argument("Wrong tuple constraint has come.");
  }
}

std::vector<size_t> ROBINMatchingHnsw::applyOutlierPruning(
    const std::vector<Eigen::Vector3f>& src_matched,
    const std::vector<Eigen::Vector3f>& tgt_matched,
    const std::string& robin_mode) {
  if (src_matched.size() != tgt_matched.size()) {
    throw std::runtime_error("The size of `src_matched` and `tgt_matched` should be same.");
  }
  if (src_matched.size() < 2 || tgt_matched.size() < 2) {
    throw std::runtime_error("Too few matched points are given.");
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> src_robin(3, src_matched.size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_robin(3, tgt_matched.size());

  num_init_corr_ = src_matched.size();
#pragma omp parallel for
  for (size_t i = 0; i < num_init_corr_; ++i) {
    src_robin.col(i) = src_matched[i].cast<double>();
    tgt_robin.col(i) = tgt_matched[i].cast<double>();
  }

  robin::IGraph* inlier_graph = robin::Make3dRegInvGraph(src_robin, tgt_robin, noise_bound_);
  const auto& filtered_indices = [&]() {
    if (robin_mode == "max_core") {
      return robin::FindInlierStructure(inlier_graph, robin::InlierGraphStructure::MAX_CORE);
    } else if (robin_mode == "max_clique") {
      return robin::FindInlierStructure(inlier_graph, robin::InlierGraphStructure::MAX_CLIQUE);
    } else {
      throw std::runtime_error("Something's wrong!");
    }
  }();

  num_pruned_corr_ = filtered_indices.size();
  return filtered_indices;
}

}  // namespace robust_matcher
