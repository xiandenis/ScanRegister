set(ROBUST_MATCHER_SOURCES
  FasterPFH.cpp
  GncSolver.cpp
  ROBINMatching.cpp
  ROBINMatchingHnsw.cpp
  RobustMatcher.cpp
)

# Explicit STATIC per project decision (robust_rough leaf).
add_library(robust_matcher STATIC ${ROBUST_MATCHER_SOURCES})

target_include_directories(robust_matcher PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_ROUGH_ROOT}>"
  ${SCAN_REGISTER_THIRDPARTY_INCLUDES}
)

target_link_libraries(robust_matcher PUBLIC
  robin
  matcher_points
  matcher_kdtree
  matcher_hnswlib
  matcher_tsl
  Eigen3::Eigen
  TBB::tbb
  flann::flann_cpp
)

set_target_properties(robust_matcher PROPERTIES POSITION_INDEPENDENT_CODE ON)

option(BUILD_AB_FEATURE_KNN_BENCH "Build FLANN vs HNSW feature knn microbench" ${SCAN_REGISTER_IS_TOP_LEVEL})
if(BUILD_AB_FEATURE_KNN_BENCH)
  add_executable(ab_feature_knn_bench ab_feature_knn_bench.cpp)
  target_link_libraries(ab_feature_knn_bench PRIVATE
    robust_matcher
    Eigen3::Eigen
    TBB::tbb
    flann::flann_cpp
    matcher_hnswlib
  )
  scan_register_apply_compat(ab_feature_knn_bench)
endif()

scan_register_apply_compat(robust_matcher)
scan_register_apply_openmp(robust_matcher)

if(MSVC)
  target_compile_definitions(robust_matcher PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
