# Default SHARED per repo shared-library policy（仓库根；不迁入 robust_refine/）。
add_library(trim_icp SHARED
  trim_icp.cpp
  trim_icp_solver.cpp
  overlap_voxel_crop.cpp
  knn_normal_overlap_crop.cpp
  implicit_surface_report.cpp
  explicit_surface_report.cpp
  registration_report.cpp
  overlap_estimator.cpp
)
add_library(trim_icp::trim_icp ALIAS trim_icp)

# #include <trim_icp/...>  → 仓库根 SCAN_REGISTER_INCLUDE_ROOT
target_include_directories(trim_icp PUBLIC
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(trim_icp PUBLIC
  basic_icp
  octree
  Eigen3::Eigen
  pcl_common
  pcl_search
  pcl_io
)

set_target_properties(trim_icp PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

scan_register_apply_compat(trim_icp)
scan_register_apply_openmp(trim_icp)

if(MSVC)
  target_compile_options(trim_icp PRIVATE /utf-8 /bigobj)
endif()

option(BUILD_TRIM_ICP_COMPILE_TEST "Build trim_icp compile smoke test" ${SCAN_REGISTER_IS_TOP_LEVEL})
if(BUILD_TRIM_ICP_COMPILE_TEST)
  add_executable(trim_icp_compile_test trim_icp_compile_test.cpp)
  target_link_libraries(trim_icp_compile_test PRIVATE trim_icp)
  scan_register_apply_compat(trim_icp_compile_test)
  scan_register_apply_openmp(trim_icp_compile_test)
  if(MSVC)
    target_compile_options(trim_icp_compile_test PRIVATE /utf-8)
  endif()
endif()

option(BUILD_TRIM_ICP_OVERLAP_TEST "Build trim_icp overlap/behavior test" ${SCAN_REGISTER_IS_TOP_LEVEL})
if(BUILD_TRIM_ICP_OVERLAP_TEST)
  add_executable(trim_icp_overlap_test trim_icp_overlap_test.cpp)
  target_link_libraries(trim_icp_overlap_test PRIVATE trim_icp)
  scan_register_apply_compat(trim_icp_overlap_test)
  scan_register_apply_openmp(trim_icp_overlap_test)
  if(MSVC)
    target_compile_options(trim_icp_overlap_test PRIVATE /utf-8)
  endif()
endif()

option(BUILD_TRIM_ICP_OVERLAP_BENCH "Build trim_icp overlap crop micro-benchmark" ${SCAN_REGISTER_IS_TOP_LEVEL})
if(BUILD_TRIM_ICP_OVERLAP_BENCH)
  add_executable(trim_icp_overlap_bench trim_icp_overlap_bench.cpp)
  target_link_libraries(trim_icp_overlap_bench PRIVATE trim_icp)
  scan_register_apply_compat(trim_icp_overlap_bench)
  scan_register_apply_openmp(trim_icp_overlap_bench)
  if(MSVC)
    target_compile_options(trim_icp_overlap_bench PRIVATE /utf-8)
  endif()
endif()

option(BUILD_TRIM_ICP_PAIR_BENCH "Build trim_icp single-pair timing bench" ${SCAN_REGISTER_IS_TOP_LEVEL})
if(BUILD_TRIM_ICP_PAIR_BENCH)
  add_executable(trim_icp_pair_bench trim_icp_pair_bench.cpp)
  target_link_libraries(trim_icp_pair_bench PRIVATE trim_icp)
  scan_register_apply_compat(trim_icp_pair_bench)
  scan_register_apply_openmp(trim_icp_pair_bench)
  if(MSVC)
    target_compile_options(trim_icp_pair_bench PRIVATE /utf-8)
  endif()
endif()
