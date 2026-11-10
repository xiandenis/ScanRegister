# Explicit STATIC (module CMake as provided / coarse global registration leaf).
add_library(robust_global_reg STATIC
  robGlob_reg_parameter.cpp
  t_coarse_check.cpp
  robust_global_reg.cpp
)

target_include_directories(robust_global_reg PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_ROUGH_ROOT}>"
)

if(SCAN_REGISTER_RAPIDJSON_INCLUDE)
  target_include_directories(robust_global_reg PRIVATE "${SCAN_REGISTER_RAPIDJSON_INCLUDE}")
elseif(SCAN_REGISTER_THIRDPARTY_INCLUDES)
  target_include_directories(robust_global_reg PRIVATE ${SCAN_REGISTER_THIRDPARTY_INCLUDES})
endif()

target_link_libraries(robust_global_reg PUBLIC
  common
  logging
  file_system
  point_cloud_io
  robust_matcher
  Eigen3::Eigen
)

set_target_properties(robust_global_reg PROPERTIES POSITION_INDEPENDENT_CODE ON)

scan_register_apply_compat(robust_global_reg)
scan_register_apply_openmp(robust_global_reg)

if(MSVC)
  target_compile_options(robust_global_reg PRIVATE /utf-8)
endif()

option(BUILD_ROBUST_GLOBAL_REG_TEST "Build robust_global_reg_test executable" OFF)
if(BUILD_ROBUST_GLOBAL_REG_TEST)
  add_executable(robust_global_reg_test robust_global_reg_test.cpp)
  target_link_libraries(robust_global_reg_test PRIVATE robust_global_reg)
  scan_register_apply_compat(robust_global_reg_test)
  if(MSVC)
    target_compile_options(robust_global_reg_test PRIVATE /utf-8)
    set_target_properties(robust_global_reg_test PROPERTIES LINK_FLAGS "/ENTRY:wmainCRTStartup")
  endif()
endif()

option(BUILD_GLOBAL_FAST_GICP_PIPELINE_TEST
  "Build global reg + FastGICP end-to-end pipeline test" OFF)
if(BUILD_GLOBAL_FAST_GICP_PIPELINE_TEST)
  if(NOT TARGET fast_gicp OR NOT TARGET octree OR NOT TARGET overlap_estimation)
    message(FATAL_ERROR
      "BUILD_GLOBAL_FAST_GICP_PIPELINE_TEST=ON requires fast_gicp, octree, overlap_estimation")
  endif()
  add_executable(global_fast_gicp_pipeline_test global_fast_gicp_pipeline_test.cpp)
  target_link_libraries(global_fast_gicp_pipeline_test PRIVATE
    robust_global_reg
    fast_gicp
    octree
    overlap_estimation
    pcl_io
  )
  scan_register_apply_compat(global_fast_gicp_pipeline_test)
  scan_register_apply_openmp(global_fast_gicp_pipeline_test)
  if(MSVC)
    target_compile_options(global_fast_gicp_pipeline_test PRIVATE /utf-8)
    set_target_properties(global_fast_gicp_pipeline_test PROPERTIES LINK_FLAGS "/ENTRY:wmainCRTStartup")
  endif()
endif()

option(BUILD_GLOBAL_IMLS_ICP_PIPELINE_TEST
  "Build global reg + IMLS-ICP end-to-end pipeline test" OFF)
option(BUILD_GLOBAL_FINE_REG_BENCHMARK_TEST
  "Build FastGICP vs IMLS-ICP fine registration benchmark" OFF)

if(BUILD_GLOBAL_IMLS_ICP_PIPELINE_TEST)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/global_imls_icp_pipeline_test.cpp")
    message(WARNING "BUILD_GLOBAL_IMLS_ICP_PIPELINE_TEST=ON but source is missing; skipping")
  elseif(NOT TARGET imls_icp)
    message(WARNING "BUILD_GLOBAL_IMLS_ICP_PIPELINE_TEST=ON but target imls_icp is missing; skipping")
  else()
    add_executable(global_imls_icp_pipeline_test global_imls_icp_pipeline_test.cpp)
    target_link_libraries(global_imls_icp_pipeline_test PRIVATE
      robust_global_reg
      imls_icp
      octree
      overlap_estimation
    )
    scan_register_apply_compat(global_imls_icp_pipeline_test)
    scan_register_apply_openmp(global_imls_icp_pipeline_test)
    if(MSVC)
      target_compile_options(global_imls_icp_pipeline_test PRIVATE /utf-8)
      set_target_properties(global_imls_icp_pipeline_test PROPERTIES LINK_FLAGS "/ENTRY:wmainCRTStartup")
    endif()
  endif()
endif()

if(BUILD_GLOBAL_FINE_REG_BENCHMARK_TEST)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/global_fine_reg_benchmark_test.cpp")
    message(WARNING "BUILD_GLOBAL_FINE_REG_BENCHMARK_TEST=ON but source is missing; skipping")
  elseif(NOT TARGET imls_icp)
    message(WARNING "BUILD_GLOBAL_FINE_REG_BENCHMARK_TEST=ON but target imls_icp is missing; skipping")
  else()
    add_executable(global_fine_reg_benchmark_test global_fine_reg_benchmark_test.cpp)
    target_link_libraries(global_fine_reg_benchmark_test PRIVATE
      robust_global_reg
      fast_gicp
      imls_icp
      octree
    )
    scan_register_apply_compat(global_fine_reg_benchmark_test)
    scan_register_apply_openmp(global_fine_reg_benchmark_test)
    if(MSVC)
      target_compile_options(global_fine_reg_benchmark_test PRIVATE /utf-8)
      set_target_properties(global_fine_reg_benchmark_test PROPERTIES LINK_FLAGS "/ENTRY:wmainCRTStartup")
    endif()
  endif()
endif()
