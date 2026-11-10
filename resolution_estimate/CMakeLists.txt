# Default SHARED per repo shared-library policy.
add_library(resolution_estimate SHARED
  resolution_estimate.cpp
  percentile_resolution_estimate.cpp
  histogram.cpp
)

target_include_directories(resolution_estimate PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(resolution_estimate PUBLIC
  logging
  pcl_common
  Eigen3::Eigen
)

set_target_properties(resolution_estimate PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

scan_register_apply_compat(resolution_estimate)
scan_register_apply_openmp(resolution_estimate)

if(MSVC)
  target_compile_options(resolution_estimate PRIVATE /utf-8)
endif()

option(BUILD_RESOLUTION_ESTIMATE_TEST "Build resolution_estimate_test executable" ${SCAN_REGISTER_IS_TOP_LEVEL})

if(BUILD_RESOLUTION_ESTIMATE_TEST)
  add_executable(resolution_estimate_test resolution_estimate_test.cpp)
  target_link_libraries(resolution_estimate_test PRIVATE
    resolution_estimate
    point_cloud_io
    pcl_io
    pcl_features
  )
  scan_register_apply_compat(resolution_estimate_test)
  scan_register_apply_openmp(resolution_estimate_test)
  if(MSVC)
    target_compile_options(resolution_estimate_test PRIVATE /utf-8)
    set_target_properties(resolution_estimate_test PROPERTIES LINK_FLAGS "/ENTRY:wmainCRTStartup")
  endif()
endif()
