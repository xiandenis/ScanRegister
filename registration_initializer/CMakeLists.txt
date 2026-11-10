# Default SHARED per repo shared-library policy.
# AngleDownsample is inlined here until cloud_sample lands in this tree.
add_library(registration_initializer SHARED
  registration_initializer.cpp
  normal_estimate.cpp
  angle_downsample.cpp
)

target_include_directories(registration_initializer PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(registration_initializer PUBLIC
  common
  logging
  file_system
  registration_type
  point_cloud_io
  depth_map_tool
  resolution_estimate
  octree
  small_gicp
  pcl_common
  pcl_io
  pcl_features
  opencv_world
  Eigen3::Eigen
  TBB::tbb
)

set_target_properties(registration_initializer PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

scan_register_apply_compat(registration_initializer)
scan_register_apply_openmp(registration_initializer)

if(MSVC)
  target_compile_options(registration_initializer PRIVATE /utf-8)
  target_compile_definitions(registration_initializer PRIVATE _USE_MATH_DEFINES)
endif()

option(BUILD_REGISTRATION_INITIALIZER_TEST "Build registration_initializer tests" ${SCAN_REGISTER_IS_TOP_LEVEL})

if(BUILD_REGISTRATION_INITIALIZER_TEST)
  add_executable(registration_initializer_test registration_initializer_test.cpp)
  target_link_libraries(registration_initializer_test PRIVATE registration_initializer)
  scan_register_apply_compat(registration_initializer_test)
  if(MSVC)
    target_compile_options(registration_initializer_test PRIVATE /utf-8)
    set_target_properties(registration_initializer_test PROPERTIES LINK_FLAGS "/ENTRY:wmainCRTStartup")
  endif()

  add_executable(normal_estimate_test normal_estimate_test.cpp)
  target_link_libraries(normal_estimate_test PRIVATE registration_initializer)
  scan_register_apply_compat(normal_estimate_test)
  if(MSVC)
    target_compile_options(normal_estimate_test PRIVATE /utf-8)
  endif()
endif()
