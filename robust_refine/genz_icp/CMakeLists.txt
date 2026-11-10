# Explicit STATIC per project decision (robust_refine leaf).
add_library(genz_icp STATIC
  genz_icp.cpp
  core/Registration.cpp
  core/Deskew.cpp
  core/VoxelHashMap.cpp
  core/Preprocessing.cpp
  core/Threshold.cpp
  pipeline/GenZICP.cpp
)
add_library(genz_icp::genz_icp ALIAS genz_icp)

target_include_directories(genz_icp PUBLIC
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_REFINE_ROOT}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/sophus>"
)

target_compile_definitions(genz_icp PUBLIC SOPHUS_USE_BASIC_LOGGING)

target_link_libraries(genz_icp PUBLIC
  Eigen3::Eigen
  TBB::tbb
  matcher_tsl
  pcl_common
)

set_target_properties(genz_icp PROPERTIES POSITION_INDEPENDENT_CODE ON)

scan_register_apply_compat(genz_icp)

if(MSVC)
  target_compile_options(genz_icp PRIVATE /utf-8 /bigobj)
endif()

option(BUILD_GENZ_ICP_COMPILE_TEST "Build genz_icp compile smoke test" ${SCAN_REGISTER_IS_TOP_LEVEL})
if(BUILD_GENZ_ICP_COMPILE_TEST)
  add_executable(genz_icp_compile_test genz_icp_compile_test.cpp)
  target_link_libraries(genz_icp_compile_test PRIVATE genz_icp)
  scan_register_apply_compat(genz_icp_compile_test)
  if(MSVC)
    target_compile_options(genz_icp_compile_test PRIVATE /utf-8)
  endif()
endif()

# Requires point_cloud_io (not yet in this repo).
option(BUILD_GENZ_ICP_PAIR "Build genz_icp pair registration tool" OFF)
if(BUILD_GENZ_ICP_PAIR)
  if(NOT TARGET point_cloud_io)
    message(FATAL_ERROR "BUILD_GENZ_ICP_PAIR=ON requires target point_cloud_io")
  endif()
  add_executable(genz_icp_pair genz_icp_pair.cpp)
  target_link_libraries(genz_icp_pair PRIVATE genz_icp point_cloud_io)
  scan_register_apply_compat(genz_icp_pair)
  if(MSVC)
    target_compile_options(genz_icp_pair PRIVATE /utf-8)
  endif()
endif()
