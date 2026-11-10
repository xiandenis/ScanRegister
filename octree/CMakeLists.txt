# Header-only octree / voxel downsample utilities (Medoid hash grid, FastOctree, CenterNearest).
add_library(octree INTERFACE)

target_include_directories(octree INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(octree INTERFACE
  pcl_common
  Eigen3::Eigen
  TBB::tbb
)

if(NOT DEFINED BUILD_CENTER_NEAREST_OCTREE_TEST)
  option(BUILD_CENTER_NEAREST_OCTREE_TEST "Build center_nearest_octree_test executable" OFF)
endif()
if(NOT DEFINED BUILD_CENTER_NEAREST_OCTREE_UNIT_TEST)
  option(BUILD_CENTER_NEAREST_OCTREE_UNIT_TEST "Build center_nearest_octree unit tests" ON)
endif()

if(BUILD_CENTER_NEAREST_OCTREE_UNIT_TEST AND SCAN_REGISTER_IS_TOP_LEVEL)
  add_executable(center_nearest_octree_unit_test center_nearest_octree_unit_test.cpp)
  target_link_libraries(center_nearest_octree_unit_test PRIVATE octree)
  scan_register_apply_compat(center_nearest_octree_unit_test)
  if(MSVC)
    target_compile_options(center_nearest_octree_unit_test PRIVATE /utf-8)
  endif()
endif()

option(BUILD_VOXEL_HASH_MEDOID_TEST "Build voxel_hash_medoid_test executable" ON)
if(BUILD_VOXEL_HASH_MEDOID_TEST AND SCAN_REGISTER_IS_TOP_LEVEL)
  add_executable(voxel_hash_medoid_test voxel_hash_medoid_test.cpp)
  target_link_libraries(voxel_hash_medoid_test PRIVATE octree)
  scan_register_apply_compat(voxel_hash_medoid_test)
  if(MSVC)
    target_compile_options(voxel_hash_medoid_test PRIVATE /utf-8)
  endif()
endif()

if(BUILD_CENTER_NEAREST_OCTREE_TEST AND SCAN_REGISTER_IS_TOP_LEVEL)
  if(NOT TARGET point_cloud_io)
    message(WARNING
      "BUILD_CENTER_NEAREST_OCTREE_TEST=ON but target 'point_cloud_io' is missing; "
      "skipping center_nearest_octree_test")
  else()
    add_executable(center_nearest_octree_test center_nearest_octree_test.cpp)
    target_link_libraries(center_nearest_octree_test PRIVATE octree point_cloud_io)
    scan_register_apply_compat(center_nearest_octree_test)
    if(MSVC)
      target_compile_options(center_nearest_octree_test PRIVATE /utf-8)
      set_target_properties(center_nearest_octree_test PROPERTIES LINK_FLAGS "/ENTRY:wmainCRTStartup")
    endif()
  endif()
endif()
