# Explicit STATIC (module CMake as provided / panorama depth-map utility leaf).
add_library(depth_map_tool STATIC
  depth_map_tool.cpp
)

target_include_directories(depth_map_tool PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(depth_map_tool PUBLIC
  common
  opencv_world
  pcl_common
  TBB::tbb
  Eigen3::Eigen
)

target_compile_definitions(depth_map_tool PRIVATE _USE_MATH_DEFINES)

set_target_properties(depth_map_tool PROPERTIES POSITION_INDEPENDENT_CODE ON)

scan_register_apply_compat(depth_map_tool)

if(MSVC)
  target_compile_options(depth_map_tool PRIVATE /utf-8)
endif()
