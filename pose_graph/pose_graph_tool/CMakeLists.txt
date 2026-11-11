# Default SHARED per repo shared-library policy.
add_library(pose_graph_tool SHARED
  pose_base.cpp
  pose_global_system.cpp
  pose_graph_grouper.cpp
)

target_include_directories(pose_graph_tool PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_POSE_GRAPH_ROOT}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(pose_graph_tool PUBLIC
  common
  logging
  registration_type
  graph
  Eigen3::Eigen
)

set_target_properties(pose_graph_tool PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

scan_register_apply_compat(pose_graph_tool)

if(MSVC)
  target_compile_options(pose_graph_tool PRIVATE /utf-8)
endif()
