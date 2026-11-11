# Default SHARED per repo shared-library policy.
add_library(pose_graph_opt SHARED
  se3_math.cpp
  ceres_pose_graph_solver.cpp
  pose_graph_optimizer.cpp
  align_result_loader.cpp
)

target_include_directories(pose_graph_opt PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_POSE_GRAPH_ROOT}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(pose_graph_opt PUBLIC
  logging
  registration_type
  Eigen3::Eigen
  Ceres::ceres
)

set_target_properties(pose_graph_opt PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

scan_register_apply_compat(pose_graph_opt)

if(MSVC)
  target_compile_options(pose_graph_opt PRIVATE /utf-8)
endif()

option(BUILD_POSE_GRAPH_OPT_TEST "Build pose_graph_opt tests" ON)
if(BUILD_POSE_GRAPH_OPT_TEST AND SCAN_REGISTER_IS_TOP_LEVEL)
  add_executable(se3_math_test se3_math_test.cpp)
  target_link_libraries(se3_math_test PRIVATE pose_graph_opt)
  scan_register_apply_compat(se3_math_test)
  if(MSVC)
    target_compile_options(se3_math_test PRIVATE /utf-8)
  endif()

  add_executable(pose_graph_opt_test pose_graph_opt_test.cpp)
  target_link_libraries(pose_graph_opt_test PRIVATE pose_graph_opt)
  scan_register_apply_compat(pose_graph_opt_test)
  if(MSVC)
    target_compile_options(pose_graph_opt_test PRIVATE /utf-8)
    # Ceres NumericDiff uses large stack frames; default 1MB overflows after optimize().
    target_link_options(pose_graph_opt_test PRIVATE /STACK:8388608)
  endif()
endif()
