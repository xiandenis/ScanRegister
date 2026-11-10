add_library(common SHARED
  common.cpp
  defines.cpp
  transformations_3d.cpp
  panorama_util.cpp
  e57_scan_geometry.cpp
  random_3d.cpp
  mem_cost_model.cpp
  mem_aware_scheduler.cpp
)

target_include_directories(common PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(common PUBLIC
  logging
  Eigen3::Eigen
)

target_compile_definitions(common PRIVATE _USE_MATH_DEFINES)

set_target_properties(common PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

scan_register_apply_compat(common)

if(MSVC)
  target_compile_options(common PRIVATE /utf-8)
endif()

option(BUILD_COMMON_TEST "Build common tests" ON)
if(BUILD_COMMON_TEST AND SCAN_REGISTER_IS_TOP_LEVEL)
  add_executable(e57_scan_geometry_test e57_scan_geometry_test.cpp)
  target_link_libraries(e57_scan_geometry_test PRIVATE common)
  scan_register_apply_compat(e57_scan_geometry_test)
  if(MSVC)
    target_compile_options(e57_scan_geometry_test PRIVATE /utf-8)
  endif()

  add_executable(mem_aware_scheduler_test mem_aware_scheduler_test.cpp)
  target_link_libraries(mem_aware_scheduler_test PRIVATE common)
  scan_register_apply_compat(mem_aware_scheduler_test)
  if(MSVC)
    target_compile_options(mem_aware_scheduler_test PRIVATE /utf-8)
  endif()

  add_executable(working_frame_test working_frame_test.cpp)
  target_link_libraries(working_frame_test PRIVATE common)
  scan_register_apply_compat(working_frame_test)
  if(MSVC)
    target_compile_options(working_frame_test PRIVATE /utf-8)
  endif()
endif()
