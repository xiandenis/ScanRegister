# Default SHARED per repo shared-library policy.
add_library(fastreg_filter SHARED
  spatial_perspective_filter.cpp
  panorama_filter.cpp
)

target_include_directories(fastreg_filter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(fastreg_filter PUBLIC
  logging
  opencv_world
  Eigen3::Eigen
)

target_compile_definitions(fastreg_filter PRIVATE _USE_MATH_DEFINES)

set_target_properties(fastreg_filter PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

scan_register_apply_compat(fastreg_filter)
scan_register_apply_openmp(fastreg_filter)

if(MSVC)
  target_compile_options(fastreg_filter PRIVATE /utf-8)
endif()
