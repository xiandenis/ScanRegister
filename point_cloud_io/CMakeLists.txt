# Point cloud I/O (LAS/SLAS/PLY/PCD/PCSE). Default SHARED per repo policy.
add_library(point_cloud_io SHARED
  point_cloud_io.cpp
  las_fast_reader.cpp
  pcse_reader.cpp
  point_cloud_stream_convert.cpp
)

target_include_directories(point_cloud_io PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(point_cloud_io PUBLIC
  logging
  pcl_io
  pcl_common
  Eigen3::Eigen
  pdalcpp
)

if(SCAN_REGISTER_RAPIDJSON_INCLUDE)
  target_include_directories(point_cloud_io PRIVATE "${SCAN_REGISTER_RAPIDJSON_INCLUDE}")
elseif(SCAN_REGISTER_THIRDPARTY_INCLUDES)
  target_include_directories(point_cloud_io PRIVATE ${SCAN_REGISTER_THIRDPARTY_INCLUDES})
else()
  message(FATAL_ERROR "RapidJSON not found (need rapidjson/document.h)")
endif()

set_target_properties(point_cloud_io PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

scan_register_apply_compat(point_cloud_io)

if(MSVC)
  target_compile_options(point_cloud_io PRIVATE /utf-8)
endif()
