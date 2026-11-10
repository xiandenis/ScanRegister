# Shared registration_type: types / IO used across static modules and Session consumers.
# STATIC would duplicate type RTTI / IO across boundaries when mixed with other SHARED libs.
add_library(registration_type SHARED
  registration_type.cpp
  lidar_frame_io.cpp
  align_result_io.cpp
  station_status_io.cpp
)

target_include_directories(registration_type PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

if(SCAN_REGISTER_RAPIDJSON_INCLUDE)
  target_include_directories(registration_type PRIVATE "${SCAN_REGISTER_RAPIDJSON_INCLUDE}")
elseif(SCAN_REGISTER_THIRDPARTY_INCLUDES)
  target_include_directories(registration_type PRIVATE ${SCAN_REGISTER_THIRDPARTY_INCLUDES})
else()
  message(FATAL_ERROR "RapidJSON not found (need rapidjson/document.h)")
endif()

target_link_libraries(registration_type PUBLIC
  common
  logging
  Eigen3::Eigen
  pcl_common
  pcl_search
)

# MSVC: export all public symbols from the DLL without per-symbol REGISTRATION_TYPE_API.
# Linux/macOS: default visibility exports out-of-line symbols from the .so.
target_compile_definitions(registration_type
  PRIVATE REGISTRATION_TYPE_BUILD
)

set_target_properties(registration_type PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

scan_register_apply_compat(registration_type)

if(MSVC)
  target_compile_options(registration_type PRIVATE /utf-8)
endif()
