# Explicit STATIC (module CMake as provided / global pose JSON I/O leaf).
add_library(global_matrix_file STATIC
  global_matrix_file_io.cpp
)

target_include_directories(global_matrix_file PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

if(SCAN_REGISTER_RAPIDJSON_INCLUDE)
  target_include_directories(global_matrix_file PRIVATE "${SCAN_REGISTER_RAPIDJSON_INCLUDE}")
elseif(SCAN_REGISTER_THIRDPARTY_INCLUDES)
  target_include_directories(global_matrix_file PRIVATE ${SCAN_REGISTER_THIRDPARTY_INCLUDES})
else()
  message(FATAL_ERROR "RapidJSON not found (need rapidjson/document.h)")
endif()

target_link_libraries(global_matrix_file PUBLIC
  common
  logging
  Eigen3::Eigen
)

set_target_properties(global_matrix_file PROPERTIES POSITION_INDEPENDENT_CODE ON)

scan_register_apply_compat(global_matrix_file)

if(MSVC)
  target_compile_options(global_matrix_file PRIVATE /utf-8)
endif()
