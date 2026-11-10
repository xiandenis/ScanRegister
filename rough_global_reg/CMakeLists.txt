# Explicit STATIC (business facade over robust_global_reg; matches domain leaf convention).
add_library(rough_global_reg STATIC
  rough_global_reg.cpp
)

target_include_directories(rough_global_reg PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(rough_global_reg PUBLIC
  Eigen3::Eigen
  registration_type
  robust_global_reg
  point_cloud_io
  file_system
  logging
)

set_target_properties(rough_global_reg PROPERTIES POSITION_INDEPENDENT_CODE ON)

scan_register_apply_compat(rough_global_reg)
scan_register_apply_openmp(rough_global_reg)

if(MSVC)
  target_compile_options(rough_global_reg PRIVATE /utf-8)
endif()

option(BUILD_ROUGH_GLOBAL_REG_TEST "Build rough_global_reg_test executable" ${SCAN_REGISTER_IS_TOP_LEVEL})

if(BUILD_ROUGH_GLOBAL_REG_TEST)
  add_executable(rough_global_reg_test rough_global_reg_test.cpp)
  target_link_libraries(rough_global_reg_test PRIVATE
    rough_global_reg
    registration_initializer
  )
  scan_register_apply_compat(rough_global_reg_test)
  if(MSVC)
    target_compile_options(rough_global_reg_test PRIVATE /utf-8)
    set_target_properties(rough_global_reg_test PROPERTIES LINK_FLAGS "/ENTRY:wmainCRTStartup")
  endif()
endif()
