# Explicit STATIC per project decision (robust_refine leaf).
add_library(basic_icp STATIC
  basic_icp.cpp
)

target_include_directories(basic_icp PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_REFINE_ROOT}>"
)
target_include_directories(basic_icp PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(basic_icp PUBLIC
  Eigen3::Eigen
)

set_target_properties(basic_icp PROPERTIES POSITION_INDEPENDENT_CODE ON)

scan_register_apply_compat(basic_icp)
scan_register_apply_openmp(basic_icp)

if(MSVC)
  target_compile_options(basic_icp PRIVATE /bigobj /utf-8)
endif()

option(BUILD_BASIC_ICP_GUARD_TEST "Build basic_icp guard / regression test" ${SCAN_REGISTER_IS_TOP_LEVEL})
if(BUILD_BASIC_ICP_GUARD_TEST)
  add_executable(basic_icp_guard_test basic_icp_guard_test.cpp)
  target_link_libraries(basic_icp_guard_test PRIVATE basic_icp)
  scan_register_apply_compat(basic_icp_guard_test)
  scan_register_apply_openmp(basic_icp_guard_test)
  if(MSVC)
    target_compile_options(basic_icp_guard_test PRIVATE /utf-8)
  endif()
endif()
