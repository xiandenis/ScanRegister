# Header-only Fast-GICP / NDT stack. Kept as INTERFACE (no translation units to archive).
add_library(fast_gicp INTERFACE)

target_include_directories(fast_gicp INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_REFINE_ROOT}>"
)

target_link_libraries(fast_gicp INTERFACE
  pcl_common
  pcl_search
  pcl_registration
  Eigen3::Eigen
)

if(TARGET OpenMP::OpenMP_CXX)
  target_link_libraries(fast_gicp INTERFACE OpenMP::OpenMP_CXX)
endif()

if(MSVC)
  target_compile_options(fast_gicp INTERFACE /openmp:llvm)
elseif(OpenMP_CXX_FLAGS)
  target_compile_options(fast_gicp INTERFACE ${OpenMP_CXX_FLAGS})
endif()

option(BUILD_FAST_GICP_COMPILE_TEST "Build fast_gicp CPU header compile smoke test" OFF)
if(BUILD_FAST_GICP_COMPILE_TEST)
  add_executable(fast_gicp_compile_test fast_gicp_compile_test.cpp)
  target_link_libraries(fast_gicp_compile_test PRIVATE fast_gicp)
  scan_register_apply_compat(fast_gicp_compile_test)
  if(MSVC)
    target_compile_options(fast_gicp_compile_test PRIVATE /utf-8)
  endif()
endif()
