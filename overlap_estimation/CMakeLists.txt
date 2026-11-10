# Header-only overlap estimation / registration overlap subset extraction.
add_library(overlap_estimation INTERFACE)

target_include_directories(overlap_estimation INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

target_link_libraries(overlap_estimation INTERFACE
  pcl_common
  pcl_search
  Eigen3::Eigen
)

if(TARGET OpenMP::OpenMP_CXX)
  target_link_libraries(overlap_estimation INTERFACE OpenMP::OpenMP_CXX)
endif()

if(MSVC)
  target_compile_options(overlap_estimation INTERFACE /openmp:llvm)
elseif(OpenMP_CXX_FLAGS)
  target_compile_options(overlap_estimation INTERFACE ${OpenMP_CXX_FLAGS})
endif()

option(BUILD_OVERLAP_ESTIMATION_TEST "Build overlap_estimation unit tests" ON)
option(BUILD_OVERLAP_ESTIMATION_BENCHMARK "Build overlap_estimation_benchmark_test executable" OFF)

if(BUILD_OVERLAP_ESTIMATION_TEST AND SCAN_REGISTER_IS_TOP_LEVEL)
  add_executable(overlap_estimation_test overlap_estimation_test.cpp)
  target_link_libraries(overlap_estimation_test PRIVATE overlap_estimation)
  scan_register_apply_compat(overlap_estimation_test)
  scan_register_apply_openmp(overlap_estimation_test)
  if(MSVC)
    target_compile_options(overlap_estimation_test PRIVATE /utf-8)
  endif()

  add_executable(registration_overlap_extractor_test registration_overlap_extractor_test.cpp)
  target_link_libraries(registration_overlap_extractor_test PRIVATE overlap_estimation)
  scan_register_apply_compat(registration_overlap_extractor_test)
  scan_register_apply_openmp(registration_overlap_extractor_test)
  if(MSVC)
    target_compile_options(registration_overlap_extractor_test PRIVATE /utf-8)
  endif()
endif()

if(BUILD_OVERLAP_ESTIMATION_BENCHMARK AND SCAN_REGISTER_IS_TOP_LEVEL)
  add_executable(overlap_estimation_benchmark_test overlap_estimation_benchmark_test.cpp)
  target_link_libraries(overlap_estimation_benchmark_test PRIVATE overlap_estimation pcl_io)
  scan_register_apply_compat(overlap_estimation_benchmark_test)
  scan_register_apply_openmp(overlap_estimation_benchmark_test)
  if(MSVC)
    target_compile_options(overlap_estimation_benchmark_test PRIVATE /utf-8)
    set_target_properties(overlap_estimation_benchmark_test PROPERTIES LINK_FLAGS "/ENTRY:wmainCRTStartup")
  endif()
endif()
