# Vendored small_gicp (koide3/small_gicp): Eigen/OpenMP registration helper + header-only core.

# Explicit STATIC per project decision (robust_refine leaf).
add_library(small_gicp STATIC
  registration/registration.cpp
  registration/registration_helper.cpp
)

target_include_directories(small_gicp PUBLIC
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_REFINE_ROOT}>"
)

target_link_libraries(small_gicp PUBLIC
  Eigen3::Eigen
)

set_target_properties(small_gicp PROPERTIES POSITION_INDEPENDENT_CODE ON)

scan_register_apply_compat(small_gicp)
scan_register_apply_openmp(small_gicp)

if(MSVC)
  target_compile_definitions(small_gicp PRIVATE _USE_MATH_DEFINES)
  target_compile_options(small_gicp PRIVATE /bigobj /utf-8)
endif()

option(BUILD_SMALL_GICP_COMPILE_TEST "Build small_gicp helper compile smoke test" OFF)
if(BUILD_SMALL_GICP_COMPILE_TEST)
  add_executable(small_gicp_compile_test small_gicp_compile_test.cpp)
  target_link_libraries(small_gicp_compile_test PRIVATE small_gicp)
  scan_register_apply_compat(small_gicp_compile_test)
  if(MSVC)
    target_compile_options(small_gicp_compile_test PRIVATE /utf-8)
  endif()
endif()
