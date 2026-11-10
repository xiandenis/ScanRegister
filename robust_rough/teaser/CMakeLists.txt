# Vendored TEASER++ core + TeaserAlign / TeaserRespondence.
# certification.cpp needs Spectra and is excluded unless TEASER_WITH_SPECTRA=ON.

set(TEASER_SOURCES
  fpfh.cpp
  graph.cpp
  matcher.cpp
  registration.cpp
  teaser_align.cpp
  teaser_respondence.cpp
)

option(TEASER_WITH_SPECTRA "Build teaser certification.cpp (requires Spectra)" OFF)
if(TEASER_WITH_SPECTRA)
  find_path(SPECTRA_INCLUDE_DIR NAMES Spectra/SymEigsSolver.h)
  if(NOT SPECTRA_INCLUDE_DIR)
    message(FATAL_ERROR "TEASER_WITH_SPECTRA=ON but Spectra headers not found")
  endif()
  list(APPEND TEASER_SOURCES certification.cpp)
endif()

# Explicit STATIC per project decision (robust_rough leaf).
add_library(teaser STATIC ${TEASER_SOURCES})

target_include_directories(teaser
  PUBLIC
    "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_ROUGH_ROOT}>"
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

if(TEASER_WITH_SPECTRA)
  target_include_directories(teaser PRIVATE "${SPECTRA_INCLUDE_DIR}")
endif()

target_link_libraries(teaser PUBLIC
  Eigen3::Eigen
  pmc
  flann::flann_cpp
  pcl_common
  pcl_features
  pcl_search
)

# PCL feature estimation may need these transitive bits on some installs.
if(TARGET pcl_io)
  target_link_libraries(teaser PUBLIC pcl_io)
endif()

set_target_properties(teaser PROPERTIES POSITION_INDEPENDENT_CODE ON)

scan_register_apply_compat(teaser)
scan_register_apply_openmp(teaser)

if(MSVC)
  target_compile_definitions(teaser PRIVATE _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
  target_compile_options(teaser PRIVATE /utf-8)
endif()

option(BUILD_TEASER_COMPILE_TEST "Build teaser_compile_test executable" ${SCAN_REGISTER_IS_TOP_LEVEL})
if(BUILD_TEASER_COMPILE_TEST)
  add_executable(teaser_compile_test teaser_compile_test.cpp)
  target_link_libraries(teaser_compile_test PRIVATE teaser)
  scan_register_apply_compat(teaser_compile_test)
  if(MSVC)
    target_compile_options(teaser_compile_test PRIVATE /utf-8)
  endif()
endif()
