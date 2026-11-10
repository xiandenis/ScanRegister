# Header-only general graph algorithms (MST / CC / shortest path). No Boost.
# Distinct from robust_rough/robin graph (registration inlier compatibility graph).
add_library(graph INTERFACE)

target_include_directories(graph INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

option(BUILD_GRAPH_COMPILE_TEST "Build graph header compile smoke test" OFF)
if(BUILD_GRAPH_COMPILE_TEST)
  add_executable(graph_compile_test graph_compile_test.cpp)
  target_link_libraries(graph_compile_test PRIVATE graph)
  scan_register_apply_compat(graph_compile_test)
  if(MSVC)
    target_compile_options(graph_compile_test PRIVATE /utf-8)
  endif()
endif()

option(BUILD_GRAPH_TEST "Build graph unit tests" ON)
if(BUILD_GRAPH_TEST AND SCAN_REGISTER_IS_TOP_LEVEL)
  add_executable(graph_test graph_test.cpp)
  target_link_libraries(graph_test PRIVATE graph)
  scan_register_apply_compat(graph_test)
  if(MSVC)
    target_compile_options(graph_test PRIVATE /utf-8)
  endif()
endif()
