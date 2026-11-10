set(ROBIN_SOURCES
  core.cpp
  graph.cpp
  graph_core.cpp
  graph_io.cpp
  graph_solvers.cpp
  pkc.cpp
  problems.cpp
  robin.cpp
  utils.cpp
)

# Explicit STATIC per project decision (robust_rough leaf).
add_library(robin STATIC ${ROBIN_SOURCES})

target_include_directories(robin PUBLIC
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_ROUGH_ROOT}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
)

target_link_libraries(robin PUBLIC
  pmc
  xenium
  Eigen3::Eigen
)

set_target_properties(robin PROPERTIES POSITION_INDEPENDENT_CODE ON)

scan_register_apply_compat(robin)
scan_register_apply_openmp(robin)

if(MSVC)
  target_compile_definitions(robin PRIVATE _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
endif()
