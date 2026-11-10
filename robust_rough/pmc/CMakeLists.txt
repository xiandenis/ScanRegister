set(PMC_SOURCES
  pmc_platform.cpp
  pmc_utils.cpp
  pmc_graph.cpp
  pmc_cores.cpp
  pmc_clique_utils.cpp
  pmc_heu.cpp
  pmc_maxclique.cpp
  pmcx_maxclique.cpp
  pmcx_maxclique_basic.cpp
  pmc_lib.cpp
)

# Explicit STATIC per project decision (robust_rough leaf).
add_library(pmc STATIC ${PMC_SOURCES})

target_include_directories(pmc PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_ROUGH_ROOT}>"
)

set_target_properties(pmc PROPERTIES POSITION_INDEPENDENT_CODE ON)

scan_register_apply_compat(pmc)
scan_register_apply_openmp(pmc)

if(WIN32)
  target_link_libraries(pmc PUBLIC psapi)
endif()

if(MSVC)
  target_compile_definitions(pmc PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
