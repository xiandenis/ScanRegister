# Shared logging library: keep a single LogManager / spdlog registry across
# modules and the final executable (static logging would duplicate state).
add_library(logging SHARED
  logging.cpp
  logging_initi.cpp
  mem_peak.cpp
)

target_include_directories(logging PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_SPDLOG_INCLUDE}>"
)

target_link_libraries(logging PUBLIC scan_register_spdlog)

if(WIN32)
  target_link_libraries(logging PRIVATE Psapi)
endif()

find_package(Threads REQUIRED)
target_link_libraries(logging PUBLIC Threads::Threads)

# LOGGING_LOG_EXPORT => dllexport when building this target;
# consumers (no define) => dllimport via logging.hpp / logging_initi.hpp.
target_compile_definitions(logging
  PRIVATE LOGGING_LOG_EXPORT
  PRIVATE SPDLOG_COMPILED_LIB
)

set_target_properties(logging PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  WINDOWS_EXPORT_ALL_SYMBOLS OFF
)

scan_register_apply_compat(logging)

if(MSVC)
  target_compile_options(logging PRIVATE /utf-8)
endif()
