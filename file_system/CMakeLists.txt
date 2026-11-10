add_library(file_system SHARED
  file_system.cpp
  portability_fixes.cpp
  strings.cpp
  tokenizer.cpp
  wildcard.cpp
)

target_include_directories(file_system PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${SCAN_REGISTER_INCLUDE_ROOT}>"
)

set_target_properties(file_system PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

scan_register_apply_compat(file_system)

if(MSVC)
  target_compile_options(file_system PRIVATE /utf-8)
endif()
