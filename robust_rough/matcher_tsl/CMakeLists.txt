add_library(matcher_tsl INTERFACE)

target_include_directories(matcher_tsl INTERFACE
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_ROUGH_ROOT}>"
)
