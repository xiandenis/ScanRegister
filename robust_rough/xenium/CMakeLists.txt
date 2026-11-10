# Header-only lock-free structures (vendored). Used by robin.
add_library(xenium INTERFACE)

target_include_directories(xenium INTERFACE
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_ROUGH_ROOT}>"
)
