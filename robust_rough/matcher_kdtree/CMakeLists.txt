add_library(matcher_kdtree INTERFACE)

target_include_directories(matcher_kdtree INTERFACE
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_ROUGH_ROOT}>"
)

target_link_libraries(matcher_kdtree INTERFACE
  matcher_points
)
