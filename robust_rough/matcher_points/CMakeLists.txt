add_library(matcher_points INTERFACE)

target_include_directories(matcher_points INTERFACE
  "$<BUILD_INTERFACE:${SCAN_REGISTER_ROBUST_ROUGH_ROOT}>"
)

target_link_libraries(matcher_points INTERFACE
  Eigen3::Eigen
  TBB::tbb
)
