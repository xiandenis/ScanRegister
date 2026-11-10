# robust_rough：鲁棒粗匹配子系统（子模块默认 STATIC / header INTERFACE）
# include 风格：<matcher_points/...>、<robin/...>、<xenium/...>
# → 本目录作为 SCAN_REGISTER_ROBUST_ROUGH_ROOT

set(SCAN_REGISTER_ROBUST_ROUGH_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

# Leaf → dependents
add_subdirectory(xenium)
add_subdirectory(matcher_tsl)
add_subdirectory(matcher_points)
add_subdirectory(matcher_kdtree)
add_subdirectory(matcher_hnswlib)
add_subdirectory(pmc)
add_subdirectory(teaser)
add_subdirectory(robin)
add_subdirectory(robust_matcher)
