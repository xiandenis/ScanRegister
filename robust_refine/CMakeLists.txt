# robust_refine：精配准 / ICP 算法集合（子模块全部 STATIC；header-only 用 INTERFACE）
# include 风格：<basic_icp/...>、<fast_gicp/...>、<small_gicp/...>、<genz_icp/...>
# → 本目录作为 SCAN_REGISTER_ROBUST_REFINE_ROOT

set(SCAN_REGISTER_ROBUST_REFINE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(basic_icp)
add_subdirectory(fast_gicp)
add_subdirectory(small_gicp)
# genz_icp links matcher_tsl from robust_rough/
add_subdirectory(genz_icp)
