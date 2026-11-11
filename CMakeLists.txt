cmake_minimum_required(VERSION 3.23)

project(scan_register
  VERSION 2.0.2
  DESCRIPTION "ScanRegister point-cloud library for terrestrial scan registration"
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Modules default to shared libraries unless a target overrides.
option(BUILD_SHARED_LIBS "Build modules as shared libraries" ON)

option(SCAN_REGISTER_INSTALL "Generate install/export for find_package(scan_register)" ON)
set(SCAN_REGISTER_INSTALL_SCOPE "party" CACHE STRING
  "Install scope: party | full | lidarstudio")
set_property(CACHE SCAN_REGISTER_INSTALL_SCOPE PROPERTY STRINGS party full lidarstudio)

option(SCAN_REGISTER_BUILD_ROS2 "Build ROS 2 packages (Linux only)" OFF)
if(WIN32)
  set(SCAN_REGISTER_BUILD_ROS2 OFF CACHE BOOL "Build ROS 2 packages (Linux only)" FORCE)
endif()

# Top-level vs add_subdirectory embed.
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(SCAN_REGISTER_IS_TOP_LEVEL ON)
else()
  set(SCAN_REGISTER_IS_TOP_LEVEL OFF)
endif()

# Repo root for #include <logging/...> / <common/...> and header install.
set(SCAN_REGISTER_INCLUDE_ROOT "${PROJECT_SOURCE_DIR}")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Keep DLLs next to test executables on multi-config generators.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
foreach(_cfg DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  string(TOUPPER "${_cfg}" _CFG)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_CFG} "${CMAKE_BINARY_DIR}/bin")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${_CFG} "${CMAKE_BINARY_DIR}/lib")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${_CFG} "${CMAKE_BINARY_DIR}/lib")
endforeach()

include(Dependencies)

# logging first: common links it.
add_subdirectory(logging)
add_subdirectory(file_system)
add_subdirectory(common)
add_subdirectory(graph)
add_subdirectory(registration_type)
add_subdirectory(point_cloud_io)
add_subdirectory(fastreg_filter)
add_subdirectory(global_matrix_file)
add_subdirectory(depth_map_tool)
add_subdirectory(octree)
add_subdirectory(overlap_estimation)
add_subdirectory(resolution_estimate)

# Robust coarse-matching stack (STATIC / header INTERFACE leaves under robust_rough/).
add_subdirectory(robust_rough)

# Fine / ICP refine stack (STATIC leaves under robust_refine/; needs matcher_tsl).
add_subdirectory(robust_refine)

# TrimICP（仓库根 trim_icp/；依赖 robust_refine/basic_icp + octree；不并入 robust_refine）。
add_subdirectory(trim_icp)

# Station init (needs small_gicp normals + resolution_estimate + octree Medoid).
add_subdirectory(registration_initializer)

# Coarse global registration (needs robust_matcher + point_cloud_io).
add_subdirectory(robust_global_reg)

# Coarse global registration business facade (ScanInfo → FramedCloud → RobustGlobalReg).
add_subdirectory(rough_global_reg)

# Pose graph: Ceres PGO (pose_graph_opt) + connectivity / global-frame tools (pose_graph_tool).
add_subdirectory(pose_graph)

# Umbrella INTERFACE for Session/foundation consumers: scan_register::scan_register
# (refine / rough / pose_graph targets are opt-in: link scan_register::<target> explicitly.)
add_library(scan_register INTERFACE)
add_library(scan_register::scan_register ALIAS scan_register)
target_link_libraries(scan_register INTERFACE
  common
  file_system
  logging
  graph
  registration_type
)

if(SCAN_REGISTER_INSTALL AND SCAN_REGISTER_IS_TOP_LEVEL)
  include(InstallExport)
endif()

message(STATUS "scan_register ${PROJECT_VERSION}")
message(STATUS "  top-level : ${SCAN_REGISTER_IS_TOP_LEVEL}")
message(STATUS "  shared    : ${BUILD_SHARED_LIBS}")
message(STATUS "  install   : ${SCAN_REGISTER_INSTALL} (scope=${SCAN_REGISTER_INSTALL_SCOPE})")
message(STATUS "  ros2      : ${SCAN_REGISTER_BUILD_ROS2}")
