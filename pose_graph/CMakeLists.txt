# Pose graph stack: Ceres PGO (pose_graph_opt) + connectivity / global-frame tools
# (pose_graph_tool). Leaves stay independent (no cross-link).

set(SCAN_REGISTER_POSE_GRAPH_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(pose_graph_opt)
add_subdirectory(pose_graph_tool)
