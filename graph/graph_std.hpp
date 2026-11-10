#pragma once

// 已弃用：请改用 <graph/graph.hpp>（纯 std 实现，无 Boost）。
// 本头文件保留 Ddx::graph_std 别名，兼容旧代码。

#include <graph/graph.hpp>

namespace Ddx
{
namespace graph_std
{

using NodeID = Ddx::NodeID;
using GraphNode = Ddx::GraphNode;
using GraphEdge = Ddx::GraphEdge;
using WeightType = Ddx::WeightType;

inline constexpr NodeID gInvalidNodeId = Ddx::gInvalidNodeId;
inline constexpr WeightType gDefaultWeight = Ddx::gDefaultWeight;
inline constexpr WeightType gInvalidWeight = Ddx::gInvalidWeight;

template <typename NodeType, typename EdgeType>
using MinimumSpanningTree = Ddx::MinimumSpanningTree<NodeType, EdgeType>;

template <typename NodeType, typename EdgeType>
using ConnectedComponentsSubgraph = Ddx::ConnectedComponentsSubgraph<NodeType, EdgeType>;

template <typename NodeType, typename EdgeType>
using ShortestPaths = Ddx::ShortestPaths<NodeType, EdgeType>;

}  // namespace graph_std
}  // namespace Ddx
