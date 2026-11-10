#pragma once

// 纯标准库通用图算法（无 Boost 依赖）：MST / 连通分量 / 最短路径。
// 与 robin/graph.hpp（配准外点兼容图）无关。

#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Ddx
{

using NodeID = uint32_t;
inline constexpr NodeID gInvalidNodeId = std::numeric_limits<NodeID>::max();

struct GraphNode
{
  NodeID id = gInvalidNodeId;
  std::string name_;

  GraphNode() = default;

  GraphNode(NodeID idx)
      : id(idx), name_(std::to_string(idx))
  {
  }

  GraphNode(NodeID idx, std::string const& name)
      : id(idx), name_(name)
  {
  }

  bool operator==(GraphNode const& node) const { return id == node.id; }

  bool operator<(GraphNode const& node) const { return id < node.id; }
};

using WeightType = double;
inline constexpr WeightType gDefaultWeight = 1.0;
inline constexpr WeightType gInvalidWeight = std::numeric_limits<WeightType>::max();

struct GraphEdge
{
  NodeID src = gInvalidNodeId;
  NodeID dst = gInvalidNodeId;
  WeightType weight = gDefaultWeight;

  GraphEdge() = default;

  GraphEdge(NodeID i, NodeID j)
      : src(i), dst(j), weight(gDefaultWeight)
  {
  }

  GraphEdge(NodeID i, NodeID j, WeightType w)
      : src(i), dst(j), weight(w)
  {
  }

  bool operator==(GraphEdge const& edge) const
  {
    return src == edge.src && dst == edge.dst;
  }
};

namespace graph_detail
{

inline bool edge_weight_less(GraphEdge const& a, GraphEdge const& b)
{
  return a.weight < b.weight;
}

class UnionFind
{
public:
  explicit UnionFind(std::vector<NodeID> const& ids)
  {
    parent_.reserve(ids.size());
    rank_.reserve(ids.size());
    for (NodeID id : ids) {
      parent_.emplace(id, id);
      rank_.emplace(id, 0);
    }
  }

  NodeID find(NodeID x)
  {
    NodeID root = x;
    while (parent_.at(root) != root) {
      root = parent_.at(root);
    }
    for (NodeID cur = x; cur != root;) {
      NodeID const next = parent_.at(cur);
      parent_[cur] = root;
      cur = next;
    }
    return root;
  }

  bool unite(NodeID a, NodeID b)
  {
    NodeID ra = find(a);
    NodeID rb = find(b);
    if (ra == rb) {
      return false;
    }
    int& rank_a = rank_.at(ra);
    int& rank_b = rank_.at(rb);
    if (rank_a < rank_b) {
      parent_[ra] = rb;
    } else if (rank_a > rank_b) {
      parent_[rb] = ra;
    } else {
      parent_[rb] = ra;
      ++rank_a;
    }
    return true;
  }

private:
  std::unordered_map<NodeID, NodeID> parent_;
  std::unordered_map<NodeID, int> rank_;
};

struct IndexedGraph
{
  std::unordered_map<NodeID, std::size_t> id_to_index;
  std::vector<GraphNode> nodes;
  std::vector<std::vector<std::pair<std::size_t, WeightType>>> adj;

  void clear()
  {
    id_to_index.clear();
    nodes.clear();
    adj.clear();
  }

  void build(std::vector<GraphNode> const& node_vec,
               std::vector<GraphEdge> const& edge_vec)
  {
    clear();
    nodes = node_vec;
    adj.assign(nodes.size(), {});

    for (std::size_t i = 0; i < nodes.size(); ++i) {
      id_to_index.emplace(nodes[i].id, i);
    }

    for (GraphEdge const& edge : edge_vec) {
      auto const src_it = id_to_index.find(edge.src);
      auto const dst_it = id_to_index.find(edge.dst);
      if (src_it == id_to_index.end() || dst_it == id_to_index.end()) {
        continue;
      }
      std::size_t const u = src_it->second;
      std::size_t const v = dst_it->second;
      if (u == v) {
        continue;
      }
      adj[u].emplace_back(v, edge.weight);
      adj[v].emplace_back(u, edge.weight);
    }
  }

  bool try_index(NodeID id, std::size_t& out_index) const
  {
    auto const it = id_to_index.find(id);
    if (it == id_to_index.end()) {
      return false;
    }
    out_index = it->second;
    return true;
  }
};

inline std::vector<NodeID> collect_node_ids(std::vector<GraphNode> const& node_vec)
{
  std::vector<NodeID> ids;
  ids.reserve(node_vec.size());
  for (GraphNode const& node : node_vec) {
    ids.push_back(node.id);
  }
  return ids;
}

inline std::vector<GraphEdge> kruskal_mst(std::vector<GraphNode> const& node_vec,
                                          std::vector<GraphEdge> const& edge_vec)
{
  std::vector<GraphEdge> valid_edges;
  valid_edges.reserve(edge_vec.size());

  std::unordered_map<NodeID, bool> node_exists;
  for (GraphNode const& node : node_vec) {
    node_exists.emplace(node.id, true);
  }

  for (GraphEdge const& edge : edge_vec) {
    if (edge.src == edge.dst) {
      continue;
    }
    if (!node_exists.count(edge.src) || !node_exists.count(edge.dst)) {
      continue;
    }
    valid_edges.push_back(edge);
  }

  std::sort(valid_edges.begin(), valid_edges.end(), edge_weight_less);

  UnionFind uf(collect_node_ids(node_vec));
  std::vector<GraphEdge> mst;
  mst.reserve(node_vec.empty() ? 0 : node_vec.size() - 1);

  for (GraphEdge const& edge : valid_edges) {
    if (uf.unite(edge.src, edge.dst)) {
      mst.push_back(edge);
    }
    if (node_vec.size() > 1 && mst.size() + 1 >= node_vec.size()) {
      break;
    }
  }
  return mst;
}

inline std::vector<GraphEdge> prim_mst(IndexedGraph const& graph)
{
  if (graph.nodes.empty()) {
    return {};
  }

  struct HeapEntry
  {
    WeightType weight;
    std::size_t src;
    std::size_t dst;

    bool operator>(HeapEntry const& other) const { return weight > other.weight; }
  };

  std::vector<bool> in_tree(graph.nodes.size(), false);
  std::vector<GraphEdge> mst;
  mst.reserve(graph.nodes.size() > 0 ? graph.nodes.size() - 1 : 0);

  for (std::size_t start = 0; start < graph.nodes.size(); ++start) {
    if (in_tree[start]) {
      continue;
    }

    std::priority_queue<HeapEntry, std::vector<HeapEntry>, std::greater<HeapEntry>> heap;
    in_tree[start] = true;

    for (auto const& neighbor : graph.adj[start]) {
      heap.push({neighbor.second, start, neighbor.first});
    }

    while (!heap.empty()) {
      HeapEntry const entry = heap.top();
      heap.pop();

      if (in_tree[entry.dst]) {
        continue;
      }

      in_tree[entry.dst] = true;
      mst.emplace_back(graph.nodes[entry.src].id, graph.nodes[entry.dst].id, entry.weight);

      for (auto const& neighbor : graph.adj[entry.dst]) {
        if (!in_tree[neighbor.first]) {
          heap.push({neighbor.second, entry.dst, neighbor.first});
        }
      }
    }
  }

  return mst;
}

inline std::vector<int> connected_component_labels(IndexedGraph const& graph)
{
  std::vector<int> labels(graph.nodes.size(), -1);
  int component_id = 0;

  for (std::size_t start = 0; start < graph.nodes.size(); ++start) {
    if (labels[start] >= 0) {
      continue;
    }

    std::vector<std::size_t> stack;
    stack.push_back(start);
    labels[start] = component_id;

    while (!stack.empty()) {
      std::size_t const u = stack.back();
      stack.pop_back();
      for (auto const& neighbor : graph.adj[u]) {
        std::size_t const v = neighbor.first;
        if (labels[v] < 0) {
          labels[v] = component_id;
          stack.push_back(v);
        }
      }
    }
    ++component_id;
  }

  return labels;
}

inline std::vector<GraphNode> dijkstra_path(IndexedGraph const& graph,
                                            std::size_t src_index,
                                            std::size_t dst_index)
{
  if (src_index >= graph.nodes.size() || dst_index >= graph.nodes.size()) {
    return {};
  }
  if (src_index == dst_index) {
    return {graph.nodes[src_index]};
  }

  struct HeapEntry
  {
    WeightType dist;
    std::size_t vertex;

    bool operator>(HeapEntry const& other) const { return dist > other.dist; }
  };

  std::vector<WeightType> dist(graph.nodes.size(), gInvalidWeight);
  std::vector<std::size_t> predecessor(graph.nodes.size(), graph.nodes.size());

  std::priority_queue<HeapEntry, std::vector<HeapEntry>, std::greater<HeapEntry>> heap;
  dist[src_index] = 0.0;
  heap.push({0.0, src_index});

  while (!heap.empty()) {
    HeapEntry const entry = heap.top();
    heap.pop();

    if (entry.dist > dist[entry.vertex]) {
      continue;
    }
    if (entry.vertex == dst_index) {
      break;
    }

    for (auto const& neighbor : graph.adj[entry.vertex]) {
      WeightType const next_dist = entry.dist + neighbor.second;
      if (next_dist < dist[neighbor.first]) {
        dist[neighbor.first] = next_dist;
        predecessor[neighbor.first] = entry.vertex;
        heap.push({next_dist, neighbor.first});
      }
    }
  }

  if (dist[dst_index] >= gInvalidWeight) {
    return {};
  }

  std::vector<GraphNode> path;
  for (std::size_t cur = dst_index; cur != src_index;) {
    path.push_back(graph.nodes[cur]);
    if (predecessor[cur] == cur || predecessor[cur] >= graph.nodes.size()) {
      return {};
    }
    cur = predecessor[cur];
  }
  path.push_back(graph.nodes[src_index]);
  return path;
}

}  // namespace graph_detail

template <typename NodeType, typename EdgeType>
class MinimumSpanningTree
{
public:
  std::vector<EdgeType> kruskalRun(std::vector<NodeType> const& node_vec,
                                    std::vector<EdgeType> const& edge_vec)
  {
    return kruskalRun2(node_vec, edge_vec);
  }

  std::vector<EdgeType> kruskalRun2(std::vector<NodeType> const& node_vec,
                                    std::vector<EdgeType> const& edge_vec)
  {
    std::vector<GraphEdge> const mst =
        graph_detail::kruskal_mst(to_node_vector(node_vec), to_input_edges(edge_vec));
    return from_graph_edges(mst);
  }

  std::vector<EdgeType> primRun(std::vector<NodeType> const& node_vec,
                                std::vector<EdgeType> const& edge_vec)
  {
    graph_detail::IndexedGraph graph;
    graph.build(to_node_vector(node_vec), to_input_edges(edge_vec));
    std::vector<GraphEdge> const mst = graph_detail::prim_mst(graph);
    return from_graph_edges(mst);
  }

private:
  static std::vector<GraphNode> to_node_vector(std::vector<NodeType> const& node_vec)
  {
    std::vector<GraphNode> nodes;
    nodes.reserve(node_vec.size());
    for (NodeType const& node : node_vec) {
      nodes.emplace_back(node.id, node.name_);
    }
    return nodes;
  }

  static std::vector<GraphEdge> to_input_edges(std::vector<EdgeType> const& edge_vec)
  {
    std::vector<GraphEdge> edges;
    edges.reserve(edge_vec.size());
    for (EdgeType const& edge : edge_vec) {
      edges.emplace_back(edge.src, edge.dst, edge.weight);
    }
    return edges;
  }

  static std::vector<EdgeType> from_graph_edges(std::vector<GraphEdge> const& edges)
  {
    std::vector<EdgeType> out;
    out.reserve(edges.size());
    for (GraphEdge const& edge : edges) {
      out.emplace_back(edge.src, edge.dst, edge.weight);
    }
    return out;
  }
};

template <typename NodeType, typename EdgeType>
class ConnectedComponentsSubgraph
{
public:
  std::vector<std::vector<EdgeType>> run(std::vector<NodeType> const& node_vec,
                                         std::vector<EdgeType> const& edge_vec)
  {
    graph_detail::IndexedGraph graph;
    graph.build(to_node_vector(node_vec), to_input_edges(edge_vec));

    std::vector<int> const labels = graph_detail::connected_component_labels(graph);
    std::unordered_map<NodeID, int> id_to_component;
    for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
      id_to_component.emplace(graph.nodes[i].id, labels[i]);
    }

    std::map<int, std::vector<EdgeType>> edge_groups;
    for (EdgeType const& edge : edge_vec) {
      auto const src_it = id_to_component.find(edge.src);
      auto const dst_it = id_to_component.find(edge.dst);
      if (src_it == id_to_component.end() || dst_it == id_to_component.end()) {
        continue;
      }
      if (src_it->second != dst_it->second) {
        continue;
      }
      edge_groups[src_it->second].push_back(edge);
    }

    std::vector<std::vector<EdgeType>> result;
    result.reserve(edge_groups.size());
    for (auto const& group : edge_groups) {
      result.push_back(group.second);
    }
    return result;
  }

  std::vector<std::vector<NodeType>> run2(std::vector<NodeType> const& node_vec,
                                          std::vector<EdgeType> const& edge_vec)
  {
    graph_detail::IndexedGraph graph;
    graph.build(to_node_vector(node_vec), to_input_edges(edge_vec));

    std::vector<int> const labels = graph_detail::connected_component_labels(graph);
    int const component_count =
        labels.empty() ? 0 : (*std::max_element(labels.begin(), labels.end()) + 1);

    std::vector<std::vector<NodeType>> result(static_cast<std::size_t>(component_count));
    for (std::size_t i = 0; i < node_vec.size(); ++i) {
      if (labels[i] >= 0) {
        result[static_cast<std::size_t>(labels[i])].push_back(node_vec[i]);
      }
    }
    return result;
  }

private:
  static std::vector<GraphNode> to_node_vector(std::vector<NodeType> const& node_vec)
  {
    std::vector<GraphNode> nodes;
    nodes.reserve(node_vec.size());
    for (NodeType const& node : node_vec) {
      nodes.emplace_back(node.id, node.name_);
    }
    return nodes;
  }

  static std::vector<GraphEdge> to_input_edges(std::vector<EdgeType> const& edge_vec)
  {
    std::vector<GraphEdge> edges;
    edges.reserve(edge_vec.size());
    for (EdgeType const& edge : edge_vec) {
      edges.emplace_back(edge.src, edge.dst, edge.weight);
    }
    return edges;
  }
};

template <typename NodeType, typename EdgeType>
class ShortestPaths
{
public:
  void build(std::vector<NodeType> const& node_vec,
             std::vector<EdgeType> const& edge_vec)
  {
    graph_.build(to_node_vector(node_vec), to_input_edges(edge_vec));
  }

  std::vector<NodeType> run(NodeType node_a, NodeType node_b)
  {
    std::size_t src_index = 0;
    std::size_t dst_index = 0;
    if (!graph_.try_index(node_a.id, src_index) || !graph_.try_index(node_b.id, dst_index)) {
      return {};
    }

    std::vector<GraphNode> const path =
        graph_detail::dijkstra_path(graph_, src_index, dst_index);

    std::vector<NodeType> result;
    result.reserve(path.size());
    for (GraphNode const& node : path) {
      result.emplace_back(node.id, node.name_);
    }
    return result;
  }

private:
  graph_detail::IndexedGraph graph_;

  static std::vector<GraphNode> to_node_vector(std::vector<NodeType> const& node_vec)
  {
    std::vector<GraphNode> nodes;
    nodes.reserve(node_vec.size());
    for (NodeType const& node : node_vec) {
      nodes.emplace_back(node.id, node.name_);
    }
    return nodes;
  }

  static std::vector<GraphEdge> to_input_edges(std::vector<EdgeType> const& edge_vec)
  {
    std::vector<GraphEdge> edges;
    edges.reserve(edge_vec.size());
    for (EdgeType const& edge : edge_vec) {
      edges.emplace_back(edge.src, edge.dst, edge.weight);
    }
    return edges;
  }
};

}  // namespace Ddx
