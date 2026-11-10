#include <graph/graph.hpp>

int main()
{
  std::vector<Ddx::GraphNode> nodes;
  nodes.emplace_back(0);
  nodes.emplace_back(1);

  std::vector<Ddx::GraphEdge> edges;
  edges.emplace_back(0, 1, 1.0);

  Ddx::MinimumSpanningTree<Ddx::GraphNode, Ddx::GraphEdge> mst;
  const auto tree = mst.kruskalRun2(nodes, edges);
  return tree.empty() ? 1 : 0;
}
