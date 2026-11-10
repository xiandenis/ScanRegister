#include <graph/graph.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <tuple>
#include <vector>

namespace
{

using Node = Ddx::GraphNode;
using Edge = Ddx::GraphEdge;

struct EdgeKey
{
  Ddx::NodeID a;
  Ddx::NodeID b;
  double w;

  bool operator<(EdgeKey const& other) const
  {
    return std::tie(a, b, w) < std::tie(other.a, other.b, other.w);
  }
};

EdgeKey normalize(Edge const& edge)
{
  if (edge.src <= edge.dst) {
    return {edge.src, edge.dst, edge.weight};
  }
  return {edge.dst, edge.src, edge.weight};
}

double mst_weight(std::vector<Edge> const& edges)
{
  double total = 0.0;
  for (Edge const& edge : edges) {
    total += edge.weight;
  }
  return total;
}

int g_failures = 0;

void check(char const* name, bool ok)
{
  std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << '\n';
  if (!ok) {
    ++g_failures;
  }
}

std::vector<Node> make_nodes(std::vector<Ddx::NodeID> const& ids)
{
  std::vector<Node> nodes;
  for (Ddx::NodeID id : ids) {
    nodes.emplace_back(id);
  }
  return nodes;
}

}  // namespace

int main()
{
  std::cout << "=== graph.hpp unit tests (std implementation) ===\n\n";

  {
    auto nodes = make_nodes({0, 1, 2});
    std::vector<Edge> edges = {Edge(0, 1, 1.0), Edge(1, 2, 2.0), Edge(0, 2, 3.0)};
    Ddx::MinimumSpanningTree<Node, Edge> mst;
    auto const tree = mst.kruskalRun2(nodes, edges);
    check("kruskal triangle edge count", tree.size() == 2);
    check("kruskal triangle weight", std::abs(mst_weight(tree) - 3.0) < 1e-9);
  }

  {
    auto nodes = make_nodes({10, 20, 30, 40});
    std::vector<Edge> edges = {Edge(10, 20, 2.0), Edge(20, 30, 1.0), Edge(30, 40, 3.0), Edge(10, 40, 10.0)};
    Ddx::MinimumSpanningTree<Node, Edge> mst;
    auto const k = mst.kruskalRun2(nodes, edges);
    auto const p = mst.primRun(nodes, edges);
    check("prim non-sequential NodeID edges", p.size() == 3);
    check("kruskal vs prim weight", std::abs(mst_weight(k) - mst_weight(p)) < 1e-9);
  }

  {
    Ddx::ConnectedComponentsSubgraph<Node, Edge> cc;
    auto groups = cc.run2(make_nodes({0, 1, 2, 3}), {Edge(0, 1, 1.0), Edge(2, 3, 1.0)});
    check("connected components count", groups.size() == 2);
  }

  {
    Ddx::ShortestPaths<Node, Edge> sp;
    auto nodes = make_nodes({0, 1, 2});
    sp.build(nodes, {Edge(0, 1, 1.0), Edge(1, 2, 1.0), Edge(0, 2, 5.0)});
    auto path = sp.run(Node(0), Node(2));
    check("shortest path length", path.size() == 3);
    check("shortest path endpoints", path.front().id == 2 && path.back().id == 0);
    check("unreachable empty", sp.run(Node(0), Node(99)).empty());
  }

  {
    std::mt19937 rng(42);
    std::uniform_int_distribution<Ddx::NodeID> id_dist(100, 999);
    std::uniform_real_distribution<double> weight_dist(0.1, 20.0);
    int ok_trials = 0;
    for (int trial = 0; trial < 30; ++trial) {
      std::vector<Ddx::NodeID> ids;
      int const n = 5 + trial % 8;
      while (static_cast<int>(ids.size()) < n) {
        Ddx::NodeID id = id_dist(rng);
        if (std::find(ids.begin(), ids.end(), id) == ids.end()) {
          ids.push_back(id);
        }
      }
      auto nodes = make_nodes(ids);
      std::vector<Edge> edges;
      for (int e = 0; e < n + trial; ++e) {
        Ddx::NodeID s = ids[static_cast<std::size_t>(e % n)];
        Ddx::NodeID t = ids[static_cast<std::size_t>((e * 3 + 1) % n)];
        if (s != t) {
          edges.emplace_back(s, t, weight_dist(rng));
        }
      }
      Ddx::MinimumSpanningTree<Node, Edge> mst;
      auto k = mst.kruskalRun2(nodes, edges);
      auto p = mst.primRun(nodes, edges);
      if (k.size() == p.size() && std::abs(mst_weight(k) - mst_weight(p)) < 1e-9) {
        ++ok_trials;
      }
    }
    check("random kruskal vs prim (30 trials)", ok_trials == 30);
  }

  std::cout << '\n';
  if (g_failures == 0) {
    std::cout << "Result: ALL PASSED\n";
  } else {
    std::cout << "Result: " << g_failures << " FAILED\n";
  }
  return g_failures;
}
