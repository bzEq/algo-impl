#include "core.h"

#include <gtest/gtest.h>
#include <limits>

struct FlowGraph {
  std::unique_ptr<Graph> graph;
  unsigned source, target;
  std::map<std::tuple<unsigned, unsigned>, int> residual_capacity;
  std::map<std::tuple<unsigned, unsigned>, int> flow;

  explicit FlowGraph(std::unique_ptr<Graph> &graph)
      : graph(std::move(graph)), source(0), target(this->graph->succ.size()) {}

  void InitCapacity(unsigned u, unsigned v, const int c) {
    assert(u != v);
    assert(c >= 0);
    assert(graph->succ[u].count(v));
    std::get<0>(residual_capacity.insert({{u, v}, 0}))->second = c;
    std::get<0>(residual_capacity.insert({{v, u}, 0}))->second = 0;
  }

  int GetCapacity(unsigned u, unsigned v) {
    auto it = residual_capacity.find({u, v});
    if (it == residual_capacity.end())
      return 0;
    return it->second;
  }

  int GetFlow(unsigned u, unsigned v) {
    auto it = flow.find({u, v});
    if (it == flow.end())
      return 0;
    return it->second;
  }

  void UpdateFlow(unsigned u, unsigned v, const int f) {
    const int c = GetCapacity(u, v);
    assert(f <= c);
    auto update = [&, this](unsigned u, unsigned v, const int f) {
      std::get<0>(flow.insert({{u, v}, 0}))->second = f;
    };
    update(u, v, f);
    update(v, u, -f);
    std::get<0>(residual_capacity.insert({{v, u}, 0}))->second = c - f;
  }
};

struct PushAndRelabel {
  static const unsigned INF = std::numeric_limits<unsigned>::max();
  FlowGraph &network;
  std::vector<int> excess;
  std::vector<unsigned> distance;

  PushAndRelabel(FlowGraph &network)
      : network(network), excess(network.graph->succ.size(), 0),
        distance(network.graph->succ.size(), INF) {
    distance[network.target] = 0;
    distance[network.source] = network.graph->succ.size();
  }

  void Push(unsigned u, unsigned v) {
    int diff = std::min(network.GetCapacity(u, v), excess[u]);
    network.UpdateFlow(u, v, network.GetFlow(u, v) + diff);
    excess[u] -= diff;
    excess[v] += diff;
  }

  void Relabel(unsigned u) {
    for (auto v : network.graph->succ[u]) {
      if (distance[v] != INF)
        distance[u] = std::min(distance[u], distance[v] + 1);
    }
    for (auto v : network.graph->pred[u]) {
      if (distance[v] != INF)
        distance[u] = std::min(distance[u], distance[v] + 1);
    }
  }

  int CalculateMaxFlow() { return 0; }
};

namespace {

inline std::unique_ptr<FlowGraph>
GenerateRandomFlowGraph(const size_t num_of_vertexes, const size_t num_of_edges,
                        const unsigned max_capacity) {
  auto g = GenerateRandomDirectedGraph(num_of_vertexes, num_of_edges);
  auto fg = std::make_unique<FlowGraph>(g);
  Random rnd(std::time(nullptr));
  for (unsigned u = 0; u < fg->graph->succ.size(); ++u) {
    for (auto v : fg->graph->succ[u]) {
      if (u == v)
        continue;
      int c = v == fg->source ? 0 : (unsigned)rnd.NextInt() % max_capacity + 1;
      fg->InitCapacity(u, v, c);
    }
  }
  return fg;
}

TEST(MaxFlowTest, SimpleFlowGraph) {
  auto g = std::make_unique<Graph>(5, true);
  g->AddEdge(0, 1);
  g->AddEdge(1, 2);
  g->AddEdge(1, 3);
  g->AddEdge(2, 3);
  g->AddEdge(2, 4);
  g->AddEdge(3, 4);
  FlowGraph fg(g);
  fg.InitCapacity(0, 1, 5);
  auto it = fg.residual_capacity.find({0, 1});
  EXPECT_TRUE(it != fg.residual_capacity.end());
  EXPECT_TRUE(it->second == 5);
}

TEST(MaxFlowTest, GenFlowGraph) {
  auto fg = GenerateRandomFlowGraph(100, 100, 1000);
  (void)fg;
}

} // namespace
