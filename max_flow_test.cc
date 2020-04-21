#include "core.h"

#include <gtest/gtest.h>
#include <limits>

struct FlowGraph {
  std::unique_ptr<Graph> graph;
  const size_t size;
  unsigned source, target;
  std::map<std::tuple<unsigned, unsigned>, int> capacity;
  std::map<std::tuple<unsigned, unsigned>, int> flow;

  explicit FlowGraph(std::unique_ptr<Graph> &graph)
      : graph(std::move(graph)), size(this->graph->succ.size()), source(0),
        target(size - 1) {}

  void InitCapacity(unsigned u, unsigned v, const int c) {
    assert(u != v);
    assert(c >= 0);
    assert(graph->succ[u].count(v));
    std::get<0>(capacity.insert({{u, v}, 0}))->second = c;
    std::get<0>(capacity.insert({{v, u}, 0}))->second = 0;
  }

  int GetCapacity(unsigned u, unsigned v) {
    auto it = capacity.find({u, v});
    if (it == capacity.end())
      return 0;
    return it->second;
  }

  int GetFlow(unsigned u, unsigned v) {
    auto it = flow.find({u, v});
    if (it == flow.end())
      return 0;
    return it->second;
  }

  int GetResidualCapacity(unsigned u, unsigned v) {
    return GetCapacity(u, v) - GetFlow(u, v);
  }

  void UpdateFlow(unsigned u, unsigned v, const int f) {
    const int c = GetCapacity(u, v);
    assert(f <= c);
    std::get<0>(flow.insert({{u, v}, 0}))->second = f;
    std::get<0>(flow.insert({{v, u}, 0}))->second = -f;
  }
};

struct PushAndRelabel {
  static const unsigned INF = std::numeric_limits<unsigned>::max();
  FlowGraph &network;
  std::vector<int> excess;
  std::vector<unsigned> distance, worklist, seen;

  PushAndRelabel(FlowGraph &network)
      : network(network), excess(network.size, 0), distance(network.size, INF) {
    distance[network.target] = 0;
    distance[network.source] = network.size;
  }

  int Push(unsigned u, unsigned v) {
    int diff = std::min(network.GetResidualCapacity(u, v), excess[u]);
    network.UpdateFlow(u, v, network.GetFlow(u, v) + diff);
    excess[u] -= diff;
    excess[v] += diff;
    return diff;
  }

  unsigned Relabel(unsigned u) {
    unsigned dis = INF;
    for (unsigned v = 0; v < network.size; ++v) {
      if (network.GetResidualCapacity(u, v) > 0)
        dis = std::min(dis, distance[v]);
    }
    if (dis != INF)
      distance[u] = dis + 1;
    return distance[u];
  }

  int CalculateMaxFlow() {
    int max_flow = 0;
    return max_flow;
  }
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
  auto it = fg.capacity.find({0, 1});
  EXPECT_TRUE(it != fg.capacity.end());
  EXPECT_TRUE(it->second == 5);
}

TEST(MaxFlowTest, GenFlowGraph) {
  auto fg = GenerateRandomFlowGraph(100, 100, 1000);
  (void)fg;
}

} // namespace
