#include "core.h"

#include <gtest/gtest.h>
#include <limits>
#include <queue>

struct FlowGraph {
  std::unique_ptr<Graph> graph;
  const size_t size;
  std::map<std::tuple<unsigned, unsigned>, int> capacity;
  std::map<std::tuple<unsigned, unsigned>, int> flow;

  explicit FlowGraph(std::unique_ptr<Graph> &graph)
      : graph(std::move(graph)), size(this->graph->succ.size()) {}

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
  std::vector<unsigned> distance, seen;
  std::queue<unsigned> worklist;

  PushAndRelabel(FlowGraph &network)
      : network(network), excess(network.size, 0), distance(network.size, 0) {
    distance[network.size - 1] = 0;
    distance[0] = network.size;
  }

  void Push(unsigned u, unsigned v) {
    int diff = std::min(network.GetResidualCapacity(u, v), excess[u]);
    // printf("Before: RC of (%u, %u): %d\n", u, v,
    //        network.GetResidualCapacity(u, v));
    // printf("Before: Flow of (%u, %u): %d\n", u, v, network.GetFlow(u, v));
    network.UpdateFlow(u, v, network.GetFlow(u, v) + diff);
    // printf("After: RC of (%u, %u): %d\n", u, v,
    //        network.GetResidualCapacity(u, v));
    // printf("After: Flow of (%u, %u): %d\n", u, v, network.GetFlow(u, v));
    excess[u] -= diff;
    excess[v] += diff;
    if (diff && excess[v] == diff)
      worklist.push(v);
  }

  void Relabel(unsigned u) {
    unsigned dis = INF;
    for (unsigned v = 0; v < network.size; ++v) {
      if (network.GetResidualCapacity(u, v) > 0)
        dis = std::min(dis, distance[v]);
    }
    if (dis != INF)
      distance[u] = dis + 1;
  }

  void Discharge(unsigned u) {
    while (excess[u] > 0) {
      unsigned v = seen[u];
      // printf("%d seen %d\n", u, v);
      if (v < network.size) {
        if (network.GetResidualCapacity(u, v) > 0 &&
            distance[u] > distance[v]) {
          Push(u, v);
        } else {
          ++seen[u];
        }
      } else {
        Relabel(u);
        seen[u] = 0;
      }
    }
  }

  int CalculateMaxFlow() {
    excess[0] = std::numeric_limits<int>::max();
    for (unsigned u = 1; u < network.size; ++u) {
      Push(0, u);
    }
    seen.resize(network.size, 0);
    while (!worklist.empty()) {
      unsigned u = worklist.front();
      worklist.pop();
      if (u != 0 && u != network.size - 1)
        Discharge(u);
    }
    int max_flow = 0;
    for (unsigned u = 0; u < network.size; ++u)
      max_flow += network.GetFlow(0, u);
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
      int c = v == 0 ? 0 : (unsigned)rnd.NextInt() % max_capacity + 1;
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
  fg.InitCapacity(0, 1, 1);
  fg.InitCapacity(1, 2, 2);
  fg.InitCapacity(2, 3, 3);
  fg.InitCapacity(3, 4, 4);
  auto it = fg.capacity.find({0, 1});
  EXPECT_TRUE(it != fg.capacity.end());
  EXPECT_TRUE(it->second == 1);
  PushAndRelabel calc(fg);
  EXPECT_TRUE(calc.CalculateMaxFlow() == 1);
}

TEST(MaxFlowTest, RandomNetwork) {
  auto fg = GenerateRandomFlowGraph(1000, 1000, 1000);
  PushAndRelabel calc(*fg);  
  calc.CalculateMaxFlow();
}

} // namespace
