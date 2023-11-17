// Copyright (c) 2020 Kai Luo <gluokai@gmail.com>. All rights reserved.

#include "core.h"

#include <gtest/gtest.h>
#include <limits>
#include <queue>

struct Network {
  // std::unique_ptr<Graph> graph;
  const DirectedGraph &graph;
  const size_t size;
  const unsigned source, target;
  std::map<std::tuple<unsigned, unsigned>, int> capacity, flow, cost;

  explicit Network(const DirectedGraph &graph, unsigned source, unsigned target)
      : graph(graph), size(graph.size()), source(source), target(target) {}

  explicit Network(const DirectedGraph &graph)
      : graph(graph), size(graph.size()), source(0), target(size - 1) {}

  void InitCapacity(unsigned u, unsigned v, const int c) {
    assert(u != v);
    assert(c >= 0);
    std::get<0>(capacity.insert({{u, v}, 0}))->second = c;
  }

  void InitCost(unsigned u, unsigned v, const int c) {
    assert(u != v);
    std::get<0>(cost.insert({{u, v}, 0}))->second = c;
  }

  int GetCapacity(unsigned u, unsigned v) {
    auto it = capacity.find({u, v});
    if (it == capacity.end())
      return 0;
    return it->second;
  }

  int GetCost(unsigned u, unsigned v) {
    auto it = cost.find({u, v});
    if (it == cost.end())
      return 1;
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

  void AddToFlow(unsigned u, unsigned v, int diff) {
    std::get<0>(flow.insert({{u, v}, 0}))->second += diff;
  }
};

struct PushAndRelabel {
  static const unsigned INF = std::numeric_limits<unsigned>::max();
  Network &network;
  std::vector<int> excess;
  std::vector<unsigned> height, seen;
  std::function<bool(unsigned, unsigned)> height_less;
  std::priority_queue<unsigned, std::vector<unsigned>, decltype(height_less)>
      worklist;

  PushAndRelabel(Network &network)
      : network(network), height_less([this](unsigned u, unsigned v) {
          return height[u] < height[v];
        }),
        worklist(height_less) {}

  void Push(unsigned u, unsigned v) {
    assert(excess[u] >= 0);
    int diff = std::min(network.GetResidualCapacity(u, v), excess[u]);
    // printf("Before: RC of (%u, %u): %d\n", u, v,
    //        network.GetResidualCapacity(u, v));
    // printf("Before: Flow of (%u, %u): %d\n", u, v, network.GetFlow(u, v));
    network.AddToFlow(u, v, diff);
    network.AddToFlow(v, u, -diff);
    // printf("After: RC of (%u, %u): %d\n", u, v,
    //        network.GetResidualCapacity(u, v));
    // printf("After: Flow of (%u, %u): %d\n", u, v, network.GetFlow(u, v));
    excess[u] -= diff;
    excess[v] += diff;
    if (diff && excess[v] == diff)
      worklist.push(v);
  }

  void Relabel(unsigned u) {
    unsigned h = INF;
    for (unsigned v = 0; v < network.size; ++v) {
      if (network.GetResidualCapacity(u, v) > 0)
        h = std::min(h, height[v]);
    }
    if (h != INF)
      height[u] = h + 1;
  }

  void Discharge(unsigned u) {
    while (excess[u] > 0) {
      unsigned v = seen[u];
      // printf("%d seen %d\n", u, v);
      if (v < network.size) {
        if (network.GetResidualCapacity(u, v) > 0 && height[u] > height[v]) {
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
    height.resize(network.size, 0);
    height[network.source] = network.size;
    excess.resize(network.size, 0);
    excess[network.source] = std::numeric_limits<int>::max();
    seen.resize(network.size, 0);
    for (unsigned u = 0; u < network.size; ++u) {
      if (u != network.source)
        Push(network.source, u);
    }
    while (!worklist.empty()) {
      unsigned u = worklist.top();
      worklist.pop();
      if (u != network.source && u != network.target)
        Discharge(u);
    }
    int max_flow = 0;
    for (unsigned u = 0; u < network.size; ++u)
      max_flow += network.GetFlow(u, network.target);
    return max_flow;
  }
};

namespace {

struct GenNetwork {
  DirectedGraph g;
  Network n;
  GenNetwork(size_t n) : g(n), n(g) {}
};

inline std::unique_ptr<GenNetwork>
GenerateRandomNetwork(const size_t num_vertexes, const size_t num_edges,
                      const unsigned max_capacity) {
  auto nw = std::make_unique<GenNetwork>(num_vertexes);
  DirectedGraph::RandomGraph(nw->g, num_edges);
  auto &network = nw->n;
  Random rnd(std::time(nullptr));
  for (unsigned u = 0; u < network.size; ++u) {
    for (auto v : network.graph.succ(u)) {
      if (u == v)
        continue;
      int c =
          v == network.source ? 0 : (unsigned)rnd.NextInt() % max_capacity + 1;
      network.InitCapacity(u, v, c);
    }
  }
  return nw;
}

TEST(MaxFlowTest, SimpleNetwork) {
  DirectedGraph g(5);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  g.AddEdge(1, 3);
  g.AddEdge(2, 3);
  g.AddEdge(2, 4);
  g.AddEdge(3, 4);
  Network network(g);
  network.InitCapacity(0, 1, 1);
  network.InitCapacity(1, 2, 2);
  network.InitCapacity(2, 3, 3);
  network.InitCapacity(3, 4, 4);
  auto it = network.capacity.find({0, 1});
  EXPECT_TRUE(it != network.capacity.end());
  EXPECT_TRUE(it->second == 1);
  PushAndRelabel calc(network);
  EXPECT_TRUE(calc.CalculateMaxFlow() == 1);
}

TEST(MaxFlowTest, LuoGu3376) {
  DirectedGraph g(4);
  g.AddEdge(3, 1);
  g.AddEdge(3, 2);
  g.AddEdge(1, 2);
  g.AddEdge(1, 0);
  g.AddEdge(0, 2);
  Network network(g, 3, 2);
  network.InitCapacity(3, 1, 30);
  network.InitCapacity(3, 2, 20);
  network.InitCapacity(1, 2, 20);
  network.InitCapacity(1, 0, 30);
  network.InitCapacity(0, 2, 40);
  PushAndRelabel calc(network);
  int max_flow = calc.CalculateMaxFlow();
  // std::cout << max_flow << std::endl;
  EXPECT_TRUE(max_flow == 50);
}

TEST(MaxFlowTest, Partitioned) {
  DirectedGraph g(3);
  g.AddEdge(0, 1);
  Network network(g);
  network.InitCapacity(0, 1, 1);
  PushAndRelabel calc(network);
  EXPECT_TRUE(calc.CalculateMaxFlow() == 0);
  EXPECT_TRUE(calc.height[1] == network.size + 1);
}

TEST(MaxFlowTest, RandomNetwork) {
  auto network = GenerateRandomNetwork(1000, 1000, 1000);
  PushAndRelabel calc(network->n);
  calc.CalculateMaxFlow();
}

} // namespace
