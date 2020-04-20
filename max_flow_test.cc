#include "core.h"

#include <gtest/gtest.h>

struct FlowGraph {
  std::unique_ptr<Graph> graph;
  const unsigned source, target;
  std::map<std::tuple<unsigned, unsigned>, int> capacity;

  explicit FlowGraph(std::unique_ptr<Graph> &graph)
      : graph(std::move(graph)), source(0), target(this->graph->succ.size()) {}

  bool SetCapacity(unsigned u, unsigned v, int c) {
    assert(graph->succ[u].count(v));
    auto res = capacity.insert({{u, v}, c});
    return std::get<1>(res);
  }
};

namespace {

inline std::unique_ptr<FlowGraph>
GenerateRandomFlowGraph(size_t num_of_vertexes, size_t num_of_edges,
                        const unsigned max_capacity) {
  auto g = GenerateRandomDirectedGraph(num_of_vertexes, num_of_edges);
  auto fg = std::make_unique<FlowGraph>(g);
  Random rnd(std::time(nullptr));
  for (unsigned u = 0; u < fg->graph->succ.size(); ++u) {
    for (auto v : fg->graph->succ[u]) {
      unsigned c = v == 0 ? 0 : rnd.NextInt() % max_capacity + 1;
      if (fg->capacity.find({v, u}) == fg->capacity.end())
        fg->SetCapacity(u, v, c);
    }
  }
  return fg;
}

TEST(MaxFlowTest, FlowGraph) {
  auto g = std::make_unique<Graph>(5, true);
  g->AddEdge(0, 1);
  g->AddEdge(1, 2);
  g->AddEdge(1, 3);
  g->AddEdge(2, 3);
  g->AddEdge(2, 4);
  g->AddEdge(3, 4);
  FlowGraph fg(g);
  EXPECT_TRUE(fg.SetCapacity(0, 1, 5));
  EXPECT_FALSE(fg.SetCapacity(0, 1, 6));
  auto it = fg.capacity.find({0, 1});
  EXPECT_TRUE(it != fg.capacity.end());
  EXPECT_TRUE(it->second == 5);
  EXPECT_TRUE(fg.SetCapacity(1, 2, 3));
  EXPECT_TRUE(fg.SetCapacity(1, 3, 5));
}

TEST(MaxFlowTest, GenFlowGraph) {
  auto fg = GenerateRandomFlowGraph(100, 100, 1000);
  (void)fg;
}

} // namespace
