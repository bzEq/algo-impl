#include "core.h"

#include "gtest/gtest.h"
#include <bitset>
#include <vector>

struct Sched {
  static constexpr unsigned kMaxSize = 1024;
  const Graph &graph;
  std::vector<std::bitset<kMaxSize>> sub_pos;
  explicit Sched(const Graph &graph) : graph(graph), sub_pos(graph.size) {}

  std::vector<std::vector<unsigned>> Group() {
    assert(IsDAG(graph));
    const size_t size = graph.size;
    std::vector<unsigned> dfo(size, UNDEF), rpo(size, UNDEF);
    unsigned depth_first_order = 0, depth_post_order = 0;
    auto pre_visit = [&](unsigned parent, unsigned u) {
      dfo[u] = depth_first_order++;
      sub_pos[u].set(u);
    };
    auto non_tree_visit = [&](unsigned u, unsigned v) {
      assert(dfo[v] < dfo[u]);
      sub_pos[u] |= sub_pos[v];
    };
    auto post_visit = [&](unsigned u, unsigned parent) {
      rpo[u] = (size - 1) - depth_post_order;
      ++depth_post_order;
      if (parent != UNDEF)
        sub_pos[parent] |= sub_pos[u];
    };
    IterativeDepthFirstVisit(graph, pre_visit, non_tree_visit, post_visit);
    std::vector<unsigned> worklist(size);
    std::iota(worklist.begin(), worklist.end(), 0);
    std::sort(worklist.begin(), worklist.end(),
              [&](unsigned u, unsigned v) { return rpo[u] < rpo[v]; });
    std::vector<std::vector<unsigned>> result;
    std::vector<unsigned> active_group;
    std::bitset<kMaxSize> active_set;
    for (unsigned u : worklist) {
      if (!active_set.test(u)) {
        active_set |= sub_pos[u];
        active_group.emplace_back(u);
      } else {
        result.emplace_back(std::move(active_group));
        active_group.emplace_back(u);
        active_set.reset();
        active_set |= sub_pos[u];
      }
    }
    if (!active_group.empty())
      result.emplace_back(std::move(active_group));
    return result;
  }
};

TEST(SchedTest, Basic) {
  Graph dag(6, true);
  dag.AddEdge(0, 1);
  dag.AddEdge(0, 2);
  dag.AddEdge(1, 3);
  dag.AddEdge(1, 4);
  dag.AddEdge(2, 4);
  dag.AddEdge(2, 5);
  assert(IsDAG(dag));
  Sched s(dag);
  auto result = s.Group();
  for (size_t i = 0; i < result.size(); ++i) {
    std::cout << "Group #" << i << ":\n";
    for (unsigned u : result[i])
      std::cout << u << std::endl;
  }
}
