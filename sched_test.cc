#include "core.h"

#include <algorithm>
#include <bitset>
#include <gtest/gtest.h>
#include <vector>

struct Sched {
  static constexpr unsigned kMaxSize = 1024;
  const DirectedGraph &graph;
  std::vector<std::bitset<kMaxSize>> sub_pos;
  explicit Sched(const DirectedGraph &graph)
      : graph(graph), sub_pos(graph.size()) {}

  std::vector<std::vector<unsigned>> Group() {
    assert(DirectedGraph::IsDAG(graph));
    const size_t size = graph.size();
    assert(size <= kMaxSize);
    std::vector<unsigned> dfo(size, UNDEF), rpo(size, UNDEF);
    unsigned depth_first_order = 0, depth_post_order = 0;
    DirectedGraph::DepthFirstVisitor DFV;
    DFV.tree_visit = [&](unsigned parent, unsigned u) {
      dfo[u] = depth_first_order++;
      sub_pos[u].set(u);
    };
    DFV.non_tree_visit = [&](unsigned u, unsigned v) {
      if (dfo[v] < dfo[u])
        sub_pos[u] |= sub_pos[v];
    };
    DFV.post_visit = [&](unsigned u, unsigned parent) {
      rpo[u] = (size - 1) - depth_post_order;
      ++depth_post_order;
      if (parent != UNDEF)
        sub_pos[parent] |= sub_pos[u];
    };
    graph.Visit(DFV);
    std::vector<unsigned> worklist(size);
    std::iota(worklist.begin(), worklist.end(), 0);
    std::vector<unsigned> bfo(size, UNDEF);
    unsigned breadth_first_order = 0;
    DirectedGraph::BreadthFirstVisitor BFV;
    BFV.tree_visit = [&](unsigned parent, unsigned u) {
      bfo[u] = breadth_first_order++;
    };
    graph.Visit(BFV);
    std::sort(worklist.begin(), worklist.end(),
              [&](unsigned u, unsigned v) { return bfo[u] > bfo[v]; });
    std::vector<std::vector<unsigned>> result;
    std::vector<unsigned> active_group;
    std::bitset<kMaxSize> active_set;
    for (unsigned u : worklist) {
      if ((active_set & sub_pos[u]).none()) {
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

  static void OutputResult(std::ostream &os,
                           const std::vector<std::vector<unsigned>> &result) {
    for (size_t i = 0; i < result.size(); ++i) {
      os << "Group #" << i << ":\n";
      for (unsigned u : result[i])
        os << u << std::endl;
    }
  }
};

TEST(SchedTest, Basic) {
  DirectedGraph dag(6);
  dag.AddEdge(0, 1);
  dag.AddEdge(0, 2);
  dag.AddEdge(1, 3);
  dag.AddEdge(1, 4);
  dag.AddEdge(2, 4);
  dag.AddEdge(2, 5);
  dag.AddEdge(5, 4);
  assert(DirectedGraph::IsDAG(dag));
  Sched s(dag);
  auto result = s.Group();
  Sched::OutputResult(std::cout, result);
}

TEST(SchedTest, Basic1) {
  DirectedGraph dag(6);
  dag.AddEdge(0, 1);
  dag.AddEdge(1, 2);
  dag.AddEdge(2, 3);
  dag.AddEdge(3, 4);
  dag.AddEdge(4, 5);
  assert(DirectedGraph::IsDAG(dag));
  Sched s(dag);
  auto result = s.Group();
  Sched::OutputResult(std::cout, result);
}

TEST(SchedTest, Basic2) {
  DirectedGraph dag(6);
  dag.AddEdge(0, 1);
  dag.AddEdge(1, 2);
  dag.AddEdge(2, 3);
  dag.AddEdge(0, 4);
  dag.AddEdge(1, 4);
  dag.AddEdge(2, 5);
  dag.AddEdge(4, 5);
  assert(DirectedGraph::IsDAG(dag));
  Sched s(dag);
  auto result = s.Group();
  Sched::OutputResult(std::cout, result);
}
