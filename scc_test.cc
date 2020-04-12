#include "core.h"

#include <gtest/gtest.h>

struct SCC {
  const Graph &graph;
  const size_t size;
  const unsigned UNDEF;
  std::vector<unsigned> lowest_ancestor, dfo, tree_parent;
  std::vector<std::set<unsigned>> sccs;
  std::function<bool(unsigned, unsigned)> dfs_less;

  SCC(const Graph &graph)
      : graph(graph), size(graph.succ.size()), UNDEF(size),
        lowest_ancestor(size, UNDEF), dfo(size, UNDEF),
        tree_parent(size, UNDEF), sccs(size),
        dfs_less([this](unsigned u, unsigned v) { return dfo[u] < dfo[v]; }) {}

  void Calculate() {
    std::vector<unsigned> scc_stack;
    std::vector<bool> instack(size, false);
    unsigned depth_first_order = 0;
    auto pre_visit = [&, this](unsigned parent, unsigned u) {
      dfo[u] = depth_first_order++;
      tree_parent[u] = parent;
      lowest_ancestor[u] = u;
      scc_stack.push_back(u);
      instack[u] = true;
    };
    auto non_tree_visit = [&, this](unsigned u, unsigned v) {
      assert(dfo[u] != UNDEF && dfo[v] != UNDEF);
      if (instack[v])
        lowest_ancestor[u] =
            std::min(lowest_ancestor[u], lowest_ancestor[v], dfs_less);
    };
    auto post_visit = [&, this](unsigned u, unsigned parent) {
      if (parent != UNDEF) {
        lowest_ancestor[parent] =
            std::min(lowest_ancestor[u], lowest_ancestor[parent], dfs_less);
      }
      if (lowest_ancestor[u] == u) {
        unsigned v = scc_stack.back();
        while (v != u) {
          scc_stack.pop_back();
          instack[v] = false;
          sccs[u].insert(v);
          v = scc_stack.back();
        }
        assert(v == u);
        scc_stack.pop_back();
        instack[v] = false;
        sccs[u].insert(v);
      }
    };
    IterativeDepthDirstVisit(graph, pre_visit, non_tree_visit, post_visit);
  }
};

namespace {

TEST(SCCTest, Graph0) {
  Graph g(5, true);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  g.AddEdge(2, 3);
  g.AddEdge(3, 4);
  g.AddEdge(4, 1);
  SCC scc(g);
  scc.Calculate();
  EXPECT_TRUE(scc.lowest_ancestor[0] == 0);
  EXPECT_TRUE(scc.lowest_ancestor[1] == 1);
  EXPECT_TRUE(scc.lowest_ancestor[2] == 1);
  EXPECT_TRUE(scc.lowest_ancestor[3] == 1);
  EXPECT_TRUE(scc.lowest_ancestor[4] == 1);
  EXPECT_TRUE(scc.sccs[1].size() == 4);
}

TEST(SCCTest, Graph1) {
  Graph g(6, true);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  g.AddEdge(2, 3);
  g.AddEdge(3, 4);
  g.AddEdge(4, 1);
  g.AddEdge(0, 5);
  g.AddEdge(5, 4);
  SCC scc(g);
  scc.Calculate();
  EXPECT_TRUE(scc.sccs[scc.lowest_ancestor[1]].size() == 4);
  EXPECT_TRUE(scc.sccs[scc.lowest_ancestor[5]].size() == 1);
  EXPECT_TRUE(scc.sccs[scc.lowest_ancestor[0]].size() == 1);
}

TEST(SCCTest, RandomCFG) {
  const unsigned n = 100000, m = 300000;
  auto g = GenerateRandomControlFlowGraph(n, m);
  SCC scc(*g);
  scc.Calculate();
}

} // namespace
