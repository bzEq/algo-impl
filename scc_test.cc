// Copyright (c) 2020 Kai Luo <gluokai@gmail.com>. All rights reserved.

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
      : graph(graph), size(graph.succ.size()), UNDEF(~0U),
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
        lowest_ancestor[u] = std::min(lowest_ancestor[u], v, dfs_less);
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
    CompressLowestAncestor();
  }

  void CompressLowestAncestor() {
    std::function<unsigned(unsigned)> compress = [&, this](unsigned u) {
      if (lowest_ancestor[u] == u)
        return u;
      unsigned lowest = compress(lowest_ancestor[u]);
      lowest_ancestor[u] = lowest;
      return lowest;
    };
    for (unsigned u = 0; u < size; ++u) {
      assert(lowest_ancestor[u] != UNDEF);
      compress(u);
    }
  }

  std::unique_ptr<Graph> DeriveDAG() {
    auto dag = std::unique_ptr<Graph>(new Graph(size, true));
    for (unsigned u = 0; u < size; ++u) {
      unsigned U = lowest_ancestor[u];
      assert(lowest_ancestor[U] == U);
      for (unsigned v : graph.succ[u]) {
        unsigned V = lowest_ancestor[v];
        assert(lowest_ancestor[V] == V);
        if (U != V)
          dag->AddEdge(U, V);
      }
    }
    return dag;
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

TEST(SCCTest, DAGTest0) {
  Graph g(6, true);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  g.AddEdge(2, 3);
  g.AddEdge(3, 4);
  g.AddEdge(4, 1);
  g.AddEdge(0, 5);
  g.AddEdge(5, 4);
  EXPECT_TRUE(not IsDAG(g));
}

TEST(SCCTest, DAGTest1) {
  Graph g(7, true);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  g.AddEdge(2, 3);
  g.AddEdge(3, 4);
  g.AddEdge(4, 5);
  g.AddEdge(0, 5);
  EXPECT_TRUE(IsDAG(g));
}

TEST(SCCTest, RandomCFG) {
  const unsigned n = 100000, m = 300000;
  auto g = GenerateRandomDirectedGraph(n, m);
  SCC scc(*g);
  scc.Calculate();
  for (unsigned u = 0; u < n; ++u) {
    EXPECT_TRUE(u == scc.lowest_ancestor[u] ||
                scc.dfs_less(scc.lowest_ancestor[u], u));
    unsigned p = scc.lowest_ancestor[u];
    EXPECT_TRUE(scc.lowest_ancestor[p] == p);
  }
  auto dag = scc.DeriveDAG();
  EXPECT_TRUE(IsDAG(*dag));
}

} // namespace
