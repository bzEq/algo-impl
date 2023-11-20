// Copyright (c) 2020 Kai Luo <gluokai@gmail.com>. All rights reserved.

#include "core.h"

#include <algorithm>
#include <gtest/gtest.h>
#include <iostream>

struct DominatorTree {
  const DirectedGraph &cfg;
  const size_t size;
  using Vertex = DirectedGraph::Vertex;
  std::vector<unsigned> dfo, rpo, dfs_tree_parent, semi, idom, lt_ancestor,
      best;
  std::vector<std::set<Vertex>> dominance_frontier, lt_bucket;
  std::function<bool(Vertex, Vertex)> dfs_less, dfs_greater;

  DominatorTree(const DirectedGraph &graph)
      : cfg(graph), size(cfg.size()), semi(size, UNDEF), idom(size, UNDEF),
        lt_ancestor(size, UNDEF), best(size, UNDEF), dominance_frontier(size),
        lt_bucket(size), dfs_less([this](Vertex u, Vertex v) {
          return dfo[u] < dfo[v];
        }),
        dfs_greater([this](Vertex u, Vertex v) {
          return dfo[u] > dfo[v];
        }) {
    assert(cfg.entry() == 0);
    DirectedGraph::DepthFirstLabel(cfg, &dfs_tree_parent, &rpo, &dfo, nullptr);
  }

  void Link(Vertex u, Vertex v) {
    assert(u != UNDEF && v != UNDEF && dfo[u] < dfo[v]);
    lt_ancestor[v] = u;
    best[v] = v;
  }

  Vertex Eval(Vertex v) {
    Vertex a = lt_ancestor[v];
    assert(a != UNDEF);
    if (lt_ancestor[a] != UNDEF) {
      Vertex b = Eval(a);
      lt_ancestor[v] = lt_ancestor[a];
      if (dfs_less(semi[b], semi[best[v]])) {
        best[v] = b;
      }
    }
    assert(best[v] != UNDEF);
    return best[v];
  }

  bool ReachableFromOrigin(Vertex w) {
    if (dfo[w] == UNDEF)
      return false;
    assert(dfo[0] != UNDEF && dfo[0] <= dfo[w]);
    return (size - 1 - rpo[0]) >= (size - 1 - rpo[w]);
  }

  // Simple Lengauer-Tarjan algorithm.
  void CalculateDTViaSLT() {
    idom[0] = 0;
    std::vector<Vertex> worklist(size);
    std::iota(worklist.begin(), worklist.end(), 0);
    std::sort(worklist.begin(), worklist.end(), dfs_greater);
    for (Vertex w : worklist) {
      if (!ReachableFromOrigin(w) || dfs_tree_parent[w] == UNDEF)
        continue;
      const Vertex p = dfs_tree_parent[w];
      semi[w] = p;
      for (Vertex v : cfg.pred(w)) {
        if (!ReachableFromOrigin(v))
          continue;
        if (dfs_less(v, w)) {
          semi[w] = std::min(semi[w], v, dfs_less);
        } else if (dfs_greater(v, w)) {
          semi[w] = std::min(semi[w], semi[Eval(v)], dfs_less);
        }
      }
      assert(semi[w] < size);
      Link(p, w);
      lt_bucket[semi[w]].insert(w);
      for (Vertex v : lt_bucket[p]) {
        assert(semi[v] == p);
        Vertex u = Eval(v);
        if (dfs_less(semi[u], p))
          idom[v] = u;
        else
          idom[v] = semi[v];
      }
      lt_bucket[p].clear();
    }
    for (auto it = worklist.rbegin(); it != worklist.rend(); ++it) {
      Vertex w = *it;
      if (semi[w] != UNDEF && idom[w] != semi[w]) {
        assert(idom[w] != UNDEF);
        idom[w] = idom[idom[w]];
      }
    }
  }

  Vertex CalculateNCA(Vertex u, Vertex v) {
    while (u != v) {
      while (dfs_greater(u, v))
        u = idom[u];
      while (dfs_less(u, v))
        v = idom[v];
    }
    return u;
  }

  void CalculateDTViaDataFlow() {
    idom[0] = 0;
    std::vector<Vertex> worklist(size);
    std::iota(worklist.begin(), worklist.end(), 0);
    std::sort(worklist.begin(), worklist.end(),
              [this](Vertex u, Vertex v) { return rpo[u] < rpo[v]; });
    bool changed = true;
    while (changed) {
      changed = false;
      for (auto u : worklist) {
        if (!ReachableFromOrigin(u) || idom[u] == UNDEF)
          continue;
        for (auto v : cfg.succ(u)) {
          Vertex new_idom = idom[v];
          if (new_idom != UNDEF)
            new_idom = CalculateNCA(new_idom, u);
          else
            new_idom = u;
          if (new_idom != idom[v]) {
            idom[v] = new_idom;
            changed = true;
          }
        }
      }
    }
  }

  void CalculateDT() { CalculateDTViaSLT(); }

  void CalculateDF() {
    for (Vertex u : cfg.all_vertex()) {
      if (idom[u] == UNDEF)
        continue;
      for (Vertex v : cfg.succ(u)) {
        if (idom[v] == UNDEF)
          continue;
        Vertex nca = CalculateNCA(u, v);
        Vertex w = u;
        while (w != nca) {
          dominance_frontier[w].insert(v);
          w = idom[w];
        }
        if (w == v)
          dominance_frontier[w].insert(v);
      }
    }
  }
};

namespace {
TEST(DominatorTreeTest, LinearGraph) {
  DirectedGraph g(5);
  g.SetEntry(0);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  g.AddEdge(2, 3);
  g.AddEdge(3, 4);
  DominatorTree dt(g);
  dt.CalculateDTViaSLT();
  DominatorTree dt1(g);
  dt1.CalculateDTViaDataFlow();
  auto check = [](DominatorTree &dt) {
    EXPECT_TRUE(dt.idom[4] == 3);
    EXPECT_TRUE(dt.idom[3] == 2);
    EXPECT_TRUE(dt.idom[2] == 1);
    EXPECT_TRUE(dt.idom[1] == 0);
    EXPECT_TRUE(dt.idom[0] == 0);
  };
  check(dt);
  check(dt1);
}

TEST(DominatorTreeTest, Graph0) {
  DirectedGraph g(6);
  g.SetEntry(0);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  g.AddEdge(1, 3);
  g.AddEdge(3, 1);
  g.AddEdge(3, 5);
  g.AddEdge(0, 4);
  g.AddEdge(4, 5);
  DominatorTree dt(g);
  dt.CalculateDTViaSLT();
  DominatorTree dt1(g);
  dt1.CalculateDTViaDataFlow();
  auto check = [](DominatorTree &dt) {
    EXPECT_TRUE(dt.idom[5] == 0);
    EXPECT_TRUE(dt.idom[4] == 0);
    EXPECT_TRUE(dt.idom[3] == 1);
    EXPECT_TRUE(dt.idom[2] == 1);
    EXPECT_TRUE(dt.idom[1] == 0);
    EXPECT_TRUE(dt.idom[0] == 0);
  };
  check(dt);
  check(dt1);
}

TEST(DominatorTreeTest, Graph1) {
  DirectedGraph g(3);
  g.SetEntry(0);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  DominatorTree dt(g);
  dt.CalculateDTViaSLT();
}

// Based on the figure of
// https://www.cs.princeton.edu/courses/archive/fall03/cs528/handouts/a%20fast%20algorithm%20for%20finding.pdf
TEST(DominatorTreeTest, Tarjan79) {
  DirectedGraph g(13);
  g.SetEntry(0);
  g.AddEdge(0, 1);  // R->A
  g.AddEdge(0, 2);  // R->B
  g.AddEdge(0, 3);  // R->C
  g.AddEdge(1, 4);  // A->D
  g.AddEdge(2, 1);  // B->A
  g.AddEdge(2, 4);  // B->D
  g.AddEdge(2, 5);  // B->E
  g.AddEdge(3, 6);  // C->F
  g.AddEdge(3, 7);  // C->G
  g.AddEdge(4, 12); // D->L
  g.AddEdge(5, 8);  // E->H
  g.AddEdge(6, 9);  // F->I
  g.AddEdge(7, 9);  // G->I
  g.AddEdge(7, 10); // G->J
  g.AddEdge(8, 11); // H->K
  g.AddEdge(8, 5);  // H->E
  g.AddEdge(9, 11); // I->K
  g.AddEdge(10, 9); // J->I
  g.AddEdge(11, 9); // K->I
  g.AddEdge(11, 0); // K->R
  g.AddEdge(12, 8); // L->H
  DominatorTree dt(g);
  dt.CalculateDTViaSLT();
  DominatorTree dt1(g);
  dt1.CalculateDTViaDataFlow();
  auto check = [](DominatorTree &dt) {
    EXPECT_TRUE(dt.idom[0] == 0);  // R
    EXPECT_TRUE(dt.idom[1] == 0);  // A
    EXPECT_TRUE(dt.idom[2] == 0);  // B
    EXPECT_TRUE(dt.idom[3] == 0);  // C
    EXPECT_TRUE(dt.idom[4] == 0);  // D
    EXPECT_TRUE(dt.idom[5] == 0);  // E
    EXPECT_TRUE(dt.idom[6] == 3);  // F
    EXPECT_TRUE(dt.idom[7] == 3);  // G
    EXPECT_TRUE(dt.idom[8] == 0);  // H
    EXPECT_TRUE(dt.idom[9] == 0);  // I
    EXPECT_TRUE(dt.idom[10] == 7); // J
    EXPECT_TRUE(dt.idom[11] == 0); // K
    EXPECT_TRUE(dt.idom[12] == 4); // L
  };
  check(dt);
  check(dt1);
}

TEST(DominatorTreeTest, SelfLoop) {
  DirectedGraph g(3);
  g.SetEntry(0);
  g.AddEdge(0, 1);
  g.AddEdge(1, 1);
  g.AddEdge(1, 2);
  g.AddEdge(2, 2);
  DominatorTree dt(g);
  dt.CalculateDTViaSLT();
  EXPECT_TRUE(dt.idom[2] == 1);
  EXPECT_TRUE(dt.idom[1] == 0);
  EXPECT_TRUE(dt.idom[0] == 0);
  dt.CalculateDF();
  EXPECT_TRUE(dt.dominance_frontier[0].empty());
  EXPECT_TRUE(dt.dominance_frontier[1].size() == 1);
  EXPECT_TRUE(dt.dominance_frontier[1].count(1));
  EXPECT_TRUE(dt.dominance_frontier[2].size() == 1);
  EXPECT_TRUE(dt.dominance_frontier[2].count(2));
}

TEST(DominatorTreeTest, SimpleLoop) {
  DirectedGraph g(4);
  g.SetEntry(0);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  g.AddEdge(2, 1);
  g.AddEdge(2, 3);
  DominatorTree dt(g);
  EXPECT_TRUE(dt.dfs_tree_parent[0] == UNDEF);
  EXPECT_TRUE(dt.dfs_tree_parent[1] == 0);
  EXPECT_TRUE(dt.dfs_tree_parent[2] == 1);
  EXPECT_TRUE(dt.dfs_tree_parent[3] == 2);
  dt.CalculateDTViaSLT();
  dt.CalculateDF();
  EXPECT_TRUE(dt.dominance_frontier[0].empty());
  EXPECT_TRUE(dt.dominance_frontier[1].size() == 1);
  EXPECT_TRUE(dt.dominance_frontier[1].count(1));
  EXPECT_TRUE(dt.dominance_frontier[2].size() == 1);
  EXPECT_TRUE(dt.dominance_frontier[2].count(1));
  EXPECT_TRUE(dt.dominance_frontier[3].empty());
}

TEST(DominatorTreeTest, WeirdGraph) {
  DirectedGraph g(4);
  g.SetEntry(0);
  g.AddEdge(1, 0);
  g.AddEdge(2, 0);
  g.AddEdge(1, 2);
  g.AddEdge(0, 3);
  DominatorTree dt(g);
  DominatorTree dt1(g);
  dt.CalculateDTViaSLT();
  dt1.CalculateDTViaDataFlow();
  for (auto u : g.all_vertex()) {
    // std::cout << dt.idom[u] << " " << dt1.idom[u] << "\n";
    EXPECT_TRUE(dt.idom[u] == dt1.idom[u]);
  }
}

TEST(DominatorTreeTest, RandomCFG) {
  const unsigned n = 100000, m = 300000;
  DirectedGraph g(n);
  DirectedGraph::RandomGraph(g, m);
  g.SetEntry(0);
  DominatorTree dt(g);
  DominatorTree dt1(g);
  dt.CalculateDTViaSLT();
  dt1.CalculateDTViaDataFlow();
  for (auto u : g.all_vertex()) {
    // std::cout << dt.idom[u] << " " << dt1.idom[u] << "\n";
    EXPECT_TRUE(dt.idom[u] == dt1.idom[u]);
  }
}

static DirectedGraph *GetBenchmarkGraph() {
  const unsigned n = 100000, m = 500000;
  struct Once {
    DirectedGraph g;
    Once() : g(n) {
      DirectedGraph::RandomGraph(g, m);
      g.SetEntry(0);
    }
  };
  static Once o;
  return &o.g;
}

TEST(DominatorTreeBenchmark, GenBenchmarkGraph) { GetBenchmarkGraph(); }

TEST(DominatorTreeBenchmark, BenchmarkSLT) {
  auto g = GetBenchmarkGraph();
  DominatorTree dt(*g);
  dt.CalculateDTViaSLT();
  dt.CalculateDF();
}

TEST(DominatorTreeBenchmark, BenchmarkDataFlow) {
  auto g = GetBenchmarkGraph();
  DominatorTree dt(*g);
  dt.CalculateDTViaDataFlow();
  dt.CalculateDF();
}

} // namespace
