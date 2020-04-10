#include "core.h"

#include <gtest/gtest.h>
#include <iostream>

struct DominatorTree {
  const Graph *cfg;
  const unsigned size, UNDEF;
  std::vector<unsigned> sdom, idom, dfo, rpo;
  std::vector<unsigned> dfs_tree_parent;
  std::vector<std::unordered_set<unsigned>> dominance_frontier;

  DominatorTree(const Graph *graph)
      : cfg(graph), size(cfg->succ.size()), UNDEF(cfg->succ.size()) {
    sdom.resize(size);
    idom.resize(size);
    dominance_frontier.resize(size);
    SimpleIterativeDFS(*cfg, &dfo, &rpo, &dfs_tree_parent);
  }

  unsigned CalculateNCA(unsigned u, unsigned v) {
    while (u != v) {
      while (dfo[u] > dfo[v])
        u = idom[u];
      while (dfo[u] < dfo[v])
        v = idom[v];
    }
    return u;
  }

  void CalculateDTViaDataFlow() {
    idom[0] = 0;
    for (unsigned i = 1; i < size; ++i) {
      idom[i] = UNDEF;
    }
    std::vector<unsigned> worklist(size);
    std::iota(worklist.begin(), worklist.end(), 0);
    std::sort(worklist.begin(), worklist.end(),
              [this](unsigned u, unsigned v) { return rpo[u] < rpo[v]; });
    bool changed = true;
    while (changed) {
      changed = false;
      for (auto u : worklist) {
        if (idom[u] == UNDEF)
          continue;
        for (auto v : cfg->succ[u]) {
          unsigned new_idom = idom[v];
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

  void CalculateDT() { CalculateDTViaDataFlow(); }

  void CalculateDF() {
    for (unsigned u = 0; u < size; ++u) {
      if (idom[u] == UNDEF)
        continue;
      for (const unsigned v : cfg->succ[u]) {
        if (idom[v] == UNDEF)
          continue;
        unsigned nca = CalculateNCA(u, v);
        unsigned w = u;
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
  Graph g(5, true);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  g.AddEdge(2, 3);
  g.AddEdge(3, 4);
  DominatorTree dt(&g);
  dt.CalculateDT();
  EXPECT_TRUE(dt.idom[4] == 3);
  EXPECT_TRUE(dt.idom[3] == 2);
  EXPECT_TRUE(dt.idom[2] == 1);
  EXPECT_TRUE(dt.idom[1] == 0);
  EXPECT_TRUE(dt.idom[0] == 0);
}

TEST(DominatorTreeTest, Graph0) {
  Graph g(6, true);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  g.AddEdge(1, 3);
  g.AddEdge(3, 1);
  g.AddEdge(3, 5);
  g.AddEdge(0, 4);
  g.AddEdge(4, 5);
  DominatorTree dt(&g);
  dt.CalculateDT();
  EXPECT_TRUE(dt.idom[5] == 0);
  EXPECT_TRUE(dt.idom[4] == 0);
  EXPECT_TRUE(dt.idom[3] == 1);
  EXPECT_TRUE(dt.idom[2] == 1);
  EXPECT_TRUE(dt.idom[1] == 0);
  EXPECT_TRUE(dt.idom[0] == 0);
}

// Based on the figure of
// https://www.cs.princeton.edu/courses/archive/fall03/cs528/handouts/a%20fast%20algorithm%20for%20finding.pdf
TEST(DominatorTreeTest, Tarjan79) {
  Graph g(13, true);
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
  DominatorTree dt(&g);
  dt.CalculateDT();
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
}

TEST(DominatorTreeTest, SelfLoop) {
  Graph g(3, true);
  g.AddEdge(0, 1);
  g.AddEdge(1, 1);
  g.AddEdge(1, 2);
  g.AddEdge(2, 2);
  DominatorTree dt(&g);
  dt.CalculateDT();
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
  Graph g(4, true);
  g.AddEdge(0, 1);
  g.AddEdge(1, 2);
  g.AddEdge(2, 1);
  g.AddEdge(2, 3);
  DominatorTree dt(&g);
  EXPECT_TRUE(dt.dfs_tree_parent[0] == 0);
  EXPECT_TRUE(dt.dfs_tree_parent[1] == 0);
  EXPECT_TRUE(dt.dfs_tree_parent[2] == 1);
  EXPECT_TRUE(dt.dfs_tree_parent[3] == 2);
  dt.CalculateDT();
  dt.CalculateDF();
  EXPECT_TRUE(dt.dominance_frontier[0].empty());
  EXPECT_TRUE(dt.dominance_frontier[1].size() == 1);
  EXPECT_TRUE(dt.dominance_frontier[1].count(1));
  EXPECT_TRUE(dt.dominance_frontier[2].size() == 1);
  EXPECT_TRUE(dt.dominance_frontier[2].count(1));
  EXPECT_TRUE(dt.dominance_frontier[3].empty());
}

TEST(DominatorTreeTest, RandomCFG) {
  const unsigned n = 100000, m = 300000;
  auto g = GenerateRandomControlFlowGraph(n, m);
  DominatorTree dt(g.get());
  dt.CalculateDT();
  dt.CalculateDF();
}

} // namespace
