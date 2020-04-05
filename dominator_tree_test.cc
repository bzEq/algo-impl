#include "core.h"

#include <gtest/gtest.h>
#include <iostream>

struct DominatorTree {
  Graph *cfg;
  const unsigned size, UNDEF;
  std::vector<unsigned> idom;
  DominatorTree(Graph *graph)
      : cfg(graph), size(cfg->adjacents.size()), UNDEF(cfg->adjacents.size()) {
    idom.resize(size);
  }
  void Calculate() {
    std::vector<unsigned> dfo, rpo;
    SimpleIterativeDFS(*cfg, &dfo, &rpo);
    idom[0] = 0;
    for (unsigned i = 1; i < size; ++i) {
      idom[i] = UNDEF;
    }
    auto Intersect = [&](unsigned u, unsigned v) {
      while (u != v) {
        while (dfo[u] > dfo[v])
          u = idom[u];
        while (dfo[u] < dfo[v])
          v = idom[v];
      }
      return u;
    };
    std::vector<bool> inqueue(size, false);
    std::vector<unsigned> worklist;
    // std::iota(worklist.begin(), worklist.end(), 0);
    // std::sort(worklist.begin(), worklist.end(),
    //           [](unsigned u, unsigned v) { return rpo[u] < rpo[v]; });
    worklist.push_back(0);
    inqueue[0] = true;
    while (!worklist.empty()) {
      unsigned u = worklist.back();
      worklist.pop_back();
      inqueue[u] = false;
      assert(idom[u] != UNDEF);
      for (auto v : cfg->adjacents[u]) {
        unsigned new_idom = idom[v];
        if (new_idom != UNDEF)
          new_idom = Intersect(new_idom, u);
        else
          new_idom = u;
        if (new_idom != idom[v]) {
          idom[v] = new_idom;
          if (!inqueue[v]) {
            worklist.push_back(v);
            inqueue[v] = true;
          }
        }
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
  dt.Calculate();
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
  dt.Calculate();
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
  dt.Calculate();
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
  dt.Calculate();
  EXPECT_TRUE(dt.idom[2] == 1);
  EXPECT_TRUE(dt.idom[1] == 0);
  EXPECT_TRUE(dt.idom[0] == 0);
}

TEST(DominatorTreeTest, RandomCFG) {
  const unsigned n = 100000, m = 1000000;  
  auto g = GenerateRandomControlFlowGraph(n, m);
  DominatorTree dt(g.get());
  dt.Calculate();
}

} // namespace
