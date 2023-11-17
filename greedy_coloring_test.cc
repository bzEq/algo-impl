// Copyright (c) 2023 Kai Luo <gluokai@gmail.com>. All rights reserved.

#include "core.h"

#include <gtest/gtest.h>

struct GreedyColoring {
  const UndirectedGraph &graph;
  std::vector<unsigned> colors;
  GreedyColoring(UndirectedGraph &g) : graph(g), colors(g.size(), UNDEF) {}

  void Coloring() {
    UndirectedGraph::BreadthFirstVisitor BFV;
    BFV.tree_visit = [&](UndirectedGraph::Vertex _, UndirectedGraph::Vertex u) {
      std::unordered_set<unsigned> used_colors;
      for (UndirectedGraph::Vertex v : graph.succ(u)) {
        if (colors[v] != UNDEF)
          used_colors.insert(colors[v]);
      }
      unsigned mex = 0;
      while (used_colors.count(mex))
        ++mex;
      assert(!used_colors.count(mex));
      colors[u] = mex;
    };
    graph.Visit(BFV);
  }

  size_t NumColors() const {
    std::unordered_set<unsigned> used_colors;
    for (UndirectedGraph::Vertex u : graph.all_vertex())
      used_colors.insert(colors[u]);
    return used_colors.size();
  }
};

TEST(GCTest, Simple) {
  UndirectedGraph g(2);
  g.AddEdge(0, 1);
  GreedyColoring GC(g);
  GC.Coloring();
  EXPECT_TRUE(GC.NumColors() == 2);
}

TEST(GCTest, Simple3) {
  UndirectedGraph g(3);
  g.AddEdge(0, 1);
  g.AddEdge(0, 2);
  GreedyColoring GC(g);
  GC.Coloring();
  EXPECT_TRUE(GC.NumColors() == 2);
}

TEST(GCTest, Triangle) {
  UndirectedGraph g(3);
  g.AddEdge(0, 1);
  g.AddEdge(0, 2);
  g.AddEdge(1, 2);
  GreedyColoring GC(g);
  GC.Coloring();
  EXPECT_TRUE(GC.NumColors() == 3);
}

TEST(GCTest, CompleteGraph) {
  const size_t n = 1UL << 10;
  UndirectedGraph g(n);
  for (size_t i = 0; i < n; ++i)
    for (size_t j = i + 1; j < n; ++j)
      g.AddEdge(i, j);
  GreedyColoring GC(g);
  GC.Coloring();
  EXPECT_TRUE(GC.NumColors() == n);
}

TEST(GCTest, Random) {
  UndirectedGraph g(1UL << 16);
  UndirectedGraph::RandomGraph(g, 1UL << 20);
  GreedyColoring GC(g);
  GC.Coloring();
  std::cout << GC.NumColors() << "\n";
}
