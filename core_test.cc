// Copyright (c) 2020 Kai Luo <gluokai@gmail.com>. All rights reserved.

#include "core.h"

#include <gtest/gtest.h>

namespace {

TEST(RandomGraphTest, DFS) {
  const unsigned n = 10000, m = 1000000;
  DirectedGraph g = CreateRandomDirectedGraph(n, m);
  std::vector<unsigned> rpo, dfo, dfs_tree_parent;
  DirectedGraph::DepthFirstLabel(g, &dfs_tree_parent, &rpo, &dfo, nullptr);
}

TEST(RandomGraphTest, BFS) {
  const unsigned n = 10000, m = 1000000;
  UndirectedGraph g = CreateRandomUndirectedGraph(n, m);
  UndirectedGraph::BreadthFirstVisitor V;
  g.Visit(V);
}

TEST(BitVectorTest, Basic) {
  BitVector BV;
  EXPECT_FALSE(BV.test(0));
  EXPECT_FALSE(BV.test(1));
  BV.set(1);
  BV.set(64);
  EXPECT_TRUE(BV.test(1));
  EXPECT_TRUE(BV.test(64));
  EXPECT_FALSE(BV.test(4096));
  BV.set(1UL << 20);
  EXPECT_TRUE(BV.test(1UL << 20));
}

} // namespace
