// Copyright (c) 2020 Kai Luo <gluokai@gmail.com>. All rights reserved.

#include "core.h"

#include <gtest/gtest.h>

namespace {

TEST(RandomGraphTest, DFS) {
  const unsigned n = 10000, m = 1000000;
  DirectedGraph g(n);
  DirectedGraph::RandomGraph(g, m);
  std::vector<unsigned> rpo, dfo, dfs_tree_parent;
  DirectedGraph::DepthFirstLabel(g, &dfs_tree_parent, &rpo, &dfo, nullptr);
}

TEST(RandomGraphTest, BFS) {
  const unsigned n = 10000, m = 1000000;
  DirectedGraph g(n);
  DirectedGraph::RandomGraph(g, m);
  DirectedGraph::DepthFirstLabel(g, nullptr, nullptr, nullptr, nullptr);
}

} // namespace
