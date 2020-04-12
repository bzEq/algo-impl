// Copyright (c) 2020 Kai Luo <gluokai@gmail.com>. All rights reserved.

#include "core.h"

#include <gtest/gtest.h>

namespace {

TEST(RandomGraphTest, DFS) {
  const unsigned n = 10000, m = 1000000;
  auto g = GenerateRandomControlFlowGraph(n, m);
  std::vector<unsigned> rpo, dfo, dfs_tree_parent;
  SimpleIterativeDFS(*g, &dfo, &rpo, &dfs_tree_parent);
}
} // namespace
