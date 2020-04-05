// Copyright (c) 2020 Kai Luo <gluokai@gmail.com>. All rights reserved.

#include "core.h"

#include <gtest/gtest.h>

namespace {

TEST(RandomGraphTest, DFS) {
  const unsigned n = 10000, m = 1000000;
  auto g = GenerateRandomControlFlowGraph(n, m);
  std::vector<unsigned> idfs_dfo, idfs_rpo, dfs_dfo, dfs_rpo;
  SimpleIterativeDFS(*g, &idfs_dfo, &idfs_rpo);
  SimpleDFS(*g, &dfs_dfo, &dfs_rpo);
  for (unsigned i = 0; i < n; ++i) {
    EXPECT_TRUE(idfs_rpo[i] == dfs_rpo[i]);
    EXPECT_TRUE(idfs_dfo[i] == dfs_dfo[i]);
  }
}
} // namespace
