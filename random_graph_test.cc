// Copyright (c) 2020 Kai Luo <gluokai@gmail.com>. All rights reserved.

#include "core.h"

#include <gtest/gtest.h>

namespace {

TEST(RandomGraphTest, DFS) {
  const unsigned n = 10000, m = 1000000;
  auto g = GenerateRandomControlFlowGraph(n, m);
  std::vector<unsigned> INum, Num;
  SimpleIterativeDFS(*g, &INum);
  SimpleDFS(*g, &Num);
  for (unsigned i = 0; i < n; ++i) {
    EXPECT_TRUE(INum[i] == Num[i]);
  }
}
} // namespace
