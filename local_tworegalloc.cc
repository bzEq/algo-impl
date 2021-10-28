#include <assert.h>
#include <gtest/gtest.h>

enum {
  USE_A,
  USE_B,
};

struct LocalTwoRegAlloc {
  // Assign 2 virtual registers to 1 physical register in a single basicblock
  // and calculate the smallest spill/reload cost.
  std::vector<int> &bb;
  LocalTwoRegAlloc(std::vector<int> &bb) : bb(bb) {}
  int CalcMemAccessCost() {
    if (bb.empty())
      return 0;
    const int n = bb.size();
    std::vector<std::vector<int>> dp(2);
    // 0 for A on stack, B in register.
    dp[0].resize(n);
    // 1 for A in register, B on stack.
    dp[1].resize(n);
    switch (bb[0]) {
    case USE_A:
      dp[0][0] = 1;
      dp[1][0] = 0;
      break;
    case USE_B:
      dp[0][0] = 0;
      dp[1][0] = 1;
      break;
    }
    for (size_t i = 1; i < n; ++i) {
      int use = bb[i];
      dp[0][i] = std::min(dp[0][i - 1] + (use == USE_A ? 3 : 0),
                          dp[1][i - 1] + (use == USE_A ? 1 : 2));
      dp[1][i] = std::min(dp[0][i - 1] + (use == USE_B ? 1 : 2),
                          dp[1][i - 1] + (use == USE_B ? 3 : 0));
    }
    return std::min(dp[0][n - 1], dp[1][n - 1]);
  }
};

namespace {
TEST(LocalTwoRegAlloc, Case0) {
  std::vector<int> bb;
  for (size_t i = 0; i < 1024; ++i)
    bb.emplace_back(USE_A);
  LocalTwoRegAlloc ra(bb);
  EXPECT_TRUE(ra.CalcMemAccessCost() == 0);
}

TEST(LocalTwoRegAlloc, Case1) {
  std::vector<int> bb{USE_A, USE_B};
  LocalTwoRegAlloc ra(bb);
  EXPECT_TRUE(ra.CalcMemAccessCost() == 1);
}

TEST(LocalTwoRegAlloc, Case2) {
  std::vector<int> bb{USE_A, USE_A, USE_B, USE_A, USE_B};
  LocalTwoRegAlloc ra(bb);
  int cost = ra.CalcMemAccessCost();
  std::cout << cost << std::endl;
}

} // namespace
