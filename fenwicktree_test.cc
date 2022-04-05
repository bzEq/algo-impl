#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

#include <gtest/gtest.h>

struct FenwickTree {
  std::vector<int64_t> s, d;
  FenwickTree() = default;
  explicit FenwickTree(size_t len) : s(len, 0), d(len, 0) {}

  FenwickTree &Add(size_t p, int64_t delta) {
    for (size_t i = p + 1; i <= s.size(); i += i & (-i)) {
      s[i - 1] += delta;
      d[i - 1] += p * delta;
    }
    return *this;
  }

  FenwickTree &Add(size_t l, size_t r, int64_t delta) {
    return Add(l, delta).Add(r, -delta);
  }

  int64_t GetSum(size_t p) {
    assert(p >= 0);
    int64_t res = 0;
    for (size_t i = p + 1; i > 0; i -= i & (-i))
      res += (p + 1) * s[i - 1] - d[i - 1];
    return res;
  }

  int64_t GetValue(size_t i) {
    if (i == 0) return GetSum(0);
    return GetSum(i) - GetSum(i - 1);
  }
};

namespace {
TEST(FenwickTreeTest, Basic) {
  FenwickTree ft(16);
  ft.Add(0, 5, 1);
  ft.Add(5, 16, 2);
  ft.Add(4, 6, -1);
  EXPECT_TRUE(ft.GetValue(4) == 0);
  EXPECT_TRUE(ft.GetValue(5) == 1);
  EXPECT_TRUE(ft.GetValue(0) == 1);
}

TEST(FenwickTreeTest, Empty) {
  FenwickTree ft;
  EXPECT_TRUE(ft.GetValue(0) == 0);
}

} // namespace
