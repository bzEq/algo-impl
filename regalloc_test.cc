#include "core.h"

#include <gtest/gtest.h>
#include <unordered_map>

class RegAlloc {
  std::unordered_map<unsigned, LiveInterval> LIS;
  std::unordered_map<unsigned, int> RegMap;
  unsigned Val = 0;

public:
  unsigned TrackLiveness(size_t B, size_t E) {
    unsigned V = Val++;
    LIS.insert({V, {B, E}});
    return V;
  }

  size_t CalcActiveRegs(size_t B, size_t E) const {
    size_t ans = 0;
    LiveInterval LI{B, E};
    for (auto &I : LIS) {
      if (LI.Interfere(I.second))
        ++ans;
    }
    return ans;
  }

  size_t CalcRegPressure(unsigned Val) const {
    auto I = LIS.find(Val);
    if (I == LIS.end())
      return 0;
    return CalcActiveRegs(I->second.B, I->second.E);
  }
};

TEST(RegAllocTest, ActiveRegs) {
  RegAlloc RA;
  RA.TrackLiveness(0, 8);
  EXPECT_EQ(RA.CalcActiveRegs(0, 4), 1);
  EXPECT_EQ(RA.CalcActiveRegs(8, 12), 0);
  RA.TrackLiveness(4, 8);
  EXPECT_EQ(RA.CalcActiveRegs(0, 4), 1);
  EXPECT_EQ(RA.CalcActiveRegs(6, 8), 2);
}

TEST(RegAllocTest, RegPressure) {
  RegAlloc RA;
  RA.TrackLiveness(0, 3);
  RA.TrackLiveness(4, 8);
  RA.TrackLiveness(6, 10);
  RA.TrackLiveness(2, 14);
  EXPECT_EQ(RA.CalcRegPressure(3), 4);
  EXPECT_EQ(RA.CalcRegPressure(2), 3);
  EXPECT_EQ(RA.CalcRegPressure(1), 3);
  EXPECT_EQ(RA.CalcRegPressure(0), 2);
}
