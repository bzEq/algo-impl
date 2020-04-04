#include "graph.h"

#include <iostream>

int main() {
  const unsigned n = 100000, m = 10000000;
  auto g = GenerateRandomGraph(n, m, true);
  std::vector<unsigned> INum, Num;
  SimpleIterativeDFS(*g, &INum);
  SimpleDFS(*g, &Num);
  for (unsigned i = 0; i < n; ++i) {
    assert(INum[i] == Num[i]);
  }
  return 0;
}
