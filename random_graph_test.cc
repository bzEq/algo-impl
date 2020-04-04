#include "graph.h"

#include <iostream>

int main() {
  const unsigned n = 10000, m = 1000000;
  auto g = GenerateRandomControlFlowGraph(n, m);
  std::vector<unsigned> INum, Num;
  SimpleIterativeDFS(*g, &INum);
  SimpleDFS(*g, &Num);
  for (unsigned i = 0; i < n; ++i) {
    assert(INum[i] == Num[i]);
  }
  return 0;
}
