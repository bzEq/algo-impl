#include "graph.h"

#include <iostream>

int main() {
  const unsigned n = 1000, m = n*(n-1);
  auto g = GenerateRandomGraph(n, m, true);
  unsigned c = 0;
  for (unsigned i = 0; i < n; ++i) {
    c += g->adjacents[i].size();
  }
  assert(c == m);
  return 0;
}
