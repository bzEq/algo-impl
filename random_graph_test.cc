#include "graph.h"

#include <iostream>

int main() {
  const unsigned n = 100, m = 1000;
  auto g = GenerateRandomGraph(n, m, true);
  for (unsigned i = 0; i < n; ++i) {
    std::cout << g->adjacents[i].size() << "\n";
  }
  return 0;
}
