#include <assert.h>
#include <iostream>
#include <memory>
#include <random>
#include <time.h>
#include <unordered_set>
#include <vector>

class Random {
public:
  Random() : distribution_(0, 1) {}

  explicit Random(int64_t seed) : generator_(seed), distribution_(0, 1) {}

  double Next() { return distribution_(generator_); }

  int64_t NextInt() { return int_distribution_(generator_); }

private:
  std::mt19937_64 generator_;
  std::uniform_real_distribution<double> distribution_;
  std::uniform_int_distribution<int64_t> int_distribution_;
};

struct Graph {
  std::vector<std::unordered_set<unsigned>> adjacents;
  Graph(size_t n) { adjacents.resize(n); }

  bool AddEdge(unsigned u, unsigned v, bool is_directed = false) {
    assert(u < adjacents.size() && v < adjacents.size() && "Invalid vertex id");
    auto res = adjacents[u].insert(v);
    if (!is_directed)
      adjacents[v].insert(u);
    return std::get<1>(res);
  }
};

inline std::unique_ptr<Graph> GenerateRandomGraph(size_t num_of_vertexes,
                                                  size_t num_of_edges,
                                                  bool is_directed = false) {
  auto g = std::make_unique<Graph>(num_of_vertexes);
  unsigned c = std::min(
      num_of_edges, is_directed ? num_of_vertexes * (num_of_vertexes - 1)
                                : num_of_vertexes * (num_of_vertexes - 1) / 2);
  Random rnd(time(nullptr));
  while (c) {
    unsigned u = static_cast<unsigned>(rnd.NextInt()) % num_of_vertexes,
             v = static_cast<unsigned>(rnd.NextInt()) % num_of_vertexes;
    if (u == v) {
      continue;
    }
    if (g->AddEdge(u, v, is_directed)) {
      --c;
    }
  }
  return std::move(g);
}
