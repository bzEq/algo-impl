// Copyright (c) 2020 Kai Luo <gluokai@gmail.com>. All rights reserved.

#include <assert.h>
#include <functional>
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
  std::vector<std::unordered_set<unsigned>> succ, pred;
  const bool is_directed;
  Graph(size_t n, bool is_directed = false) : is_directed(is_directed) {
    succ.resize(n);
    pred.resize(n);
  }

  bool AddEdge(unsigned u, unsigned v) {
    assert(u < succ.size() && v < succ.size() && "Invalid vertex id");
    auto res = succ[u].insert(v);
    pred[v].insert(u);
    if (!is_directed) {
      succ[v].insert(u);
      pred[u].insert(v);
    }
    return std::get<1>(res);
  }
};

inline std::unique_ptr<Graph> GenerateRandomGraph(size_t num_of_vertexes,
                                                  size_t num_of_edges) {
  auto g = std::make_unique<Graph>(num_of_vertexes);
  unsigned c =
      std::min(num_of_edges, g->is_directed
                                 ? num_of_vertexes * (num_of_vertexes - 1)
                                 : num_of_vertexes * (num_of_vertexes - 1) / 2);
  Random rnd(time(nullptr));
  while (c) {
    unsigned u = static_cast<unsigned>(rnd.NextInt()) % num_of_vertexes,
             v = static_cast<unsigned>(rnd.NextInt()) % num_of_vertexes;
    if (g->AddEdge(u, v)) {
      --c;
    }
  }
  return g;
}

inline std::unique_ptr<Graph>
GenerateRandomControlFlowGraph(size_t num_of_vertexes, size_t num_of_edges) {
  auto g = std::make_unique<Graph>(num_of_vertexes, true);
  unsigned c = std::min(num_of_edges, num_of_vertexes * num_of_vertexes);
  Random rnd(time(nullptr));
  while (c) {
    unsigned u = static_cast<unsigned>(rnd.NextInt()) % num_of_vertexes,
             v = static_cast<unsigned>(rnd.NextInt()) % num_of_vertexes;
    if (g->AddEdge(u, v)) {
      --c;
    }
  }
  return g;
}

inline void SimpleIterativeDFS(const Graph &graph, std::vector<unsigned> *dfo,
                               std::vector<unsigned> *rpo,
                               std::vector<unsigned> *dfs_tree_parent,
                               bool single_source = false) {
  if (graph.succ.empty())
    return;
  unsigned depth_first_order = 0, post_order = 0;
  const size_t size = graph.succ.size();
  const unsigned UNDEF = size;
  dfo->resize(size, UNDEF);
  rpo->resize(size, UNDEF);
  dfs_tree_parent->resize(size);
  std::iota(dfs_tree_parent->begin(), dfs_tree_parent->end(), 0);
  std::vector<bool> visited(size, false);
  struct State {
    unsigned u;
    decltype(graph.succ[0].begin()) next;
  };
  std::function<void(unsigned)> DFS = [&](unsigned o) {
    std::vector<State> dfs_stack;
    dfs_stack.emplace_back(State{o, graph.succ[o].begin()});
    while (!dfs_stack.empty()) {
      State &s = dfs_stack.back();
      if (!visited[s.u]) {
        visited[s.u] = true;
        (*dfo)[s.u] = depth_first_order++;
      }
      for (; s.next != graph.succ[s.u].end(); ++s.next) {
        if (!visited[*s.next])
          break;
      }
      if (s.next == graph.succ[s.u].end()) {
        assert(size >= post_order);
        (*rpo)[s.u] = size - post_order;
        ++post_order;
        dfs_stack.pop_back();
        continue;
      }
      unsigned v = *s.next;
      assert(v < size && !visited[v]);
      (*dfs_tree_parent)[v] = s.u;
      dfs_stack.emplace_back(State{v, graph.succ[v].begin()});
    }
  };
  if (single_source)
    DFS(0);
  else
    for (unsigned u = 0; u < size; ++u)
      if (!visited[u])
        DFS(u);
}

inline void SimpleDFS(const Graph &graph, std::vector<unsigned> *dfo,
                      std::vector<unsigned> *rpo) {
  if (graph.succ.empty())
    return;
  const size_t size = graph.succ.size();
  unsigned depth_first_order = 0, post_order = 0;
  dfo->resize(size);
  rpo->resize(size);
  std::vector<bool> visited(size, false);
  std::function<void(unsigned)> DFS = [&](unsigned u) {
    if (visited[u])
      return;
    visited[u] = true;
    (*dfo)[u] = depth_first_order++;
    for (auto v : graph.succ[u]) {
      DFS(v);
    }
    (*rpo)[u] = size - post_order;
    ++post_order;
  };
  for (unsigned u = 0; u < size; ++u)
    if (!visited[u])
      DFS(u);
}
