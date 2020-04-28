// Copyright (c) 2020 Kai Luo <gluokai@gmail.com>. All rights reserved.

#include <assert.h>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <set>
#include <time.h>
#include <vector>
#include <x86intrin.h>

static constexpr unsigned UNDEF = ~0U;

struct Graph {
  std::vector<std::set<unsigned>> succ, pred;
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

inline void IterativeDepthFirstVisit(
    const Graph &graph,
    const std::function<void(unsigned parent, unsigned u)> &pre_visit,
    const std::function<void(unsigned u, unsigned v)> &non_tree_visit,
    const std::function<void(unsigned u, unsigned parent)> &post_visit) {
  if (graph.succ.empty())
    return;
  const size_t size = graph.succ.size();
  std::vector<bool> visited(size, false);
  struct State {
    unsigned parent, u;
    decltype(graph.succ[0].begin()) next;
  };
  std::function<void(unsigned)> DFS = [&](unsigned o) {
    std::vector<State> dfs_stack;
    dfs_stack.emplace_back(State{UNDEF, o, graph.succ[o].begin()});
    while (!dfs_stack.empty()) {
      State &s = dfs_stack.back();
      if (!visited[s.u]) {
        visited[s.u] = true;
        pre_visit(s.parent, s.u);
      }
      for (; s.next != graph.succ[s.u].end(); ++s.next) {
        if (!visited[*s.next])
          break;
        else
          non_tree_visit(s.u, *s.next);
      }
      if (s.next == graph.succ[s.u].end()) {
        post_visit(s.u, s.parent);
        dfs_stack.pop_back();
        continue;
      }
      unsigned v = *s.next;
      ++s.next;
      assert(v < size && !visited[v]);
      dfs_stack.emplace_back(State{s.u, v, graph.succ[v].begin()});
    }
  };
  for (unsigned u = 0; u < size; ++u)
    if (!visited[u])
      DFS(u);
}

inline void SimpleIterativeDFS(const Graph &graph, std::vector<unsigned> *dfo,
                               std::vector<unsigned> *rpo,
                               std::vector<unsigned> *dfs_tree_parent) {
  unsigned depth_first_order = 0, depth_post_order = 0;
  const size_t size = graph.succ.size();
  dfo->resize(size, UNDEF);
  rpo->resize(size, UNDEF);
  dfs_tree_parent->resize(size, UNDEF);
  auto non_tree_visit = [](unsigned u, unsigned v) {};
  auto pre_visit = [&](unsigned parent, unsigned u) {
    (*dfs_tree_parent)[u] = parent;
    (*dfo)[u] = depth_first_order++;
  };
  auto post_visit = [&](unsigned u, unsigned parent) {
    (*rpo)[u] = (size - 1) - depth_post_order;
    ++depth_post_order;
  };
  IterativeDepthFirstVisit(graph, pre_visit, non_tree_visit, post_visit);
}

inline bool IsDAG(const Graph &graph) {
  if (graph.succ.empty())
    return false;
  bool answer = true;
  unsigned depth_first_order = 0, depth_post_order = 0;
  const size_t size = graph.succ.size();
  std::vector<unsigned> dfo(size, UNDEF), dpo(size, UNDEF);
  auto non_tree_visit = [&](unsigned u, unsigned v) {
    if (dfo[v] <= dfo[u] && dpo[v] == UNDEF)
      answer = false;
  };
  auto pre_visit = [&](unsigned parent, unsigned u) {
    dfo[u] = depth_first_order++;
  };
  auto post_visit = [&](unsigned u, unsigned parent) {
    dpo[u] = depth_post_order++;
  };
  IterativeDepthFirstVisit(graph, pre_visit, non_tree_visit, post_visit);
  return answer;
}

inline std::unique_ptr<Graph> DeriveDFSTree(const Graph &graph) {
  auto tree = std::unique_ptr<Graph>(new Graph(graph.succ.size()));
  auto pre_visit = [&](unsigned parent, unsigned u) {
    if (parent != UNDEF)
      tree->AddEdge(parent, u);
  };
  auto non_tree_visit = [&](unsigned u, unsigned v) {};
  auto post_visit = [&](unsigned u, unsigned parent) {};
  IterativeDepthFirstVisit(graph, pre_visit, non_tree_visit, post_visit);
  return tree;
}

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
GenerateRandomDirectedGraph(size_t num_of_vertexes, size_t num_of_edges) {
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

inline unsigned CountLeadingZeros(uint64_t x) { return _lzcnt_u64(x); }

inline unsigned Log2Ceil(uint64_t x) { return 64 - CountLeadingZeros(x - 1); }

inline unsigned Log2Floor(uint64_t x) { return 63 - CountLeadingZeros(x); }
