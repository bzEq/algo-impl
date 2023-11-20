// Copyright (c) 2020 Kai Luo <gluokai@gmail.com>. All rights reserved.

#pragma once

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include <functional>
#include <iostream>
#include <memory>
#include <queue>
#include <random>
#include <ranges>
#include <set>
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

inline unsigned CountLeadingZeros(uint64_t x) { return __builtin_clzl(x); }

inline unsigned Log2Ceil(uint64_t x) { return 64 - CountLeadingZeros(x - 1); }

inline unsigned Log2Floor(uint64_t x) { return 63 - CountLeadingZeros(x); }

static constexpr unsigned UNDEF = ~0U;

template <bool IsDirected = true> class SimpleGraph {
public:
  using Vertex = uint32_t;
  static constexpr Vertex UNDEF = ~(Vertex(0));

private:
  Vertex entry_;
  std::vector<std::unordered_set<Vertex>> succ_, pred_;

public:
  using SuccConstIterator = decltype(succ_[0].cbegin());

  explicit SimpleGraph(size_t size) : entry_(UNDEF), succ_(size), pred_(size) {}

  SimpleGraph &AddEdge(Vertex u, Vertex v) {
    assert(u < succ_.size() && v < succ_.size() && "Vertex out of bound");
    succ_[u].insert(v);
    pred_[v].insert(u);
    if (!IsDirected) {
      succ_[v].insert(u);
      pred_[u].insert(v);
    }
    return *this;
  }

  SimpleGraph &RemoveEdge(Vertex u, Vertex v) {
    assert(u < succ_.size() && v < succ_.size() && "Vertex out of bound");
    succ_[u].erase(v);
    pred_[v].erase(u);
    if (!IsDirected) {
      succ_[v].erase(u);
      pred_[u].erase(v);
    }
    return *this;
  }

  SimpleGraph &SetEntry(Vertex u) {
    entry_ = u;
    return *this;
  }

  size_t size() const { return succ_.size(); }

  auto succ(Vertex u) const {
    assert(u < succ_.size() && "Vertex out of bound");
    return std::ranges::subrange(succ_[u]);
  }

  auto pred(Vertex v) const {
    assert(v < pred_.size() && "Vertex out of bound");
    return std::ranges::subrange(pred_[v]);
  }

  bool empty() const { return size() == 0; }

  bool HasEntry() const { return entry_ != UNDEF; }

  bool HasEdge(Vertex u, const Vertex v) const { return succ_[u].count(v); }

  constexpr bool directed() const { return IsDirected; }

  Vertex entry() const { return entry_; }

  auto all_vertex() const {
    std::vector<Vertex> vs(size());
    std::iota(vs.begin(), vs.end(), 0);
    return vs;
  }

  template <typename V> void Visit(V &visitor) const { visitor.Visit(*this); }

  using EdgeVisitor = std::function<void(Vertex, Vertex)>;

  class DepthFirstVisitor {
  public:
    EdgeVisitor tree_visit = nullptr, non_tree_visit = nullptr,
                post_visit = nullptr;

    void Visit(const SimpleGraph &graph) {
      visited_.clear();
      states_.clear();
      if (graph.HasEntry())
        return Visit(graph, graph.entry());
      for (Vertex u : graph.all_vertex())
        Visit(graph, u);
    }

  private:
    struct VisitState {
      Vertex parent, current;
      SuccConstIterator next, end;
    };
    std::unordered_set<Vertex> visited_;
    std::vector<VisitState> states_;

    void Visit(const SimpleGraph &graph, Vertex start) {
      if (visited_.count(start))
        return;
      auto succ_range = graph.succ(start);
      states_.emplace_back(
          VisitState{UNDEF, start, succ_range.begin(), succ_range.end()});
      while (!states_.empty()) {
        VisitState &s = states_.back();
        if (!visited_.count(s.current)) {
          if (tree_visit)
            tree_visit(s.parent, s.current);
          visited_.insert(s.current);
        }
        for (; s.next != s.end; ++s.next) {
          Vertex succ = *s.next;
          assert(succ < graph.size() && "Vertex out of bound");
          if (visited_.count(succ)) {
            if (non_tree_visit)
              non_tree_visit(s.current, succ);
          } else
            break;
        }
        if (s.next == s.end) {
          if (post_visit)
            post_visit(s.current, s.parent);
          states_.pop_back();
          continue;
        }
        Vertex succ = *s.next;
        ++s.next;
        assert(succ < graph.size() && "Vertex out of bound");
        assert(!visited_.count(succ) && "Should not have visted");
        states_.emplace_back(VisitState{
            s.current, succ, graph.succ(succ).begin(), graph.succ(succ).end()});
      }
    }
  };

  class BreadthFirstVisitor {
  public:
    EdgeVisitor tree_visit = nullptr;

    void Visit(const SimpleGraph &graph) {
      visited_.clear();
      if (graph.HasEntry())
        return Visit(graph, graph.entry());
      for (Vertex u : graph.all_vertex())
        Visit(graph, u);
    }

  private:
    std::unordered_set<Vertex> visited_;
    std::queue<std::pair<Vertex, Vertex>> worklist_;

    void Visit(const SimpleGraph &graph, Vertex start) {
      if (visited_.count(start))
        return;
      worklist_.push({UNDEF, start});
      while (!worklist_.empty()) {
        auto u = worklist_.front();
        worklist_.pop();
        if (visited_.count(u.second))
          continue;
        if (tree_visit)
          tree_visit(u.first, u.second);
        visited_.insert(u.second);
        for (Vertex v : graph.succ(u.second)) {
          if (!visited_.count(v))
            worklist_.push({u.second, v});
        }
      }
    }
  };

  static void RandomGraph(SimpleGraph &graph, size_t edges_to_add) {
    size_t max_edges = graph.size() * (graph.size() - 1);
    if (!graph.directed())
      max_edges /= 2;
    edges_to_add = std::min(max_edges, edges_to_add);
    Random rnd(time(nullptr));
    while (edges_to_add) {
      Vertex u = Vertex(rnd.NextInt() % graph.size());
      Vertex v = Vertex(rnd.NextInt() % graph.size());
      if (!graph.HasEdge(u, v)) {
        graph.AddEdge(u, v);
        --edges_to_add;
      }
    }
  }

  static bool IsDAG(const SimpleGraph &graph) {
    if (!graph.directed())
      return false;
    bool answer = true;
    uint32_t dfo = 0, dpo = 0;
    const size_t size = graph.size();
    std::vector<uint32_t> DFO(size, ~0U), DPO(size, ~0U);
    DepthFirstVisitor DFV;
    DFV.non_tree_visit = [&](Vertex u, Vertex v) {
      if (DFO[v] <= DFO[u] && DPO[v] == ~0U)
        answer = false;
    };
    DFV.tree_visit = [&](Vertex parent, Vertex u) { DFO[u] = dfo++; };
    DFV.post_visit = [&](Vertex u, Vertex parent) { DPO[u] = dpo++; };
    graph.Visit(DFV);
    return answer;
  }

  static void DeriveDFSTree(SimpleGraph &graph, SimpleGraph<false> &tree) {
    DepthFirstVisitor DFV;
    DFV.tree_visit = [&](Vertex parent, Vertex u) {
      if (parent != UNDEF)
        tree.AddEdge(parent, u);
    };
    graph.Visit(DFV);
  }

  static void DeriveBFSTree(SimpleGraph &graph, SimpleGraph<false> &tree) {
    BreadthFirstVisitor BFV;
    BFV.tree_visit = [&](Vertex parent, Vertex u) {
      if (parent != UNDEF)
        tree.AddEdge(parent, u);
    };
    graph.Visit(BFV);
  }

  static void DepthFirstLabel(const SimpleGraph &graph,
                              std::vector<Vertex> *tree_parent,
                              std::vector<unsigned> *RPO,
                              std::vector<unsigned> *DFO,
                              std::vector<unsigned> *DPO) {
    unsigned dfo = 0, dpo = 0;
    const size_t size = graph.size();
    if (tree_parent)
      tree_parent->resize(size, UNDEF);
    if (RPO)
      RPO->resize(size, UNDEF);
    if (DFO)
      DFO->resize(size, UNDEF);
    if (DPO)
      DPO->resize(size, UNDEF);
    DepthFirstVisitor DFV;
    DFV.tree_visit = [&](Vertex parent, Vertex u) {
      if (tree_parent)
        tree_parent->at(u) = parent;
      if (DFO)
        DFO->at(u) = dfo++;
    };
    DFV.post_visit = [&](Vertex u, Vertex parent) {
      if (RPO)
        RPO->at(u) = size - 1 - dpo;
      if (DPO)
        DPO->at(u) = dpo++;
    };
    graph.Visit(DFV);
  }
};

using DirectedGraph = SimpleGraph<true>;

using UndirectedGraph = SimpleGraph<false>;

template<bool IsDirected = true>
SimpleGraph<IsDirected> CreateRandomSimpleGraph(size_t num_vertex, size_t num_edges) {
  SimpleGraph<IsDirected> G(num_vertex);
  SimpleGraph<IsDirected>::RandomGraph(G, num_edges);
  return G;
}
