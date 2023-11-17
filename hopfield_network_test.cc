#include "core.h"

#include <gtest/gtest.h>

template<bool UseMinus1 = false>
class HopfieldNetwork {
public:
  using Float = float;
  using Vertex = UndirectedGraph::Vertex;

  explicit HopfieldNetwork(UndirectedGraph &graph)
    : graph(graph) {}

  void SetLink(Vertex u, Vertex v, Float w) {
    graph.AddEdge(u, v);
    std::get<0>(weight.insert({{u, v}, 0}))->second = w;
    std::get<0>(weight.insert({{v, u}, 0}))->second = w;
  }

  bool Activate(unsigned u) {
    Float sum = 0;
    for (auto v : graph.pred(u))
      sum += weight[{v, u}] * states[v];
    if (sum >= threshold[u]) {
      states[u] = 1;
      return true;
    }
    states[u] = UseMinus1 ? -1 : 0;
    return false;
  }

  HopfieldNetwork &SetState(unsigned u, int state) {
    assert(state == (UseMinus1 ? -1 : 0) || state == 1);
    states[u] = state;
    return *this;
  }

  int GetState(unsigned u) { return states[u]; }

  HopfieldNetwork &SetThreshold(unsigned u, Float t) {
    threshold[u] = t;
    return *this;
  }

  void Iterate(size_t max_num_iteration = 1024, std::ostream *out = nullptr) {
    bool changed = true;
    Float prev = std::numeric_limits<Float>::max();
    for (size_t i = 0; i < max_num_iteration; ++i) {
      changed = false;
      for (unsigned u = 0; u < graph.size(); ++u) {
        int old = states[u];
        Activate(u);
        if (old != states[u])
          changed = true;
      }
      Float current = energy();
      if (out != nullptr)
        *out << "Energy after iteration #" << i << ":\t" << std::setprecision(12)
             << current << std::endl;
      assert(current <= prev);
      prev = current;
      if (!changed)
        break;
    }
  }

  Float energy() {
    Float T = 0, W = 0;
    for (size_t i = 0; i < graph.size(); ++i) {
      T += threshold[i] * states[i];
      for (size_t j = 0; j < graph.size(); ++j) {
        if (i != j)
          W += states[i] * states[j] * weight[{i, j}];
      }
    }
    Float E = -0.5 * W + T;
    return E;
  }

  void Print(std::ostream &out) {
    for (unsigned u = 0; u < graph.size(); ++u) {
      out << states[u];
      if (u == graph.size() - 1)
        out << "\n";
      else
        out << " ";
    }
  }

private:
  UndirectedGraph &graph;
  std::map<std::tuple<unsigned, unsigned>, Float> weight;
  std::map<unsigned, int> states;
  std::map<unsigned, Float> threshold;
};

namespace {
TEST(HopfieldNetworkTest, Flip) {
  UndirectedGraph graph(4);
  HopfieldNetwork hn(graph);
  for (unsigned u = 0; u < graph.size(); ++u) {
    hn.SetThreshold(u, -1);
  }
  hn.SetLink(0, 1, -2);
  hn.SetLink(0, 2, -2);
  hn.SetLink(0, 3, -2);
  hn.SetLink(1, 2, -2);
  hn.SetLink(1, 3, -2);
  hn.SetLink(2, 3, -2);
  hn.Iterate();
  EXPECT_TRUE(hn.GetState(0) == 1);
  EXPECT_TRUE(hn.GetState(1) == 0);
  EXPECT_TRUE(hn.GetState(2) == 0);
  EXPECT_TRUE(hn.GetState(3) == 0);
  hn.Print(std::cout);
}

TEST(HopfieldNetwork, EightRooks) {
  UndirectedGraph graph(64);
  HopfieldNetwork hn(graph);
  for (unsigned u = 0; u < graph.size(); ++u) {
    hn.SetThreshold(u, -1);
  }
  for (unsigned i = 0; i < 8; ++i) {
    unsigned base = 8 * i;
    for (unsigned j = 0; j < 8; ++j) {
      for (unsigned k = j + 1; k < 8; ++k) {
        hn.SetLink(base + j, base + k, -2);
      }
    }
  }
  for (unsigned i = 0; i < 8; ++i) {
    for (unsigned j = 0; j < 8; ++j) {
      for (unsigned k = j + 1; k < 8; ++k) {
        hn.SetLink(i + j * 8, i + k * 8, -2);
      }
    }
  }
  hn.Iterate();
  for (unsigned u = 0; u < graph.size();) {
    std::cout << hn.GetState(u++);
    if (u & 7)
      std::cout << "\t";
    else
      std::cout << "\n";
  }
}

TEST(HopfieldNetwork, Random) {
  Random rnd(std::time(nullptr));
  const uint64_t M = 1 << 10;
  UndirectedGraph g(M);
  HopfieldNetwork<true> hn(g);
  for (unsigned u = 0; u < g.size(); ++u) {
    hn.SetThreshold(u, rnd.Next());
    hn.SetState(u, rnd.NextInt() % 2 ? -1 : 1);
  }
  size_t num_edges = rnd.NextInt() % (M * M);
  for (size_t i = 0; i < num_edges; ++i) {
    unsigned u = rnd.NextInt() % g.size();
    unsigned v = rnd.NextInt() % g.size();
    hn.SetLink(u, v, rnd.Next() - 0.5);
  }
  hn.Iterate(1024, &std::cout);
  hn.Print(std::cout);
}

} // namespace
