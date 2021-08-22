#include "core.h"

#include <gtest/gtest.h>

struct HopfieldNetwork {
  using Float = float;
  Graph &graph;
  std::map<std::tuple<unsigned, unsigned>, Float> weight;
  std::map<unsigned, int> states;
  std::map<unsigned, Float> threshold;
  explicit HopfieldNetwork(Graph &graph) : graph(graph) {}
  void SetLink(unsigned u, unsigned v, Float w) {
    graph.AddEdge(u, v);
    std::get<0>(weight.insert({{u, v}, 0}))->second = w;
    std::get<0>(weight.insert({{v, u}, 0}))->second = w;
  }
  bool Activate(unsigned u) {
    Float sum = 0;
    for (auto v : graph.pred[u])
      sum += weight[{v, u}] * states[v];
    if (sum >= threshold[u]) {
      states[u] = 1;
      return true;
    }
    return false;
  }
  void SetState(unsigned u, int state) { states[u] = state; }
  void Iterate(size_t max_num_iteration = 1024) {
    bool changed = true;
    while (max_num_iteration-- && changed) {
      changed = false;
      for (unsigned u = 0; u < graph.size; ++u) {
        int old = states[u];
        Activate(u);
        if (old != states[u])
          changed = true;
      }
      // std::cout << energy() << std::endl;
    }
  }
  Float energy() {
    Float T = 0, W = 0;
    for (int i = 0; i < graph.size; ++i) {
      T += threshold[i] * states[i];
      for (int j = 0; j < graph.size; ++j) {
        W += states[i] * states[j] * weight[{i, j}];
      }
    }
    Float E = -0.5 * W + T;
    return E;
  }
};

namespace {
TEST(HopfieldNetworkTest, Flip) {
  Graph graph(4, false);
  HopfieldNetwork hn(graph);
  hn.SetLink(0, 1, -2);
  hn.SetLink(0, 2, -2);
  hn.SetLink(0, 3, -2);
  hn.SetLink(1, 2, -2);
  hn.SetLink(1, 3, -2);
  hn.SetLink(2, 3, -2);
  hn.Iterate();
  EXPECT_TRUE(hn.states[0] == 1);
  EXPECT_TRUE(hn.states[1] == 0);
  EXPECT_TRUE(hn.states[2] == 0);
  EXPECT_TRUE(hn.states[3] == 0);
}

TEST(HopfieldNetwork, EightRooks) {
  Graph graph(64, false);
  HopfieldNetwork hn(graph);
  for (unsigned u = 0; u < graph.size; ++u) {
    hn.threshold[u] = -1;
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
  for (unsigned u = 0; u < graph.size;) {
    std::cout << hn.states[u++];
    if (u & 7)
      std::cout << " ";
    else
      std::cout << std::endl;
  }
}

} // namespace
