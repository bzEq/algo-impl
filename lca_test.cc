#include "core.h"

#include <gtest/gtest.h>

struct LCA {
  const Graph &graph;
  const size_t size;
  const unsigned UNDEF;
  std::vector<unsigned> tree_parent, label;
  struct Range {
    unsigned begin, end;
  };
  std::vector<Range> dfo;

  LCA(const Graph &graph)
      : graph(graph), size(graph.succ.size()), UNDEF(size),
        tree_parent(size, UNDEF), label(size, UNDEF), dfo(size) {
    unsigned depth_first_order = 0;
    auto pre_visit = [&](unsigned parent, unsigned u) {
      dfo[u].begin = depth_first_order;
      dfo[u].end = depth_first_order;
      label[depth_first_order] = u;
      ++depth_first_order;
      tree_parent[u] = parent;
    };
    auto non_tree_visit = [&](unsigned u, unsigned v) {};
    auto post_visit = [&](unsigned u, unsigned parent) {
      if (parent != UNDEF) {
        dfo[parent].end = std::max(dfo[parent].end, dfo[u].end);
      }
    };
    IterativeDepthDirstVisit(graph, pre_visit, non_tree_visit, post_visit);
  }

  unsigned GetLCA(unsigned u, unsigned v) {
    assert(u < size && v < size);
    if (dfo[u].begin > dfo[v].begin)
      std::swap(u, v);
    if (dfo[u].begin <= dfo[v].begin && dfo[v].begin <= dfo[u].end)
      return u;
    assert(dfo[v].begin > dfo[u].end);
    assert(dfo[u].end + 1 < size);
    unsigned w = label[dfo[u].end + 1];
    assert(tree_parent[w] != UNDEF);
    if (v == w)
      return tree_parent[w];
    return GetLCA(tree_parent[w], v);
  }
};

TEST(LCATest, Simple) {
  Graph g(8);
  g.AddEdge(0, 1);
  g.AddEdge(0, 5);
  g.AddEdge(1, 2);
  g.AddEdge(2, 3);
  g.AddEdge(1, 4);
  g.AddEdge(5, 6);
  g.AddEdge(5, 7);
  LCA lca(g);
  EXPECT_TRUE(lca.tree_parent[4] == 1);
  EXPECT_TRUE(lca.GetLCA(0, 0) == 0);
  EXPECT_TRUE(lca.GetLCA(1, 2) == 1);
  EXPECT_TRUE(lca.GetLCA(3, 4) == 1);
  EXPECT_TRUE(lca.label[lca.dfo[1].end] == 4);
  EXPECT_TRUE(lca.GetLCA(1, 5) == 0);
  EXPECT_TRUE(lca.GetLCA(3, 5) == 0);
  EXPECT_TRUE(lca.GetLCA(4, 5) == 0);
  EXPECT_TRUE(lca.GetLCA(6, 7) == 5);
  EXPECT_TRUE(lca.GetLCA(3, 6) == 0);
}
