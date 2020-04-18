#include "core.h"

#include <gtest/gtest.h>

struct LCA {
  const Graph &graph;
  const size_t size;
  const unsigned UNDEF;
  std::vector<unsigned> tree_parent, label, height;
  struct Range {
    unsigned begin, end;
  };
  std::vector<Range> dfo;
  std::vector<std::vector<unsigned>> ancestor;

  LCA(const Graph &graph)
      : graph(graph), size(graph.succ.size()), UNDEF(size),
        tree_parent(size, UNDEF), label(size, UNDEF), height(size, UNDEF),
        dfo(size), ancestor(size) {
    unsigned depth_first_order = 0, current_height = 0;
    auto pre_visit = [&](unsigned parent, unsigned u) {
      height[u] = current_height;
      ++current_height;
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
      assert(current_height > 0);
      --current_height;
    };
    IterativeDepthDirstVisit(graph, pre_visit, non_tree_visit, post_visit);
    const unsigned MAX_ORDER = Log2Ceil(size);
    for (unsigned u = 0; u < size; ++u) {
      if (Covers(label[0], u) && height[u] != 0 && height[u] != UNDEF)
        ancestor[u].resize(Log2Ceil(height[u]) + 1, label[0]);
    }
    for (unsigned o = 0; o < MAX_ORDER; ++o) {
      for (unsigned dfn = 0; dfn < size; ++dfn) {
        unsigned u = label[dfn];
        if (!Covers(label[0], u))
          continue;
        if (o == 0) {
          if (tree_parent[u] != UNDEF && 0 < ancestor[u].size()) {
            ancestor[u][0] = tree_parent[u];
          }
        } else {
          if (o < ancestor[u].size()) {
            unsigned v = ancestor[u][o - 1];
            if (o - 1 < ancestor[v].size())
              ancestor[u][o] = ancestor[v][o - 1];
          }
        }
      }
    }
  }

  bool Covers(unsigned u, unsigned v) {
    return dfo[u].begin <= dfo[v].begin && dfo[v].begin <= dfo[u].end;
  }

  unsigned GetAncestor(unsigned u, const unsigned distance) {
    if (distance == 0)
      return u;
    unsigned o = Log2Floor(distance);
    assert(o < ancestor[u].size());
    return GetAncestor(ancestor[u][o], distance - (1 << o));
  }

  unsigned GetLCA(unsigned u, unsigned v) {
    assert(u < size && v < size);
    if (!Covers(label[0], u) || !Covers(label[0], v))
      return UNDEF;
    assert(height[u] != UNDEF && height[v] != UNDEF);
    if (height[u] < height[v]) {
      std::swap(u, v);
    }
    if (v == label[0])
      return v;
    u = GetAncestor(u, height[u] - height[v]);
    if (u == v)
      return u;
    assert(height[u] == height[v]);
    assert(ancestor[u].size() == ancestor[v].size());
    const unsigned o = ancestor[u].size();
    for (int i = o - 1; i >= 0; --i) {
      if (ancestor[u][i] != ancestor[v][i]) {
        u = ancestor[u][i];
        v = ancestor[v][i];
      }
    }
    return ancestor[u][0];
  }
};

TEST(LCATest, Log2Ceil) {
  EXPECT_TRUE(Log2Ceil(2) == 1);
  EXPECT_TRUE(Log2Ceil(4) == 2);
  EXPECT_TRUE(Log2Ceil(6) == 3);
  EXPECT_TRUE(Log2Ceil(12) == 4);
  EXPECT_TRUE(Log2Ceil(16) == 4);
}

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
  EXPECT_TRUE(lca.height[4] == 2);
  EXPECT_TRUE(lca.ancestor[4].size() == 2);
  EXPECT_TRUE(lca.ancestor[4][0] == 1);
  EXPECT_TRUE(lca.ancestor[4][1] == 0);
  EXPECT_TRUE(lca.tree_parent[4] == 1);
  EXPECT_TRUE(lca.GetLCA(0, 0) == 0);
  EXPECT_TRUE(lca.GetLCA(1, 2) == 1);
  EXPECT_TRUE(lca.GetLCA(3, 4) == 1);
  EXPECT_TRUE(lca.label[lca.dfo[1].end] == 4);
  EXPECT_TRUE(lca.GetLCA(1, 5) == 0);
  EXPECT_TRUE(lca.height[3] == 3);
  EXPECT_TRUE(lca.ancestor[3].size() == 3);
  EXPECT_TRUE(lca.ancestor[3][0] == 2);
  EXPECT_TRUE(lca.ancestor[3][1] == 1);
  EXPECT_TRUE(lca.ancestor[3][2] == 0);
  EXPECT_TRUE(lca.GetLCA(3, 5) == 0);
  EXPECT_TRUE(lca.GetLCA(4, 5) == 0);
  EXPECT_TRUE(lca.GetLCA(6, 7) == 5);
  EXPECT_TRUE(lca.ancestor[3].back() == 0);
  EXPECT_TRUE(lca.ancestor[6].back() == 0);
  EXPECT_TRUE(lca.GetLCA(3, 6) == 0);
}

TEST(LCATest, Random) {
  const unsigned n = 500000, m = 500000;
  Random rnd(std::time(nullptr));
  auto g = GenerateRandomGraph(n, m);
  LCA lca(*g);
  for (unsigned i = 0; i < m; ++i) {
    unsigned u = rnd.NextInt() % n, v = rnd.NextInt() % n;
    lca.GetLCA(u, v);
  }
}
