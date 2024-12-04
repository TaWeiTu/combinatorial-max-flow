#pragma once

#include <algorithm>
#include <cassert>
#include <queue>
#include <vector>

#include "graph.h"
#include "link_cut_tree.h"

template <typename U, typename V>
class NaiveLinkCutTree : public LinkCutTree<U, V> {
 public:
  void Link(Edge e) override {
    Vertex u = edges_[e].first;
    Vertex v = edges_[e].second;
    tree_[u].push_back(e);
    tree_[v].push_back(e);
  };

  void Cut(Edge e) override {
    for (Vertex z : {edges_[e].first, edges_[e].second}) {
      auto iter = std::find(tree_[z].begin(), tree_[z].end(), e);
      assert(iter != tree_[z].end());
      tree_[z].erase(iter);
    }
  }

  void Update(Vertex u, Vertex v, U update) override {
    Traverse(u, v, [update, this](Edge e) {
      values_[e] = U::Apply(update, values_[e]);
    });
  }

  V Query(Vertex u, Vertex v) override {
    V result = V();
    Traverse(u, v, [&result, this](Edge e) { result = result + values_[e]; });
    return result;
  }

  NaiveLinkCutTree(int n, const std::vector<std::pair<Vertex, Vertex>> &edges)
      : tree_(n), edges_(edges), values_(edges.size()) {}

 private:
  std::vector<V> values_;
  std::vector<std::vector<Edge>> tree_;
  std::vector<std::pair<Vertex, Vertex>> edges_;

  template <typename F>
  void Traverse(Vertex u, Vertex v, F &&f) {
    std::queue<Vertex> que;
    que.push(u);
    std::vector<Edge> from(tree_.size(), -1);
    while (!que.empty()) {
      Vertex x = que.front();
      que.pop();
      for (Edge e : tree_[x]) {
        Vertex to = edges_[e].first ^ edges_[e].second ^ x;
        if (from[to] == -1) {
          from[to] = e;
          que.push(to);
        }
      }
    }
    assert(from[v] != -1);
    while (v != u) {
      f(from[v]);
      v = edges_[from[v]].first ^ edges_[from[v]].second ^ v;
    }
  }
};
