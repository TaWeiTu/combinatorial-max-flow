#include "../src/graph.h"

#include <catch2/catch_all.hpp>
#include <queue>
#include <random>

#include "random_graph_generator.h"

TEST_CASE("strongly connected components", "[scc][stress]") {
  std::mt19937_64 rng(Catch::rngSeed());

  auto g = GENERATE(take(200, random_graph()), take(100, random_graph(50)),
                    take(30, random_graph(200)));

  CAPTURE(g);

  SECTION("is correct") {
    auto scc = g.SCC();

    auto ReachableSet = [&](Vertex x) {
      std::vector<bool> reachable(g.n);
      std::queue<Vertex> que;
      que.push(x);
      reachable[x] = true;
      while (!que.empty()) {
        Vertex u = que.front();
        que.pop();
        for (Edge e : g.out_edges[u]) {
          if (!reachable[g.head[e]]) {
            reachable[g.head[e]] = true;
            que.push(g.head[e]);
          }
        }
      }
      return reachable;
    };

    std::vector<std::vector<bool>> closure(g.n);
    for (Vertex u : g.Vertices()) {
      closure[u] = ReachableSet(u);
    }
    for (Vertex x : g.Vertices()) {
      for (Vertex y : g.Vertices()) {
        REQUIRE((closure[x][y] && closure[y][x]) == (scc[x] == scc[y]));
      }
    }
    for (Edge e : g.Edges()) {
      REQUIRE(scc[g.tail[e]] <= scc[g.head[e]]);
    }
  }
}

TEST_CASE("respecting order", "[order]") {
  SECTION("path") {
    Graph g =
        Graph::FromEdgeList(5, {{0, 1, 0}, {1, 2, 0}, {2, 4, 0}, {4, 3, 0}});
    {
      auto order = RespectingOrder(g, {0, 0, 0, 0, 0});
      REQUIRE(order == std::vector{0, 1, 2, 4, 3});
    }
    {
      auto order = RespectingOrder(g, {0, 0, 1, 0, 0});
      REQUIRE(order == std::vector{0, 1, 2, 4, 3});
    }
    {
      auto order = RespectingOrder(g, {0, 0, 2, 1, 2});
      REQUIRE(order == std::vector{0, 1, 2, 4, 3});
    }
  }

  SECTION("fixed1") {
    Graph g = Graph::FromEdgeList(7, {{0, 1, 0},
                                      {1, 2, 0},
                                      {2, 0, 0},
                                      {2, 3, 0},
                                      {3, 4, 0},
                                      {4, 5, 0},
                                      {5, 3, 0},
                                      {5, 0, 0},
                                      {2, 6, 0}});
    auto order = RespectingOrder(g, {0, 0, 1, 0, 0, 0, 1, 2, 0});
    REQUIRE(order == std::vector{0, 1, 2, 3, 4, 5, 6});
  }

  SECTION("fixed2") {
    Graph g = Graph::FromEdgeList(11, {
                                          {1, 2, 0},
                                          {2, 3, 0},
                                          {5, 4, 0},
                                          {4, 0, 0},
                                          {6, 7, 0},
                                          {8, 9, 0},
                                          {9, 6, 0},
                                          {1, 10, 0},
                                          {3, 4, 0},
                                          {10, 6, 0},
                                          {5, 8, 0},
                                          {3, 1, 1},
                                          {7, 8, 1},
                                          {0, 5, 1},
                                          {9, 10, 2},
                                      });
    std::vector<WeightT> level;
    for (auto c : g.capacity) level.emplace_back(c);
    auto order = RespectingOrder(g, level);
    // this is the unique answer
    REQUIRE(order == std::vector{5, 0, 1, 2, 4, 3, 9, 10, 7, 8, 6});
  }
}
