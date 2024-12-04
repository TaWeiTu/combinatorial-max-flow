#include <catch2/catch_all.hpp>

#include "../src/graph.h"

using namespace std;

TEST_CASE("graph add vertices and edges", "[graph]") {
  Graph g;
  REQUIRE(g.n == 0);
  REQUIRE(g.m == 0);
  Vertex x = g.AddVertex();
  Vertex y = g.AddVertex();
  Vertex z = g.AddVertex();
  REQUIRE(g.n == 3);
  REQUIRE(g.m == 0);
  Edge xy = g.AddEdge(x, y);
  Edge yz = g.AddEdge(y, z, 5);
  REQUIRE(g.n == 3);
  REQUIRE(g.m == 2);
  using vv = vector<Vertex>;
  REQUIRE(g.out_edges[x] == vv{xy});
  REQUIRE(g.out_edges[y] == vv{yz});
  REQUIRE(g.out_edges[z] == vv{});
  REQUIRE(g.in_edges[x] == vv{});
  REQUIRE(g.in_edges[y] == vv{xy});
  REQUIRE(g.in_edges[z] == vv{yz});
  REQUIRE(g.EdgeInfo(xy) == tuple{x, y, 0});
  REQUIRE(g.EdgeInfo(yz) == tuple{y, z, 5});
}
TEST_CASE("graph from edge list", "[graph]") {
  Graph g = Graph::FromEdgeList(3, {{0, 1, 7}, {1, 2, 5}});
  REQUIRE(g.n == 3);
  REQUIRE(g.m == 2);
  REQUIRE(g.EdgeInfo(0) == tuple{0, 1, 7});
  REQUIRE(g.EdgeInfo(1) == tuple{1, 2, 5});
}
