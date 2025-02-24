#include "../src/maximum_flow.h"

#include <catch2/catch_all.hpp>
#include <random>

#include "correct_flow.h"
#include "random_graph_generator.h"
using namespace std;

TEST_CASE("maximum flow on small fixed graphs", "[mf]") {
  Vertex source = 0, sink = 1;

  SECTION("no edges") {
    auto g = Graph::FromEdgeList(2, {});

    auto [flow_value, flow] = MaximumFlow(g, source, sink);

    REQUIRE(flow_value == 0);
    REQUIRE(flow == vector<CapacityT>{});
  }

  SECTION("single edge") {
    auto g = Graph::FromEdgeList(2, {{0, 1, 7}});

    auto [flow_value, flow] = MaximumFlow(g, source, sink);

    REQUIRE(flow_value == 7);
    REQUIRE(flow == vector<CapacityT>{7});
  }

  SECTION("two edges") {
    auto g = Graph::FromEdgeList(3, {{0, 2, 10}, {2, 1, 7}});

    auto [flow_value, flow] = MaximumFlow(g, source, sink);

    REQUIRE(flow_value == 7);
    REQUIRE(flow == vector<CapacityT>{7, 7});
  }

  SECTION("triangle cycle") {
    auto g = Graph::FromEdgeList(3, {{0, 2, 10}, {1, 0, 9}, {2, 1, 7}});

    auto [flow_value, flow] = MaximumFlow(g, source, sink);

    REQUIRE(flow == vector<CapacityT>{7, 0, 7});
    REQUIRE(flow_value == 7);
  }

  SECTION("failed previously 1") {
    auto g = Graph::FromEdgeList(3, {{0, 2, 13},
                                     {2, 1, 77},
                                     {2, 0, 57},
                                     {2, 0, 65},
                                     {1, 2, 33},
                                     {0, 2, 82}});

    auto [flow_value, flow] = MaximumFlow(g, 1, 0);

    REQUIRE(flow_value == 33);
  }

  SECTION("failed previously 2") {
    auto g = Graph::FromEdgeList(4, {{2, 0, 80}, {3, 2, 24}, {1, 3, 91}});

    auto [flow_value, flow] = MaximumFlow(g, 1, 2);

    REQUIRE(flow_value == 24);
  }
}
