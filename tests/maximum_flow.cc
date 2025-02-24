#include "../src/maximum_flow.h"

#include <catch2/catch_all.hpp>
#include <random>

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

    REQUIRE(flow_value == 7);
    REQUIRE(flow == vector<CapacityT>{7, 0, 7});
  }
}
