#include "../src/weighted_push_relabel.h"

#include <catch2/catch_all.hpp>

using namespace std;

TEST_CASE("weighted push relabel small fixed graph", "[wpr]") {
  auto g = Graph::FromEdgeList(3, {{1, 2, 3}, {1, 0, 9}, {0, 2, 4}});
  auto [flow_value, flow] =
      WeightedPushRelabel(g, {0, 100, -100}, {1, 3, 4}, 100);
  CAPTURE(flow_value, flow);
  REQUIRE(flow_value == 7);
  REQUIRE(flow == vector<CapacityT>{3, 4, 4});
}
