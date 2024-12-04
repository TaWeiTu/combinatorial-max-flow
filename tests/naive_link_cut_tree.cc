#include "../src/naive_link_cut_tree.h"

#include <catch2/catch_all.hpp>

using namespace std;

namespace sum_size_increment {
struct V {
  int val = 0;
  int sz = 0;
  friend V operator+(V a, V b) {
    return V{.val = a.val + b.val, .sz = a.sz + b.sz};
  };
};
struct U {
  int x = 0;
  static V Apply(U a, V b) { return V{a.x * b.sz + b.val, b.sz}; }
  static U Compose(U a, U b) { return U(a.x + b.x); }
};
}  // namespace sum_size_increment

TEST_CASE("naive link cut tree", "[lct]") {
  using namespace sum_size_increment;
  NaiveLinkCutTree<U, V> lct(10);

  lct.Link(3, 2);
  lct.SetParentEdge(3, {10000, 1});
  lct.Link(4, 3);
  lct.SetParentEdge(4, {1000, 1});
  lct.Link(5, 3);
  lct.SetParentEdge(5, {100, 1});
  lct.Link(6, 5);
  lct.SetParentEdge(6, {10, 1});

  // 6 -> 5 -> 3 -> 2
  //      4 ---^

  REQUIRE(lct.GetRoot(3) == 2);
  REQUIRE(lct.GetRoot(4) == 2);
  REQUIRE(lct.GetRoot(5) == 2);
  REQUIRE(lct.GetRoot(6) == 2);
  REQUIRE(lct.GetRoot(2) == 2);
  REQUIRE(lct.GetRoot(1) == 1);

  REQUIRE(lct.QueryParentEdge(5).val == 100);
  REQUIRE(lct.QueryPathToRoot(6).val == 10110);
  REQUIRE(lct.QueryPathToRoot(6).sz == 3);
  REQUIRE(lct.QueryPathToRoot(1).sz == 0);

  lct.UpdatePathToRoot(6, {+1});

  SECTION("when query") {
    REQUIRE(lct.QueryPathToRoot(6).val == 10113);
    REQUIRE(lct.QueryParentEdge(5).val == 101);
    REQUIRE(lct.QueryPathToRoot(5).val == 10102);
  }

  SECTION("when cut") {
    lct.CutParent(5);

    // 6 -> 5
    // 4 -> 3 -> 2

    REQUIRE(lct.GetRoot(3) == 2);
    REQUIRE(lct.GetRoot(6) == 5);
    REQUIRE(lct.QueryPathToRoot(6).val == 11);
  }
}
