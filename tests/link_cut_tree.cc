#include "../src/link_cut_tree.h"

#include <catch2/catch_all.hpp>
#include <random>
#include <ranges>

#include "../src/naive_link_cut_tree.h"

using namespace std;

namespace sum_increment {
struct V {
  int val = 0;
  int sz = 0;
  friend V operator+(V a, V b) {
    return V{.val = a.val + b.val, .sz = a.sz + b.sz};
  };
};
struct U {
  int x = 0;
  U() = default;
  U(int x) : x(x) {}
  static V Apply(U a, V b) { return V{a.x * b.sz + b.val, b.sz}; }
  static U Compose(U a, U b) { return U(a.x + b.x); }
};
}  // namespace sum_increment

TEMPLATE_TEST_CASE("link cut tree", "[lct]",
                   (LinkCutTree<sum_increment::U, sum_increment::V>),
                   (NaiveLinkCutTree<sum_increment::U, sum_increment::V>)) {
  TestType lct(10);

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

namespace free_monoid {
struct V {
  vector<vector<int>> list;
  friend V operator+(V a, const V& b) {
    a.list.insert(end(a.list), begin(b.list), end(b.list));
    return a;
  }
};
struct U {
  vector<int> updates;
  static V Apply(const U& a, V b) {
    if (a.updates == vector<int>{106}) {
      for (auto bb : b.list)
        for (auto bbb : bb) REQUIRE(bbb != 106);
    }
    for (auto& v : b.list) v.insert(end(v), begin(a.updates), end(a.updates));
    return b;
  }
  static U Compose(U a, const U& b) {
    a.updates.insert(end(a.updates), begin(b.updates), end(b.updates));
    return a;
  }
};
}  // namespace free_monoid

TEST_CASE("link cut tree stress", "[lct][stress]") {
  using namespace free_monoid;
  int uid = 99;

  // try different values of n
  auto n = GENERATE(0, 1, 2, 3, 4, 5, 6, 7, 10, 15, 20, 25, 99, 101);
  mt19937 rng(Catch::rngSeed() + n);
  CAPTURE(n);
  NaiveLinkCutTree<U, V> naive(n);
  LinkCutTree<U, V> lct(n);
  if (n == 0) return;

  SECTION("random") {
    int max_len = 0;
    // perform 2000 random updates
    for ([[maybe_unused]] auto _ : views::iota(0, 2000)) {
      int type = rng() % 10;
      Vertex u = rng() % n, v = rng() % n;
      if (type < 5) {  // Link
        if (naive.GetRoot(u) != u || naive.GetRoot(v) == u) continue;
        V data{{{-(++uid)}}};
        naive.Link(u, v);
        lct.Link(u, v);
        naive.SetParentEdge(u, data);
        lct.SetParentEdge(u, data);
      } else if (type == 5) {  // CutParent
        if (naive.GetRoot(u) == u) continue;
        naive.CutParent(u);
        lct.CutParent(u);
      } else if (type == 6) {  // GetRoot
        REQUIRE(naive.GetRoot(u) == lct.GetRoot(u));
      } else if (type == 7) {  // QueryParentEdge
        REQUIRE(naive.QueryParentEdge(u).list == lct.QueryParentEdge(u).list);
      } else if (type == 8) {  // QueryPathToRoot
        max_len = max<int>(max_len, ssize(naive.QueryPathToRoot(u).list));
        REQUIRE(naive.QueryPathToRoot(u).list == lct.QueryPathToRoot(u).list);
      } else if (type == 9) {  // UpdatePathToRoot
        U update{{++uid}};
        naive.UpdatePathToRoot(u, update);
        lct.UpdatePathToRoot(u, update);
      }
    }
  }
  SECTION("path") {
    for (int i : views::iota(1, n)) {
      V data{{{-(++uid)}}};
      naive.Link(i, i - 1);
      lct.Link(i, i - 1);
      naive.SetParentEdge(i, data);
      lct.SetParentEdge(i, data);
    }
    for ([[maybe_unused]] auto _ : views::iota(1, 100)) {
      Vertex u = rng() % n;
      if (rng() % 2) {
        REQUIRE(naive.QueryPathToRoot(u).list == lct.QueryPathToRoot(u).list);
      } else {
        U update{{++uid}};
        naive.UpdatePathToRoot(u, update);
        lct.UpdatePathToRoot(u, update);
      }
    }
  }
}
