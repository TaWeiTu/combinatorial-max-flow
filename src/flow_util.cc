#include "flow_util.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <queue>
#include <vector>

#include "graph.h"
#include "link_cut_tree.h"

namespace {

struct V {
  CapacityT flow_routed, flow_to_route;
  Vertex min_vertex;
  V() : flow_routed(std::numeric_limits<CapacityT>::max()) {}
  V(CapacityT x, CapacityT y, Vertex u)
      : flow_routed(x), flow_to_route(y), min_vertex(u) {}

  V operator+(const V &rhs) const {
    return flow_routed < rhs.flow_routed ? *this : rhs;
  }
};

struct U {
  CapacityT update_flow_routed, update_flow_to_route;
  U() = default;
  U(CapacityT x, CapacityT y)
      : update_flow_routed(x), update_flow_to_route(y) {}

  static U Compose(U lhs, U rhs) {
    return U(lhs.update_flow_routed + rhs.update_flow_routed,
             lhs.update_flow_to_route + rhs.update_flow_to_route);
  }
  static V Apply(U upd, V val) {
    return V(val.flow_routed + upd.update_flow_routed,
             val.flow_to_route + upd.update_flow_to_route, val.min_vertex);
  }
};

}  // namespace

FlowDecomposition::FlowDecomposition(const Graph &g,
                                     const std::vector<CapacityT> &flow)
    : g_(g), flow_(flow) {
  RouteInternal({}, true);
}

std::vector<CapacityT> FlowDecomposition::RouteInternal(
    std::map<std::pair<Vertex, Vertex>, CapacityT> subdemand, bool init) {
  assert(!init || subdemand.empty());
  assert(static_cast<int>(flow_.size()) == g_.m);
  if (!init) {
    for (auto [k, v] : subdemand) {
      assert(multi_commodity_demand_.find(k) != multi_commodity_demand_.end() &&
             multi_commodity_demand_[k] >= v);
    }
  }

  // Find sources and sinks in this flow.
  std::vector<CapacityT> net_flow(g_.n);
  for (Edge e : g_.Edges()) {
    net_flow[g_.tail[e]] += flow_[e];
    net_flow[g_.head[e]] -= flow_[e];
  }
  CapacityT total_flow = 0;
  std::vector<Vertex> sinks;
  for (Vertex v = 0; v < g_.n; ++v) {
    if (net_flow[v] > 0) {
      total_flow += net_flow[v];
    } else if (net_flow[v] < 0) {
      sinks.push_back(v);
    }
  }

  LinkCutTree<U, V> link_cut_tree(g_.n);
  std::vector<Edge> parent(g_.n, -1);
  auto in_edges = g_.in_edges;

  auto GetNewParent = [&](Vertex v) {
    while (!in_edges[v].empty() && flow_[in_edges[v].back()] == 0) {
      in_edges[v].pop_back();
    }
    if (in_edges[v].empty()) {
      return -1;
    }
    Edge e = in_edges[v].back();
    in_edges[v].pop_back();
    return e;
  };

  CapacityT total_routed = 0;
  std::vector<CapacityT> subflow(g_.m);
  std::queue<Vertex> no_parent;
  for (Vertex v : g_.Vertices()) {
    no_parent.push(v);
  }

  auto InvalidateParent = [&](Vertex u) {
    assert(parent[u] != -1);
    subflow[parent[u]] = link_cut_tree.QueryParentEdge(u).flow_to_route;
    link_cut_tree.CutParent(u);
    parent[u] = -1;
    no_parent.push(u);
  };

  auto SendFlowToRoot = [&](Vertex u, CapacityT f) {
    link_cut_tree.UpdatePathToRoot(u, U(-f, 0));
    while (parent[u] != -1) {
      auto val = link_cut_tree.QueryPathToRoot(u);
      if (val.flow_routed > 0) {
        break;
      }
      InvalidateParent(val.min_vertex);
    }
  };

  while (!sinks.empty()) {
    if (!no_parent.empty()) {
      Vertex u = no_parent.front();
      no_parent.pop();
      while (true) {
        parent[u] = GetNewParent(u);
        if (parent[u] == -1) {
          break;
        }
        // Check if a cycle is formed
        Vertex v = g_.tail[parent[u]];
        // Special case: self-loop
        if (u == v) {
          continue;
        }
        CapacityT remaining_flow = flow_[parent[u]];
        if (link_cut_tree.GetRoot(v) == u) {
          V cycle = link_cut_tree.QueryPathToRoot(v);
          assert(cycle.flow_routed > 0);
          SendFlowToRoot(v, std::min(remaining_flow, cycle.flow_routed));
          remaining_flow -= std::min(remaining_flow, cycle.flow_routed);
        }
        if (remaining_flow > 0) {
          link_cut_tree.Link(u, v, V(remaining_flow, 0, u));
          break;
        }
      }
    } else {
      Vertex t = sinks.back();
      sinks.pop_back();
      Vertex s = link_cut_tree.GetRoot(t);
      assert(net_flow[s] > 0 && "The sink must now be connected to a source");
      V value = link_cut_tree.QueryPathToRoot(t);
      assert(value.flow_routed > 0);
      CapacityT f = std::min({value.flow_routed, net_flow[s], -net_flow[t]});
      total_routed += f;
      if (init) {
        multi_commodity_demand_[std::make_pair(s, t)] += f;
      }
      CapacityT to_route = std::min(subdemand[std::make_pair(s, t)], f);
      subdemand[std::make_pair(s, t)] -= to_route;
      link_cut_tree.UpdatePathToRoot(t, U(0, to_route));
      SendFlowToRoot(t, f);
      net_flow[t] += f;
      net_flow[s] -= f;
      assert(net_flow[t] <= 0);
      assert(net_flow[s] >= 0);
      if (net_flow[t] < 0) {
        sinks.push_back(t);
      }
    }
  }
  assert(total_flow == total_routed);
  assert(std::ranges::all_of(net_flow, [](auto v) { return v == 0; }));

  for (auto [k, v] : subdemand) {
    assert(v == 0);
  }
  for (Vertex u : g_.Vertices()) {
    if (parent[u] != -1) {
      subflow[parent[u]] = link_cut_tree.QueryParentEdge(u).flow_to_route;
    }
  }
  return subflow;
}

std::vector<CapacityT> FlowDecomposition::Route(
    const std::map<std::pair<Vertex, Vertex>, CapacityT> &subdemand) {
  return RouteInternal(subdemand);
}
