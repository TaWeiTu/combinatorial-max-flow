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
  CapacityT flow_remaining, subflow;
  Vertex min_vertex;
  V() : flow_remaining(std::numeric_limits<CapacityT>::max()) {}
  V(CapacityT x, CapacityT y, Vertex u)
      : flow_remaining(x), subflow(y), min_vertex(u) {}

  V operator+(const V &rhs) const {
    return flow_remaining < rhs.flow_remaining ? *this : rhs;
  }
};

struct U {
  CapacityT update_flow_remaining, update_subflow;
  U() = default;
  U(CapacityT x, CapacityT y) : update_flow_remaining(x), update_subflow(y) {}

  static U Compose(U lhs, U rhs) {
    return U(lhs.update_flow_remaining + rhs.update_flow_remaining,
             lhs.update_subflow + rhs.update_subflow);
  }
  static V Apply(U upd, V val) {
    return V(val.flow_remaining + upd.update_flow_remaining,
             val.subflow + upd.update_subflow, val.min_vertex);
  }
};

std::pair<MultiCommodityDemand, std::vector<CapacityT>> RouteInternal(
    const Graph &g, std::vector<CapacityT> flow,
    MultiCommodityDemand subdemand) {
  assert(static_cast<int>(flow.size()) == g.m);

  // Find sources and sinks in this flow.
  std::vector<CapacityT> net_flow(g.n);
  for (Edge e : g.Edges()) {
    net_flow[g.tail[e]] += flow[e];
    net_flow[g.head[e]] -= flow[e];
  }
  CapacityT total_flow = 0;
  std::vector<Vertex> sinks;
  for (Vertex v = 0; v < g.n; ++v) {
    if (net_flow[v] > 0) {
      total_flow += net_flow[v];
    } else if (net_flow[v] < 0) {
      sinks.push_back(v);
    }
  }

  std::vector<Edge> parent(g.n, -1);
  auto in_edges = g.in_edges;

  auto GetNewParent = [&](Vertex v) {
    while (!in_edges[v].empty() && flow[in_edges[v].back()] == 0) {
      in_edges[v].pop_back();
    }
    if (in_edges[v].empty()) {
      return -1;
    }
    Edge e = in_edges[v].back();
    in_edges[v].pop_back();
    return e;
  };

  std::queue<Vertex> no_parent;
  for (Vertex v : g.Vertices()) {
    no_parent.push(v);
  }
  LinkCutTree<U, V> link_cut_tree(g.n);
  std::vector<CapacityT> subflow(g.m);

  // Mark the parent edge of u as invalid since it no longer has flow on it.
  auto InvalidateParent = [&](Vertex u) {
    assert(parent[u] != -1);
    // Extract the subflow value on the edge before removal.
    subflow[parent[u]] = link_cut_tree.QueryParentEdge(u).subflow;
    link_cut_tree.CutParent(u);
    parent[u] = -1;
    no_parent.push(u);
  };

  // Send f units of flow from the root to u.
  auto SendFlowToRoot = [&](Vertex u, CapacityT f) {
    link_cut_tree.UpdatePathToRoot(u, U(-f, 0));
    // Sending flow may zero-out the flow values of several edges along this
    // path. Extract and invalidate them in the root-to-u order.
    while (parent[u] != -1) {
      auto val = link_cut_tree.QueryPathToRoot(u);
      if (val.flow_remaining > 0) {
        break;
      }
      InvalidateParent(val.min_vertex);
    }
  };

  MultiCommodityDemand demand;
  CapacityT total_routed =
      0;  // To check all demands have been successfully routed at the end.

  while (!sinks.empty()) {
    if (!no_parent.empty()) {
      // First make sure every vertex has a parent edge.
      Vertex u = no_parent.front();
      no_parent.pop();
      while (true) {
        parent[u] = GetNewParent(u);
        if (parent[u] == -1) {
          break;
        }
        Vertex v = g.tail[parent[u]];
        if (u == v) continue;
        CapacityT remainingflow = flow[parent[u]];
        if (link_cut_tree.GetRoot(v) == u) {  // This creates a cycle.
          V cycle = link_cut_tree.QueryPathToRoot(v);
          assert(cycle.flow_remaining > 0);
          SendFlowToRoot(v, std::min(remainingflow, cycle.flow_remaining));
          remainingflow -= std::min(remainingflow, cycle.flow_remaining);
        }
        if (remainingflow > 0) {
          link_cut_tree.Link(u, v, V(remainingflow, 0, u));
          break;
        }
      }
    } else {
      // If every vertex has a parent edge, send some unit of flow from a source
      // to a sink so that the flow value on at least one edge is zeroed-out.
      Vertex t = sinks.back();
      sinks.pop_back();
      Vertex s = link_cut_tree.GetRoot(t);
      assert(net_flow[s] > 0 && "The sink must now be connected to a source");
      V value = link_cut_tree.QueryPathToRoot(t);
      assert(value.flow_remaining > 0 &&
             "Link cut tree must contain edges with positive flow values");
      CapacityT f = std::min({value.flow_remaining, net_flow[s], -net_flow[t]});
      total_routed += f;
      demand[std::make_pair(s, t)] += f;
      CapacityT to_route =
          std::min(subdemand[std::make_pair(s, t)],
                   f);  // Amount of subflow to route along this path.
      subdemand[std::make_pair(s, t)] -= to_route;
      link_cut_tree.UpdatePathToRoot(t, U(0, to_route));
      SendFlowToRoot(t, f);
      net_flow[t] += f;
      net_flow[s] -= f;
      assert(net_flow[t] <= 0 && net_flow[s] >= 0 &&
             "A source/sink remains a source/sink");
      // The sink is not yet saturated, push it back to the queue.
      if (net_flow[t] < 0) sinks.push_back(t);
    }
  }
  assert(total_flow == total_routed &&
         std::ranges::all_of(net_flow, [](auto v) { return v == 0; }) &&
         "Failed to route all demands");
  assert(
      std::ranges::all_of(subdemand, [](auto kv) { return kv.second == 0; }) &&
      "Failed to route subdemand");

  // Extract subflow value for edges remained in the link cut tree.
  for (Vertex u : g.Vertices()) {
    if (parent[u] != -1)
      subflow[parent[u]] = link_cut_tree.QueryParentEdge(u).subflow;
  }
  return std::make_pair(demand, subflow);
}

}  // namespace

FlowDecomposition::FlowDecomposition(const Graph &g,
                                     const std::vector<CapacityT> &flow)
    : g_(g), flow_(flow) {
  std::vector<CapacityT> _;
  std::tie(demand_, _) = RouteInternal(g_, flow_, {});
}

std::vector<CapacityT> FlowDecomposition::Route(
    const std::map<std::pair<Vertex, Vertex>, CapacityT> &subdemand) {
  for (auto [k, v] : subdemand) {
    assert(demand_.find(k) != demand_.end() && v <= demand_[k] &&
           "Input must be a subdemand");
  }
  return RouteInternal(g_, flow_, subdemand).second;
}
