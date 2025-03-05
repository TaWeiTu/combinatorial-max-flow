#include "flow_util.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <queue>
#include <vector>

#include "graph.h"
#include "link_cut_tree.h"

namespace flow_decomposition {

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
  static V Reverse(V v) { return v; }
  static U Reverse(U u) { return u; }
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

}  // namespace flow_decomposition

FlowDecomposition::FlowDecomposition(const Graph &g,
                                     const std::vector<CapacityT> &flow)
    : g_(g), flow_(flow) {
  std::vector<CapacityT> _;
  std::tie(demand_, _) = flow_decomposition::RouteInternal(g_, flow_, {});
}

std::vector<CapacityT> FlowDecomposition::Route(
    const MultiCommodityDemand &subdemand) {
  CapacityT scale = std::numeric_limits<CapacityT>::max();
  for (auto [k, v] : subdemand) {
    assert(demand_.find(k) != demand_.end() && "input must be a subdemand");
    if (v != 0) scale = std::min(scale, demand_[k] / v);
  }
  assert(scale >= 1 && "input must be a subdemand");
  MultiCommodityDemand scaled_up_subdemand = subdemand;
  for (auto &[k, v] : scaled_up_subdemand) v *= scale;
  auto scaled_up_flow =
      flow_decomposition::RouteInternal(g_, flow_, scaled_up_subdemand).second;
  return FlowRoundingExact(g_, scaled_up_flow, scale);
}

namespace flow_rounding {

struct V {
  CapacityT min_fwd_flow, min_bwd_flow;
  Edge min_fwd_edge, min_bwd_edge;

  V()
      : min_fwd_flow(std::numeric_limits<CapacityT>::max()),
        min_bwd_flow(std::numeric_limits<CapacityT>::max()),
        min_fwd_edge(-1),
        min_bwd_edge(-1) {}

  V(CapacityT a, CapacityT b, Edge c, Edge d)
      : min_fwd_flow(a), min_bwd_flow(b), min_fwd_edge(c), min_bwd_edge(d) {}

  V operator+(const V &rhs) const {
    V result;
    if (min_fwd_flow < rhs.min_fwd_flow) {
      result.min_fwd_flow = min_fwd_flow;
      result.min_fwd_edge = min_fwd_edge;
    } else {
      result.min_fwd_flow = rhs.min_fwd_flow;
      result.min_fwd_edge = rhs.min_fwd_edge;
    }
    if (min_bwd_flow < rhs.min_bwd_flow) {
      result.min_bwd_flow = min_bwd_flow;
      result.min_bwd_edge = min_bwd_edge;
    } else {
      result.min_bwd_flow = rhs.min_bwd_flow;
      result.min_bwd_edge = rhs.min_bwd_edge;
    }
    return result;
  }
};

struct U {
  CapacityT delta;

  U() : delta(0) {}
  U(CapacityT d) : delta(d) {}

  static U Compose(U lhs, U rhs) { return U(lhs.delta + rhs.delta); }
  static V Apply(U upd, V val) {
    if (val.min_fwd_flow == std::numeric_limits<CapacityT>::max()) {
      assert(val.min_bwd_flow == std::numeric_limits<CapacityT>::max());
      return val;
    }
    return V(val.min_fwd_flow - upd.delta, val.min_bwd_flow + upd.delta,
             val.min_fwd_edge, val.min_bwd_edge);
  }
  static V Reverse(V v) {
    return V(v.min_bwd_flow, v.min_fwd_flow, v.min_bwd_edge, v.min_fwd_edge);
  }
  static U Reverse(U u) { return U(-u.delta); }
};

};  // namespace flow_rounding

std::vector<CapacityT> FlowRoundingExact(const Graph &g,
                                         const std::vector<CapacityT> &flow,
                                         CapacityT scale) {
  auto demand = FlowToDemand(g, flow);
  assert(std::ranges::all_of(g.Vertices(),
                             [&](Vertex v) { return demand[v] % scale == 0; }));

  using namespace flow_rounding;
  LinkCutTree<U, V> link_cut_tree(g.n + g.m);

  auto rounded_flow = flow;
  for (auto &v : rounded_flow) v /= scale;

  auto FinalizeEdge = [&](auto e) {
    link_cut_tree.MakeRoot(g.head[e]);
    auto value = link_cut_tree.QueryParentEdge(e + g.n);
    assert(value.min_fwd_flow == 0 || value.min_bwd_flow == 0);
    if (value.min_fwd_flow == 0) rounded_flow[e]++;
  };

  for (Edge e : g.Edges()) {
    if (g.head[e] == g.tail[e]) {
      rounded_flow[e] = 0;
      continue;
    }
    if (flow[e] % scale == 0) continue;
    Vertex u = g.tail[e], v = g.head[e];
    // Can send fwd flow in the forward direction and bwd flow in the backward
    // direction.
    CapacityT fwd = scale - flow[e] % scale, bwd = flow[e] % scale;
    while (fwd > 0 && bwd > 0 &&
           link_cut_tree.GetRoot(u) == link_cut_tree.GetRoot(v)) {
      link_cut_tree.MakeRoot(u);
      V value = link_cut_tree.QueryPathToRoot(v);
      if (value.min_fwd_flow == 0 || value.min_bwd_flow == 0) {
        Edge z =
            value.min_fwd_flow == 0 ? value.min_fwd_edge : value.min_bwd_edge;
        FinalizeEdge(z);
        Vertex p = link_cut_tree.GetParent(z + g.n);
        assert(p == g.tail[z] || p == g.head[z]);
        Vertex q = g.tail[z] ^ g.head[z] ^ p;
        assert(link_cut_tree.GetParent(q) == z + g.n);
        link_cut_tree.CutParent(q);
        link_cut_tree.CutParent(z + g.n);
      } else {
        CapacityT fwd_send = std::min(value.min_fwd_flow, fwd);
        CapacityT bwd_send = std::min(value.min_bwd_flow, bwd);
        CapacityT send = fwd_send < bwd_send ? fwd_send : -bwd_send;
        link_cut_tree.UpdatePathToRoot(v, U(send));
        fwd -= send;
        bwd += send;
      }
    }
    if (fwd > 0 && bwd > 0) {
      link_cut_tree.MakeRoot(u);
      link_cut_tree.Link(u, e + g.n, V{});
      link_cut_tree.Link(e + g.n, v, V{fwd, bwd, e, e});
    } else {
      if (fwd == 0) rounded_flow[e]++;
    }
  }
  for (Edge e : g.Edges()) {
    if (link_cut_tree.GetParent(e + g.n) != -1) FinalizeEdge(e);
  }
  {
    auto rounded_demand = FlowToDemand(g, rounded_flow);
    for (auto &v : rounded_demand) v *= scale;
    assert(rounded_demand == demand);
  }
  return rounded_flow;
}
std::vector<CapacityT> FlowRoundingRoundedDown(
    const Graph &g, const std::vector<CapacityT> &flow, CapacityT scale) {
  auto demand = FlowToDemand(g, flow);
  auto new_graph = g;
  Vertex s = new_graph.AddVertex();
  auto new_flow = flow;
  for (Vertex v : g.Vertices()) {
    if (demand[v] % scale == 0) continue;
    if (demand[v] > 0) {
      new_graph.AddEdge(v, s);
      new_flow.push_back(scale - demand[v] % scale);
    } else {
      new_graph.AddEdge(s, v);
      new_flow.push_back(scale - std::abs(demand[v]) % scale);
    }
  }
  auto rounded_flow = FlowRoundingExact(new_graph, new_flow, scale);
  auto prefix = std::vector(rounded_flow.begin(), rounded_flow.begin() + g.m);
  auto routed_demand = FlowToDemand(g, prefix);
  assert(std::ranges::all_of(g.Vertices(), [&](Vertex v) {
    return demand[v] * routed_demand[v] >= 0 &&
           std::abs(routed_demand[v]) >= std::abs(demand[v]) / scale;
  }));
  return prefix;
}
