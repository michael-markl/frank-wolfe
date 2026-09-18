use std::{cmp::min, sync::RwLock};

use log::warn;
use ordered_float::NotNan;

use crate::{
    routing::astar::AStarTable,
    routing::astar_tree::{AStarTree, CostValuesOps},
    network::bundle_index::BundleIndex,
    common::{BundleIdx, EdgeIdx, Float, NodeIdx, PathIdx},
    network::demand::Demand,
    optimization::edge_based_solution::EdgeBasedSolution,
    network::graph_ops::GraphOps,
    network::path_index::{Path, PathIndex},
};

pub fn path_decomposition(
    edge_flow: &mut EdgeBasedSolution,
    demand: &Demand,
    graph: &impl GraphOps,
    a_star_table: &AStarTable,
    costs: &impl CostValuesOps,
    path_index: &mut PathIndex,
    bundles: &RwLock<BundleIndex>,
) -> Vec<Vec<(PathIdx, Float)>> {
    demand.check_solution(edge_flow, graph);
    struct SubGraph<'a, G: GraphOps> {
        graph: &'a G,
        flow: &'a EdgeBasedSolution,
    }

    let total_flow_before: Float = edge_flow.edge_flow().iter().sum::<Float>() + edge_flow.permit_flow().iter().sum::<Float>();

    let mut omitted_demand = 0.0;

    impl<'a, G: GraphOps> GraphOps for SubGraph<'a, G> {
        fn num_edges(&self) -> usize {
            self.graph.num_edges()
        }

        fn num_nodes(&self) -> usize {
            self.graph.num_nodes()
        }
        fn num_permits(&self) -> usize {
            self.graph.num_permits()
        }

        fn incoming_edges(&self, node_idx: NodeIdx) -> impl Iterator<Item = EdgeIdx> {
            self.graph
                .incoming_edges(node_idx)
                .filter(move |edge_idx| self.flow.edge_flow()[*edge_idx] > 0.0)
        }

        fn outgoing_edges(&self, node_idx: NodeIdx) -> impl Iterator<Item = EdgeIdx> {
            self.graph
                .outgoing_edges(node_idx)
                .filter(move |edge_idx| self.flow.edge_flow()[*edge_idx] > 0.0)
        }

        fn edge_cost_lower_bound(&self, edge_idx: EdgeIdx) -> Float {
            self.graph.edge_cost_lower_bound(edge_idx)
        }

        fn edge_tail(&self, edge_idx: EdgeIdx) -> NodeIdx {
            self.graph.edge_tail(edge_idx)
        }

        fn edge_head(&self, edge_idx: EdgeIdx) -> NodeIdx {
            self.graph.edge_head(edge_idx)
        }

        fn edge_bundle(&self, edge_idx: EdgeIdx) -> BundleIdx {
            self.graph.edge_bundle(edge_idx)
        }

        fn node_allows_through_traffic(&self, node_idx: NodeIdx) -> bool {
            self.graph.node_allows_through_traffic(node_idx)
        }
    }

    let res = demand
        .commodities()
        .iter()
        .enumerate()
        .map(|(commodity_idx, commodity)| {
            let mut paths = Vec::new();
            let mut remaining_flow = NotNan::new(commodity.demand).unwrap();
            while remaining_flow.into_inner() > 0.0 {
                let path = AStarTree::new(commodity.origin, a_star_table)
                .compute_shortest_path(
                    &SubGraph { graph, flow: edge_flow },
                    demand,
                    costs,
                    commodity.destination_idx,
                    bundles,
                );
                if path.is_none() {
                    if remaining_flow.into_inner() > 1e-8 {
                        warn!("No path found for commodity {} with remaining flow {} ({:.3}%), but flow is not fully decomposed.", commodity_idx, remaining_flow, 100.0 * remaining_flow.into_inner() / commodity.demand);
                    }
                    omitted_demand += remaining_flow.into_inner();
                    break;
                }
                let (path_cost, path, bundle_idx) = path.unwrap();
                let min_flow_on_edge = path.iter()
                    .map(|edge_idx| NotNan::new(edge_flow.edge_flow()[*edge_idx]).unwrap()).min().unwrap_or(NotNan::new(Float::INFINITY).unwrap());
                
                let path_flow = min(remaining_flow, min_flow_on_edge);

                let path_idx = path_index.transfer_element(Path::from_edges(path));
                paths.push((path_idx, path_flow.into()));
                edge_flow.reduce(path_index.get_payload(path_idx), bundles.read().unwrap().get_payload(bundle_idx), path_flow.into_inner());
                remaining_flow -= path_flow;
            }
            paths
        })
        .collect();
    
    if omitted_demand > 1e-8 {
        warn!("Total omitted demand in path decomposition: {} ({:.3}%)", omitted_demand, 100.0 * omitted_demand / demand.commodities().iter().map(|it| it.demand).sum::<Float>());        
    }
    let total_flow: Float = edge_flow.edge_flow().iter().sum::<Float>() + edge_flow.permit_flow().iter().sum::<Float>();
    if total_flow > 1e-8 {
        warn!("Total edge & permit flow remaining after path decomposition: {} ({:.3}%)", total_flow, 100.0 * total_flow / total_flow_before);
    }

    res
}
