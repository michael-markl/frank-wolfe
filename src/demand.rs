use crate::{
    col::{HashMap, map_new},
    common::{CommodityIdx, DestinationIdx, Float, NodeIdx},
    graph_ops::GraphOps,
};
use log::warn;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};

pub trait DemandOps {
    fn node_idx_by_destination(&self, destination_idx: DestinationIdx) -> NodeIdx;

    fn num_destinations(&self) -> usize;
}

pub struct Commodity {
    pub origin: NodeIdx,
    pub destination_idx: DestinationIdx,
    pub demand: Float,
}

pub struct Demand {
    destinations: Vec<NodeIdx>,
    destination_by_node_idx: HashMap<NodeIdx, DestinationIdx>,
    commodities_by_origin: HashMap<NodeIdx, Vec<CommodityIdx>>,
    commodities: Vec<Commodity>,
}

impl Demand {
    pub fn empty() -> Self {
        Self {
            destinations: Vec::new(),
            destination_by_node_idx: map_new(),
            commodities_by_origin: map_new(),
            commodities: Vec::new(),
        }
    }

    pub fn commodities(&self) -> &Vec<Commodity> {
        &self.commodities
    }

    pub fn add_commodity(
        &mut self,
        origin: NodeIdx,
        destination: NodeIdx,
        demand: Float,
    ) -> CommodityIdx {
        let destination_idx = self
            .destination_by_node_idx
            .entry(destination)
            .or_insert_with(|| {
                self.destinations.push(destination);
                self.destinations.len() - 1
            });

        let commodity_idx = self.commodities.len();
        self.commodities.push(Commodity {
            origin,
            destination_idx: *destination_idx,
            demand,
        });
        self.commodities_by_origin
            .entry(origin)
            .or_default()
            .push(commodity_idx);

        commodity_idx
    }

    pub fn get_commodity(&self, commodity_idx: CommodityIdx) -> &Commodity {
        &self.commodities[commodity_idx]
    }

    pub fn par_iter_by_origin(
        &self,
    ) -> impl ParallelIterator<Item = (&NodeIdx, &Vec<CommodityIdx>)> {
        self.commodities_by_origin.par_iter()
    }

    pub fn check_solution(
        &self,
        solution: &crate::edge_based_solution::EdgeBasedSolution,
        graph: &crate::graph::Graph,
    ) {
        // Check flow conservation at every node
        for node_idx in 0..graph.num_nodes() {
            let mut inflow = 0.0;
            let mut outflow = 0.0;

            for incoming_edge in graph.incoming_edges(node_idx as NodeIdx) {
                inflow += solution.edge_flow()[incoming_edge];
            }

            for outgoing_edge in graph.outgoing_edges(node_idx as NodeIdx) {
                outflow += solution.edge_flow()[outgoing_edge];
            }

            // Calculate net demand at this node
            let mut origin_demand = 0.0;
            let mut destination_demand = 0.0;
            for commodity in &self.commodities {
                if commodity.origin == node_idx as NodeIdx {
                    origin_demand += commodity.demand;
                }
                if self.destinations[commodity.destination_idx] == node_idx as NodeIdx {
                    destination_demand += commodity.demand;
                }
            }

            let balance = inflow - destination_demand - outflow + origin_demand;
            let check_thru = graph.node_allows_through_traffic(node_idx)
                || ((inflow - destination_demand).abs() < 1e-8
                    && (outflow - origin_demand).abs() < 1e-8);

            if balance.abs() >= 1e-8 || !check_thru {
                warn!(
                    "Flow conservation violated at node {}: inflow={}, outflow={}, origin_demand={}, destination_demand={}, balance={}",
                    node_idx, inflow, outflow, origin_demand, destination_demand, balance
                );
            }
        }
    }
}

impl DemandOps for Demand {
    fn node_idx_by_destination(&self, destination_idx: DestinationIdx) -> NodeIdx {
        self.destinations[destination_idx]
    }
    fn num_destinations(&self) -> usize {
        self.destinations.len()
    }
}
