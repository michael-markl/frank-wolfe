use accurate::traits::DotWithAccumulator;
use log::{info, warn};

use crate::{
    collections::HashMap,
    common::{BUNDLE_IDX_EMPTY, CommodityIdx, Float, MyDotAccumulator, PathIdx},
    network::bundle_index::{Bundle, BundleIndex},
    network::demand::Demand,
    network::graph_ops::GraphOps,
    network::path_index::PathIndex,
    optimization::frank_wolfe::SolutionOps,
};

#[derive(Clone)]
pub struct PathBasedSolution {
    edge_flow: Vec<f64>,
    permit_flow: Vec<f64>,
    path_flow: HashMap<(CommodityIdx, PathIdx), Float>,
}

impl PathBasedSolution {
    pub fn edge_flow(&self) -> &Vec<Float> {
        &self.edge_flow
    }

    pub fn permit_flow(&self) -> &Vec<Float> {
        &self.permit_flow
    }

    pub fn path_flow(&self) -> &HashMap<(CommodityIdx, PathIdx), Float> {
        &self.path_flow
    }

    pub fn from_vec(
        edge_flow: Vec<f64>,
        permit_flow: Vec<f64>,
        path_flow: HashMap<(CommodityIdx, PathIdx), Float>,
    ) -> Self {
        PathBasedSolution {
            edge_flow,
            permit_flow,
            path_flow,
        }
    }

    pub fn inner_prod(&self, other: &Self) -> Float {
        self.edge_flow
            .iter()
            .copied()
            .zip(other.edge_flow.iter().copied())
            .chain(
                self.permit_flow
                    .iter()
                    .copied()
                    .zip(other.permit_flow.iter().copied()),
            )
            .dot_with_accumulator::<MyDotAccumulator>()
    }

    pub fn check_consistency(
        &self,
        path_index: &PathIndex,
        bundle_index: &BundleIndex,
        demand: &Demand,
        graph: &impl GraphOps,
    ) {
        info!("Checking consistency...");
        let mut implied_edge_flow = vec![0.0; self.edge_flow.len()];
        let mut implied_permit_flow = vec![0.0; self.permit_flow.len()];
        let mut implied_demand = vec![0.0; demand.commodities().len()];

        for ((commodity_idx, path_idx), &flow) in self.path_flow.iter() {
            let mut bundle = Bundle::empty();
            for edge_idx in path_index.get_payload(*path_idx).edges() {
                implied_edge_flow[edge_idx] += flow;
                let edge_bundle_idx = graph.edge_bundle(edge_idx);
                if edge_bundle_idx != BUNDLE_IDX_EMPTY {
                    bundle = bundle.union(bundle_index.get_payload(edge_bundle_idx));
                }
            }
            for permit_idx in bundle.permits() {
                implied_permit_flow[permit_idx] += flow;
            }
            implied_demand[*commodity_idx] += flow;
        }

        let mut max_inconsistency = 0.0;
        for (idx, (implied, actual)) in implied_edge_flow.iter().zip(&self.edge_flow).enumerate() {
            let diff = (implied - actual).abs();
            max_inconsistency = if diff > max_inconsistency {
                diff
            } else {
                max_inconsistency
            };
            if diff >= 1e-6 {
                warn!(
                    "Inconsistent edge flow for edge {}: implied {}, actual {}",
                    idx, implied, actual
                );
            }
        }

        for (idx, (implied, actual)) in implied_permit_flow
            .iter()
            .zip(&self.permit_flow)
            .enumerate()
        {
            let diff = (implied - actual).abs();
            max_inconsistency = if diff > max_inconsistency {
                diff
            } else {
                max_inconsistency
            };
            if diff >= 1e-6 {
                warn!(
                    "Inconsistent permit flow for permit {}: implied {}, actual {}",
                    idx, implied, actual
                );
            }
        }

        for (idx, (implied, commodity)) in
            implied_demand.iter().zip(demand.commodities()).enumerate()
        {
            let diff = (implied - commodity.demand).abs();
            max_inconsistency = if diff > max_inconsistency {
                diff
            } else {
                max_inconsistency
            };
            if diff >= 1e-6 {
                warn!(
                    "Inconsistent demand flow for commodity {}: implied {}, actual {}",
                    idx, implied, commodity.demand
                );
            }
        }
    }
}

impl SolutionOps for PathBasedSolution {
    fn from_linear_combination(sol1: &Self, scale2: Float, sol2: &Self) -> Self {
        let edge_flow = sol1
            .edge_flow
            .iter()
            .zip(&sol2.edge_flow)
            .map(|(f1, f2)| f1 + scale2 * f2)
            .collect();
        let permit_flow = sol1
            .permit_flow
            .iter()
            .zip(&sol2.permit_flow)
            .map(|(f1, f2)| f1 + scale2 * f2)
            .collect();

        let mut path_flow: HashMap<(CommodityIdx, PathIdx), Float> = sol1.path_flow.clone();
        for (key, &val) in &sol2.path_flow {
            *path_flow.entry(*key).or_insert(0.0) += scale2 * val;
        }

        PathBasedSolution {
            edge_flow,
            permit_flow,
            path_flow,
        }
    }

    fn assign_linear_combination(&mut self, sol1: &Self, scale2: Float, sol2: &Self) {
        for ((f1, f2), f) in sol1
            .edge_flow
            .iter()
            .zip(&sol2.edge_flow)
            .zip(&mut self.edge_flow)
        {
            *f = f1 + scale2 * f2;
        }
        for ((f1, f2), f) in sol1
            .permit_flow
            .iter()
            .zip(&sol2.permit_flow)
            .zip(&mut self.permit_flow)
        {
            *f = f1 + scale2 * f2;
        }

        self.path_flow = sol1.path_flow.clone();
        for (key, &val) in &sol2.path_flow {
            *self.path_flow.entry(*key).or_insert(0.0) += scale2 * val;
        }
    }

    fn add_scaled(&mut self, scale: Float, other: &Self) {
        for (f_other, f_self) in other.edge_flow.iter().zip(&mut self.edge_flow) {
            *f_self += scale * f_other;
        }
        for (f_other, f_self) in other.permit_flow.iter().zip(&mut self.permit_flow) {
            *f_self += scale * f_other;
        }

        for (key, &val) in &other.path_flow {
            *self.path_flow.entry(*key).or_insert(0.0) += scale * val;
        }
    }
}
