use std::mem::replace;

use crate::{
    col::{HashMap, map_new},
    common::{BundleIdx, Float, PathIdx},
    frank_wolfe::SolutionOps,
    path_index::PathIndex,
};

#[derive(Clone)]
enum EdgeFlowState {
    Valid(Vec<Float>),
    InvalidAllocated(Vec<Float>),
    InvalidUnallocated(usize), // the number of edges, to initialize the flow vector
}

#[derive(Clone)]
struct PathBasedSolution {
    path_flow: HashMap<PathIdx, (BundleIdx, Float)>,
    edge_flow: EdgeFlowState,
}

fn compute_edge_flow(
    path_flow: &HashMap<PathIdx, (BundleIdx, Float)>,
    path_index: &PathIndex,
    edge_flow: &mut Vec<Float>,
) {
    for (path_idx, (_bundle_idx, flow)) in path_flow {
        let path = path_index.get_payload(*path_idx);
        for edge_idx in path.edges() {
            edge_flow[*edge_idx] += flow;
        }
    }
}

impl PathBasedSolution {
    pub fn empty(num_edges: usize) -> Self {
        PathBasedSolution {
            path_flow: map_new(),
            edge_flow: EdgeFlowState::InvalidUnallocated(num_edges),
        }
    }

    pub fn num_edges(&self) -> usize {
        match &self.edge_flow {
            EdgeFlowState::Valid(edge_flow) => edge_flow.len(),
            EdgeFlowState::InvalidAllocated(edge_flow) => edge_flow.len(),
            EdgeFlowState::InvalidUnallocated(num_edges) => *num_edges,
        }
    }

    pub fn edge_flow<'a>(&'a mut self, path_index: &'a PathIndex) -> &'a Vec<Float> {
        let num_edges = self.num_edges();
        let edge_flow = replace(
            &mut self.edge_flow,
            EdgeFlowState::InvalidUnallocated(num_edges),
        );

        self.edge_flow = match edge_flow {
            EdgeFlowState::Valid(edge_flow) => EdgeFlowState::Valid(edge_flow),
            EdgeFlowState::InvalidUnallocated(capacity) => {
                let mut edge_flow = vec![0.0; capacity];
                compute_edge_flow(&self.path_flow, path_index, &mut edge_flow);
                EdgeFlowState::Valid(edge_flow)
            }
            EdgeFlowState::InvalidAllocated(mut edge_flow) => {
                compute_edge_flow(&self.path_flow, path_index, &mut edge_flow);
                EdgeFlowState::Valid(edge_flow)
            }
        };

        match &self.edge_flow {
            EdgeFlowState::Valid(edge_flow) => edge_flow,
            _ => unreachable!(),
        }
    }

    fn invalidate_edge_flow(&mut self) {
        let num_edges = self.num_edges();
        let edge_flow = replace(
            &mut self.edge_flow,
            EdgeFlowState::InvalidUnallocated(num_edges),
        );
        self.edge_flow = match edge_flow {
            EdgeFlowState::Valid(edge_flow) | EdgeFlowState::InvalidAllocated(edge_flow) => {
                EdgeFlowState::InvalidAllocated(edge_flow)
            }
            EdgeFlowState::InvalidUnallocated(_) => EdgeFlowState::InvalidUnallocated(num_edges),
        };
    }
}

impl SolutionOps for PathBasedSolution {
    fn from_linear_combination(sol1: &Self, scale2: Float, sol2: &Self) -> Self {
        let mut path_flow = map_new();
        for (path_idx, (bundle_idx, flow)) in &sol1.path_flow {
            path_flow.insert(*path_idx, (*bundle_idx, *flow));
        }
        for (path_idx, (bundle_idx, flow)) in &sol2.path_flow {
            path_flow
                .entry(*path_idx)
                .and_modify(|(_bundle_idx, existing_flow)| *existing_flow += scale2 * flow)
                .or_insert((*bundle_idx, scale2 * flow));
        }
        PathBasedSolution {
            path_flow,
            edge_flow: EdgeFlowState::InvalidUnallocated(sol1.num_edges()),
        }
    }

    fn assign_linear_combination(&mut self, sol1: &Self, scale2: Float, sol2: &Self) {
        self.path_flow.clear();
        for (path_idx, (bundle_idx, flow)) in &sol1.path_flow {
            self.path_flow.insert(*path_idx, (*bundle_idx, *flow));
        }
        for (path_idx, (bundle_idx, flow)) in &sol2.path_flow {
            self.path_flow
                .entry(*path_idx)
                .and_modify(|(_bundle_idx, existing_flow)| *existing_flow += scale2 * flow)
                .or_insert((*bundle_idx, scale2 * flow));
        }
        self.invalidate_edge_flow();
    }

    fn add_scaled(&mut self, scale: Float, other: &Self) {
        for (path_idx, (bundle_idx, flow)) in &other.path_flow {
            self.path_flow
                .entry(*path_idx)
                .and_modify(|(_bundle_idx, existing_flow)| *existing_flow += scale * flow)
                .or_insert((*bundle_idx, scale * flow));
        }
        self.invalidate_edge_flow();
    }
}
