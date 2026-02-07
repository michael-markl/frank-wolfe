use std::{mem::replace, sync::RwLock};

use rayon::iter::ParallelIterator;

use crate::{
    astar::AStarTable,
    astar_tree::{AStarTree, ShortestPathCostOps},
    bundle_index::{Bundle, BundleIndex},
    col::{HashMap, map_new},
    common::{BundleIdx, EdgeIdx, Float, PathIdx},
    demand::DemandOps,
    frank_wolfe::{ConvexProgramInstance, SolutionOps},
    graph::{Edge, Graph},
    graph_ops::GraphOps,
    path_index::PathIndex,
    tntp::TNTPNet,
};

mod astar;
mod astar_tree;
mod bundle_index;
mod col;
mod common;
mod demand;
mod frank_wolfe;
mod graph;
mod graph_ops;
mod index;
mod iter;
mod path_index;
mod tntp;

fn main() {
    let tntpnet = tntp::read_net_file(std::path::Path::new(
        "C:\\Users\\markl07\\git\\TransportationNetworks\\Berlin-Center\\berlin-center_net.tntp",
    ))
    .map_err(|err| eprintln!("Error reading net file: {}", err))
    .unwrap();
    let demand= tntp::read_trips_file(std::path::Path::new("C:\\Users\\markl07\\git\\TransportationNetworks\\Berlin-Center\\berlin-center_trips.tntp"), &tntpnet)
        .map_err(|err| eprintln!("Error reading trips file: {}", err))
        .unwrap();

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
                EdgeFlowState::InvalidUnallocated(_) => {
                    EdgeFlowState::InvalidUnallocated(num_edges)
                }
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

        fn inner_prod(&self, other: &Self) -> Float {
            todo!()
        }
    }

    struct Instance {}

    impl ConvexProgramInstance<PathBasedSolution> for Instance {
        fn directional_derivative(
            &self,
            at: &PathBasedSolution,
            direction: &PathBasedSolution,
        ) -> Float {
            todo!()
        }

        fn compute_objective(&self, solution: &PathBasedSolution) -> Float {
            todo!()
        }

        fn solve_subproblem(
            &self,
            x: &PathBasedSolution,
        ) -> frank_wolfe::LinearizedSubProblemSolution<PathBasedSolution> {
            todo!()
        }
    }

    let graph = tntpnet.graph;
    let mut astar_table = AStarTable::create(graph.num_nodes(), demand.num_destinations());
    astar_table.fill_table(&graph, &demand);

    let bundle_index = RwLock::new(BundleIndex::new());
    bundle_index
        .write()
        .unwrap()
        .transfer_element(Bundle::from_permits(vec![])); // TODO: Handle empty set more efficiently.

    struct Costs<'a>(&'a Graph);

    impl ShortestPathCostOps for Costs<'_> {
        fn get_edge_cost(&self, edge_idx: EdgeIdx) -> Float {
            self.0.edge_cost_lower_bound(edge_idx)
        }

        fn get_permit_cost(&self, permit_idx: common::PermitIdx) -> Float {
            todo!()
        }
    }

    let costs = Costs(&graph);

    demand
        .par_iter_by_origin()
        .for_each(|(&origin, commodity_indices)| {
            let mut tree = AStarTree::new(origin);

            for &commodity_idx in commodity_indices {
                let destination_idx = demand.get_commodity(commodity_idx).destination_idx;
                tree.compute_distance(
                    &astar_table,
                    &graph,
                    &demand,
                    &costs,
                    destination_idx,
                    &bundle_index,
                );
            }
        });

    //frank_wolfe::solve_convex_program(initial_solution, instance)
}
