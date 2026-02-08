use std::sync::RwLock;

use rayon::iter::ParallelIterator;

use crate::{
    BMWFunction,
    astar::AStarTable,
    astar_tree::{AStarTree, ShortestPathCostOps},
    bundle_index::BundleIndex,
    common::{self, EdgeIdx, Float},
    demand::Demand,
    edge_based_solution::EdgeBasedSolution,
    frank_wolfe::{ConvexProgramInstance, LinearizedSubProblemSolution, SolutionOps},
    graph::Graph,
    graph_ops::GraphOps,
};

pub struct EdgeBasedConvexProgramInstance<'g, 'd, 't, 'b> {
    pub graph: &'g Graph,
    pub demand: &'d Demand,
    pub astar_table: &'t AStarTable,
    pub bundle_index: &'b RwLock<BundleIndex<'b>>,
}

impl<'g, 'd, 't, 'b> EdgeBasedConvexProgramInstance<'g, 'd, 't, 'b> {
    pub fn compute_shortest_path_flow(
        &self,
        edge_costs: &Vec<Float>,
        permit_costs: &Vec<Float>,
    ) -> EdgeBasedSolution {
        struct Costs<'a>(&'a Vec<Float>, &'a Vec<Float>);

        impl ShortestPathCostOps for Costs<'_> {
            fn get_edge_cost(&self, edge_idx: EdgeIdx) -> Float {
                self.0[edge_idx]
            }

            fn get_permit_cost(&self, permit_idx: common::PermitIdx) -> Float {
                self.1[permit_idx]
            }
        }

        let costs = Costs(edge_costs, permit_costs);

        let (edge_flow, permit_flow) = self
            .demand
            .par_iter_by_origin()
            .map(|(&origin, commodity_indices)| {
                let mut origin_edge_flow = vec![0.0; self.graph.num_edges()];
                let mut origin_permit_flow = vec![0.0; self.graph.num_permits()];
                let mut tree = AStarTree::new(origin);
                for &commodity_idx in commodity_indices {
                    let commodity = self.demand.get_commodity(commodity_idx);
                    let destination_idx = commodity.destination_idx;
                    let (_cost, path, bundle_idx) = tree.compute_shortest_path(
                        self.astar_table,
                        self.graph,
                        self.demand,
                        &costs,
                        destination_idx,
                        self.bundle_index,
                    );
                    for edge_idx in path {
                        origin_edge_flow[edge_idx] += commodity.demand;
                    }
                    for permit_idx in self.bundle_index.read().unwrap().get_payload(bundle_idx).permits() {
                        origin_permit_flow[permit_idx] += commodity.demand;
                    }
                }
                (origin_edge_flow, origin_permit_flow)
            })
            .reduce(
                || {
                    (
                        vec![0.0; self.graph.num_edges()],
                        vec![0.0; self.graph.num_permits()],
                    )
                },
                |(mut a_edges, mut a_permits), (b_edges, b_permits)| {
                    for (f_a, f_b) in a_edges.iter_mut().zip(b_edges.iter()) {
                        *f_a += f_b;
                    }
                    for (f_a, f_b) in a_permits.iter_mut().zip(b_permits.iter()) {
                        *f_a += f_b;
                    }
                    (a_edges, a_permits)
                },
            );

        EdgeBasedSolution::from_vec(edge_flow, permit_flow)
    }
}

impl<'g, 'd, 't, 'b> ConvexProgramInstance<EdgeBasedSolution>
    for EdgeBasedConvexProgramInstance<'g, 'd, 't, 'b>
{
    fn directional_derivative(
        &self,
        at: &EdgeBasedSolution,
        direction: &EdgeBasedSolution,
    ) -> Float {
        at.edge_flow()
            .iter()
            .enumerate()
            .map(|(edge_idx, it)| BMWFunction::derivative(self.graph.edge(edge_idx), *it))
            .zip(direction.edge_flow().iter())
            .map(|(a, b)| a * b)
            .sum::<Float>()
            + at.permit_flow()
                .iter()
                .enumerate()
                .map(|(permit_idx, _it)| self.graph.permit(permit_idx).params.alpha) // TODO: PERMIT COSTS
                .zip(direction.permit_flow().iter())
                .map(|(a, b)| a * b)
                .sum::<Float>()
    }

    fn compute_objective(&self, solution: &EdgeBasedSolution) -> Float {
        solution
            .edge_flow()
            .iter()
            .enumerate()
            .map(|(edge_idx, it)| BMWFunction::evaluate(self.graph.edge(edge_idx), *it))
            .sum::<Float>()
            + solution
                .permit_flow()
                .iter()
                .enumerate()
                .map(|(permit_idx, it)| self.graph.permit(permit_idx).params.alpha * *it) // TODO: PERMIT COSTS
                .sum::<Float>()
    }

    fn solve_subproblem(
        &self,
        x: &EdgeBasedSolution,
    ) -> LinearizedSubProblemSolution<EdgeBasedSolution> {
        let edge_gradient_at_x = x
            .edge_flow()
            .iter()
            .enumerate()
            .map(|(edge_idx, it)| BMWFunction::derivative(self.graph.edge(edge_idx), *it))
            .collect::<Vec<_>>();

        let permit_gradient_at_x = x
            .permit_flow()
            .iter()
            .enumerate()
            .map(|(permit_idx, _it)| self.graph.permit(permit_idx).params.alpha) // TODO: PERMIT COSTS
            .collect::<Vec<_>>();

        // TODO: PERMIT COSTS
        let y = self.compute_shortest_path_flow(&edge_gradient_at_x, &permit_gradient_at_x);
        let diff = EdgeBasedSolution::from_linear_combination(&y, -1.0, &x);
        let inner_product =
            EdgeBasedSolution::from_vec(edge_gradient_at_x, permit_gradient_at_x).inner_prod(&diff);
        LinearizedSubProblemSolution {
            solution: y,
            direction: diff,
            inner_product,
        }
    }
}
