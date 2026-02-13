use std::sync::RwLock;

use accurate::dot::traits::DotWithAccumulator;
use accurate::traits::{SumAccumulator, SumWithAccumulator};
use rayon::iter::ParallelIterator;

use crate::common::{MyDotAccumulator, MySumAccumulator};
use crate::{
    BMWFunction,
    astar::AStarTable,
    astar_tree::{AStarTree, CostValuesOps},
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

        impl CostValuesOps for Costs<'_> {
            fn get_edge_cost(&self, edge_idx: EdgeIdx) -> Float {
                self.0[edge_idx]
            }

            fn get_permit_cost(&self, permit_idx: common::PermitIdx) -> Float {
                self.1[permit_idx]
            }
        }

        let costs = Costs(edge_costs, permit_costs);
        let mut edge_flow = vec![MySumAccumulator::zero(); self.graph.num_edges()];
        let mut permit_flow = vec![MySumAccumulator::zero(); self.graph.num_permits()];

        let results = self
            .demand
            .par_iter_by_origin()
            .map(|(&origin, commodity_indices)| {
                let mut tree = AStarTree::new(origin, self.astar_table);
                let costs = &costs;
                commodity_indices.iter().map(move |&commodity_idx| {
                    let commodity = self.demand.get_commodity(commodity_idx);
                    let destination_idx = commodity.destination_idx;
                    let (_cost, path, bundle_idx) = tree.compute_shortest_path(
                        self.graph,
                        self.demand,
                        costs,
                        destination_idx,
                        self.bundle_index,
                    );
                    (path, bundle_idx, commodity.demand)
                })
            })
            .flatten_iter()
            .collect::<Vec<_>>();

        for (path, bundle_idx, demand) in results.into_iter() {
            for edge_idx in path {
                edge_flow[edge_idx] += demand;
            }
            let binding = self.bundle_index.read().unwrap();
            let bundle = binding.get_payload(bundle_idx);
            for permit_idx in bundle.permits() {
                permit_flow[permit_idx] += demand;
            }
            drop(binding);
        }

        EdgeBasedSolution::from_vec(
            edge_flow.into_iter().map(|it| it.sum()).collect(),
            permit_flow.into_iter().map(|it| it.sum()).collect(),
        )
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
            .map(|(edge_idx, &it)| BMWFunction::derivative(&self.graph.edge(edge_idx).params, it))
            .zip(direction.edge_flow().iter().copied())
            .chain(
                at.permit_flow()
                    .iter()
                    .enumerate()
                    .map(|(permit_idx, &it)| {
                        BMWFunction::derivative(&self.graph.permit(permit_idx).params, it)
                    })
                    .zip(direction.permit_flow().iter().copied()),
            )
            .dot_with_accumulator::<MyDotAccumulator>()
    }

    fn compute_objective(&self, solution: &EdgeBasedSolution) -> Float {
        solution
            .edge_flow()
            .iter()
            .enumerate()
            .map(|(edge_idx, &it)| BMWFunction::evaluate(&self.graph.edge(edge_idx).params, it))
            .chain(
                solution
                    .permit_flow()
                    .iter()
                    .enumerate()
                    .map(|(permit_idx, &it)| {
                        BMWFunction::evaluate(&self.graph.permit(permit_idx).params, it)
                    }),
            )
            .sum_with_accumulator::<MySumAccumulator>()
    }

    fn solve_subproblem(
        &self,
        x: &EdgeBasedSolution,
    ) -> LinearizedSubProblemSolution<EdgeBasedSolution> {
        let edge_gradient_at_x = x
            .edge_flow()
            .iter()
            .enumerate()
            .map(|(edge_idx, &it)| BMWFunction::derivative(&self.graph.edge(edge_idx).params, it))
            .collect::<Vec<_>>();

        let permit_gradient_at_x = x
            .permit_flow()
            .iter()
            .enumerate()
            .map(|(permit_idx, &it)| {
                BMWFunction::derivative(&self.graph.permit(permit_idx).params, it)
            })
            .collect::<Vec<_>>();

        let y = self.compute_shortest_path_flow(&edge_gradient_at_x, &permit_gradient_at_x);
        let diff = EdgeBasedSolution::from_linear_combination(&y, -1.0, x);
        let inner_product =
            EdgeBasedSolution::from_vec(edge_gradient_at_x, permit_gradient_at_x).inner_prod(&diff);
        LinearizedSubProblemSolution {
            solution: y,
            direction: diff,
            inner_product,
        }
    }
}
