use std::sync::RwLock;

use accurate::dot::traits::DotWithAccumulator;
use accurate::traits::{SumAccumulator, SumWithAccumulator};
use rayon::iter::{ParallelBridge, ParallelIterator};

use crate::col::HashMap;
use crate::common::{BUNDLE_IDX_EMPTY, MyDotAccumulator, MySumAccumulator};
use crate::iter::for_each_with_thread_local::ForEachWithThreadLocal;
use crate::path_based_solution::PathBasedSolution;
use crate::path_index::{Path, PathIndex};
use crate::{
    BMWFunction,
    astar::AStarTable,
    astar_tree::{AStarTree, CostValuesOps},
    bundle_index::BundleIndex,
    common::{self, EdgeIdx, Float},
    demand::Demand,
    frank_wolfe::{ConvexProgramInstance, LinearizedSubProblemSolution, SolutionOps},
    graph::Graph,
    graph_ops::GraphOps,
};

pub struct PathBasedConvexProgramInstance<'g, 'd, 't, 'b, 'idx, 'p> {
    pub graph: &'g Graph,
    pub demand: &'d Demand,
    pub astar_table: &'t AStarTable,
    pub bundle_index: &'b RwLock<BundleIndex<'b>>,
    pub path_index: &'p mut PathIndex<'idx>,
}

impl<'g, 'd, 't, 'b, 'idx, 'p> PathBasedConvexProgramInstance<'g, 'd, 't, 'b, 'idx, 'p> {
    pub fn compute_initial_solution(&mut self) -> PathBasedSolution {
        let edge_costs: Vec<f64> = (0..self.graph.num_edges())
            .map(|edge_idx| BMWFunction::derivative(&self.graph.edge(edge_idx).params, 0.0))
            .collect::<Vec<_>>();
        let permit_costs = (0..self.graph.num_permits())
            .map(|permit_idx| BMWFunction::derivative(&self.graph.permit(permit_idx).params, 0.0))
            .collect::<Vec<_>>();
        self.compute_shortest_path_flow(&edge_costs, &permit_costs)
    }

    pub fn compute_shortest_path_flow(
        &mut self,
        edge_costs: &Vec<Float>,
        permit_costs: &Vec<Float>,
    ) -> PathBasedSolution {
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

        let init_flows = || {
            (
                vec![MySumAccumulator::zero(); self.graph.num_edges()],
                vec![MySumAccumulator::zero(); self.graph.num_permits()],
                vec![],
            )
        };

        let flows_iterator = self.demand.par_iter_by_origin().for_each_with_thread_local(
            init_flows,
            |(&origin, commodity_indices), (edge_flow, permit_flow, paths)| {
                let mut tree = AStarTree::new(origin, self.astar_table);
                let costs = &costs;
                commodity_indices.iter().for_each(|&commodity_idx| {
                    let commodity = self.demand.get_commodity(commodity_idx);
                    let destination_idx = commodity.destination_idx;
                    let (_cost, path, bundle_idx) = tree
                        .compute_shortest_path(
                            self.graph,
                            self.demand,
                            costs,
                            destination_idx,
                            self.bundle_index,
                        )
                        .unwrap();
                    let path = Path::from_edges(path);
                    for edge_idx in path.edges() {
                        edge_flow[edge_idx] += commodity.demand;
                    }
                    if bundle_idx != BUNDLE_IDX_EMPTY {
                        let binding = self.bundle_index.read().unwrap();
                        let bundle = binding.get_payload(bundle_idx);
                        for permit_idx in bundle.permits() {
                            permit_flow[permit_idx] += commodity.demand;
                        }
                    }
                    paths.push((commodity_idx, path));
                });
            },
        );

        let (edge_flow, permit_flow, paths) = flows_iterator.par_bridge().reduce(
            init_flows,
            |acc, (edge_flow, permit_flow, paths)| {
                let (mut edge_flow_acc, mut permit_flow_acc, mut paths_acc) = acc;
                for (acc, flow) in edge_flow_acc.iter_mut().zip(edge_flow.into_iter()) {
                    *acc = acc.clone() + flow;
                }
                for (acc, flow) in permit_flow_acc.iter_mut().zip(permit_flow.into_iter()) {
                    *acc = acc.clone() + flow;
                }
                paths_acc.extend(paths);
                (edge_flow_acc, permit_flow_acc, paths_acc)
            },
        );

        let path_flow = paths
            .into_iter()
            .map(|(commodity_idx, path)| {
                let path_idx = self.path_index.transfer_element(path);
                (
                    (commodity_idx, path_idx),
                    self.demand.get_commodity(commodity_idx).demand,
                )
            })
            .collect::<HashMap<_, _>>();

        PathBasedSolution::from_vec(
            edge_flow.into_iter().map(|it| it.sum()).collect(),
            permit_flow.into_iter().map(|it| it.sum()).collect(),
            path_flow,
        )
    }
}

impl<'g, 'd, 't, 'b, 'idx, 'p> ConvexProgramInstance<PathBasedSolution>
    for PathBasedConvexProgramInstance<'g, 'd, 't, 'b, 'idx, 'p>
{
    fn directional_derivative(
        &mut self,
        at: &PathBasedSolution,
        direction: &PathBasedSolution,
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

    fn compute_objective(&mut self, solution: &PathBasedSolution) -> Float {
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
        &mut self,
        x: &PathBasedSolution,
    ) -> LinearizedSubProblemSolution<PathBasedSolution> {
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
        let diff = PathBasedSolution::from_linear_combination(&y, -1.0, x);
        let inner_product = edge_gradient_at_x
            .iter()
            .copied()
            .zip(diff.edge_flow().iter().copied())
            .chain(
                permit_gradient_at_x
                    .iter()
                    .copied()
                    .zip(diff.permit_flow().iter().copied()),
            )
            .dot_with_accumulator::<MyDotAccumulator>();
        LinearizedSubProblemSolution {
            solution: y,
            direction: diff,
            inner_product,
        }
    }
}
