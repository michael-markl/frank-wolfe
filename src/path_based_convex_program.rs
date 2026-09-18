use std::sync::RwLock;

use accurate::dot::traits::DotWithAccumulator;
use accurate::traits::{SumAccumulator, SumWithAccumulator};
use itertools::Either;
use log::{debug, info};
use rayon::iter::{IntoParallelRefIterator, ParallelBridge, ParallelIterator};

use crate::astar_tree::ShortestPathResult;
use crate::bundle_index::Bundle;
use crate::col::{HashMap, map_new};
use crate::common::{
    BUNDLE_IDX_EMPTY, BundleIdx, CommodityIdx, MyDotAccumulator, MySumAccumulator, PathIdx,
};
use crate::edge_based_solution::EdgeBasedSolution;
use crate::frank_wolfe::{FrankWolfeResult, frank_wolfe_step};
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

pub struct PathBasedConvexProgramInstance<'g, 'd, 't, 'b, 'bi, 'idx, 'p> {
    pub graph: &'g Graph,
    pub demand: &'d Demand,
    pub astar_table: &'t AStarTable,
    pub bundle_index: &'b RwLock<BundleIndex<'bi>>,
    pub path_index: &'p mut PathIndex<'idx>,
}

pub fn compute_path_cost(
    path: &Path,
    costs: &impl CostValuesOps,
    bundle_index: &BundleIndex,
    graph: &impl GraphOps,
) -> Float {
    let mut cost = MySumAccumulator::zero();
    let mut current_bundle = Bundle::empty();
    for edge_idx in path.edges() {
        cost += costs.get_edge_cost(edge_idx);
        let edge_bundle_idx = graph.edge_bundle(edge_idx);
        if edge_bundle_idx != BUNDLE_IDX_EMPTY {
            let edge_bundle = bundle_index.get_payload(edge_bundle_idx);
            for permit_idx in edge_bundle.set_minus_iter(&current_bundle) {
                cost += costs.get_permit_cost(*permit_idx);
            }
            current_bundle = current_bundle.into_union(edge_bundle);
        }
    }
    cost.sum()
}

pub fn get_paths_by_commodity<Costs: CostValuesOps + Sync, Graph: GraphOps + Sync>(
    path_flow: &HashMap<(CommodityIdx, PathIdx), Float>,
    path_index: &PathIndex,
    costs: &Costs,
    bundle_index: &BundleIndex,
    graph: &Graph,
) -> HashMap<CommodityIdx, Vec<(PathIdx, Float, Float)>> {
    let paths_by_commodity: Vec<_> = path_flow
        .par_iter()
        .map(|(&(commodity_idx, path_idx), &flow)| {
            let path = path_index.get_payload(path_idx);
            let cost = compute_path_cost(path, costs, bundle_index, graph);
            (commodity_idx, (path_idx, cost, flow))
        })
        .collect();
    paths_by_commodity
        .into_iter()
        .fold(map_new(), |mut acc, (commodity_idx, path_info)| {
            acc.entry(commodity_idx).or_insert(vec![]).push(path_info);
            acc
        })
}

struct Costs<'a>(&'a Vec<Float>, &'a Vec<Float>);

impl CostValuesOps for Costs<'_> {
    fn get_edge_cost(&self, edge_idx: EdgeIdx) -> Float {
        self.0[edge_idx]
    }

    fn get_permit_cost(&self, permit_idx: common::PermitIdx) -> Float {
        self.1[permit_idx]
    }
}

pub fn compute_bmw_gradient_from_solution(
    solution: &PathBasedSolution,
    graph: &Graph,
) -> (Vec<Float>, Vec<Float>) {
    let edge_costs = (0..graph.num_edges())
        .map(|edge_idx| {
            BMWFunction::derivative(&graph.edge(edge_idx).params, solution.edge_flow()[edge_idx])
        })
        .collect::<Vec<_>>();
    let permit_costs = (0..graph.num_permits())
        .map(|permit_idx| {
            BMWFunction::derivative(
                &graph.permit(permit_idx).params,
                solution.permit_flow()[permit_idx],
            )
        })
        .collect::<Vec<_>>();
    (edge_costs, permit_costs)
}

pub trait AddVector {
    fn add_vector(&mut self, other: Self);
}

impl AddVector for Vec<MySumAccumulator> {
    fn add_vector(&mut self, other: Self) {
        assert!(self.len() == other.len());
        for (acc, flow) in self.iter_mut().zip(other.into_iter()) {
            *acc = acc.clone() + flow;
        }
    }
}

struct EdgeFlowAccumulator {
    edge_flow: Vec<MySumAccumulator>,
    permit_flow: Vec<MySumAccumulator>,
}

impl EdgeFlowAccumulator {
    fn new(num_edges: usize, num_permits: usize) -> Self {
        Self {
            edge_flow: vec![MySumAccumulator::zero(); num_edges],
            permit_flow: vec![MySumAccumulator::zero(); num_permits],
        }
    }

    fn add_flows(&mut self, edge_flow: Vec<MySumAccumulator>, permit_flow: Vec<MySumAccumulator>) {
        self.edge_flow.add_vector(edge_flow);
        self.permit_flow.add_vector(permit_flow);
    }

    fn into_flows(self) -> (Vec<Float>, Vec<Float>) {
        (
            self.edge_flow.into_iter().map(|acc| acc.sum()).collect(),
            self.permit_flow.into_iter().map(|acc| acc.sum()).collect(),
        )
    }

    fn add(&mut self, other: Self) {
        self.add_flows(other.edge_flow, other.permit_flow);
    }

    fn add_path(
        &mut self,
        path: &Path,
        flow: Float,
        graph: &impl GraphOps,
        bundle_index: &RwLock<BundleIndex>,
    ) {
        let mut current_bundle = Bundle::empty();
        let mut binding = None;
        for edge_idx in path.edges() {
            self.edge_flow[edge_idx] += flow;
            let edge_bundle_idx = graph.edge_bundle(edge_idx);
            if edge_bundle_idx != BUNDLE_IDX_EMPTY {
                let binding = binding.get_or_insert_with(|| bundle_index.read().unwrap());
                let edge_bundle = binding.get_payload(edge_bundle_idx);
                for permit_idx in edge_bundle.set_minus_iter(&current_bundle) {
                    self.permit_flow[*permit_idx] += flow;
                }
                current_bundle = current_bundle.into_union(edge_bundle);
            }
        }
    }

    fn add_path_bundle(
        &mut self,
        path: &Path,
        bundle_idx: BundleIdx,
        bundle_index: &RwLock<BundleIndex>,
        flow: Float,
    ) {
        for edge_idx in path.edges() {
            self.edge_flow[edge_idx] += flow;
        }
        if bundle_idx != BUNDLE_IDX_EMPTY {
            let binding = bundle_index.read().unwrap();
            let bundle = binding.get_payload(bundle_idx);
            for permit_idx in bundle.permits() {
                self.permit_flow[permit_idx] += flow;
            }
        }
    }
}

impl<'g, 'd, 't, 'b, 'bi, 'idx, 'p> PathBasedConvexProgramInstance<'g, 'd, 't, 'b, 'bi, 'idx, 'p> {
    pub fn compute_initial_solution(&mut self) -> PathBasedSolution {
        let edge_costs: Vec<f64> = (0..self.graph.num_edges())
            .map(|edge_idx| BMWFunction::derivative(&self.graph.edge(edge_idx).params, 0.0))
            .collect::<Vec<_>>();
        let permit_costs = (0..self.graph.num_permits())
            .map(|permit_idx| BMWFunction::derivative(&self.graph.permit(permit_idx).params, 0.0))
            .collect::<Vec<_>>();
        self.compute_shortest_path_flow(&Costs(&edge_costs, &permit_costs))
    }

    pub fn compute_shortest_path_flow(
        &mut self,
        costs: &(impl CostValuesOps + Sync),
    ) -> PathBasedSolution {
        let init_flows = || {
            (
                EdgeFlowAccumulator::new(self.graph.num_edges(), self.graph.num_permits()),
                vec![],
            )
        };

        let flows_iterator = self.demand.par_iter_by_origin().for_each_with_thread_local(
            init_flows,
            |(&origin, commodity_indices), (flow_acc, paths)| {
                let mut tree = AStarTree::new(origin, self.astar_table);
                commodity_indices.iter().for_each(|&commodity_idx| {
                    let commodity = self.demand.get_commodity(commodity_idx);
                    let destination_idx = commodity.destination_idx;
                    let ShortestPathResult {
                        distance: _cost,
                        path,
                        bundle_idx,
                    } = tree
                        .compute_shortest_path(
                            self.graph,
                            self.demand,
                            costs,
                            destination_idx,
                            self.bundle_index,
                        )
                        .unwrap();
                    let path = Path::from_edges(path);
                    flow_acc.add_path_bundle(
                        &path,
                        bundle_idx,
                        self.bundle_index,
                        commodity.demand,
                    );
                    paths.push((commodity_idx, path));
                });
            },
        );

        let (flow_acc, paths) =
            flows_iterator
                .par_bridge()
                .reduce(init_flows, |acc, (flow, paths)| {
                    let (mut flow_acc, mut paths_acc) = acc;
                    flow_acc.add(flow);
                    paths_acc.extend(paths);
                    (flow_acc, paths_acc)
                });

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

        let (edge_flow, permit_flow) = flow_acc.into_flows();
        PathBasedSolution::from_vec(edge_flow, permit_flow, path_flow)
    }

    pub fn compute_flow_altering_only_high_regret_commodities(
        &mut self,
        x: &PathBasedSolution,
        costs_at_x: &(impl CostValuesOps + Sync),
        max_relative_regret_fraction: Float,
    ) -> (PathBasedSolution, Float, Float, Float, EdgeBasedSolution) {
        let paths_by_commodity = {
            let bundle_index = self.bundle_index.read().unwrap();
            get_paths_by_commodity(
                x.path_flow(),
                self.path_index,
                costs_at_x,
                &bundle_index,
                self.graph,
            )
        };

        let shortest_paths = {
            let paths: Vec<Vec<(CommodityIdx, (Float, Vec<EdgeIdx>, BundleIdx))>> = self
                .demand
                .par_iter_by_origin()
                .map(|(&origin_idx, commodities)| {
                    let mut tree = AStarTree::new(origin_idx, self.astar_table);
                    commodities
                        .iter()
                        .map(|&commodity_idx| {
                            let commodity = self.demand.get_commodity(commodity_idx);
                            let ShortestPathResult {
                                distance,
                                path,
                                bundle_idx,
                            } = tree
                                .compute_shortest_path(
                                    self.graph,
                                    self.demand,
                                    costs_at_x,
                                    commodity.destination_idx,
                                    self.bundle_index,
                                )
                                .expect("Destination is reachable.");
                            (commodity_idx, (distance, path, bundle_idx))
                        })
                        .collect()
                })
                .collect();
            paths.into_iter().flatten().collect::<HashMap<_, _>>()
        };

        let (max_relative_regret, weighted_relative_regret_sum, total_flow) = paths_by_commodity
            .iter()
            .fold((0.0, 0.0, 0.0), |acc, (&commodity_idx, paths)| {
                let (mut max_regret, mut weighted_sum, mut flow_sum) = acc;
                let distance = shortest_paths
                    .get(&commodity_idx)
                    .map(|(distance, _, _)| *distance)
                    .expect("Shortest distance missing for commodity.");
                for (_, cost, flow) in paths.iter().filter(|(_, _, flow)| *flow > 0.0) {
                    let relative_regret = if distance > 0.0 {
                        *cost / distance - 1.0
                    } else {
                        0.0
                    };
                    max_regret = partial_max(max_regret, relative_regret);
                    weighted_sum += relative_regret * *flow;
                    flow_sum += *flow;
                }
                (max_regret, weighted_sum, flow_sum)
            })
            ;
        let mean_regret = if total_flow > 0.0 {
            weighted_relative_regret_sum / total_flow
        } else {
            0.0
        };
        let high_regret_threshold = max_relative_regret_fraction * max_relative_regret;

        let init_flows = || {
            (
                EdgeFlowAccumulator::new(self.graph.num_edges(), self.graph.num_permits()),
                EdgeFlowAccumulator::new(self.graph.num_edges(), self.graph.num_permits()),
                vec![],
                1.0,
            )
        };

        let flow_iter = self.demand.par_iter_by_origin().for_each_with_thread_local(
            init_flows,
            |(_, commodities), (shortest_flow, reduced_flow, new_paths, approximation)| {
                commodities.iter().for_each(|&commodity_idx| {
                    let commodity = self.demand.get_commodity(commodity_idx);
                    let paths = paths_by_commodity.get(&commodity_idx);
                    if paths.is_none() {
                        return;
                    }
                    let paths = paths.unwrap();
                    let (distance, shortest_path_edges, bundle_idx) = shortest_paths
                        .get(&commodity_idx)
                        .expect("Shortest path missing for commodity.");
                    let distance = *distance;
                    let path = Path::from_edges(shortest_path_edges.clone());

                    shortest_flow.add_path_bundle(
                        &path,
                        *bundle_idx,
                        self.bundle_index,
                        commodity.demand,
                    );

                    // For the reduced flow, keep paths whose relative regret is at most
                    // max_relative_regret_fraction of the global maximum relative regret.
                    {
                        // Update approximation
                        let distance_reciprocal = if distance > 0.0 { 1.0 / distance } else { 0.0 };
                        let flow_eps = 1e-8 * commodity.demand;
                        for (_, cost, flow) in paths.iter() {
                            let path_approx = cost * distance_reciprocal;
                            if *flow > flow_eps && path_approx > *approximation {
                                *approximation = path_approx;
                            }
                        }
                    }

                    let flow_below_target = paths
                        .iter()
                        .filter(|(_, cost, _)| {
                            let relative_regret = if distance > 0.0 {
                                *cost / distance - 1.0
                            } else {
                                0.0
                            };
                            relative_regret <= high_regret_threshold
                        })
                        .map(|(_, _, flow)| *flow)
                        .sum_with_accumulator::<MySumAccumulator>();
                    let new_path_flow = partial_max(0.0, commodity.demand - flow_below_target);

                    paths
                        .iter()
                        .filter(|(_, cost, _)| {
                            let relative_regret = if distance > 0.0 {
                                *cost / distance - 1.0
                            } else {
                                0.0
                            };
                            relative_regret <= high_regret_threshold
                        })
                        .for_each(|(path_idx, _, flow)| {
                            let path = self.path_index.get_payload(*path_idx);
                            reduced_flow.add_path(path, *flow, self.graph, self.bundle_index);
                            new_paths.push((commodity_idx, Either::Left(*path_idx), *flow));
                        });

                    if new_path_flow > 0.0 {
                        reduced_flow.add_path_bundle(
                            &path,
                            *bundle_idx,
                            self.bundle_index,
                            new_path_flow,
                        );
                        new_paths.push((commodity_idx, Either::Right(path), new_path_flow));
                    }
                });
            },
        );

        let (shortest_flow, reduced_flow, new_paths, approximation) =
            flow_iter.par_bridge().reduce(
                init_flows,
                |acc, (shortest_flow, reduced_flow, paths, approx)| {
                    let (
                        mut shortest_flow_acc,
                        mut reduced_flow_acc,
                        mut paths_acc,
                        mut approx_acc,
                    ) = acc;
                    shortest_flow_acc.add(shortest_flow);
                    reduced_flow_acc.add(reduced_flow);
                    paths_acc.extend(paths);
                    approx_acc = partial_max(approx_acc, approx);
                    (shortest_flow_acc, reduced_flow_acc, paths_acc, approx_acc)
                },
            );

        let mut path_flow = map_new();
        for (commodity_idx, path_idx_or_box, flow) in new_paths.into_iter() {
            let path_idx = match path_idx_or_box {
                Either::Left(path_idx) => path_idx,
                Either::Right(path) => self.path_index.transfer_element(path),
            };
            *path_flow.entry((commodity_idx, path_idx)).or_insert(0.0) += flow;
        }

        let (edge_flow, permit_flow) = reduced_flow.into_flows();
        let new_solution = PathBasedSolution::from_vec(edge_flow, permit_flow, path_flow);

        let (edge_flow, permit_flow) = shortest_flow.into_flows();
        let shortest_flow = EdgeBasedSolution::from_vec(edge_flow, permit_flow);

        (
            new_solution,
            approximation,
            max_relative_regret,
            mean_regret,
            shortest_flow,
        )
    }

    pub fn remove_high_regret_paths(
        &mut self,
        result: FrankWolfeResult<PathBasedSolution>,
    ) -> FrankWolfeResult<PathBasedSolution> {
        self.remove_high_regret_paths_with_on_step(result, 0.5, |_, _, _, _, _, _| {})
    }

    pub fn remove_high_regret_paths_with_on_step(
        &mut self,
        mut result: FrankWolfeResult<PathBasedSolution>,
        max_relative_regret_fraction: Float,
        mut on_step: impl FnMut(usize, Float, Float, Float, Float, &FrankWolfeResult<PathBasedSolution>),
    ) -> FrankWolfeResult<PathBasedSolution> {
        assert!(
            (0.0..=1.0).contains(&max_relative_regret_fraction),
            "max_relative_regret_fraction must be in [0, 1]"
        );
        let desired_approximation: f64 = 1.0 + 2.0f64.powi(-10);
        let max_iterations = 300;
        let mut iteration = 0;

        info!("Removing high regret paths.");

        loop {
            if iteration >= max_iterations {
                info!("Reached maximum number of iterations when removing high regret paths.");
                return result;
            }
            iteration += 1;

            let (edge_costs, permit_costs) =
                compute_bmw_gradient_from_solution(&result.solution, self.graph);
            let costs = Costs(&edge_costs, &permit_costs);

            let (new_solution, approximation, max_regret, mean_regret, shortest_flow) =
                self.compute_flow_altering_only_high_regret_commodities(
                    &result.solution,
                    &costs,
                    max_relative_regret_fraction,
                );

            {
                // Compute new gap
                let direction_edges = shortest_flow
                    .edge_flow()
                    .iter()
                    .zip(result.solution.edge_flow().iter())
                    .map(|(&new, &old)| new - old)
                    .collect::<Vec<_>>();
                let direction_permits = shortest_flow
                    .permit_flow()
                    .iter()
                    .zip(result.solution.permit_flow().iter())
                    .map(|(&new, &old)| new - old)
                    .collect::<Vec<_>>();
                let inner_product =
                    directional_derivative(&costs, &direction_edges, &direction_permits);

                result.improve_gap(-inner_product);
            };

            if cfg!(debug_assertions) {
                new_solution.check_consistency(
                    self.path_index,
                    &self.bundle_index.read().unwrap(),
                    self.demand,
                    self.graph,
                );
            }

            let num_paths = new_solution.path_flow().len();
            debug!(
                "Iteration {}, approximation: {}, #paths: {}",
                iteration, approximation, num_paths,
            );

            let direction =
                PathBasedSolution::from_linear_combination(&new_solution, -1.0, &result.solution);
            let (new_result, step_size) = frank_wolfe_step(result, self, new_solution, &direction);
            result = new_result;
            on_step(
                iteration,
                step_size,
                max_regret,
                approximation,
                mean_regret,
                &result,
            );
            debug!(
                "After FW-step: objective value: {:.6e}, optimality gap: {:.6e}, relative gap: {:.6e}",
                result.objective_value, result.optimality_gap, result.relative_optimality_gap
            );

            if step_size == 0.0 {
                info!("Step size is 0.0 during high regret path removal. Stopping.");
                return result;
            }

            if approximation < desired_approximation {
                info!(
                    "Desired approximation reached ({} < {}). Stopping.",
                    approximation, desired_approximation
                );
                return result;
            }
        }
    }
}

fn partial_max(v1: f64, v2: f64) -> f64 {
    if v1 > v2 { v1 } else { v2 }
}

fn directional_derivative(
    costs: &impl CostValuesOps,
    direction_edges: &[Float],
    direction_permits: &[Float],
) -> Float {
    direction_edges
        .iter()
        .enumerate()
        .map(|(edge_idx, &it)| (costs.get_edge_cost(edge_idx), it))
        .chain(
            direction_permits
                .iter()
                .enumerate()
                .map(|(permit_idx, &it)| (costs.get_permit_cost(permit_idx), it)),
        )
        .dot_with_accumulator::<MyDotAccumulator>()
}

impl<'g, 'd, 't, 'b, 'bi, 'idx, 'p> ConvexProgramInstance<PathBasedSolution>
    for PathBasedConvexProgramInstance<'g, 'd, 't, 'b, 'bi, 'idx, 'p>
{
    fn directional_derivative(
        &mut self,
        at: &PathBasedSolution,
        direction: &PathBasedSolution,
    ) -> Float {
        struct DynCosts<'a, 'b> {
            graph: &'a Graph,
            solution: &'b PathBasedSolution,
        }
        impl CostValuesOps for DynCosts<'_, '_> {
            fn get_edge_cost(&self, edge_idx: EdgeIdx) -> Float {
                BMWFunction::derivative(
                    &self.graph.edge(edge_idx).params,
                    self.solution.edge_flow()[edge_idx],
                )
            }

            fn get_permit_cost(&self, permit_idx: common::PermitIdx) -> Float {
                BMWFunction::derivative(
                    &self.graph.permit(permit_idx).params,
                    self.solution.permit_flow()[permit_idx],
                )
            }
        }
        directional_derivative(
            &DynCosts {
                graph: self.graph,
                solution: at,
            },
            direction.edge_flow(),
            direction.permit_flow(),
        )
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
        let (edge_costs, permit_costs) = compute_bmw_gradient_from_solution(x, self.graph);
        let costs = Costs(&edge_costs, &permit_costs);

        if cfg!(debug_assertions) {       
            x.check_consistency(
                self.path_index,
                &self.bundle_index.read().unwrap(),
                self.demand,
                self.graph,
            );
        }

        let y = self.compute_shortest_path_flow(&costs);
        let diff = PathBasedSolution::from_linear_combination(&y, -1.0, x);
        let inner_product = edge_costs
            .iter()
            .copied()
            .zip(diff.edge_flow().iter().copied())
            .chain(
                permit_costs
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
