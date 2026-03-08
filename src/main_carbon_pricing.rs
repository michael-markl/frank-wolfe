use std::sync::RwLock;

use accurate::traits::{DotWithAccumulator, ParallelSumWithAccumulator, SumWithAccumulator};
use clap_derive::Parser;
use log::{info, trace};
use rayon::iter::ParallelIterator;

use crate::{
    astar::AStarTable,
    bmw_function::BMWFunction,
    bundle_index::{Bundle, BundleIndex},
    col::{HashMap, map_new},
    common::{EdgeIdx, Float, MyDotAccumulator, MySumAccumulator, PermitIdx},
    demand::{Demand, DemandOps},
    edge_based_convex_program::EdgeBasedConvexProgramInstance,
    edge_based_solution::EdgeBasedSolution,
    frank_wolfe::{FrankWolfeResult, solve_convex_program},
    graph::{EdgeParams, Graph, LpfMode},
    graph_ops::GraphOps,
    io::{self},
    path_based_convex_program::PathBasedConvexProgramInstance,
    path_based_solution::PathBasedSolution,
    path_index::PathIndex,
};

#[derive(Parser, Debug)]
pub struct CarbonPricingArgs {
    #[arg(long = "graph")]
    graph: std::path::PathBuf,

    #[arg(long = "demand")]
    demand: std::path::PathBuf,

    #[arg(long = "min_per_time_unit")]
    min_per_time_unit: Option<Float>,

    #[arg(long = "km_per_distance_unit")]
    km_per_distance_unit: Option<Float>,

    #[arg(long = "cordon_edge_map")]
    cordon_edge_map: Option<std::path::PathBuf>,

    #[arg(long = "permit_based", default_value_t = false)]
    permit_based: bool,

    #[arg(long = "min_price")]
    min_price: Float,

    #[arg(long = "max_price")]
    max_price: Float,

    #[arg(long = "steps")]
    steps: usize,

    #[arg(
        long = "reuse_solution",
        default_value_t = true,
        help = "Whether to reuse the solution from the previous price as the initial solution for the next price. \
                Speeds up computation, but may lead to some noticeable artifacts if accuracy is low."
    )]
    reuse_solution: bool,

    #[arg(
        long = "rel_gap",
        default_value_t = 1e-6,
        help = "Desired relative optimality of the Frank-Wolfe algorithm gap for each price."
    )]
    rel_gap: Float,

    #[arg(
        long = "max_iter",
        default_value_t = 20000,
        help = "Maximum number of iterations of the Frank-Wolfe algorithm for each price."
    )]
    max_iter: usize,

    #[arg(long = "out_csv")]
    csv_output_path: Option<std::path::PathBuf>,

    #[arg(long = "out_flow_template")]
    flow_output_path: Option<std::path::PathBuf>,

    #[arg(long = "with_paths", default_value_t = false)]
    with_paths: bool,
}

pub trait TollsStrategy {
    fn set_tolls(&self, graph: &mut Graph, price: Float);
}

impl TollsStrategy for Box<dyn TollsStrategy> {
    fn set_tolls(&self, graph: &mut Graph, price: Float) {
        (**self).set_tolls(graph, price);
    }
}

pub fn main_carbon_pricing(args: CarbonPricingArgs) {
    assert!(args.min_price <= args.max_price);
    assert!(args.steps > 0);
    assert!(
        (args.steps == 1) == (args.min_price == args.max_price),
        "Steps must be 1 if and only if min_price equals max_price"
    );

    let ext_graph = io::read_graph(&args.graph);
    let (demand, _commodity_idx_by_id) = io::read_demand(&args.demand, &ext_graph);

    let mut graph = ext_graph.graph;

    if let Some(min_per_time_unit) = args.min_per_time_unit {
        for edge_idx in 0..graph.num_edges() {
            let edge = graph.edge_mut(edge_idx);
            edge.params.ff_time *= min_per_time_unit;
        }
    }

    if let Some(km_per_distance_unit) = args.km_per_distance_unit {
        for edge_idx in 0..graph.num_edges() {
            let edge = graph.edge_mut(edge_idx);
            edge.params.length *= km_per_distance_unit;
        }
    }

    let bundle_index = RwLock::new(BundleIndex::new());

    let cordon_pricing_map = args.cordon_edge_map.map(CordonPricingMap::from_csv);

    let tolls_strategy: Box<dyn TollsStrategy> =
        if let Some(cordon_pricing_map) = &cordon_pricing_map {
            if !args.permit_based {
                struct EdgeBasedCordonPricing {
                    map: CordonPricingMap,
                }

                impl TollsStrategy for EdgeBasedCordonPricing {
                    fn set_tolls(&self, graph: &mut Graph, price: Float) {
                        for edge_idx in 0..graph.num_edges() {
                            let edge = graph.edge_mut(edge_idx);
                            let cordon_edge = self.map.for_edge(edge_idx);
                            edge.params.toll = if cordon_edge.leads_inside { price } else { 0.0 }
                        }
                    }
                }
                Box::new(EdgeBasedCordonPricing {
                    map: cordon_pricing_map.clone(),
                })
            } else {
                let permit_idx = graph.add_permit(EdgeParams {
                    mode: LpfMode::C,
                    ff_time: 0.0,
                    beta: 0.0,
                    capacity: 999999.0,
                    length: 0.0,
                    toll: 0.0,
                    offset: 0.0,
                });
                let bundle_idx = bundle_index
                    .write()
                    .unwrap()
                    .transfer_element(Bundle::from_permits(vec![permit_idx]));
                for edge_idx in 0..graph.num_edges() {
                    let edge = graph.edge_mut(edge_idx);
                    let cordon_edge = cordon_pricing_map.for_edge(edge_idx);
                    if cordon_edge.inside {
                        edge.bundle = bundle_idx;
                    }
                }

                pub fn minimum_demand_using_permit(
                    graph: &Graph,
                    demand: &Demand,
                    bundle_index: &RwLock<BundleIndex>,
                    permit_idx: usize,
                ) -> Float {
                    demand
                        .par_iter_by_origin()
                        .map(|(&origin, commodities)| {
                            // Find all nodes reachable from origin using only edges that lie outside the cordon
                            let reachable = {
                                let mut visited = map_new();
                                let mut stack = vec![origin];
                                while let Some(node_idx) = stack.pop() {
                                    if visited.contains_key(&node_idx) {
                                        continue;
                                    }
                                    visited.insert(node_idx, true);
                                    for edge_idx in graph.outgoing_edges(node_idx) {
                                        let edge = graph.edge(edge_idx);
                                        let is_cordon_edge = bundle_index
                                            .read()
                                            .unwrap()
                                            .get_payload(edge.bundle)
                                            .permits()
                                            .any(|p_idx| p_idx == permit_idx);
                                        if !is_cordon_edge {
                                            stack.push(edge.head);
                                        }
                                    }
                                }
                                visited
                            };

                            commodities
                                .iter()
                                .filter(|&&commodity_idx| {
                                    let commodity = demand.get_commodity(commodity_idx);
                                    let dest_node_idx =
                                        demand.node_idx_by_destination(commodity.destination_idx);
                                    !reachable.contains_key(&dest_node_idx)
                                })
                                .map(|&commodity_idx| demand.get_commodity(commodity_idx).demand)
                                .sum_with_accumulator::<MySumAccumulator>()
                        })
                        .parallel_sum_with_accumulator::<MySumAccumulator>()
                }

                info!(
                    "Minimum demand using permit: {:.6e}",
                    minimum_demand_using_permit(&graph, &demand, &bundle_index, permit_idx)
                );

                struct PermitBasedCordonPricing {
                    permit_idx: PermitIdx,
                }

                impl TollsStrategy for PermitBasedCordonPricing {
                    fn set_tolls(&self, graph: &mut Graph, price: Float) {
                        graph.permit_mut(self.permit_idx).params.toll = price;
                    }
                }

                Box::new(PermitBasedCordonPricing { permit_idx })
            }
        } else {
            Box::new(CarbonPricing {})
        };

    let mut csv_writer = if let Some(csv_output_path) = &args.csv_output_path {
        if csv_output_path.exists() {
            panic!(
                "CSV output file '{}' already exists. Please remove it or choose a different path.",
                csv_output_path.display()
            );
        }
        let mut wtr = csv::Writer::from_path(csv_output_path).unwrap();
        wtr.write_record([
            "iteration",
            "price",
            "total_travel_time",
            "total_user_cost",
            "total_consumption",
            "total_consumption_inside",
            "total_entrances",
            "total_permit_flow",
            "num_iterations",
            "objective_value",
            "gap",
            "relative_gap",
        ])
        .unwrap();
        wtr.flush().unwrap();
        Some(wtr)
    } else {
        None
    };

    if args.with_paths {
        let mut path_index = PathIndex::new();
        compute_solutions_for_price_range_with_paths(
            &mut graph,
            &demand,
            &bundle_index,
            &mut path_index,
            (args.min_price, args.max_price),
            args.steps,
            tolls_strategy,
            args.rel_gap,
            args.max_iter,
            args.reuse_solution,
            |step, price, result, graph| {
                handle_step_output(
                    step,
                    price,
                    result,
                    graph,
                    result.solution.edge_flow(),
                    result.solution.permit_flow(),
                    args.steps,
                    args.flow_output_path.as_ref(),
                    cordon_pricing_map.as_ref(),
                    &mut csv_writer,
                );
            },
        );
    } else {
        compute_solutions_for_price_range(
            &mut graph,
            &demand,
            &bundle_index,
            (args.min_price, args.max_price),
            args.steps,
            tolls_strategy,
            args.rel_gap,
            args.max_iter,
            args.reuse_solution,
            |step, price, result, graph| {
                demand.check_solution(&result.solution, graph);

                handle_step_output(
                    step,
                    price,
                    result,
                    graph,
                    result.solution.edge_flow(),
                    result.solution.permit_flow(),
                    args.steps,
                    args.flow_output_path.as_ref(),
                    cordon_pricing_map.as_ref(),
                    &mut csv_writer,
                );
            },
        );
    }
}

fn write_flow_csv(edge_flows: &[Float], flow_csv_path: &std::path::PathBuf, graph: &Graph) {
    let mut wtr = csv::Writer::from_path(flow_csv_path).expect("Failed to create flow CSV writer");

    wtr.write_record([
        "edge_id",
        "flow",
        "adjusted-length",
        "capacity",
        "utilization",
        "travel_time_per_unit",
    ])
    .expect("Failed to write header");

    for edge_idx in 0..graph.num_edges() {
        let edge = graph.edge(edge_idx);
        let flow = edge_flows[edge_idx];
        let length = edge.params.length;
        let capacity = edge.params.capacity;
        let utilization = flow / capacity;
        let travel_time_per_unit = BMWFunction::derivative(&edge.params, flow);

        wtr.write_record(&[
            edge_idx.to_string(),
            flow.to_string(),
            length.to_string(),
            capacity.to_string(),
            utilization.to_string(),
            travel_time_per_unit.to_string(),
        ])
        .expect("Failed to write flow record");
    }

    wtr.flush().expect("Failed to flush CSV writer");
}

fn flow_output_path_for_step(
    flow_output_path_template: &std::path::PathBuf,
    steps: usize,
    step: usize,
) -> std::path::PathBuf {
    if steps == 1 {
        flow_output_path_template.with_added_extension("csv")
    } else {
        flow_output_path_template.with_added_extension(format!("{:03}.csv", step))
    }
}

fn handle_step_output<Solution>(
    step: usize,
    price: Float,
    result: &FrankWolfeResult<Solution>,
    graph: &Graph,
    edge_flows: &[Float],
    permit_flows: &[Float],
    steps: usize,
    flow_output_path_template: Option<&std::path::PathBuf>,
    cordon_pricing_map: Option<&CordonPricingMap>,
    csv_writer: &mut Option<csv::Writer<std::fs::File>>,
) {
    if let Some(flow_output_path_template) = flow_output_path_template {
        let flow_csv_path = flow_output_path_for_step(flow_output_path_template, steps, step);
        write_flow_csv(edge_flows, &flow_csv_path, graph);
    }

    let total_travel_time = edge_flows
        .iter()
        .copied()
        .enumerate()
        .map(|(edge_idx, it)| {
            let p = &graph.edge(edge_idx).params;
            ((BMWFunction::derivative(p, it) - p.toll), it)
        })
        .dot_with_accumulator::<MyDotAccumulator>();

    let total_user_cost = edge_flows
        .iter()
        .copied()
        .enumerate()
        .map(|(edge_idx, it)| (BMWFunction::derivative(&graph.edge(edge_idx).params, it), it))
        .chain(permit_flows.iter().copied().enumerate().map(|(permit_idx, it)| {
            (
                BMWFunction::derivative(&graph.permit(permit_idx).params, it),
                it,
            )
        }))
        .dot_with_accumulator::<MyDotAccumulator>();

    let total_consumption = edge_flows
        .iter()
        .copied()
        .enumerate()
        .map(|(edge_idx, it)| (graph.edge(edge_idx).params.length, it))
        .dot_with_accumulator::<MyDotAccumulator>();

    let consumption_inside = cordon_pricing_map.map(|map| {
        edge_flows
            .iter()
            .copied()
            .enumerate()
            .filter(|(edge_idx, _)| map.for_edge(*edge_idx).inside)
            .map(|(edge_idx, it)| (graph.edge(edge_idx).params.length, it))
            .dot_with_accumulator::<MyDotAccumulator>()
    });

    let total_entrances = cordon_pricing_map.map(|map| {
        edge_flows
            .iter()
            .copied()
            .enumerate()
            .filter(|(edge_idx, _)| map.for_edge(*edge_idx).leads_inside)
            .map(|(_, it)| it)
            .sum_with_accumulator::<MySumAccumulator>()
    });

    let total_permit_flow = cordon_pricing_map.map(|_| {
        permit_flows
            .iter()
            .copied()
            .sum_with_accumulator::<MySumAccumulator>()
    });

    trace!(
        "Step {}: Price = {:.6e}, Total travel time = {:.6e}, Total user cost = {:.6e}, Total consumption = {:.6e}",
        step, price, total_travel_time, total_user_cost, total_consumption
    );

    csv_writer.iter_mut().for_each(|wtr| {
        wtr.write_record(&[
            step.to_string(),
            price.to_string(),
            total_travel_time.to_string(),
            total_user_cost.to_string(),
            total_consumption.to_string(),
            consumption_inside.map_or("".to_string(), |v| v.to_string()),
            total_entrances.map_or("".to_string(), |v| v.to_string()),
            total_permit_flow.map_or("".to_string(), |v| v.to_string()),
            result.num_iterations.to_string(),
            result.objective_value.to_string(),
            result.optimality_gap.to_string(),
            result.relative_optimality_gap.to_string(),
        ])
        .unwrap();
        wtr.flush().unwrap();
    });
}

pub fn compute_solutions_for_price_range(
    graph: &mut Graph,
    demand: &Demand,
    bundle_index: &RwLock<BundleIndex>,
    price_range: (Float, Float),
    steps: usize,
    tolls_strategy: impl TollsStrategy,
    rel_gap: Float,
    max_iter: usize,
    reuse_solution: bool,
    mut on_step: impl FnMut(usize, Float, &FrankWolfeResult<EdgeBasedSolution>, &Graph),
) {
    compute_solutions_for_price_range_generic(
        graph,
        demand,
        price_range,
        steps,
        tolls_strategy,
        |graph, demand, astar_table, previous_solution| {
            let mut instance = EdgeBasedConvexProgramInstance {
                graph,
                demand,
                astar_table,
                bundle_index,
            };

            let initial_solution = if reuse_solution {
                previous_solution.unwrap_or_else(|| instance.compute_initial_solution())
            } else {
                instance.compute_initial_solution()
            };

            solve_convex_program(initial_solution, &mut instance, rel_gap, max_iter)
        },
        |step, price, result, graph| on_step(step, price, result, graph),
    );
}

pub fn compute_solutions_for_price_range_with_paths(
    graph: &mut Graph,
    demand: &Demand,
    bundle_index: &RwLock<BundleIndex>,
    path_index: &mut PathIndex,
    price_range: (Float, Float),
    steps: usize,
    tolls_strategy: impl TollsStrategy,
    rel_gap: Float,
    max_iter: usize,
    reuse_solution: bool,
    mut on_step: impl FnMut(usize, Float, &FrankWolfeResult<PathBasedSolution>, &Graph),
) {
    compute_solutions_for_price_range_generic(
        graph,
        demand,
        price_range,
        steps,
        tolls_strategy,
        |graph, demand, astar_table, previous_solution| {
            let mut instance = PathBasedConvexProgramInstance {
                graph,
                demand,
                astar_table,
                bundle_index,
                path_index,
            };

            let initial_solution = if reuse_solution {
                previous_solution.unwrap_or_else(|| instance.compute_initial_solution())
            } else {
                instance.compute_initial_solution()
            };

            let result = solve_convex_program(initial_solution, &mut instance, rel_gap, max_iter);

            if cfg!(debug_assertions) {
                result.solution.check_consistency(
                    path_index,
                    &bundle_index.read().unwrap(),
                    demand,
                    graph,
                );
            }

            result
        },
        |step, price, result, graph| on_step(step, price, result, graph),
    );
}

fn compute_solutions_for_price_range_generic<Solution>(
    graph: &mut Graph,
    demand: &Demand,
    price_range: (Float, Float),
    steps: usize,
    tolls_strategy: impl TollsStrategy,
    mut solve_step: impl FnMut(
        &Graph,
        &Demand,
        &AStarTable,
        Option<Solution>,
    ) -> FrankWolfeResult<Solution>,
    mut on_step: impl FnMut(usize, Float, &FrankWolfeResult<Solution>, &Graph),
) {
    let mut solution: Option<Solution> = None;

    let mut astar_table = AStarTable::create(graph.num_nodes(), demand.num_destinations());
    astar_table.fill_table(graph, demand);

    for step in 0..steps {
        let price = if steps == 1 {
            price_range.0
        } else {
            price_range.0
                + (price_range.1 - price_range.0) * (step as Float) / ((steps - 1) as Float)
        };
        trace!("Step {}: Price = {:.6e}", step, price);
        tolls_strategy.set_tolls(graph, price);

        let result = solve_step(graph, demand, &astar_table, solution.take());

        on_step(step, price, &result, graph);
        solution = Some(result.solution);
    }
}

struct CarbonPricing {}

impl TollsStrategy for CarbonPricing {
    fn set_tolls(&self, graph: &mut Graph, price: Float) {
        for edge_idx in 0..graph.num_edges() {
            let edge = graph.edge_mut(edge_idx);
            edge.params.toll = price * edge.params.length;
        }
    }
}

#[derive(Clone, Copy)]
struct CordonEdge {
    inside: bool,
    leads_inside: bool,
}

#[derive(Clone)]
struct CordonPricingMap {
    map: HashMap<EdgeIdx, CordonEdge>,
}

impl CordonPricingMap {
    pub fn for_edge(&self, edge_idx: EdgeIdx) -> CordonEdge {
        *self.map.get(&edge_idx).unwrap_or(&CordonEdge {
            inside: false,
            leads_inside: false,
        })
    }

    pub fn from_csv(path: std::path::PathBuf) -> Self {
        let mut map = map_new();
        let mut rdr = csv::Reader::from_path(path).unwrap();
        let headers = rdr.headers().unwrap();
        let edge_id_col = headers.iter().position(|h| h == "edge_id").unwrap();
        let leads_inside_col = headers.iter().position(|h| h == "leads_inside").unwrap();
        let lies_inside_col = headers.iter().position(|h| h == "lies_inside").unwrap();
        for result in rdr.records() {
            let record = result.unwrap();
            let edge_idx: EdgeIdx = record[edge_id_col].parse().unwrap();
            let inside: bool = record[lies_inside_col].parse::<usize>().unwrap() > 0;
            let leads_inside: bool = record[leads_inside_col].parse::<usize>().unwrap() > 0;
            map.insert(
                edge_idx,
                CordonEdge {
                    inside,
                    leads_inside,
                },
            );
        }
        CordonPricingMap { map }
    }
}
