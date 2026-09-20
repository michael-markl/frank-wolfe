use std::{io::Write, sync::RwLock};

use accurate::traits::DotWithAccumulator;
use clap_derive::Parser;
use log::{error, info, trace};
use serde::Serialize;

use frank_wolfe::{
    common::{EdgeIdx, Float, MyDotAccumulator},
    io::{self},
    network::{
        bundle_index::BundleIndex,
        demand::{Demand, DemandOps},
        graph::Graph,
        graph_ops::GraphOps,
        path_index::PathIndex,
    },
    optimization::{
        bmw_function::BMWFunction,
        edge_based_solution::EdgeBasedSolution,
        frank_wolfe::{FrankWolfeResult, solve_convex_program},
        path_based_convex_program::PathBasedConvexProgramInstance,
        path_based_solution::PathBasedSolution,
    },
    routing::astar::AStarTable,
};

#[derive(Debug, Serialize)]
struct BudgetPricingCsvEntry {
    iteration: usize,
    price: Float,
    total_travel_time: Float,
    total_user_cost: Float,
    total_consumption: Float,
    num_iterations: usize,
    objective_value: Float,
    gap: Float,
    relative_gap: Float,
}

#[derive(Parser, Debug)]
pub struct BudgetPricingArgs {
    #[arg(long = "graph")]
    graph: std::path::PathBuf,

    #[arg(long = "demand")]
    demand: std::path::PathBuf,

    #[arg(long = "min_per_time_unit")]
    min_per_time_unit: Option<Float>,

    #[arg(long = "km_per_distance_unit")]
    km_per_distance_unit: Option<Float>,

    #[arg(long = "budget")]
    budget: Float,

    #[arg(long = "permit_based", default_value_t = false)]
    permit_based: bool,

    #[arg(long = "initial_price")]
    initial_price: Option<Float>,

    #[arg(long = "binary_search_steps", default_value_t = 10)]
    binary_search_steps: usize,

    #[arg(
        long = "reuse_solution",
        default_value_t = false,
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

    #[arg(long = "out_sqlite")]
    sqlite_output_path: Option<std::path::PathBuf>,
}

pub trait TollsStrategy {
    fn set_tolls(&self, graph: &mut Graph, price: Float);
}

impl TollsStrategy for Box<dyn TollsStrategy> {
    fn set_tolls(&self, graph: &mut Graph, price: Float) {
        (**self).set_tolls(graph, price);
    }
}

pub fn main_budget_pricing(args: BudgetPricingArgs) {
    let ext_graph = io::read_graph(&args.graph);
    let (demand, commodity_idx_by_id) = io::read_demand(&args.demand, &ext_graph);

    let mut graph = ext_graph.graph;
    assert!(
        (0..graph.num_edges()).all(|i| graph.edge(i).params.externality_linear == 0.0),
        "budget-pricing requires constant externalities: flow-dependent emissions need not be monotone in price; use carbon-pricing instead"
    );

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

    let tolls_strategy: Box<dyn TollsStrategy> = Box::new(CarbonPricing {});

    let mut csv_writer = if let Some(csv_output_path) = &args.csv_output_path {
        if csv_output_path.exists() {
            panic!(
                "CSV output file '{}' already exists. Please remove it or choose a different path.",
                csv_output_path.display()
            );
        }
        Some(csv::Writer::from_path(csv_output_path).unwrap())
    } else {
        None
    };

    if let Some(sqlite_output_path) = &args.sqlite_output_path
        && sqlite_output_path.exists()
    {
        error!(
            "SQLite output file '{}' already exists. Please remove it or choose a different path.",
            sqlite_output_path.display()
        );
        std::process::exit(1);
    }

    let mut path_index = PathIndex::new();

    let result = exp_search_for_budget(
        &mut graph,
        &demand,
        &bundle_index,
        args.budget,
        args.initial_price.unwrap_or(1.0),
        tolls_strategy,
        args.binary_search_steps,
        args.rel_gap,
        args.max_iter,
        args.reuse_solution,
        &mut path_index,
        |step, price, result, graph| {
            let total_travel_time = result
                .solution
                .edge_flow()
                .iter()
                .enumerate()
                .map(|(edge_idx, &it)| {
                    (
                        (BMWFunction::derivative(&graph.edge(edge_idx).params, it)
                            - graph.edge(edge_idx).params.toll),
                        it,
                    )
                })
                .dot_with_accumulator::<MyDotAccumulator>();

            let total_user_cost =
                result
                    .solution
                    .edge_flow()
                    .iter()
                    .enumerate()
                    .map(|(edge_idx, &it)| {
                        (
                            BMWFunction::derivative(&graph.edge(edge_idx).params, it),
                            it,
                        )
                    })
                    .chain(result.solution.permit_flow().iter().enumerate().map(
                        |(permit_idx, &it)| {
                            (
                                BMWFunction::derivative(&graph.permit(permit_idx).params, it),
                                it,
                            )
                        },
                    ))
                    .dot_with_accumulator::<MyDotAccumulator>();

            let total_consumption = result
                .solution
                .edge_flow()
                .iter()
                .enumerate()
                .map(|(edge_idx, &it)| (graph.edge(edge_idx).params.length, it))
                .dot_with_accumulator::<MyDotAccumulator>();

            trace!(
                "Step {}: Price = {:.6e}, Total travel time = {:.6e}, Total user cost = {:.6e}, Total consumption = {:.6e}",
                step, price, total_travel_time, total_user_cost, total_consumption
            );

            csv_writer.iter_mut().for_each(|wtr| {
                wtr.serialize(BudgetPricingCsvEntry {
                    iteration: step,
                    price,
                    total_travel_time,
                    total_user_cost,
                    total_consumption,
                    num_iterations: result.num_iterations,
                    objective_value: result.objective_value,
                    gap: result.optimality_gap,
                    relative_gap: result.relative_optimality_gap,
                })
                .unwrap();
                wtr.flush().unwrap();
            });
        },
    );

    if let Some(path) = args.sqlite_output_path {
        let solution = result.1;
        io::sqlite::write_solution(
            &path,
            solution.edge_flow(),
            &graph,
            ext_graph.edge_idx_by_id.as_ref(),
            commodity_idx_by_id.as_ref(),
            Some((solution.path_flow(), &path_index)),
        );
        let mut wrt = std::io::BufWriter::new(
            std::fs::File::create(path.with_added_extension("price.txt")).unwrap(),
        );
        writeln!(wrt, "{:}", result.0).unwrap();
        wrt.flush().unwrap();
    }
}

#[derive(Debug, Serialize)]
struct EdgeFlowCsvEntry {
    edge_id: EdgeIdx,
    flow: Float,
    #[serde(rename = "adjusted-length")]
    adjusted_length: Float,
    capacity: Float,
    utilization: Float,
    travel_time_per_unit: Float,
}

fn write_flow_csv(solution: &EdgeBasedSolution, flow_csv_path: &std::path::PathBuf, graph: &Graph) {
    let mut wtr = csv::Writer::from_path(flow_csv_path).expect("Failed to create flow CSV writer");

    let edge_flows = solution.edge_flow();
    for edge_idx in 0..graph.num_edges() {
        let edge = graph.edge(edge_idx);
        let flow = edge_flows[edge_idx];
        wtr.serialize(EdgeFlowCsvEntry {
            edge_id: edge_idx,
            flow,
            adjusted_length: edge.params.length,
            capacity: edge.params.capacity,
            utilization: flow / edge.params.capacity,
            travel_time_per_unit: BMWFunction::derivative(&edge.params, flow),
        })
        .expect("Failed to write flow record");
    }

    wtr.flush().expect("Failed to flush CSV writer");
}

pub fn exp_search_for_budget<'a>(
    graph: &mut Graph,
    demand: &Demand,
    bundle_index: &'a RwLock<BundleIndex<'a>>,
    budget: Float,
    initial_price: Float,
    tolls_strategy: impl TollsStrategy,
    binary_search_steps: usize,
    rel_gap: Float,
    max_iter: usize,
    reuse_solution: bool,
    path_index: &mut PathIndex,
    mut on_step: impl FnMut(usize, Float, &FrankWolfeResult<PathBasedSolution>, &Graph),
) -> (f64, PathBasedSolution) {
    let mut solution: Option<PathBasedSolution> = None;

    let mut astar_table = AStarTable::create(graph.num_nodes(), demand.num_destinations());
    astar_table.fill_table(graph, demand);

    info!("Exponential search until budget is fulfilled...");

    const MAX_STEPS_EXP_SEARCH: usize = 100;
    let mut step = 0;

    let mut price_lower_bound = 0.0;
    let price_upper_bound: f64;
    let solution_upper_bound: PathBasedSolution;

    loop {
        let price = if step == 0 {
            0.0
        } else {
            initial_price * 2f64.powi(step as i32 - 1)
        };

        trace!("Step {}: Price = {:.6e}", step, price);
        tolls_strategy.set_tolls(graph, price);

        let mut instance = PathBasedConvexProgramInstance {
            graph,
            demand,
            astar_table: &astar_table,
            bundle_index,
            path_index,
        };

        let initial_solution = if reuse_solution {
            solution
                .take()
                .unwrap_or_else(|| instance.compute_initial_solution())
        } else {
            instance.compute_initial_solution()
        };

        let result = solve_convex_program(
            initial_solution,
            &mut instance,
            rel_gap,
            max_iter,
            |_, _| {},
        );
        let total_consumption = result
            .solution
            .edge_flow()
            .iter()
            .enumerate()
            .map(|(edge_idx, &it)| (graph.edge(edge_idx).params.length, it))
            .dot_with_accumulator::<MyDotAccumulator>();

        on_step(step, price, &result, graph);

        if total_consumption <= budget {
            price_upper_bound = price;
            solution_upper_bound = result.solution.clone();
            solution = Some(result.solution);
            break;
        }

        price_lower_bound = price;

        solution = Some(result.solution);

        if step >= MAX_STEPS_EXP_SEARCH {
            panic!(
                "Exponential search did not find a price fulfilling the budget constraint after {} steps.",
                MAX_STEPS_EXP_SEARCH
            );
        }
        step += 1;
    }

    let mut price_upper_bound = price_upper_bound;
    let mut solution_upper_bound = solution_upper_bound;

    if price_lower_bound == price_upper_bound {
        assert!(price_lower_bound == 0.0);
        info!("Price 0 fulfills the budget constraint.");
        return (price_upper_bound, solution_upper_bound);
    }

    info!("Binary search for price fulfilling the budget constraint...");

    for binary_step in 0..binary_search_steps {
        let price = (price_lower_bound + price_upper_bound) / 2.0;

        trace!("Binary search step {}: Price = {:.6e}", binary_step, price);
        tolls_strategy.set_tolls(graph, price);

        let mut instance = PathBasedConvexProgramInstance {
            graph,
            demand,
            astar_table: &astar_table,
            bundle_index,
            path_index,
        };

        let initial_solution = if reuse_solution {
            solution
                .take()
                .unwrap_or_else(|| instance.compute_initial_solution())
        } else {
            instance.compute_initial_solution()
        };

        let result = solve_convex_program(
            initial_solution,
            &mut instance,
            rel_gap,
            max_iter,
            |_, _| {},
        );
        let result = instance.remove_high_regret_paths(result);
        let total_consumption = result
            .solution
            .edge_flow()
            .iter()
            .enumerate()
            .map(|(edge_idx, &it)| (graph.edge(edge_idx).params.length, it))
            .dot_with_accumulator::<MyDotAccumulator>();

        if total_consumption > budget {
            price_lower_bound = price;
        } else {
            price_upper_bound = price;
            solution_upper_bound = result.solution.clone();
        }

        on_step(step + binary_step + 1, price, &result, graph);
        solution = Some(result.solution);
    }

    (price_upper_bound, solution_upper_bound)
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
