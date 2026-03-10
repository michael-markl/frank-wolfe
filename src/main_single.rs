use std::sync::RwLock;

use accurate::traits::DotWithAccumulator;
use clap_derive::Parser;
use log::info;

use crate::{
    astar::AStarTable,
    bmw_function::BMWFunction,
    bundle_index::BundleIndex,
    common::{Float, MyDotAccumulator},
    demand::DemandOps,
    frank_wolfe::solve_convex_program,
    graph_ops::GraphOps,
    io::{self, csv::write_edge_flow_csv, sqlite::write_solution},
    path_based_convex_program::PathBasedConvexProgramInstance,
    path_index::PathIndex,
};

#[derive(Parser, Debug)]
pub struct SingleArgs {
    #[arg(long = "graph")]
    tntp_net: std::path::PathBuf,

    #[arg(long = "demand")]
    tntp_trips: std::path::PathBuf,

    #[arg(long = "min_per_time_unit")]
    min_per_time_unit: Option<Float>,

    #[arg(long = "km_per_distance_unit")]
    km_per_distance_unit: Option<Float>,

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

    #[arg(long = "out_flow_csv")]
    out_flow_csv: Option<std::path::PathBuf>,

    #[arg(long = "out_flow_sqlite")]
    out_flow_sqlite: Option<std::path::PathBuf>,

    #[arg(long = "with_paths")]
    with_paths: bool,
}

pub fn main_single(args: SingleArgs) {
    let ext_graph = io::read_graph(&args.tntp_net);
    let (demand, commodity_idx_by_id) = io::read_demand(&args.tntp_trips, &ext_graph);

    let mut graph = ext_graph.graph;
    let edge_idx_by_id = ext_graph.edge_idx_by_id;

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
    let mut astar_table = AStarTable::create(graph.num_nodes(), demand.num_destinations());
    astar_table.fill_table(&graph, &demand);

    let mut path_index = PathIndex::new();

    let mut instance = PathBasedConvexProgramInstance {
        graph: &graph,
        demand: &demand,
        astar_table: &astar_table,
        bundle_index: &bundle_index,
        path_index: &mut path_index,
    };

    let result = solve_convex_program(
        instance.compute_initial_solution(),
        &mut instance,
        args.rel_gap,
        args.max_iter,
    );

    if cfg!(debug_assertions) {
        result.solution.check_consistency(
            &path_index,
            &bundle_index.read().unwrap(),
            &demand,
            &graph,
        );
    }

    info!("Objective value: {:.6e}", result.objective_value);
    info!("Optimality gap: {:.6e}", result.optimality_gap);
    info!(
        "Rel. optimality gap: {:.6e}",
        result.relative_optimality_gap
    );
    info!("Iterations: {}", result.num_iterations);

    let total_travel_time = result
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
        .dot_with_accumulator::<MyDotAccumulator>();

    info!("Total Travel Time: {:.6e}", total_travel_time);

    if let Some(out_flow) = &args.out_flow_csv {
        write_edge_flow_csv(result.solution.edge_flow(), out_flow, &graph);
    }

    if let Some(out_flow_sqlite) = &args.out_flow_sqlite {
        write_solution(
            out_flow_sqlite,
            result.solution.edge_flow(),
            &graph,
            edge_idx_by_id.as_ref(),
            commodity_idx_by_id.as_ref(),
            Some((result.solution.path_flow(), &path_index)),
        );
    }
}
