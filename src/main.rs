use std::sync::RwLock;

use accurate::traits::{DotWithAccumulator, SumWithAccumulator};
use clap::{Parser, Subcommand};
use log::{error, info};

use crate::{
    astar::AStarTable,
    bmw_function::BMWFunction,
    bundle_index::BundleIndex,
    common::{MyDotAccumulator, MySumAccumulator},
    demand::DemandOps,
    edge_based_convex_program::EdgeBasedConvexProgramInstance,
    frank_wolfe::solve_convex_program,
    graph_ops::GraphOps,
    main_carbon_pricing::{CarbonPricingArgs, main_carbon_pricing},
};

mod astar;
mod astar_tree;
mod bmw_function;
mod bundle_index;
mod col;
mod common;
mod demand;
mod edge_based_convex_program;
mod edge_based_solution;
mod frank_wolfe;
mod graph;
mod graph_ops;
mod index;
mod iter;
mod main_carbon_pricing;
mod tntp;

#[derive(Parser)]
#[command(name = "frank-wolfe")]
#[command(about = "Frank-Wolfe traffic assignment experiments", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    CarbonPricing(CarbonPricingArgs),
    Test,
}

fn test() {
    let tntpnet = tntp::read_net_file(std::path::Path::new(
        "C:\\Users\\markl07\\git\\TransportationNetworks\\Berlin-Center\\berlin-center_net.tntp",
    ))
    .map_err(|err| error!("Error reading net file: {}", err))
    .unwrap();
    let demand= tntp::read_trips_file(std::path::Path::new("C:\\Users\\markl07\\git\\TransportationNetworks\\Berlin-Center\\berlin-center_trips.tntp"), &tntpnet)
        .map_err(|err| error!("Error reading trips file: {}", err))
        .unwrap();

    let graph = tntpnet.graph;
    let mut astar_table = AStarTable::create(graph.num_nodes(), demand.num_destinations());
    astar_table.fill_table(&graph, &demand);

    let bundle_index = RwLock::new(BundleIndex::new());

    let instance = EdgeBasedConvexProgramInstance {
        graph: &graph,
        demand: &demand,
        astar_table: &astar_table,
        bundle_index: &bundle_index,
    };

    let initial_solution = instance.compute_shortest_path_flow(
        &(0..graph.num_edges())
            .map(|edge_idx| graph.edge(edge_idx).params.ff_time)
            .collect::<Vec<_>>(),
        &vec![],
    );

    let result = solve_convex_program(initial_solution, instance);

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

    let total_demand = demand
        .commodities()
        .iter()
        .map(|c| c.demand)
        .sum_with_accumulator::<MySumAccumulator>();

    info!(
        "Total travel time under solution: {:.6e}",
        total_travel_time
    );

    info!("Total demand: {:.6e}", total_demand);

    info!(
        "Average travel time per unit of demand under solution: {:.6e}",
        total_travel_time / total_demand
    );

    // Write solution to CSV file
    let mut wtr = csv::Writer::from_path("solution.csv").unwrap();
    wtr.write_record(["edge_idx", "flow"]).unwrap();
    result
        .solution
        .edge_flow()
        .iter()
        .enumerate()
        .for_each(|(edge_idx, flow)| {
            wtr.write_record(&[edge_idx.to_string(), flow.to_string()])
                .unwrap();
        });
    wtr.flush().unwrap();
}

fn main() {
    env_logger::init();
    let cli = Cli::parse();
    match cli.command {
        Commands::CarbonPricing(args) => main_carbon_pricing(args),
        Commands::Test => test(),
    }
}
