use std::sync::RwLock;

use clap::{Parser, Subcommand};
use log::{error, info};

use crate::{
    astar::AStarTable,
    bundle_index::BundleIndex,
    common::Float,
    demand::DemandOps,
    edge_based_convex_program::EdgeBasedConvexProgramInstance,
    frank_wolfe::solve_convex_program,
    graph::EdgeParams,
    graph_ops::GraphOps,
    main_carbon_pricing::{CarbonPricingArgs, main_carbon_pricing},
};

mod astar;
mod astar_tree;
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

struct BMWFunction {}

impl BMWFunction {
    fn evaluate(p: &EdgeParams, x: Float) -> Float {
        // int_0^x toll + alpha (1 + beta * (y+offset/gamma)^4) dy
        // = x * (toll + alpha) + alpha * beta / gamma^4 * int_0^y (y + offset)^4 dy
        // = x * (toll + alpha) + alpha * beta / gamma^4 * [ (x + offset)^5 -
        // offset^5 ] / 5

        x * (p.toll + p.ff_time)
            + p.ff_time * p.beta / (5.0 * p.capacity.powi(4))
                * ((x + p.offset).powi(5) - p.offset.powi(5))
    }

    fn derivative(p: &EdgeParams, x: Float) -> Float {
        p.toll + p.ff_time * (1.0 + p.beta / (p.capacity.powi(4)) * (x + p.offset).powi(4))
    }
}

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

    let solution = solve_convex_program(initial_solution, instance);

    let total_travel_time = solution
        .edge_flow()
        .iter()
        .enumerate()
        .map(|(edge_idx, &it)| BMWFunction::derivative(&graph.edge(edge_idx).params, it) * it)
        .sum::<Float>();

    let total_demand = demand.commodities().iter().map(|c| c.demand).sum::<Float>();

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
    solution
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
