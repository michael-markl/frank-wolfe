use clap::{Parser, Subcommand};

use crate::{
    bmw_function::BMWFunction,
    main_carbon_pricing::{CarbonPricingArgs, main_carbon_pricing},
    main_single::{SingleArgs, main_single},
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
mod io;
mod iter;
mod main_carbon_pricing;
mod main_single;
mod path_index;
mod path_based_solution;
mod path_based_convex_program;

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
    Single(SingleArgs),
}

fn main() {
    env_logger::init();
    let cli = Cli::parse();
    match cli.command {
        Commands::CarbonPricing(args) => main_carbon_pricing(args),
        Commands::Single(args) => main_single(args),
    }
}
