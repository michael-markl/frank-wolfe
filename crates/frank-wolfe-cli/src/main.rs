use clap::{Parser, Subcommand};

use crate::{
    cli::budget_pricing::{BudgetPricingArgs, main_budget_pricing},
    cli::carbon_pricing::{CarbonPricingArgs, main_carbon_pricing},
    cli::single::{SingleArgs, main_single},
};

mod cli;

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
    BudgetPricing(BudgetPricingArgs),
}

fn main() {
    env_logger::init();
    let cli = Cli::parse();
    match cli.command {
        Commands::CarbonPricing(args) => main_carbon_pricing(args),
        Commands::Single(args) => main_single(args),
        Commands::BudgetPricing(args) => main_budget_pricing(args),
    }
}
