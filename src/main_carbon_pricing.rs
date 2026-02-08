use std::sync::RwLock;

use clap_derive::Parser;

use crate::{
    BMWFunction,
    astar::AStarTable,
    bundle_index::BundleIndex,
    common::Float,
    demand::{Demand, DemandOps},
    edge_based_convex_program::EdgeBasedConvexProgramInstance,
    edge_based_solution::EdgeBasedSolution,
    frank_wolfe::solve_convex_program,
    graph::Graph,
    graph_ops::GraphOps,
    tntp::{read_net_file, read_trips_file},
};

#[derive(Parser, Debug)]
pub struct CarbonPricingArgs {
    #[arg(long = "graph")]
    tntp_net: std::path::PathBuf,
    #[arg(long = "demand")]
    tntp_trips: std::path::PathBuf,
    #[arg(long = "out")]
    csv_output_path: std::path::PathBuf,
    #[arg(long = "min_price")]
    min_price: Float,
    #[arg(long = "max_price")]
    max_price: Float,
    #[arg(long = "steps")]
    steps: usize,
    #[arg(long = "min_per_time_unit")]
    min_per_time_unit: Option<Float>,
    #[arg(long = "km_per_distance_unit")]
    km_per_distance_unit: Option<Float>,
}

pub fn main_carbon_pricing(args: CarbonPricingArgs) {
    let tntp_net = read_net_file(&args.tntp_net)
        .map_err(|err| eprintln!("Error reading net file: {}", err))
        .unwrap();
    let demand = read_trips_file(&args.tntp_trips, &tntp_net)
        .map_err(|err| eprintln!("Error reading trips file: {}", err))
        .unwrap();

    let mut graph = tntp_net.graph;

    if let Some(min_per_time_unit) = args.min_per_time_unit {
        for edge_idx in 0..graph.num_edges() {
            let edge = graph.edge_mut(edge_idx);
            edge.edge_params.alpha *= min_per_time_unit;
        }
    }

    if let Some(km_per_distance_unit) = args.km_per_distance_unit {
        for edge_idx in 0..graph.num_edges() {
            let edge = graph.edge_mut(edge_idx);
            edge.edge_params.length *= km_per_distance_unit;
        }
    }

    let mut wtr = csv::Writer::from_path(&args.csv_output_path).unwrap();
    wtr.write_record(&[
        "step",
        "carbon_price",
        "total_travel_time",
        "total_user_cost",
        "total_emission",
    ])
    .unwrap();
    wtr.flush().unwrap();

    carbon_pricing(
        &mut graph,
        &demand,
        args.min_price,
        args.max_price,
        args.steps,
        |step, price, solution, graph| {
            let total_travel_time = solution
                .edge_flow()
                .iter()
                .enumerate()
                .map(|(edge_idx, it)| {
                    (BMWFunction::derivative(graph.edge(edge_idx), *it)
                        - graph.edge(edge_idx).edge_params.length)
                        * *it
                })
                .sum::<Float>();

            let total_user_cost = solution
                .edge_flow()
                .iter()
                .enumerate()
                .map(|(edge_idx, it)| BMWFunction::derivative(graph.edge(edge_idx), *it))
                .sum::<Float>();

            let total_emission = solution
                .edge_flow()
                .iter()
                .enumerate()
                .map(|(edge_idx, it)| graph.edge(edge_idx).edge_params.length * *it)
                .sum::<Float>();

            println!(
                "Step {}: Carbon price = {:.6e}, Total travel time = {:.6e}, Total user cost = {:.6e}, Total emission = {:.6e}",
                step, price, total_travel_time, total_user_cost, total_emission
            );

            wtr.write_record(&[
                step.to_string(),
                price.to_string(),
                total_travel_time.to_string(),
                total_user_cost.to_string(),
                total_emission.to_string(),
            ])
            .unwrap();
            wtr.flush().unwrap();
        },
    );
}

pub fn carbon_pricing(
    graph: &mut Graph,
    demand: &Demand,
    min_price: Float,
    max_price: Float,
    steps: usize,
    mut on_step: impl FnMut(usize, Float, &EdgeBasedSolution, &Graph),
) {
    let mut solution: Option<EdgeBasedSolution> = None;

    let mut astar_table = AStarTable::create(graph.num_nodes(), demand.num_destinations());
    astar_table.fill_table(graph, demand);

    let bundle_index = RwLock::new(BundleIndex::new());

    for step in 0..steps {
        let price = min_price + (max_price - min_price) * (step as Float) / ((steps - 1) as Float);
        println!("Step {}: Carbon price = {:.6e}", step, price);
        set_carbon_prices(graph, price);

        let instance = EdgeBasedConvexProgramInstance {
            graph,
            demand,
            astar_table: &astar_table,
            bundle_index: &bundle_index,
        };

        let initial_solution = solution.take().unwrap_or_else(|| {
            let edge_costs = (0..graph.num_edges())
                .map(|edge_idx| BMWFunction::derivative(graph.edge(edge_idx), 0.0))
                .collect::<Vec<_>>();
            instance.compute_shortest_path_flow(&edge_costs, &vec![])
        });

        solution = Some(solve_convex_program(initial_solution, instance));

        on_step(step, price, solution.as_ref().unwrap(), graph);
    }
}

fn set_carbon_prices(graph: &mut Graph, price: Float) {
    for edge_idx in 0..graph.num_edges() {
        let edge = graph.edge_mut(edge_idx);
        edge.edge_params.toll = price * edge.edge_params.length;
    }
}
