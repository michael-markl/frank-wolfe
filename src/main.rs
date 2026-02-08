use std::{mem::replace, sync::RwLock};

use rayon::iter::ParallelIterator;

use crate::{
    astar::AStarTable,
    astar_tree::{AStarTree, ShortestPathCostOps},
    bundle_index::{Bundle, BundleIndex},
    col::{HashMap, map_new},
    common::{BundleIdx, EdgeIdx, Float, PathIdx, PermitIdx},
    demand::{Demand, DemandOps},
    edge_based_convex_program::EdgeBasedConvexProgramInstance,
    edge_based_solution::EdgeBasedSolution,
    frank_wolfe::{
        ConvexProgramInstance, LinearizedSubProblemSolution, SolutionOps, solve_convex_program,
    },
    graph::{Edge, Graph},
    graph_ops::GraphOps,
    path_index::PathIndex,
    tntp::TNTPNet,
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
mod path_based_solution;
mod path_index;
mod tntp;

struct BMWFunction {}

impl BMWFunction {
    fn evaluate(edge: &Edge, x: Float) -> Float {
        let p = &edge.edge_params;

        // int_0^x toll + alpha (1 + beta * (y+offset/gamma)^4) dy
        // = x * (toll + alpha) + alpha * beta / gamma^4 * int_0^y (y + offset)^4 dy
        // = x * (toll + alpha) + alpha * beta / gamma^4 * [ (x + offset)^5 -
        // offset^5 ] / 5

        x * (p.toll + p.alpha)
            + p.alpha * p.beta / (5.0 * p.gamma.powi(4))
                * ((x + p.offset).powi(5) - p.offset.powi(5))
    }

    fn derivative(edge: &Edge, x: Float) -> Float {
        let p = &edge.edge_params;
        p.toll + p.alpha * (1.0 + p.beta / (p.gamma.powi(4)) * (x + p.offset).powi(4))
    }
}

fn main() {
    let tntpnet = tntp::read_net_file(std::path::Path::new(
        "C:\\Users\\markl07\\git\\TransportationNetworks\\Berlin-Center\\berlin-center_net.tntp",
    ))
    .map_err(|err| eprintln!("Error reading net file: {}", err))
    .unwrap();
    let demand= tntp::read_trips_file(std::path::Path::new("C:\\Users\\markl07\\git\\TransportationNetworks\\Berlin-Center\\berlin-center_trips.tntp"), &tntpnet)
        .map_err(|err| eprintln!("Error reading trips file: {}", err))
        .unwrap();

    let graph = tntpnet.graph;
    let mut astar_table = AStarTable::create(graph.num_nodes(), demand.num_destinations());
    astar_table.fill_table(&graph, &demand);

    let bundle_index = RwLock::new(BundleIndex::new());
    bundle_index
        .write()
        .unwrap()
        .transfer_element(Bundle::from_permits(vec![])); // TODO: Handle empty set more efficiently.

    let instance = EdgeBasedConvexProgramInstance {
        graph: &graph,
        demand: &demand,
        astar_table: &astar_table,
        bundle_index: &bundle_index,
    };

    let initial_solution = instance.compute_shortest_path_flow(
        &(0..graph.num_edges())
            .map(|edge_idx| graph.edge(edge_idx).edge_params.alpha)
            .collect::<Vec<_>>(),
        &vec![],
    );

    let solution = solve_convex_program(initial_solution, instance);

    let total_travel_time = solution
        .edge_flow()
        .iter()
        .enumerate()
        .map(|(edge_idx, it)| BMWFunction::derivative(graph.edge(edge_idx), *it) * *it)
        .sum::<Float>();

    let total_demand = demand.commodities().iter().map(|c| c.demand).sum::<Float>();

    println!(
        "Total travel time under solution: {:.6e}",
        total_travel_time
    );

    println!("Total demand: {:.6e}", total_demand);

    println!(
        "Average travel time per unit of demand under solution: {:.6e}",
        total_travel_time / total_demand
    );

    // Write solution to CSV file
    let mut wtr = csv::Writer::from_path("solution.csv").unwrap();
    wtr.write_record(&["edge_idx", "flow"]).unwrap();
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
