use std::sync::RwLock;

use clap_derive::Parser;

use crate::{
    BMWFunction,
    astar::AStarTable,
    bundle_index::{Bundle, BundleIndex},
    col::{HashMap, map_new},
    common::{BundleIdx, EdgeIdx, Float},
    demand::{Demand, DemandOps},
    edge_based_convex_program::EdgeBasedConvexProgramInstance,
    edge_based_solution::EdgeBasedSolution,
    frank_wolfe::solve_convex_program,
    graph::{EdgeMode, EdgeParams, Graph},
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
    #[arg(long = "cordon_edge_map")]
    cordon_edge_map: Option<std::path::PathBuf>,
    #[arg(long = "permit_based", default_value_t = false)]
    permit_based: bool,
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
            edge.params.alpha *= min_per_time_unit;
        }
    }

    if let Some(km_per_distance_unit) = args.km_per_distance_unit {
        for edge_idx in 0..graph.num_edges() {
            let edge = graph.edge_mut(edge_idx);
            edge.params.length *= km_per_distance_unit;
        }
    }

    let bundle_index = RwLock::new(BundleIndex::new());

    let cordon_pricing_map = args
        .cordon_edge_map
        .map(CordonPricingMap::from_csv);

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
                    alpha: 0.0,
                    beta: 0.0,
                    gamma: Float::MAX,
                    length: 0.0,
                    toll: 0.0,
                    mode: EdgeMode::BPR,
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
                struct PermitBasedCordonPricing {
                    bundle_idx: BundleIdx,
                }

                impl TollsStrategy for PermitBasedCordonPricing {
                    fn set_tolls(&self, graph: &mut Graph, price: Float) {
                        for edge_idx in 0..graph.num_edges() {
                            let edge = graph.edge_mut(edge_idx);
                            edge.params.toll = if edge.bundle == self.bundle_idx {
                                price
                            } else {
                                0.0
                            };
                        }
                    }
                }
                Box::new(PermitBasedCordonPricing { bundle_idx })
            }
        } else {
            Box::new(CarbonPricing {})
        };

    let mut wtr = csv::Writer::from_path(&args.csv_output_path).unwrap();
    wtr.write_record([
        "iteration",
        "price",
        "total_travel_time",
        "total_user_cost",
        "total_consumption",
        "total_consumption_inside",
        "total_entrances",
        "total_permit_flow",
    ])
    .unwrap();
    wtr.flush().unwrap();

    compute_solutions_for_price_range(
        &mut graph,
        &demand,
        &bundle_index,
        args.min_price,
        args.max_price,
        args.steps,
        tolls_strategy,
        |step, price, solution, graph| {
            let total_travel_time = solution
                .edge_flow()
                .iter()
                .enumerate()
                .map(|(edge_idx, &it)| {
                    (BMWFunction::derivative(&graph.edge(edge_idx).params, it)
                        - graph.edge(edge_idx).params.toll)
                        * it
                })
                .sum::<Float>();

            let total_user_cost = solution
                .edge_flow()
                .iter()
                .enumerate()
                .map(|(edge_idx, &it)| {
                    BMWFunction::derivative(&graph.edge(edge_idx).params, it) * it
                })
                .sum::<Float>()
                + solution
                    .permit_flow()
                    .iter()
                    .enumerate()
                    .map(|(permit_idx, &it)| {
                        BMWFunction::derivative(&graph.permit(permit_idx).params, it)
                    })
                    .sum::<Float>();

            let total_consumption = solution
                .edge_flow()
                .iter()
                .enumerate()
                .map(|(edge_idx, &it)| graph.edge(edge_idx).params.length * it)
                .sum::<Float>();

            let consumption_inside = cordon_pricing_map.as_ref().map(|map| {
                solution
                    .edge_flow()
                    .iter()
                    .enumerate()
                    .filter(|(edge_idx, _)| map.for_edge(*edge_idx).inside)
                    .map(|(edge_idx, &it)| graph.edge(edge_idx).params.length * it)
                    .sum::<Float>()
            });

            let total_entrances = cordon_pricing_map.as_ref().map(|map| {
                solution
                    .edge_flow()
                    .iter()
                    .enumerate()
                    .filter(|(edge_idx, _)| map.for_edge(*edge_idx).leads_inside)
                    .map(|(_edge_idx, &it)| it)
                    .sum::<Float>()
            });

            let total_permit_flow = cordon_pricing_map.as_ref().map(|_| {
                solution
                    .permit_flow()
                    .iter()
                    .enumerate()
                    .map(|(_, &it)| it)
                    .sum::<Float>()
            });

            println!(
                "Step {}: Price = {:.6e}, Total travel time = {:.6e}, Total user cost = {:.6e}, Total consumption = {:.6e}",
                step, price, total_travel_time, total_user_cost, total_consumption
            );

            wtr.write_record(&[
                step.to_string(),
                price.to_string(),
                total_travel_time.to_string(),
                total_user_cost.to_string(),
                total_consumption.to_string(),
                consumption_inside.map_or("".to_string(), |v| v.to_string()),
                total_entrances.map_or("".to_string(), |v| v.to_string()),
                total_permit_flow.map_or("".to_string(), |v| v.to_string()),
            ])
            .unwrap();
            wtr.flush().unwrap();
        },
    );
}

pub fn compute_solutions_for_price_range<'a>(
    graph: &mut Graph,
    demand: &Demand,
    bundle_index: &'a RwLock<BundleIndex<'a>>,
    min_price: Float,
    max_price: Float,
    steps: usize,
    tolls_strategy: impl TollsStrategy,
    mut on_step: impl FnMut(usize, Float, &EdgeBasedSolution, &Graph),
) {
    let mut solution: Option<EdgeBasedSolution> = None;

    let mut astar_table = AStarTable::create(graph.num_nodes(), demand.num_destinations());
    astar_table.fill_table(graph, demand);

    for step in 0..steps {
        let price = min_price + (max_price - min_price) * (step as Float) / ((steps - 1) as Float);
        println!("Step {}: Price = {:.6e}", step, price);
        tolls_strategy.set_tolls(graph, price);

        let instance = EdgeBasedConvexProgramInstance {
            graph,
            demand,
            astar_table: &astar_table,
            bundle_index,
        };

        let initial_solution = solution.take().unwrap_or_else(|| {
            let edge_costs = (0..graph.num_edges())
                .map(|edge_idx| BMWFunction::derivative(&graph.edge(edge_idx).params, 0.0))
                .collect::<Vec<_>>();
            let permit_costs = (0..graph.num_permits())
                .map(|permit_idx| BMWFunction::derivative(&graph.permit(permit_idx).params, 0.0))
                .collect::<Vec<_>>();
            instance.compute_shortest_path_flow(&edge_costs, &permit_costs)
        });

        solution = Some(solve_convex_program(initial_solution, instance));

        on_step(step, price, solution.as_ref().unwrap(), graph);
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
