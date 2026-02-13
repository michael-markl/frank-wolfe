use serde::Serialize;

use crate::{
    bmw_function::BMWFunction, common::Float, edge_based_solution::EdgeBasedSolution, graph::Graph,
    graph_ops::GraphOps,
};

#[derive(Debug, Serialize)]
struct FlowCsvEntry {
    edge_id: usize,
    flow: Float,
    adjusted_length: Float,
    capacity: Float,
    utilization: Float,
    travel_time_per_unit: Float,
}

pub fn write_flow_csv(
    solution: &EdgeBasedSolution,
    flow_csv_path: &std::path::PathBuf,
    graph: &Graph,
) {
    let mut wtr = csv::Writer::from_path(flow_csv_path).expect("Failed to create flow CSV writer");

    let edge_flows = solution.edge_flow();
    for edge_idx in 0..graph.num_edges() {
        let edge = graph.edge(edge_idx);
        let travel_time_per_unit = BMWFunction::derivative(&edge.params, edge_flows[edge_idx]);
        wtr.serialize(FlowCsvEntry {
            edge_id: edge_idx,
            flow: edge_flows[edge_idx],
            adjusted_length: edge.params.length,
            capacity: edge.params.capacity,
            utilization: edge_flows[edge_idx] / edge.params.capacity,
            travel_time_per_unit,
        })
        .expect("Failed to write flow record");
    }

    wtr.flush().expect("Failed to flush CSV writer");
}
