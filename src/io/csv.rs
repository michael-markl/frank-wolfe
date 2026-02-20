use serde::Serialize;

use crate::{
    bmw_function::BMWFunction, common::Float, graph::Graph,
    graph_ops::GraphOps,
};

#[derive(Debug, Serialize)]
struct EdgeFlowCsvEntry {
    edge_id: usize,
    flow: Float,
    adjusted_length: Float,
    capacity: Float,
    utilization: Float,
    travel_time_per_unit: Float,
}

pub fn write_edge_flow_csv(
    edge_flow: &[Float],
    flow_csv_path: &std::path::PathBuf,
    graph: &Graph,
) {
    let mut wtr = csv::Writer::from_path(flow_csv_path).expect("Failed to create flow CSV writer");

    for edge_idx in 0..graph.num_edges() {
        let edge = graph.edge(edge_idx);
        let travel_time_per_unit = BMWFunction::derivative(&edge.params, edge_flow[edge_idx]);
        wtr.serialize(EdgeFlowCsvEntry {
            edge_id: edge_idx,
            flow: edge_flow[edge_idx],
            adjusted_length: edge.params.length,
            capacity: edge.params.capacity,
            utilization: edge_flow[edge_idx] / edge.params.capacity,
            travel_time_per_unit,
        })
        .expect("Failed to write flow record");
    }

    wtr.flush().expect("Failed to flush CSV writer");
}
