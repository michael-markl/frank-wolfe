use crate::{
    col::HashMap,
    common::{CommodityIdx, EdgeIdx, NodeIdx},
    demand::Demand,
    graph::Graph,
};

pub mod csv;
pub mod sqlite;
pub mod tntp;

pub struct ExternalGraph {
    pub graph: Graph,
    pub node_idx_by_id: HashMap<i64, NodeIdx>,
    pub edge_idx_by_id: Option<HashMap<i64, EdgeIdx>>,
}

pub fn read_graph(path: &std::path::PathBuf) -> ExternalGraph {
    if path.extension().is_some_and(|it| it == "tntp") {
        let tntp_net = tntp::read_net_file(path)
            .expect(&format!("Error reading net file '{}'.", path.display()));
        ExternalGraph {
            graph: tntp_net.graph,
            node_idx_by_id: tntp_net.node_idx_by_id,
            edge_idx_by_id: None,
        }
    } else if path
        .extension()
        .is_some_and(|it| it == "sqlite" || it == "sqlite3")
    {
        let (graph, node_idx_by_id, edge_idx_by_id) = sqlite::read_graph(path);
        ExternalGraph {
            graph,
            node_idx_by_id,
            edge_idx_by_id: Some(edge_idx_by_id),
        }
    } else {
        panic!("Unsupported file extension: {:?}", path.extension());
    }
}

pub fn read_demand(
    path: &std::path::PathBuf,
    ext_graph: &ExternalGraph,
) -> (Demand, Option<HashMap<i64, CommodityIdx>>) {
    if path.extension().is_some_and(|it| it == "tntp") {
        let tntp_demand = tntp::read_trips_file(path, ext_graph)
            .unwrap_or_else(|_| panic!("Error reading trip file '{}'.", path.display()));
        (tntp_demand, None)
    } else if path
        .extension()
        .is_some_and(|it| it == "sqlite" || it == "sqlite3")
    {
        let (demand, commodity_idx_by_id) = sqlite::read_demand(path, &ext_graph.node_idx_by_id);
        (demand, Some(commodity_idx_by_id))
    } else {
        panic!("Unsupported file extension: {:?}", path.extension());
    }
}
