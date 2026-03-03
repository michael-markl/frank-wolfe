use sqlite::OpenFlags;

use crate::{
    bmw_function::BMWFunction,
    col::HashMap,
    common::{BUNDLE_IDX_EMPTY, CommodityIdx, EdgeIdx, Float, NodeIdx, PathIdx},
    demand::Demand,
    graph::{EdgeParams, Graph, LpfMode},
    path_index::PathIndex,
};

pub fn read_graph(
    path: &std::path::PathBuf,
) -> (Graph, HashMap<i64, NodeIdx>, HashMap<i64, EdgeIdx>) {
    let mut graph = Graph::empty();

    let db =
        sqlite::Connection::open_with_flags(path, sqlite::OpenFlags::default().with_read_only())
            .expect(&format!(
                "Failed to open sqlite database '{}'.",
                path.display()
            ));
    // Query the nodes table first and retrieve ids as a hashmap original-id -> index.
    let node_idx_by_id: HashMap<_, _> = db
        .prepare("SELECT ID FROM NODE ORDER BY ID;")
        .expect("Failed to prepare statement for nodes table.")
        .into_iter()
        .map(|it| {
            let new_idx = graph.add_node(true);
            let id: i64 = it.expect("Failed to read node record.").read(0);
            (id, new_idx)
        })
        .collect::<HashMap<_, _>>();

    // Query the edges table and retrieve the from and to node indices as well as the free flow time and capacity.
    let edge_idx_by_id: HashMap<_, _> = db
        .prepare("SELECT ID, NODE_FROM, NODE_TO, LENGTH, LPF_PARAM_1, LPF_PARAM_2, LPF_PARAM_3, LPF_MODE, OFFSET FROM LINK ORDER BY ID;")
        .expect("Failed to prepare statement for edges table.")
        .into_iter()
        .map(|it| {
            let record = it.expect("Failed to read edge record.");
            let id: i64 = record.read(0);
            let from_node: i64 = record.read(1);
            let to_node: i64 = record.read(2);
            let length: f64 = record.read(3);
            let lpf_param_1: f64 = record.read(4);
            let lpf_param_2: f64 = record.read(5);
            let lpf_param_3: f64 = record.read(6);
            let lpf_mode_str: &str = record.read(7);

            let lpf_mode = match lpf_mode_str.to_lowercase().as_str() {
                "bpr" => LpfMode::BPR,
                "c" => LpfMode::C,
                "op" => LpfMode::OP,
                _ => panic!("Unknown LPF mode '{}' for edge {} record.", lpf_mode_str, id),
            };

            let offset: f64 = record.read(8);

            let edge_params = EdgeParams {
                mode: lpf_mode,
                toll: 0.0,
                offset,
                ff_time: lpf_param_1,
                beta: lpf_param_2,
                capacity: lpf_param_3,
                length,
            };

            let from_node_idx = *node_idx_by_id.get(&from_node).expect(format!("Failed to find from node {} for edge {} record.", from_node, id).as_str());
            let to_node_idx = *node_idx_by_id.get(&to_node).expect(format!("Failed to find to node {} for edge {} record.", to_node, id).as_str());

            let new_edge_idx = graph.add_edge(
                from_node_idx,
                to_node_idx,
                BUNDLE_IDX_EMPTY,
                edge_params
            ).expect("Node indexes should be valid.");
            (id, new_edge_idx)
        }).collect();

    (graph, node_idx_by_id, edge_idx_by_id)
}

pub fn read_demand(
    path: &std::path::PathBuf,
    node_idx_by_id: &HashMap<i64, NodeIdx>,
) -> (Demand, HashMap<i64, CommodityIdx>) {
    let db =
        sqlite::Connection::open_with_flags(path, sqlite::OpenFlags::default().with_read_only())
            .expect(&format!(
                "Failed to open sqlite database '{}'.",
                path.display()
            ));

    let mut demand = Demand::empty();

    // Query the demand table and retrieve the from and to node indices as well as the demand.
    let commodity_idx_by_id: HashMap<_, _> = db
        .prepare("SELECT ID, NODE_FROM, NODE_TO, FLOW FROM DEMAND ORDER BY ID;")
        .expect("Failed to prepare statement for demand table.")
        .into_iter()
        .map(|it| {
            let record = it.expect("Failed to read demand record.");
            let commodity_id: i64 = record.read(0);
            let from_node: i64 = record.read(1);
            let to_node: i64 = record.read(2);
            let flow: f64 = record.read(3);

            let from_node_idx = *node_idx_by_id.get(&from_node).expect(
                format!("Failed to find from node {} for demand record.", from_node).as_str(),
            );
            let to_node_idx = *node_idx_by_id
                .get(&to_node)
                .expect(format!("Failed to find to node {} for demand record.", to_node).as_str());

            let commodity_idx: CommodityIdx =
                demand.add_commodity(from_node_idx, to_node_idx, flow);

            (commodity_id, commodity_idx)
        })
        .collect();

    (demand, commodity_idx_by_id)
}

pub fn write_solution(
    path: &std::path::PathBuf,
    edge_flow: &[Float],
    graph: &Graph,
    edge_idx_by_id: Option<&HashMap<i64, EdgeIdx>>,
    commodity_idx_by_id: Option<&HashMap<i64, CommodityIdx>>,
    path_flow: Option<&HashMap<(CommodityIdx, PathIdx), Float>>,
    path_index: &PathIndex,
) {
    let edge_id_by_idx = edge_idx_by_id.map(|map| {
        map.iter()
            .map(|(id, idx)| (*idx, *id))
            .collect::<HashMap<_, _>>()
    });

    let commodity_id_by_idx = commodity_idx_by_id.map(|map| {
        map.iter()
            .map(|(id, idx)| (*idx, *id))
            .collect::<HashMap<_, _>>()
    });

    let db = sqlite::Connection::open_with_flags(
        path,
        OpenFlags::default().with_create().with_read_write(),
    )
    .unwrap_or_else(|e| {
        panic!(
            "Failed to open sqlite database for writing '{}': {:#}",
            path.display(),
            e
        )
    });

    db.execute("BEGIN TRANSACTION;").unwrap();
    db.execute("CREATE TABLE EDGE ( ID INTEGER, FLOW REAL, UTILIZATION REAL, COST REAL, COST_WITH_TOLL REAL );")
        .unwrap();
    db.execute("CREATE TABLE GLOBAL ( COST REAL );").unwrap();

    let mut stmt = db
        .prepare("INSERT INTO EDGE (ID, FLOW, UTILIZATION, COST, COST_WITH_TOLL) VALUES (?, ?, ?, ?, ?);")
        .unwrap();
    for (edge_idx, &flow) in edge_flow.iter().enumerate() {
        let params = &graph.edge(edge_idx).params;
        let edge_id = edge_id_by_idx
            .as_ref()
            .map(|it| it[&edge_idx])
            .unwrap_or(edge_idx as i64);
        stmt.bind((1, edge_id)).unwrap();
        stmt.bind((2, flow)).unwrap();
        let utilization = if params.mode == LpfMode::C {
            0.0
        } else {
            flow / params.capacity
        };
        stmt.bind((3, utilization)).unwrap();
        let mut no_toll_params = params.clone();
        no_toll_params.toll = 0.0;
        let cost = BMWFunction::derivative(&no_toll_params, flow);
        stmt.bind((4, cost)).unwrap();
        let cost_with_toll = BMWFunction::derivative(&params, flow);
        stmt.bind((5, cost_with_toll)).unwrap();
        stmt.next().unwrap();
        stmt.reset().unwrap();
    }
    drop(stmt);

    if let Some(path_flow) = path_flow {
        db.execute("CREATE TABLE PATH ( ID INTEGER, DEMAND_ID INTEGER, FLOW REAL );")
            .unwrap();
        db.execute(
            "CREATE TABLE EDGE_PATH ( EDGE_ID INTEGER, PATH_ID INTEGER, EDGE_INDEX INTEGER );",
        )
        .unwrap();

        let mut path_stmt = db
            .prepare("INSERT INTO PATH (ID, DEMAND_ID, FLOW) VALUES (?, ?, ?);")
            .unwrap();

        for (&(commodity_idx, path_idx), &flow) in path_flow.iter() {
            let demand_id = commodity_id_by_idx
                .as_ref()
                .map(|it| it[&commodity_idx])
                .unwrap_or(commodity_idx as i64);
            path_stmt.bind((1, path_idx as i64)).unwrap();
            path_stmt.bind((2, demand_id)).unwrap();
            path_stmt.bind((3, flow)).unwrap();
            path_stmt.next().unwrap();
            path_stmt.reset().unwrap();
        }
        drop(path_stmt);

        let mut edge_path_stmt = db
            .prepare("INSERT INTO EDGE_PATH (EDGE_ID, PATH_ID, EDGE_INDEX) VALUES (?, ?, ?);")
            .unwrap();
        for (&(_, path_idx), _) in path_flow.iter() {
            let path = path_index.get_payload(path_idx);
            for (edge_position, edge_idx) in path.edges().enumerate() {
                let edge_id = edge_id_by_idx
                    .as_ref()
                    .map(|it| it[&edge_idx])
                    .unwrap_or(edge_idx as i64);
                edge_path_stmt.bind((1, edge_id)).unwrap();
                edge_path_stmt.bind((2, path_idx as i64)).unwrap();
                edge_path_stmt.bind((3, edge_position as i64)).unwrap();
                edge_path_stmt.next().unwrap();
                edge_path_stmt.reset().unwrap();
            }
        }
    }
    db.execute("END TRANSACTION;").unwrap();
}
