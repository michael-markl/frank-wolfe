use std::{
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};

use accurate::traits::SumWithAccumulator;
use log::info;

use crate::{
    collections::{HashMap, map_new},
    common::{BUNDLE_IDX_EMPTY, Float, MySumAccumulator, NodeIdx},
    io::ExternalGraph,
    network::demand::Demand,
    network::graph::{EdgeParams, Graph, LpfMode},
};

pub struct TNTPNet {
    pub graph: Graph,
    pub node_idx_by_id: HashMap<i64, NodeIdx>,
}

fn read_metadata(
    non_empty_lines: &mut impl Iterator<Item = (usize, Result<String, String>)>,
) -> Result<HashMap<String, String>, String> {
    let mut metadata = map_new();
    for (index, line) in non_empty_lines {
        let line = line?;
        let trimmed = line.trim();
        if !trimmed.starts_with('<') {
            return Ok(metadata);
        }
        if let Some((key, value)) = trimmed[1..].split_once('>') {
            if key == "END OF METADATA" {
                if value.trim().is_empty() {
                    return Ok(metadata);
                } else {
                    return Err(format!(
                        "Line {:}: Unexpected value '{:}' for <END OF METADATA> tag",
                        index, value
                    ));
                }
            }
            metadata.insert(key.trim().to_string(), value.trim().to_string());
        } else {
            return Err(format!(
                "Line {:}: Incomplete metadata tag '{}'",
                index, trimmed
            ));
        }
    }
    Ok(metadata)
}

/// Parses a .tntp net file, under the following assumptions:
/// * Lines that consist entirely of whitespace are ignored.
/// * Optionally, the first N lines may consist of metadata lines, containing a key-value pair in the format `<KEY> VALUE`. Whitespace around the key and value is trimmed.
/// * The metadata section ends at the first line that does not start with `<`, or after the line `<END OF METADATA>`, whichever comes first.
/// * The first non-metadata line is assumed to be the header line, containing column names separated by tabs.
pub fn read_net_file(path: &Path) -> Result<TNTPNet, String> {
    let file = File::open(path)
        .map_err(|err| format!("Failed to open net file {}: {}", path.display(), err))?;
    let reader = BufReader::new(file);

    let mut lines_iter = reader
        .lines()
        .map(|it| it.map_err(|err| format!("Failed to read TNTP net file: {}", err)))
        .enumerate()
        .map(|(idx, line)| (idx + 1, line))
        .filter(|(_, it)| it.is_err() || !it.as_ref().unwrap().is_empty());

    let metadata = read_metadata(&mut lines_iter)?;
    let first_through_node = metadata
        .get("FIRST THRU NODE")
        .map(|it| {
            str::parse::<i64>(it).map_err(|err| format!("Invalid FIRST THRU NODE value: {}", err))
        })
        .transpose()?;

    info!("First through node: {:?}", first_through_node);

    let (header_idx, header) = lines_iter
        .next()
        .ok_or("Net file appears empty.".to_string())?;
    let header_names: Vec<String> = header?
        .split('\t')
        .map(|it| it.trim().to_lowercase())
        .collect();
    let columns: HashMap<&str, usize> = [
        "init_node",
        "term_node",
        "capacity",
        "length",
        "free_flow_time",
        "b",
    ]
    .iter()
    .map(|&col| {
        let pos = header_names.iter().position(|it| it == col).ok_or(format!(
            "Line {:}: Header does not contain required column '{}'",
            header_idx, col
        ))?;
        Ok((col, pos))
    })
    .collect::<Result<HashMap<_, _>, String>>()?;

    let externality_linear_column = header_names
        .iter()
        .position(|it| it == "externality_linear");

    let mut node_idx_by_id = map_new();
    let mut graph = Graph::empty();

    struct RawEdge {
        tail_node: i64,
        head_node: i64,
        capacity: Float,
        length: Float,
        free_flow_time: Float,
        b: Float,
        externality_linear: Float,
    }

    let mut raw_edges = vec![];

    for (line_idx, line) in lines_iter {
        let line =
            line.map_err(|err| format!("Failed to read net file {}: {}", path.display(), err))?;
        let data = line.split('\t').map(|it| it.trim()).collect::<Vec<&str>>();

        let tail_node = data
            .get(*columns.get("init_node").unwrap())
            .ok_or_else(|| format!("Line {:}: Missing 'init_node' column", line_idx))?
            .parse::<i64>()
            .map_err(|err| format!("Line {:}: Invalid init_node value: {}", line_idx, err))?;
        let head_node = data
            .get(*columns.get("term_node").unwrap())
            .ok_or_else(|| format!("Line {:}: Missing 'term_node' column", line_idx))?
            .parse::<i64>()
            .map_err(|err| format!("Line {:}: Invalid term_node value: {}", line_idx, err))?;
        let capacity = data
            .get(*columns.get("capacity").unwrap())
            .ok_or_else(|| format!("Line {:}: Missing 'capacity' column", line_idx))?
            .parse::<Float>()
            .map_err(|err| format!("Line {:}: Invalid capacity value: {}", line_idx, err))?;
        let length = data
            .get(*columns.get("length").unwrap())
            .ok_or_else(|| format!("Line {:}: Missing 'length' column", line_idx))?
            .parse::<Float>()
            .map_err(|err| format!("Line {:}: Invalid length value: {}", line_idx, err))?;
        let free_flow_time = data
            .get(*columns.get("free_flow_time").unwrap())
            .ok_or_else(|| format!("Line {:}: Missing 'free_flow_time' column", line_idx))?
            .parse::<Float>()
            .map_err(|err| format!("Line {:}: Invalid free_flow_time value: {}", line_idx, err))?;
        let b = data
            .get(*columns.get("b").unwrap())
            .ok_or_else(|| format!("Line {:}: Missing 'b' column", line_idx))?
            .parse::<Float>()
            .map_err(|err| format!("Line {:}: Invalid b value: {}", line_idx, err))?;

        let externality_linear = match externality_linear_column {
            None => 0.0,
            Some(column) => data
                .get(column)
                .ok_or_else(|| format!("Line {}: Missing 'externality_linear' column", line_idx))?
                .parse::<Float>()
                .map_err(|err| format!("Line {}: Invalid externality_linear: {}", line_idx, err))?,
        };
        if !externality_linear.is_finite() || externality_linear < 0.0 {
            return Err(format!(
                "Line {}: externality_linear must be finite and nonnegative",
                line_idx
            ));
        }

        raw_edges.push(RawEdge {
            tail_node,
            head_node,
            capacity,
            length,
            free_flow_time,
            b,
            externality_linear,
        });
    }

    let mut all_node_ids = raw_edges
        .iter()
        .flat_map(|edge| [edge.tail_node, edge.head_node])
        .collect::<Vec<i64>>();
    all_node_ids.sort();
    all_node_ids.dedup();

    for node_id in all_node_ids {
        let allows_through = first_through_node.is_none_or(|first| node_id >= first);
        let node_idx = graph.add_node(allows_through);
        node_idx_by_id.insert(node_id, node_idx);
    }

    for edge in raw_edges {
        let tail_idx = *node_idx_by_id.get(&edge.tail_node).unwrap();
        let head_idx = *node_idx_by_id.get(&edge.head_node).unwrap();
        graph
            .add_edge(
                tail_idx,
                head_idx,
                BUNDLE_IDX_EMPTY,
                EdgeParams {
                    mode: LpfMode::BPR,
                    capacity: edge.capacity,
                    length: edge.length,
                    toll: 0.0,
                    toll_linear: 0.0,
                    externality_linear: edge.externality_linear,
                    offset: 0.0,
                    ff_time: edge.free_flow_time,
                    beta: edge.b,
                },
            )
            .expect("Node indices are valid");
    }

    Ok(TNTPNet {
        graph,
        node_idx_by_id,
    })
}

fn parse_trip_pairs(
    line: &str,
    origin_node_idx: usize,
    demand: &mut Demand,
    ext_graph: &ExternalGraph,
) -> Result<(), String> {
    let stripped_semicolon = line.replace(";", " ");
    let parts: Vec<&str> = stripped_semicolon.split_whitespace().collect();
    if !parts.len().is_multiple_of(3) {
        return Err(format!("Invalid trip pairs line format: '{}'", line));
    }
    for &[destination_id, colon, demand_value] in parts.as_chunks::<3>().0 {
        let destination_id = destination_id
            .parse::<i64>()
            .map_err(|err| format!("Invalid destination id in trip pairs: {}", err))?;
        let destination_node_idx =
            *ext_graph
                .node_idx_by_id
                .get(&destination_id)
                .ok_or_else(|| {
                    format!(
                        "Destination id {} not found in network nodes",
                        destination_id
                    )
                })?;
        if colon != ":" {
            return Err(format!(
                "Invalid trip pairs line format (expected ':' after destination id): '{}'",
                line
            ));
        }
        let demand_value = demand_value
            .parse::<Float>()
            .map_err(|err| format!("Invalid demand value in trip pairs: {}", err))?;
        demand.add_commodity(origin_node_idx, destination_node_idx, demand_value);
    }
    Ok(())
}

pub fn read_trips_file(path: &Path, ext_graph: &ExternalGraph) -> Result<Demand, String> {
    let file = File::open(path)
        .map_err(|err| format!("Failed to open trips file {}: {}", path.display(), err))?;
    let reader = BufReader::new(file);

    let mut demand = Demand::empty();
    let mut current_origin_idx: Option<NodeIdx> = None;

    let mut lines_iter = reader
        .lines()
        .map(|it| it.map_err(|err| format!("Failed to read TNTP trips file: {}", err)))
        .enumerate()
        .map(|(idx, line)| (idx + 1, line))
        .filter(|(_, it)| it.is_err() || !it.as_ref().unwrap().is_empty());

    read_metadata(&mut lines_iter)?;

    for (line_no, line) in lines_iter {
        let line =
            line.map_err(|err| format!("Failed to read trips file {}: {}", path.display(), err))?;
        let trimmed = line.trim();
        if trimmed.to_ascii_lowercase().starts_with("origin") {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() != 2 {
                return Err(format!(
                    "Line {:}: Invalid origin line format: '{}'",
                    line_no + 1,
                    trimmed
                ));
            }
            let origin_id = parts[1].parse::<i64>().map_err(|err| {
                format!("{}:{}: Invalid origin id, {}", line_no, path.display(), err)
            })?;
            let origin_node_idx = ext_graph.node_idx_by_id.get(&origin_id).ok_or_else(|| {
                format!(
                    "Line {:}: Origin id {} not found in network nodes",
                    line_no, origin_id
                )
            })?;
            current_origin_idx = Some(*origin_node_idx);
            continue;
        }

        let origin_node_idx = current_origin_idx.ok_or_else(|| {
            format!(
                "Trips entry without origin at {}:{}",
                path.display(),
                line_no
            )
        })?;

        parse_trip_pairs(trimmed, origin_node_idx, &mut demand, ext_graph)?;
    }

    info!(
        "Total demand: {:.6e}",
        demand
            .commodities()
            .iter()
            .map(|c| c.demand)
            .sum_with_accumulator::<MySumAccumulator>()
    );

    Ok(demand)
}
