use crate::{
    collections::{HashSet, set_new},
    common::{EdgeIdx, NodeIdx},
    network::graph_ops::GraphOps,
};

/// Returns the nodes reachable from `origin` along outgoing edges accepted by `edge_okay`.
/// The origin itself is always included and must be a valid node index.
/// Uses depth-first traversal and visits each reachable node once.
pub fn reachable_nodes(
    graph: &impl GraphOps,
    origin: NodeIdx,
    mut edge_okay: impl FnMut(EdgeIdx) -> bool,
) -> HashSet<NodeIdx> {
    let mut visited = set_new();
    let mut stack = vec![origin];
    while let Some(node_idx) = stack.pop() {
        if !visited.insert(node_idx) {
            continue;
        }
        for edge_idx in graph.outgoing_edges(node_idx) {
            if edge_okay(edge_idx) {
                stack.push(graph.edge_head(edge_idx));
            }
        }
    }
    visited
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        common::BUNDLE_IDX_EMPTY,
        network::graph::{EdgeParams, Graph, LpfMode},
    };

    #[test]
    fn filters_directed_edges_with_cycles_and_alternative_paths() {
        let mut graph = Graph::empty();
        for _ in 0..5 {
            graph.add_node(true);
        }
        let params = EdgeParams {
            mode: LpfMode::C,
            toll: 0.0,
            toll_linear: 0.0,
            externality_linear: 0.0,
            offset: 0.0,
            ff_time: 1.0,
            beta: 0.0,
            capacity: 1.0,
            length: 1.0,
        };
        // Reject the direct path to 2, but allow reaching it through 1.
        // Node 3 lies behind a rejected edge; node 4 has only an incoming path to 0.
        for (tail, head) in [(0, 2), (0, 1), (1, 2), (2, 0), (2, 3), (4, 0), (1, 1)] {
            graph
                .add_edge(tail, head, BUNDLE_IDX_EMPTY, params.clone())
                .unwrap();
        }
        let mut examined = Vec::new();
        let reachable = reachable_nodes(&graph, 0, |edge_idx| {
            examined.push(edge_idx);
            edge_idx != 0 && edge_idx != 4
        });
        assert_eq!(reachable, [0, 1, 2].into_iter().collect());
        examined.sort_unstable();
        assert_eq!(examined, [0, 1, 2, 3, 4, 6]);
        assert_eq!(
            reachable_nodes(&graph, 0, |_| false),
            [0].into_iter().collect()
        );
        assert_eq!(
            reachable_nodes(&graph, 3, |_| true),
            [3].into_iter().collect()
        );
    }
}
