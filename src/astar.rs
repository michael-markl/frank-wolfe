use priority_queue::PriorityQueue;
use rayon::{
    iter::{IndexedParallelIterator, ParallelIterator},
    slice::ParallelSliceMut,
};

use crate::{
    common::{DestinationIdx, Float, NodeIdx},
    demand::DemandOps,
    graph_ops::GraphOps,
};

pub struct AStarTable {
    num_nodes: NodeIdx,

    /// A row per destination, a column per node.
    /// The value distances[i, j] is a lower bound on the distance from j to i.
    distances: Vec<Float>,
}

impl AStarTable {
    pub fn create(num_nodes: NodeIdx, num_destinations: NodeIdx) -> AStarTable {
        AStarTable {
            num_nodes,
            distances: vec![0.0; num_nodes * num_destinations],
        }
    }

    pub fn fill_table(&mut self, graph: &(impl GraphOps + Sync), demand: &(impl DemandOps + Sync)) {
        self.distances.fill(Float::INFINITY);

        self.distances
            .par_chunks_exact_mut(self.num_nodes)
            .enumerate()
            .for_each(|(destination_idx, distances)| {
                fill_row(graph, demand, distances, destination_idx);
            });
    }
}

pub trait AStarBoundOps {
    /// Given a node v and a destination t, returns a lower bound on the cost of any feasible v-t-path.
    ///
    /// Here, "feasible" means that the path may not contain an inner node that disallows through traffic.
    fn get_lower_bound(&self, node_idx: NodeIdx, destination_idx: DestinationIdx) -> Float;
}

impl AStarBoundOps for AStarTable {
    fn get_lower_bound(&self, node_idx: NodeIdx, destination_idx: DestinationIdx) -> Float {
        self.distances[destination_idx * self.num_nodes + node_idx]
    }
}

fn fill_row(
    graph: &impl GraphOps,
    demand: &impl DemandOps,
    distances: &mut [Float],
    destination_idx: DestinationIdx,
) {
    // Use Float::INFINITY as earliest arrival for nodes not reaching the destination.
    for it in distances.iter_mut() {
        *it = Float::INFINITY;
    }

    let destination_node_idx = demand.node_idx_by_destination(destination_idx);

    #[derive(Debug)]
    struct QueueValue {
        cost: Float,
    }

    impl Ord for QueueValue {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            // Reverse order for min-heap
            other.cost.total_cmp(&self.cost)
        }
    }

    impl PartialOrd for QueueValue {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }

    impl PartialEq for QueueValue {
        fn eq(&self, other: &Self) -> bool {
            self.cmp(other).is_eq()
        }
    }

    impl Eq for QueueValue {}

    // Do a backwards Dijkstra search using the lower bound costs.

    let mut queue: PriorityQueue<NodeIdx, QueueValue> = PriorityQueue::new();
    queue.push(destination_node_idx, QueueValue { cost: 0.0 });

    while let Some((node_idx, QueueValue { cost })) = queue.pop() {
        distances[node_idx] = cost;

        if node_idx == destination_node_idx || graph.node_allows_through_traffic(node_idx) {
            for edge_idx in graph.incoming_edges(node_idx) {
                let tail_idx = graph.edge_tail(edge_idx);
                if distances[tail_idx] != Float::INFINITY {
                    // Already settled
                    debug_assert!(
                        distances[tail_idx] <= cost + graph.edge_cost_lower_bound(edge_idx),
                        "Destination {}: Lower bound costs must be consistent, but found a shorter path to node {} via edge {}: {} < {} + {} = {}",
                        destination_idx,
                        tail_idx,
                        edge_idx,
                        distances[tail_idx],
                        cost,
                        graph.edge_cost_lower_bound(edge_idx),
                        cost + graph.edge_cost_lower_bound(edge_idx)
                    );
                    continue;
                }
                let new_cost = cost + graph.edge_cost_lower_bound(edge_idx);
                // "push_increase", since the priority queue is a max-heap.
                queue.push_increase(tail_idx, QueueValue { cost: new_cost });
            }
        }
    }
}
