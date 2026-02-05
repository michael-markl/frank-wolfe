use priority_queue::PriorityQueue;
use rayon::{
    iter::{IndexedParallelIterator, ParallelIterator},
    slice::ParallelSliceMut,
};

use crate::{
    col::{HashMap, HashSet, map_new, set_new},
    common::{BundleIdx, Float},
};

type NodeIdx = usize;
type DestinationIdx = usize;
type EdgeIdx = usize;

pub struct AStarTable {
    num_nodes: NodeIdx,

    /// A row per destination, a column per node.
    /// The value distances[i, j] is a lower bound on the distance from j to i.
    distances: Vec<Float>,
}

pub trait GraphOps {
    fn node_idx_by_destination(&self, destination_idx: DestinationIdx) -> NodeIdx;

    fn incoming_edges(&self, node_idx: NodeIdx) -> impl Iterator<Item = EdgeIdx>;

    fn outgoing_edges(&self, node_idx: NodeIdx) -> impl Iterator<Item = EdgeIdx>;

    fn edge_cost_lower_bound(&self, edge_idx: EdgeIdx) -> Float;

    fn edge_tail(&self, edge_idx: EdgeIdx) -> NodeIdx;

    fn edge_head(&self, edge_idx: EdgeIdx) -> NodeIdx;

    fn edge_bundle(&self, edge_idx: EdgeIdx) -> BundleIdx;

    fn node_allows_through_traffic(&self, node_idx: NodeIdx) -> bool;
}

impl AStarTable {
    pub fn create(num_nodes: NodeIdx, num_destinations: NodeIdx) -> AStarTable {
        AStarTable {
            num_nodes,
            distances: vec![Float::INFINITY; num_nodes * num_destinations],
        }
    }

    pub fn fill_table(&mut self, graph: &(impl GraphOps + Sync)) {
        self.distances
            .par_chunks_exact_mut(self.num_nodes)
            .enumerate()
            .for_each(|(destination_idx, distances)| {
                fill_row(graph, distances, destination_idx);
            });
    }

    pub fn get_lower_bound(&self, node_idx: NodeIdx, destination_idx: DestinationIdx) -> Float {
        self.distances[destination_idx * self.num_nodes + node_idx]
    }
}

fn fill_row(graph: &impl GraphOps, distances: &mut [Float], destination_idx: DestinationIdx) {
    // Use Float::MAX as earliest arrival for nodes not reaching the destination.
    for it in distances.iter_mut() {
        *it = Float::MAX;
    }

    let destination_node_idx = graph.node_idx_by_destination(destination_idx);

    #[derive(PartialOrd)]
    struct QueueValue {
        cost: Float,
    }

    impl Ord for QueueValue {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            // Reverse order for min-heap
            other.cost.total_cmp(&self.cost)
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
                if distances[tail_idx] != Float::MAX {
                    // Already settled
                    debug_assert!(
                        distances[tail_idx] <= cost + graph.edge_cost_lower_bound(edge_idx)
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
