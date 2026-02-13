use std::{collections::hash_map::Entry, sync::RwLock};

use accurate::traits::SumWithAccumulator;
use log::trace;
use priority_queue::PriorityQueue;

use crate::{
    astar::AStarBoundOps,
    bundle_index::BundleIndex,
    col::{HashMap, map_new},
    common::{
        BUNDLE_IDX_EMPTY, BundleIdx, DestinationIdx, EdgeIdx, Float, MySumAccumulator, NodeIdx,
        PermitIdx,
    },
    demand::DemandOps,
    graph_ops::GraphOps,
};

pub trait CostValuesOps {
    fn get_edge_cost(&self, edge_idx: EdgeIdx) -> Float;

    fn get_permit_cost(&self, permit_idx: PermitIdx) -> Float;
}

#[derive(Debug)]
struct Predecessor {
    edge_idx: EdgeIdx,
    prev_bundle_idx: BundleIdx,
}

#[derive(Debug)]
struct AStarTreeDistanceEntry {
    by_bundle: HashMap<BundleIdx, (Float, Predecessor)>,
    cheapest: BundleIdx,
}

pub struct AStarTree<'a, B: AStarBoundOps> {
    source_idx: NodeIdx,
    bounds: &'a B,

    /// distances[w][B] is the cost of a minimum s-w-path p, including the bundle cost, among paths p that require exactly the permits in bundle B.
    distances: HashMap<NodeIdx, AStarTreeDistanceEntry>,

    destination_idx: Option<DestinationIdx>,
    // For a given destination t, the queue contains pairs (w, B) of node and bundles, with value the current best known cost of an s-w-t path
    queue: PriorityQueue<(NodeIdx, BundleIdx), TreeEntry>,
}

struct TreeEntry {
    max_cost_from_source: Float,
    /// Estimated total cost from source to destination via this node.
    /// Computed as max_cost_from_source + AStar-lower-bound
    cost_estimate_to_destination: Float,
    predecessor: Option<Predecessor>,
}

impl Ord for TreeEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Reverse order for min-heap
        other
            .cost_estimate_to_destination
            .total_cmp(&self.cost_estimate_to_destination)
            // If both are INFINITY, then prefer a smaller max_cost_from_source.
            // This makes sure that we can relax edges to nodes that don't reach the current destination.
            // However, if the destination changes, we recompute the cost_estimate_to_destination based
            // on the new destination.
            // (Technically, we could just use a different priority_queue API when relaxing edges.)
            .then_with(|| {
                other
                    .max_cost_from_source
                    .total_cmp(&self.max_cost_from_source)
            })
    }
}

impl PartialOrd for TreeEntry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for TreeEntry {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other).is_eq()
    }
}
impl Eq for TreeEntry {}

impl<'a, B: AStarBoundOps> AStarTree<'a, B> {
    pub fn new(source_idx: NodeIdx, bounds: &'a B) -> AStarTree<'a, B> {
        trace!("Creating A* tree with source node {}", source_idx);

        let mut queue = PriorityQueue::<(NodeIdx, BundleIdx), TreeEntry>::new();
        queue.push(
            (source_idx, BUNDLE_IDX_EMPTY),
            TreeEntry {
                max_cost_from_source: 0.0,
                cost_estimate_to_destination: 0.0,
                predecessor: None,
            },
        );
        AStarTree {
            source_idx,
            bounds,
            distances: map_new(),
            destination_idx: None,
            queue,
        }
    }

    fn thru_aware_lower_bound(
        source_idx: NodeIdx,
        bounds: &B,
        node_idx: NodeIdx,
        destination_node_idx: NodeIdx,
        destination_idx: DestinationIdx,
        graph: &impl GraphOps,
    ) -> Float {
        if node_idx == source_idx
            || node_idx == destination_node_idx
            || graph.node_allows_through_traffic(node_idx)
        {
            bounds.get_lower_bound(node_idx, destination_idx)
        } else {
            Float::INFINITY
        }
    }

    pub fn compute_shortest_path(
        &mut self,
        graph: &impl GraphOps,
        demand: &impl DemandOps,
        costs: &impl CostValuesOps,
        destination_idx: DestinationIdx,
        bundles: &RwLock<BundleIndex>,
    ) -> (Float, Vec<EdgeIdx>, BundleIdx) {
        let destination_node_idx = demand.node_idx_by_destination(destination_idx);
        if destination_node_idx == self.source_idx {
            return (0.0, vec![], BUNDLE_IDX_EMPTY);
        }

        let distance = self.compute_distance(graph, demand, costs, destination_idx, bundles);

        let destination_entry = self
            .distances
            .get(&destination_node_idx)
            .expect("Destination must be reachable from source");

        let mut path = vec![];

        let mut current_node = destination_node_idx;
        let mut current_bundle_idx = destination_entry.cheapest;
        while current_node != self.source_idx {
            let entry = self
                .distances
                .get(&current_node)
                .expect("Current node must be reachable from source");
            let (_distance, predecessor) = &entry.by_bundle[&current_bundle_idx];
            path.push(predecessor.edge_idx);
            current_node = graph.edge_tail(predecessor.edge_idx);
            current_bundle_idx = predecessor.prev_bundle_idx;
        }
        path.reverse();

        if cfg!(debug_assertions) {
            let (path_cost, path_bundle_idx) = compute_path_cost(&path, costs, bundles, graph);
            debug_assert!(
                (path_cost - distance).abs() < 1e-8
                    && path_bundle_idx == destination_entry.cheapest,
                "Reconstructed path cost {} differs from computed distance: {}, \
                or reconstructed path bundle idx {} differs from computed cheapest bundle idx {}, path: {:?}",
                path_cost,
                distance,
                path_bundle_idx,
                destination_entry.cheapest,
                path
            );
        }

        (distance, path, destination_entry.cheapest)
    }

    pub fn compute_distance(
        &mut self,
        graph: &impl GraphOps,
        demand: &impl DemandOps,
        costs: &impl CostValuesOps,
        destination_idx: DestinationIdx,
        bundles: &RwLock<BundleIndex>,
    ) -> Float {
        let destination_node_idx = demand.node_idx_by_destination(destination_idx);

        if destination_node_idx == self.source_idx {
            return 0.0;
        }

        if let Some(entry) = self.distances.get(&destination_node_idx) {
            return entry.by_bundle[&entry.cheapest].0;
        }

        if self.destination_idx != Some(destination_idx) {
            self.destination_idx = Some(destination_idx);
            trace!(
                "Destination changed to {}, {}",
                destination_idx, destination_node_idx
            );
            // Recompute the entry cost estimates in the queue.
            self.queue
                .iter_mut()
                .for_each(|((node_idx, bundle_idx), entry)| {
                    let lower_bound = Self::thru_aware_lower_bound(
                        self.source_idx,
                        self.bounds,
                        *node_idx,
                        destination_node_idx,
                        destination_idx,
                        graph,
                    );
                    trace!("Updating cost estimate for node {}, bundle {}, distance {:.3}, lower_bound: {:.3}, estimate from {:.3} to {:.3}", node_idx, bundle_idx, entry.max_cost_from_source, lower_bound, entry.cost_estimate_to_destination, entry.max_cost_from_source + lower_bound);
                    entry.cost_estimate_to_destination = entry.max_cost_from_source + lower_bound;
                });
        }

        while let Some(((node_idx, bundle_idx), entry)) = self.queue.pop() {
            let pred_node = entry
                .predecessor
                .as_ref()
                .map(|pred| graph.edge_tail(pred.edge_idx));
            assert!(
                pred_node
                    .iter()
                    .all(|it| it == &self.source_idx || graph.node_allows_through_traffic(*it)),
                "Predecessor nodes must allow through traffic, but found predecessor node {} for node {}, which does not allow through traffic",
                pred_node.unwrap_or(usize::MAX),
                node_idx
            );

            // Update self.distances.
            if node_idx != self.source_idx {
                let predecessor = entry
                    .predecessor
                    .expect("Predecessor must be set for non-source nodes");

                trace!(
                    "Settling node {}, bundle {}, distance from source: {:.3}, predecessor edge: {:?}, predecessor bundle: {}, pred node: {:?}",
                    node_idx,
                    bundle_idx,
                    entry.max_cost_from_source,
                    predecessor.edge_idx,
                    predecessor.prev_bundle_idx,
                    pred_node
                );

                let distance_entry = self.distances.entry(node_idx);
                match distance_entry {
                    Entry::Vacant(vacant) => {
                        let mut by_bundle = map_new();
                        by_bundle.insert(bundle_idx, (entry.max_cost_from_source, predecessor));
                        vacant.insert(AStarTreeDistanceEntry {
                            by_bundle,
                            cheapest: bundle_idx,
                        });
                    }
                    Entry::Occupied(mut occupied) => {
                        let distance_entry = occupied.get_mut();
                        let previous = distance_entry
                            .by_bundle
                            .insert(bundle_idx, (entry.max_cost_from_source, predecessor));
                        assert!(
                            previous.is_none(),
                            "Node {} was reached multiple times with the same bundle idx {}, which should not happen in A*: {:?}",
                            node_idx,
                            bundle_idx,
                            distance_entry.by_bundle
                        );
                        assert!(
                            distance_entry.by_bundle[&distance_entry.cheapest].0
                                <= entry.max_cost_from_source,
                            "New path to node {} with bundle idx {} has higher cost than existing path with bundle idx {}: {} > {}",
                            node_idx,
                            bundle_idx,
                            distance_entry.cheapest,
                            entry.max_cost_from_source,
                            distance_entry.by_bundle[&distance_entry.cheapest].0
                        );
                    }
                }
            }

            // Enqueue neighbors.
            if node_idx == self.source_idx || graph.node_allows_through_traffic(node_idx) {
                for edge_idx in graph.outgoing_edges(node_idx) {
                    let head = graph.edge_head(edge_idx);
                    if head == self.source_idx {
                        continue;
                    }

                    let (permit_aware_edge_cost, new_bundle_idx) =
                        permit_aware_edge_cost(bundle_idx, edge_idx, costs, graph, bundles);
                    let new_max_cost_from_source =
                        entry.max_cost_from_source + permit_aware_edge_cost;

                    let cost_estimate_to_destination = new_max_cost_from_source
                        + Self::thru_aware_lower_bound(
                            self.source_idx,
                            self.bounds,
                            head,
                            destination_node_idx,
                            destination_idx,
                            graph,
                        );

                    trace!(
                        "Relaxing edge {},{} -> {},{}, distance from source {:.3} + {:.3} = {:.3}, cost estimate to destination {:.3}",
                        node_idx,
                        bundle_idx,
                        head,
                        new_bundle_idx,
                        entry.max_cost_from_source,
                        permit_aware_edge_cost,
                        new_max_cost_from_source,
                        cost_estimate_to_destination
                    );

                    if let Some(existing_entry) = self
                        .distances
                        .get(&head)
                        .and_then(|it| it.by_bundle.get(&new_bundle_idx))
                    {
                        assert!(
                            existing_entry.0 <= new_max_cost_from_source + 1e-8,
                            "New path to node {} with bundle idx {} has higher cost than existing path with same bundle idx: {} > {}",
                            head,
                            new_bundle_idx,
                            new_max_cost_from_source,
                            existing_entry.0
                        );
                        continue;
                    }

                    let res = self.queue.push_increase(
                        (head, new_bundle_idx),
                        TreeEntry {
                            max_cost_from_source: new_max_cost_from_source,
                            cost_estimate_to_destination,
                            predecessor: Some(Predecessor {
                                edge_idx,
                                prev_bundle_idx: bundle_idx,
                            }),
                        },
                    );
                    if let Some(entry) = res
                        && (entry.max_cost_from_source != new_max_cost_from_source
                            || entry.cost_estimate_to_destination != cost_estimate_to_destination)
                    {
                        trace!(
                            "Existing priority ({:.3}, {:.3}) was overwritten.",
                            entry.max_cost_from_source, entry.cost_estimate_to_destination
                        );
                    }
                }
            }

            // Early stop, if we found the destination.
            if node_idx == destination_node_idx {
                return entry.max_cost_from_source;
            }
        }

        panic!(
            "No path found from source node {} to destination node {}, idx {}",
            self.source_idx, destination_node_idx, destination_idx
        );
    }
}

/// Returns the cost and a new bundle idx for traversing the edge, which may require the purchase of additional permits.
fn permit_aware_edge_cost(
    from_bundle_idx: BundleIdx,
    edge_idx: EdgeIdx,
    costs: &impl CostValuesOps,
    graph: &impl GraphOps,
    bundles: &RwLock<BundleIndex>,
) -> (Float, BundleIdx) {
    let edge_bundle_idx: BundleIdx = graph.edge_bundle(edge_idx);
    let edge_cost = costs.get_edge_cost(edge_idx);
    if edge_bundle_idx == BUNDLE_IDX_EMPTY {
        return (edge_cost, from_bundle_idx);
    }

    let guard = bundles.read().unwrap();
    let edge_bundle = guard.get_payload(edge_bundle_idx);
    let from_bundle = guard.get_payload(from_bundle_idx);
    let additional_permits_cost = edge_bundle
        .set_minus_iter(from_bundle)
        .map(|&it| costs.get_permit_cost(it))
        .sum_with_accumulator::<MySumAccumulator>();
    let new_bundle = edge_bundle.union(from_bundle);

    let tmp_new_bundle_idx = guard.find_idx(&new_bundle);
    drop(guard);
    let new_bundle_idx =
        tmp_new_bundle_idx.unwrap_or_else(|| bundles.write().unwrap().transfer_element(new_bundle));
    (edge_cost + additional_permits_cost, new_bundle_idx)
}

fn compute_path_cost(
    path: &[EdgeIdx],
    costs: &impl CostValuesOps,
    bundles: &RwLock<BundleIndex>,
    graph: &impl GraphOps,
) -> (Float, BundleIdx) {
    let edge_costs = path
        .iter()
        .map(|&edge_idx| costs.get_edge_cost(edge_idx))
        .sum::<Float>();
    let mut bundle_idx = BUNDLE_IDX_EMPTY;
    for &edge_idx in path {
        let edge_bundle_idx = graph.edge_bundle(edge_idx);
        if edge_bundle_idx != BUNDLE_IDX_EMPTY {
            let guard = bundles.read().unwrap();
            let edge_bundle = guard.get_payload(edge_bundle_idx);
            bundle_idx = guard
                .find_idx(&edge_bundle.union(guard.get_payload(bundle_idx)))
                .expect("Bundle should already be indexed..");
        }
    }

    let permit_costs = bundles
        .read()
        .unwrap()
        .get_payload(bundle_idx)
        .permits()
        .map(|permit_idx| costs.get_permit_cost(permit_idx))
        .sum::<Float>();

    (edge_costs + permit_costs, bundle_idx)
}

#[cfg(test)]
mod tests {
    use crate::astar::AStarTable;

    use super::*;

    struct TestGraph {
        edges: Vec<(NodeIdx, NodeIdx)>, // (tail, head)
        edge_costs: Vec<Float>,
    }

    impl TestGraph {
        fn new() -> Self {
            TestGraph {
                edges: Vec::new(),
                edge_costs: Vec::new(),
            }
        }

        fn add_edge(&mut self, tail: NodeIdx, head: NodeIdx, cost: Float) -> EdgeIdx {
            let edge_idx = self.edges.len();
            self.edges.push((tail, head));
            self.edge_costs.push(cost);
            edge_idx
        }

        fn build_outgoing_edges(&self) -> Vec<Vec<EdgeIdx>> {
            let max_node = self
                .edges
                .iter()
                .map(|(tail, head)| (*tail).max(*head))
                .max()
                .unwrap_or(0);
            let mut outgoing = vec![Vec::new(); max_node + 1];
            for (edge_idx, (tail, _)) in self.edges.iter().enumerate() {
                outgoing[*tail].push(edge_idx);
            }
            outgoing
        }
    }

    impl GraphOps for TestGraph {
        fn num_edges(&self) -> usize {
            self.edges.len()
        }

        fn num_nodes(&self) -> usize {
            self.edges
                .iter()
                .map(|(tail, head)| (*tail).max(*head))
                .max()
                .map(|m| m + 1)
                .unwrap_or(0)
        }

        fn num_permits(&self) -> usize {
            0
        }

        fn incoming_edges(&self, _node_idx: NodeIdx) -> impl Iterator<Item = EdgeIdx> {
            std::iter::empty()
        }

        fn outgoing_edges(&self, node_idx: NodeIdx) -> impl Iterator<Item = EdgeIdx> {
            self.edges
                .iter()
                .enumerate()
                .filter(move |(_, (tail, _))| *tail == node_idx)
                .map(|(edge_idx, _)| edge_idx)
        }

        fn edge_cost_lower_bound(&self, edge_idx: EdgeIdx) -> Float {
            self.edge_costs[edge_idx]
        }

        fn edge_tail(&self, edge_idx: EdgeIdx) -> NodeIdx {
            self.edges[edge_idx].0
        }

        fn edge_head(&self, edge_idx: EdgeIdx) -> NodeIdx {
            self.edges[edge_idx].1
        }

        fn edge_bundle(&self, _edge_idx: EdgeIdx) -> BundleIdx {
            BUNDLE_IDX_EMPTY
        }

        fn node_allows_through_traffic(&self, _node_idx: NodeIdx) -> bool {
            true
        }
    }

    struct TestDemand {
        destinations: Vec<NodeIdx>,
    }

    impl TestDemand {
        fn new(destinations: Vec<NodeIdx>) -> Self {
            TestDemand { destinations }
        }
    }

    impl DemandOps for TestDemand {
        fn node_idx_by_destination(&self, destination_idx: DestinationIdx) -> NodeIdx {
            self.destinations[destination_idx]
        }

        fn num_destinations(&self) -> usize {
            self.destinations.len()
        }
    }

    impl CostValuesOps for TestGraph {
        fn get_edge_cost(&self, edge_idx: EdgeIdx) -> Float {
            self.edge_costs[edge_idx]
        }

        fn get_permit_cost(&self, _permit_idx: PermitIdx) -> Float {
            0.0
        }
    }

    #[test]
    fn test_compute_distance_same_source_different_destinations() {
        // Create a more complex graph with multiple paths:
        //             1 ----> 4 ----> 5 (dest 1)
        //          1 /    2 /    3  /
        //           /      /       /
        // (source) 0     5/       /1
        //           \    /       /
        //          4 \  / 6     /
        //             2 ----> 3 (dest 0)
        //
        // Path to 5: 0->1->4->5 (costs: 1 + 2 + 3 = 6) or 0->2->4->5 (costs: 4 + 5 + 3 = 12)
        // Path to 3: 0->2->3 (costs: 4 + 6 = 10)

        let mut graph = TestGraph::new();
        graph.add_edge(0, 1, 1.0); // cost 1
        graph.add_edge(0, 2, 4.0); // cost 4
        graph.add_edge(1, 4, 2.0); // cost 2
        graph.add_edge(2, 4, 5.0); // cost 5
        graph.add_edge(4, 5, 3.0); // cost 3
        graph.add_edge(2, 3, 6.0); // cost 6
        graph.add_edge(3, 5, 1.0); // cost 1

        // Create demand: destination 0 is node 5, destination 1 is node 3
        let demand = TestDemand::new(vec![3, 5]);

        // Create a table for lower bounds (all zeros)
        let mut table = AStarTable::create(graph.num_nodes(), 2);
        table.fill_table(&graph, &demand);

        // Create bundle index
        let bundles = RwLock::new(BundleIndex::new());

        // Create A* tree with source 0
        let mut tree = AStarTree::new(0, &table);

        // Compute distance to destination 0 (node 5)
        let dist_to_dest_0 = tree.compute_distance(&graph, &demand, &graph, 0, &bundles);

        // Compute distance to destination 1 (node 3)
        let dist_to_dest_1 = tree.compute_distance(&graph, &demand, &graph, 1, &bundles);

        // Expected shortest path to node 3: 0->2->3 (costs: 4 + 6 = 10)
        assert!(
            (dist_to_dest_0 - 10.0).abs() < 1e-8,
            "Distance to destination 0 should be 10.0, got {}",
            dist_to_dest_0
        );

        // Expected shortest path to node 5: 0->1->4->5 (costs: 1 + 2 + 3 = 6)
        assert!(
            (dist_to_dest_1 - 6.0).abs() < 1e-8,
            "Distance to destination 1 should be 6.0, got {}",
            dist_to_dest_1
        );
    }
}
