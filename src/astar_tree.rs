use std::{collections::hash_map::Entry, sync::RwLock};

use priority_queue::PriorityQueue;

use crate::{
    astar::AStarTable,
    bundle_index::BundleIndex,
    col::{HashMap, map_new},
    common::{BUNDLE_IDX_EMPTY, BundleIdx, DestinationIdx, EdgeIdx, Float, NodeIdx, PermitIdx},
    demand::DemandOps,
    graph_ops::GraphOps,
};

pub trait ShortestPathCostOps {
    fn get_edge_cost(&self, edge_idx: EdgeIdx) -> Float;

    fn get_permit_cost(&self, permit_idx: PermitIdx) -> Float;
}

#[derive(Debug)]
struct Predecessor {
    previous_bundle_idx: BundleIdx,
    edge_idx: EdgeIdx,
}

struct AStarTreeDistanceEntry {
    by_bundle: HashMap<BundleIdx, (Float, Predecessor)>,
    cheapest: BundleIdx,
}

pub struct AStarTree {
    source_idx: NodeIdx,
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

impl AStarTree {
    pub fn new(source_idx: NodeIdx) -> AStarTree {
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
            distances: map_new(),
            destination_idx: None,
            queue,
        }
    }

    fn thru_aware_lower_bound(
        source_idx: NodeIdx,
        table: &AStarTable,
        node_idx: NodeIdx,
        destination_node_idx: NodeIdx,
        destination_idx: DestinationIdx,
        graph: &impl GraphOps,
    ) -> Float {
        if node_idx == source_idx
            || node_idx == destination_node_idx
            || graph.node_allows_through_traffic(node_idx)
        {
            table.get_lower_bound(node_idx, destination_idx)
        } else {
            Float::MAX
        }
    }

    pub fn compute_shortest_path(
        &mut self,
        table: &AStarTable,
        graph: &impl GraphOps,
        demand: &impl DemandOps,
        costs: &impl ShortestPathCostOps,
        destination_idx: DestinationIdx,
        bundles: &RwLock<BundleIndex>,
    ) -> (Float, Vec<EdgeIdx>, BundleIdx) {
        let distance = self.compute_distance(table, graph, demand, costs, destination_idx, bundles);

        let mut path = vec![];
        let mut current_node = demand.node_idx_by_destination(destination_idx);
        let bundle_idx = self
            .distances
            .get(&current_node)
            .expect("Destination must be reachable from source")
            .cheapest;
        while current_node != self.source_idx {
            let entry = self
                .distances
                .get(&current_node)
                .expect("Current node must be reachable from source");
            let (_bundle_idx, predecessor) = &entry.by_bundle[&entry.cheapest];
            path.push(predecessor.edge_idx);
            current_node = graph.edge_tail(predecessor.edge_idx);
        }
        path.reverse();

        (distance, path, bundle_idx)
    }

    pub fn compute_distance(
        &mut self,
        table: &AStarTable,
        graph: &impl GraphOps,
        demand: &impl DemandOps,
        costs: &impl ShortestPathCostOps,
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
            // Recompute the entry cost estimates in the queue.
            self.queue
                .iter_mut()
                .for_each(|((node_idx, _bundle_idx), entry)| {
                    let lower_bound = Self::thru_aware_lower_bound(
                        self.source_idx,
                        table,
                        *node_idx,
                        destination_node_idx,
                        destination_idx,
                        graph,
                    );
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
                        debug_assert!(
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
                    let edge_bundle_idx: BundleIdx = graph.edge_bundle(edge_idx);
                    // TODO: Handle empty set more efficiently.

                    let guard = bundles.read().unwrap();
                    let edge_bundle = guard.get_payload(edge_bundle_idx);
                    let current_bundle = guard.get_payload(bundle_idx);
                    let mut additional_permits_cost = 0.0;
                    for permit in edge_bundle.set_minus_iter(current_bundle) {
                        additional_permits_cost += costs.get_permit_cost(*permit);
                    }
                    let new_bundle = edge_bundle.union(current_bundle);

                    let new_bundle_idx = guard.find_idx(&new_bundle);
                    drop(guard);
                    let new_bundle_idx = new_bundle_idx
                        .unwrap_or_else(|| bundles.write().unwrap().transfer_element(new_bundle));

                    let edge_cost = costs.get_edge_cost(edge_idx);
                    let new_max_cost_from_source =
                        entry.max_cost_from_source + edge_cost + additional_permits_cost;

                    if let Some(existing_entry) = self
                        .distances
                        .get(&head)
                        .and_then(|it| it.by_bundle.get(&new_bundle_idx))
                    {
                        debug_assert!(
                            existing_entry.0 <= new_max_cost_from_source + 1e-8,
                            "New path to node {} with bundle idx {} has higher cost than existing path with same bundle idx: {} > {}",
                            head,
                            new_bundle_idx,
                            new_max_cost_from_source,
                            existing_entry.0
                        );
                        continue;
                    }

                    self.queue.push_increase(
                        (head, new_bundle_idx),
                        TreeEntry {
                            max_cost_from_source: new_max_cost_from_source,
                            cost_estimate_to_destination: new_max_cost_from_source
                                + Self::thru_aware_lower_bound(
                                    self.source_idx,
                                    table,
                                    head,
                                    destination_node_idx,
                                    destination_idx,
                                    graph,
                                ),
                            predecessor: Some(Predecessor {
                                previous_bundle_idx: bundle_idx,
                                edge_idx,
                            }),
                        },
                    );
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
