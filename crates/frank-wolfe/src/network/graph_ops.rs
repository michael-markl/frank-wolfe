use crate::common::{BundleIdx, EdgeIdx, Float, NodeIdx};

pub trait GraphOps {
    fn num_edges(&self) -> usize;

    fn num_nodes(&self) -> usize;

    fn num_permits(&self) -> usize;

    fn incoming_edges(&self, node_idx: NodeIdx) -> impl Iterator<Item = EdgeIdx>;

    fn outgoing_edges(&self, node_idx: NodeIdx) -> impl Iterator<Item = EdgeIdx>;

    fn edge_cost_lower_bound(&self, edge_idx: EdgeIdx) -> Float;

    fn edge_tail(&self, edge_idx: EdgeIdx) -> NodeIdx;

    fn edge_head(&self, edge_idx: EdgeIdx) -> NodeIdx;

    fn edge_bundle(&self, edge_idx: EdgeIdx) -> BundleIdx;

    fn node_allows_through_traffic(&self, node_idx: NodeIdx) -> bool;
}
