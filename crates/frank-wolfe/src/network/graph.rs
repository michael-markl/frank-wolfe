use crate::{
    common::{BundleIdx, EdgeIdx, Float, NodeIdx, PermitIdx},
    network::graph_ops::GraphOps,
};

#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub enum LpfMode {
    BPR,
    C,
    OP,
}

#[derive(Clone)]
pub struct EdgeParams {
    pub mode: LpfMode,
    pub toll: Float,
    pub offset: Float,
    pub ff_time: Float,
    pub beta: Float,
    pub capacity: Float,
    pub length: Float,
}

pub struct Edge {
    pub tail: NodeIdx,
    pub head: NodeIdx,
    pub bundle: BundleIdx,

    pub params: EdgeParams,
}

pub struct Permit {
    pub params: EdgeParams,
}

pub struct Node {
    incoming_edges: Vec<EdgeIdx>,
    outgoing_edges: Vec<EdgeIdx>,
    allows_through_traffic: bool,
}

/// A directed multi-graph.
///
/// Nodes are indexed from 0 to num_nodes - 1, edges are indexed from 0 to num_edges - 1.
/// We allow multiple edges to share both tail and head nodes (i.e. parallel edges).
/// We allow loops.
///
/// INVARIANT: All edge and node indices in the graph are within bounds.
/// INVARIANT: nodes[i].incoming_edges and nodes[i].outgoing_edges contain exactly the indices of edges with head or tail i, respectively.
pub struct Graph {
    edges: Vec<Edge>,
    nodes: Vec<Node>,
    permits: Vec<Permit>,
}

impl Graph {
    pub fn empty() -> Self {
        Self {
            edges: Vec::new(),
            nodes: Vec::new(),
            permits: Vec::new(),
        }
    }

    pub fn edge(&self, edge_idx: EdgeIdx) -> &Edge {
        &self.edges[edge_idx]
    }

    pub fn edge_mut(&mut self, edge_idx: EdgeIdx) -> &mut Edge {
        &mut self.edges[edge_idx]
    }

    pub fn permit(&self, permit_idx: PermitIdx) -> &Permit {
        &self.permits[permit_idx]
    }

    pub fn add_permit(&mut self, permit_params: EdgeParams) -> PermitIdx {
        let permit_idx = self.permits.len();
        self.permits.push(Permit {
            params: permit_params,
        });
        permit_idx
    }

    pub fn add_node(&mut self, allows_through_traffic: bool) -> NodeIdx {
        let node_idx = self.nodes.len();
        self.nodes.push(Node {
            incoming_edges: Vec::new(),
            outgoing_edges: Vec::new(),
            allows_through_traffic,
        });
        node_idx
    }

    pub fn add_edge(
        &mut self,
        tail: NodeIdx,
        head: NodeIdx,
        bundle: BundleIdx,
        edge_params: EdgeParams,
    ) -> Result<EdgeIdx, String> {
        if tail >= self.nodes.len() {
            return Err(format!(
                "Tail node index {} out of bounds (num nodes: {})",
                tail,
                self.nodes.len()
            ));
        }
        if head >= self.nodes.len() {
            return Err(format!(
                "Head node index {} out of bounds (num nodes: {})",
                head,
                self.nodes.len()
            ));
        }
        let edge_idx = self.edges.len();
        self.edges.push(Edge {
            tail,
            head,
            bundle,
            params: edge_params,
        });
        self.nodes[tail].outgoing_edges.push(edge_idx);
        self.nodes[head].incoming_edges.push(edge_idx);
        Ok(edge_idx)
    }

    pub fn permit_mut(&mut self, permit_idx: PermitIdx) -> &mut Permit {
        &mut self.permits[permit_idx]
    }
}

impl GraphOps for Graph {
    fn incoming_edges(&self, node_idx: NodeIdx) -> impl Iterator<Item = EdgeIdx> {
        self.nodes[node_idx].incoming_edges.iter().cloned()
    }

    fn outgoing_edges(&self, node_idx: NodeIdx) -> impl Iterator<Item = EdgeIdx> {
        self.nodes[node_idx].outgoing_edges.iter().cloned()
    }

    fn edge_tail(&self, edge_idx: EdgeIdx) -> NodeIdx {
        self.edges[edge_idx].tail
    }

    fn edge_head(&self, edge_idx: EdgeIdx) -> NodeIdx {
        self.edges[edge_idx].head
    }

    fn edge_bundle(&self, edge_idx: EdgeIdx) -> BundleIdx {
        self.edges[edge_idx].bundle
    }

    fn node_allows_through_traffic(&self, node_idx: NodeIdx) -> bool {
        self.nodes[node_idx].allows_through_traffic
    }

    fn edge_cost_lower_bound(&self, edge_idx: EdgeIdx) -> Float {
        self.edges[edge_idx].params.ff_time
    }

    fn num_edges(&self) -> usize {
        self.edges.len()
    }

    fn num_nodes(&self) -> usize {
        self.nodes.len()
    }

    fn num_permits(&self) -> usize {
        self.permits.len()
    }
}
