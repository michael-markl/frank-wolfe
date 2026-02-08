use crate::{
    common::{BundleIdx, EdgeIdx, Float, NodeIdx, PermitIdx},
    graph_ops::GraphOps,
};

pub enum EdgeMode {
    Constant,
    BPR,
    OP,
}

pub struct EdgeParams {
    pub mode: EdgeMode,
    pub toll: Float,
    pub offset: Float,
    pub alpha: Float,
    pub beta: Float,
    pub gamma: Float,
    pub length: Float,
}

pub struct Edge {
    pub tail: NodeIdx,
    pub head: NodeIdx,
    pub bundle: BundleIdx,

    pub edge_params: EdgeParams,
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

    pub fn node(&self, node_idx: NodeIdx) -> &Node {
        &self.nodes[node_idx]
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
            edge_params,
        });
        self.nodes[tail].outgoing_edges.push(edge_idx);
        self.nodes[head].incoming_edges.push(edge_idx);
        Ok(edge_idx)
    }

    pub fn new(edges: Vec<Edge>, nodes: Vec<Node>, permits: Vec<Permit>) -> Result<Self, String> {
        let num_edges = edges.len();
        let num_nodes = nodes.len();
        for (edge_idx, edge) in edges.iter().enumerate() {
            if edge.tail >= num_nodes {
                return Err(format!(
                    "Edge {} has tail node index {} out of bounds (num nodes: {})",
                    edge_idx, edge.tail, num_nodes
                ));
            }
            if edge.head >= num_nodes {
                return Err(format!(
                    "Edge {} has head node index {} out of bounds (num nodes: {})",
                    edge_idx, edge.head, num_nodes
                ));
            }
            if nodes[edge.tail]
                .outgoing_edges
                .iter()
                .filter(|&&idx| idx == edge_idx)
                .count()
                != 1
            {
                return Err(format!(
                    "Edge {} is not listed exactly once in outgoing edges of its tail node {}",
                    edge_idx, edge.tail
                ));
            }
            if nodes[edge.head]
                .incoming_edges
                .iter()
                .filter(|&&idx| idx == edge_idx)
                .count()
                != 1
            {
                return Err(format!(
                    "Edge {} is not listed exactly once in incoming edges of its head node {}",
                    edge_idx, edge.head
                ));
            }
        }

        for (node_idx, node) in nodes.iter().enumerate() {
            for &edge_idx in &node.incoming_edges {
                if edge_idx >= num_edges {
                    return Err(format!(
                        "Node {} has incoming edge index {} out of bounds (num edges: {})",
                        node_idx, edge_idx, num_edges
                    ));
                }
                if edges[edge_idx].head != node_idx {
                    return Err(format!(
                        "Node {} has incoming edge index {} whose head is not equal to the node index",
                        node_idx, edge_idx
                    ));
                }
            }
            for &edge_idx in &node.outgoing_edges {
                if edge_idx >= num_edges {
                    return Err(format!(
                        "Node {} has outgoing edge index {} out of bounds (num edges: {})",
                        node_idx, edge_idx, num_edges
                    ));
                }
                if edges[edge_idx].tail != node_idx {
                    return Err(format!(
                        "Node {} has outgoing edge index {} whose tail is not equal to the node index",
                        node_idx, edge_idx
                    ));
                }
            }
        }

        Ok(Self {
            edges,
            nodes,
            permits,
        })
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
        self.edges[edge_idx].edge_params.alpha
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
