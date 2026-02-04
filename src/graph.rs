use std::alloc::Layout;
use std::fmt::{Debug, Write};
use std::hash::Hash;
use std::iter::{empty, once};
use std::mem::MaybeUninit;

use crate::col::{map_new, set_new, HashMap, HashSet};

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct ExtStationId(pub u32);
impl Debug for ExtStationId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("s#{}", self.0))
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct StationIdx(pub u32);
impl Debug for StationIdx {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("_s#{}", self.0))
    }
}
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct EdgeIdx(pub u32);
impl Debug for EdgeIdx {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("e#{}", self.0))
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct NodeIdx(pub u32);

impl Debug for NodeIdx {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("n#{}", self.0))
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct CommodityIdx(pub u32);
impl Debug for CommodityIdx {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("c#{}", self.0))
    }
}


#[derive(PartialEq, Eq, Hash)]
pub struct Path {
    commodity_idx: CommodityIdx,
    edges: [EdgeIdx],
}

impl Path {
    pub fn commodity_idx(&self) -> CommodityIdx {
        self.commodity_idx
    }

    pub fn edges(&self) -> &[EdgeIdx] {
        &self.edges
    }

    pub fn is_outside(&self) -> bool {
        self.edges.is_empty()
    }
}

#[derive(PartialEq, Eq, Hash)]
pub struct PathBox {
    payload: Box<Path>,
}

impl Clone for PathBox {
    fn clone(&self) -> Self {
        Self::new(
            self.payload.commodity_idx,
            self.payload.edges.iter().copied(),
        )
    }
}

impl PathBox {
    pub fn payload(&self) -> &Path {
        &self.payload
    }

    pub fn new(commodity_idx: CommodityIdx, edges: impl ExactSizeIterator<Item = EdgeIdx>) -> Self {
        let layout = Layout::new::<CommodityIdx>()
            .extend(Layout::new::<EdgeIdx>().repeat(edges.len()).unwrap().0)
            .unwrap()
            .0
            .pad_to_align();

        let ptr = unsafe { std::alloc::alloc(layout) };
        if ptr.is_null() {
            std::alloc::handle_alloc_error(layout)
        } else {
            let ptr = std::ptr::from_raw_parts_mut::<Path>(ptr as *mut (), edges.len());
            // safety: ptr points to allocated (but not initialized) Dst
            unsafe {
                let commodity_idx_ptr = std::ptr::addr_of_mut!((*ptr).commodity_idx);
                commodity_idx_ptr.write(commodity_idx);

                let edges_ptr: *mut [EdgeIdx] = std::ptr::addr_of_mut!((*ptr).edges);
                // cast tail to MaybeUninit slice for convenience
                let tail_uninit = &mut *(edges_ptr as *mut [MaybeUninit<EdgeIdx>]);
                for (edge_in_box, edge) in tail_uninit.iter_mut().zip(edges) {
                    edge_in_box.write(edge);
                }
            }
            // safety: ptr points to an allocated Dst, and all fields have been initialized
            let payload = unsafe { Box::from_raw(ptr) };
            Self { payload }
        }
    }
}

#[derive(Debug)]
pub struct EdgePayload {
    pub from: NodeIdx,
    pub to: NodeIdx,
}


#[derive(Debug)]
pub struct NodePayload {
    pub incoming: Vec<EdgeIdx>,
    pub outgoing: Vec<EdgeIdx>,
}

#[derive(Debug)]
pub struct Graph {
    edges: Vec<EdgePayload>,
    nodes: Vec<NodePayload>,
    commodities: Vec<CommodityPayload>,
    station_by_node: Vec<StationIdx>,
    num_stations: usize,
}

#[derive(Debug)]
pub struct ExtODPair {
    pub origin: ExtStationId,
    pub destination: ExtStationId,
}

#[derive(Debug, Clone)]
pub struct ODPair {
    pub origin: StationIdx,
    pub destination: StationIdx,
}

pub type FVal = f64;

pub const EPS: FVal = 1e-9;
pub const EPS_L: FVal = 1e-6;

#[derive(Debug)]
pub struct ExtCommodity {
    pub od_pair: ExtODPair,
    pub demand: FVal,
}

#[derive(Debug, Clone)]
pub struct CommodityPayload {
    pub od_pair: ODPair,
    pub demand: FVal,
    pub spawn_node: Option<NodeIdx>,
}

impl Graph {

    pub fn nodes(&self) -> impl Iterator<Item = (NodeIdx, &NodePayload)> {
        self.nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (NodeIdx(i as u32), n))
    }

    pub fn edges(&self) -> impl Iterator<Item = (EdgeIdx, &EdgePayload)> {
        self.edges
            .iter()
            .enumerate()
            .map(|(i, e)| (EdgeIdx(i as u32), e))
    }

    pub fn commodity(&self, commodity_idx: CommodityIdx) -> &CommodityPayload {
        &self.commodities[commodity_idx.0 as usize]
    }

    pub fn commodities(&self) -> impl Iterator<Item = (CommodityIdx, &CommodityPayload)> {
        self.commodities
            .iter()
            .enumerate()
            .map(|(i, c)| (CommodityIdx(i as u32), c))
    }

    pub fn num_commodities(&self) -> usize {
        self.commodities.len()
    }

    pub fn outgoing_with(
        &self,
        node_id: NodeIdx,
        predicate: impl Fn(EdgeIdx, &EdgePayload) -> bool,
    ) -> Option<(EdgeIdx, &EdgePayload)> {
        self.node(node_id).outgoing.iter().find_map(|&edge_idx| {
            let edge = self.edge(edge_idx);
            match predicate(edge_idx, edge) {
                true => Some((edge_idx, edge)),
                false => None,
            }
        })
    }

    /// Adds a node to the graph unless it is a wait node which already exists for the specified station and time.
    fn add_node(
        &mut self,
        wait_node_by_station_time: &mut HashMap<(StationIdx, Time), NodeIdx>,
        time: Time,
        node_type: NodeType,
        station: StationIdx,
    ) -> NodeIdx {
        let mut new_node = || {
            let id = NodeIdx(self.nodes.len().try_into().unwrap());
            self.nodes.push(NodePayload {
                incoming: vec![],
                outgoing: vec![],
                time,
                node_type,
            });
            self.station_by_node.push(station);
            id
        };

        let node_id = match node_type {
            NodeType::Wait(station_id) => *wait_node_by_station_time
                .entry((station_id, time))
                .or_insert_with(new_node),
            _ => new_node(),
        };
        node_id
    }

    pub fn node(&self, node_id: NodeIdx) -> &NodePayload {
        &self.nodes[node_id.0 as usize]
    }

    pub fn edge(&self, edge_idx: EdgeIdx) -> &EdgePayload {
        &self.edges[edge_idx.0 as usize]
    }

    fn node_mut(&mut self, node_id: NodeIdx) -> &mut NodePayload {
        &mut self.nodes[node_id.0 as usize]
    }

    fn add_edge(&mut self, from: NodeIdx, to: NodeIdx) -> EdgeIdx {
        let edge_idx = EdgeIdx(self.edges.len().try_into().unwrap());
        let payload = EdgePayload { from, to };
        self.edges.push(payload);
        self.node_mut(from).outgoing.push(edge_idx);
        self.node_mut(to).incoming.push(edge_idx);
        edge_idx
    }

    pub fn outside_path(&self, commodity_idx: CommodityIdx) -> PathBox {
        PathBox::new(commodity_idx, empty())
    }

    pub fn station(&self, node_idx: NodeIdx) -> StationIdx {
        self.station_by_node[node_idx.0 as usize]
    }

    pub fn num_nodes(&self) -> usize {
        self.nodes.len()
    }

    pub fn num_edges(&self) -> usize {
        self.edges.len()
    }

    pub fn num_stations(&self) -> usize {
        self.num_stations
    }

    pub fn stations(&self) -> impl Iterator<Item = StationIdx> {
        (0_u32..(self.num_stations as u32)).map(StationIdx)
    }
}

pub fn reachable_nodes(
    graph: &Graph,
    source: NodeIdx,
    mut edge_okay: impl FnMut(EdgeIdx, &EdgePayload) -> bool,
) -> HashSet<NodeIdx> {
    let mut reachable: HashSet<NodeIdx> = set_new();
    let mut next = vec![source];
    while let Some(node_id) = next.pop() {
        if reachable.insert(node_id) {
            for &edge_idx in graph.node(node_id).outgoing.iter() {
                let edge = graph.edge(edge_idx);
                if edge_okay(edge_idx, edge) {
                    next.push(edge.to);
                }
            }
        }
    }
    reachable
}
