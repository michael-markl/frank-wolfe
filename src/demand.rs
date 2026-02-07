use rayon::iter::{IntoParallelRefIterator, ParallelIterator};

use crate::{
    col::{HashMap, map_new},
    common::{CommodityIdx, DestinationIdx, Float, NodeIdx},
};

pub trait DemandOps {
    fn node_idx_by_destination(&self, destination_idx: DestinationIdx) -> NodeIdx;

    fn num_destinations(&self) -> usize;
}

pub struct Commodity {
    pub origin: NodeIdx,
    pub destination_idx: DestinationIdx,
    pub demand: Float,
}

pub struct Demand {
    destinations: Vec<NodeIdx>,
    destination_by_node_idx: HashMap<NodeIdx, DestinationIdx>,
    commodities_by_origin: HashMap<NodeIdx, Vec<CommodityIdx>>,
    commodities: Vec<Commodity>,
}

impl Demand {
    pub fn empty() -> Self {
        Self {
            destinations: Vec::new(),
            destination_by_node_idx: map_new(),
            commodities_by_origin: map_new(),
            commodities: Vec::new(),
        }
    }

    pub fn add_commodity(
        &mut self,
        origin: NodeIdx,
        destination: NodeIdx,
        demand: Float,
    ) -> CommodityIdx {
        let destination_idx = self
            .destination_by_node_idx
            .entry(destination)
            .or_insert_with(|| {
                self.destinations.push(destination);
                self.destinations.len() - 1
            });

        let commodity_idx = self.commodities.len();
        self.commodities.push(Commodity {
            origin,
            destination_idx: *destination_idx,
            demand,
        });
        self.commodities_by_origin
            .entry(origin)
            .or_insert(Vec::new())
            .push(commodity_idx);

        commodity_idx
    }

    pub fn get_commodity(&self, commodity_idx: CommodityIdx) -> &Commodity {
        &self.commodities[commodity_idx]
    }

    pub fn par_iter_by_origin(
        &self,
    ) -> impl ParallelIterator<Item = (&NodeIdx, &Vec<CommodityIdx>)> {
        self.commodities_by_origin.par_iter()
    }
}

impl DemandOps for Demand {
    fn node_idx_by_destination(&self, destination_idx: DestinationIdx) -> NodeIdx {
        self.destinations[destination_idx]
    }
    fn num_destinations(&self) -> usize {
        self.destinations.len()
    }
}
