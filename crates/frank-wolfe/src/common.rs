pub type Float = f64;

pub type NodeIdx = usize;
pub type DestinationIdx = usize;
pub type EdgeIdx = usize;
pub type CommodityIdx = usize;
pub type PermitIdx = usize;
pub type BundleIdx = usize;

pub const BUNDLE_IDX_EMPTY: BundleIdx = 0;
pub type PathIdx = usize;

pub type MyDotAccumulator = accurate::dot::NaiveDot<Float>;
pub type MySumAccumulator = accurate::sum::NaiveSum<Float>;
