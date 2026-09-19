//! Shared hash-map aliases and payload interning.

pub mod index;

pub type HashMap<K, V> = rustc_hash::FxHashMap<K, V>;
pub type HashSet<T> = rustc_hash::FxHashSet<T>;

pub fn map_new<K, V>() -> HashMap<K, V> {
    rustc_hash::FxHashMap::default()
}

pub fn set_new<T>() -> HashSet<T> {
    rustc_hash::FxHashSet::default()
}
