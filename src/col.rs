pub type HashMap<K, V> = rustc_hash::FxHashMap<K, V>;

pub fn map_new<K, V>() -> HashMap<K, V> {
    rustc_hash::FxHashMap::default()
}
