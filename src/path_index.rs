use std::{
    mem,
    ops::{Deref, DerefMut},
    slice::Iter,
};

use crate::{
    common::{EdgeIdx, PathIdx},
    index::Index,
    iter::{dedup::Dedup, filter_sorted::SortedFilter, merge_sorted::merge_sorted},
};

#[derive(PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct Path([EdgeIdx]);

impl Path {
    pub fn empty() -> Box<Path> {
        Path::new(Box::new([]))
    }

    fn new(boxed_slice: Box<[EdgeIdx]>) -> Box<Self> {
        unsafe { mem::transmute(boxed_slice) }
    }

    pub fn from_edges(edges: Vec<EdgeIdx>) -> Box<Path> {
        Path::new(edges.into_boxed_slice())
    }

    pub fn union_count(&self, other: &Path) -> usize {
        merge_sorted(self.0.iter(), other.0.iter()).dedup().count()
    }

    pub fn union(&self, other: &Path) -> Box<Path> {
        let count = self.union_count(other);

        let mut array = Box::new_uninit_slice(count);

        merge_sorted(self.0.iter(), other.0.iter())
            .dedup()
            .zip(array.iter_mut())
            .for_each(|(permit, slot)| {
                slot.write(*permit);
            });
        let boxed_slice = unsafe { array.assume_init() };

        Path::new(boxed_slice)
    }

    pub fn set_minus_iter<'a>(&'a self, other: &'a Path) -> impl Iterator<Item = &'a EdgeIdx> {
        self.0.iter().sorted_filter(other.0.iter())
    }

    pub fn edges(&self) -> impl Iterator<Item = EdgeIdx> + '_ {
        self.0.iter().copied()
    }
}

impl<'a> From<&'a Path> for Iter<'a, EdgeIdx> {
    fn from(val: &'a Path) -> Self {
        val.0.iter()
    }
}

pub struct PathIndex<'a>(Index<'a, PathIdx, Path>);

impl<'a> PathIndex<'a> {
    pub fn new() -> Self {
        let mut index = Index::new();
        index.transfer_element(Path::empty());
        Self(index)
    }
}

impl<'a> Deref for PathIndex<'a> {
    type Target = Index<'a, PathIdx, Path>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<'a> DerefMut for PathIndex<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
