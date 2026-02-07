use std::mem;

use crate::{
    common::{EdgeIdx, PathIdx},
    index::Index,
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

    pub fn from_edges(mut edges: Vec<EdgeIdx>) -> Box<Path> {
        edges.sort_unstable();
        edges.dedup();
        Path::new(edges.into_boxed_slice())
    }

    pub fn edges(&self) -> &[EdgeIdx] {
        &self.0
    }
}

pub type PathIndex<'a> = Index<'a, PathIdx, Path>;
