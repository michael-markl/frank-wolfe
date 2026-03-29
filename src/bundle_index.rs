use std::{
    mem,
    ops::{Deref, DerefMut},
    slice::Iter,
};

use crate::{
    common::{BundleIdx, PermitIdx},
    index::Index,
    iter::{dedup::Dedup, filter_sorted::SortedFilter, merge_sorted::merge_sorted},
};

#[derive(PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct Bundle([PermitIdx]);

impl Bundle {
    pub fn empty() -> Box<Bundle> {
        Bundle::new(Box::new([]))
    }

    fn new(boxed_slice: Box<[PermitIdx]>) -> Box<Self> {
        unsafe { mem::transmute(boxed_slice) }
    }

    pub fn from_permits(mut permits: Vec<PermitIdx>) -> Box<Bundle> {
        permits.sort_unstable();
        permits.dedup();
        Bundle::new(permits.into_boxed_slice())
    }

    pub fn union_count(&self, other: &Bundle) -> usize {
        merge_sorted(self.0.iter(), other.0.iter()).dedup().count()
    }

    pub fn into_union(self: Box<Bundle>, other: &Bundle) -> Box<Bundle> {
        let count = self.union_count(other);
        if count == self.0.len() {
            return self;
        }

        let mut array = Box::new_uninit_slice(count);

        merge_sorted(self.0.iter(), other.0.iter())
            .dedup()
            .zip(array.iter_mut())
            .for_each(|(permit, slot)| {
                slot.write(*permit);
            });
        let boxed_slice = unsafe { array.assume_init() };

        Bundle::new(boxed_slice)
    }

    pub fn union(&self, other: &Bundle) -> Box<Bundle> {
        let count = self.union_count(other);

        let mut array = Box::new_uninit_slice(count);

        merge_sorted(self.0.iter(), other.0.iter())
            .dedup()
            .zip(array.iter_mut())
            .for_each(|(permit, slot)| {
                slot.write(*permit);
            });
        let boxed_slice = unsafe { array.assume_init() };

        Bundle::new(boxed_slice)
    }

    pub fn set_minus_iter<'a>(&'a self, other: &'a Bundle) -> impl Iterator<Item = &'a PermitIdx> {
        self.0.iter().sorted_filter(other.0.iter())
    }

    pub fn permits(&self) -> impl Iterator<Item = PermitIdx> + '_ {
        self.0.iter().copied()
    }
}

impl<'a> From<&'a Bundle> for Iter<'a, PermitIdx> {
    fn from(val: &'a Bundle) -> Self {
        val.0.iter()
    }
}

pub struct BundleIndex<'a>(Index<'a, BundleIdx, Bundle>);

impl<'a> BundleIndex<'a> {
    pub fn new() -> Self {
        let mut index = Index::new();
        index.transfer_element(Bundle::empty());
        Self(index)
    }
}

impl<'a> Deref for BundleIndex<'a> {
    type Target = Index<'a, BundleIdx, Bundle>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<'a> DerefMut for BundleIndex<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
