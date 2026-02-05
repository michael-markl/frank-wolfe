use std::hash::Hash;

use crate::col::{HashMap, map_new};

pub struct Index<'a, Idx: Copy + Into<usize> + From<usize>, Payload: ?Sized> {
    elements: Vec<Box<Payload>>,

    by_idx: HashMap<&'a Payload, Idx>,
}

impl<'a, Idx: Copy + Into<usize> + From<usize>, Payload: Eq + Hash + ?Sized>
    Index<'a, Idx, Payload>
{
    pub fn new() -> Self {
        Self {
            elements: vec![],
            by_idx: map_new(),
        }
    }

    pub fn find_idx(&self, element: &Payload) -> Option<Idx> {
        self.by_idx.get(element).copied()
    }

    pub fn get_by_idx(&self, idx: Idx) -> &Payload {
        &self.elements[Idx::into(idx)]
    }

    pub fn element_ids(&self) -> impl Iterator<Item = Idx> {
        (0..self.elements.len()).map(|id| Idx::from(id))
    }

    pub fn transfer_element(&mut self, element_box: Box<Payload>) -> Idx {
        self.by_idx
            .get(element_box.as_ref())
            .copied()
            .unwrap_or_else(|| {
                let id_usize = self.elements.len();
                let id = Idx::from(id_usize);
                self.elements.push(element_box);

                // Get unbounded ref to payload.
                let element_ptr = self.elements[id_usize].as_ref() as *const Payload;
                // SAFETY: The ref to the payload is valid until the Index is destroyed,
                // as we never remove elements from the index.
                let unbounded_ref: &'a Payload = unsafe { element_ptr.as_ref() }.unwrap();
                self.by_idx.insert(unbounded_ref, id);
                id
            })
    }
}
