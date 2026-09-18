/// An iterator that merges two sorted iterators in ascending order.
pub struct MergedIterator<I, J, T>
where
    I: Iterator<Item = T>,
    J: Iterator<Item = T>,
    T: Ord,
{
    left: Option<T>,
    right: Option<T>,
    left_iter: I,
    right_iter: J,
}

impl<I, J, T> MergedIterator<I, J, T>
where
    I: Iterator<Item = T>,
    J: Iterator<Item = T>,
    T: Ord,
{
    /// Creates a new merged iterator from two sorted iterators.
    pub fn new(mut left_iter: I, mut right_iter: J) -> Self {
        let left = left_iter.next();
        let right = right_iter.next();
        MergedIterator {
            left,
            right,
            left_iter,
            right_iter,
        }
    }
}

impl<I, J, T> Iterator for MergedIterator<I, J, T>
where
    I: Iterator<Item = T>,
    J: Iterator<Item = T>,
    T: Ord,
{
    type Item = T;

    fn next(&mut self) -> Option<T> {
        match (&self.left, &self.right) {
            (None, None) => None,
            (Some(_), None) => {
                let item = self.left.take();
                self.left = self.left_iter.next();
                item
            }
            (None, Some(_)) => {
                let item = self.right.take();
                self.right = self.right_iter.next();
                item
            }
            (Some(_), Some(_)) => {
                if self.left <= self.right {
                    let item = self.left.take();
                    self.left = self.left_iter.next();
                    item
                } else {
                    let item = self.right.take();
                    self.right = self.right_iter.next();
                    item
                }
            }
        }
    }
}

/// Merges two sorted iterators into a single sorted iterator.
pub fn merge_sorted<I, J, T>(left: I, right: J) -> MergedIterator<I, J, T>
where
    I: Iterator<Item = T>,
    J: Iterator<Item = T>,
    T: Ord,
{
    MergedIterator::new(left, right)
}
