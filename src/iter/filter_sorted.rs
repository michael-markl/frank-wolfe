/// A trait for filtering elements from a sorted iterator that appear in another sorted iterator.
pub trait SortedFilter: Iterator {
    /// Returns an iterator that yields items from `self` that are not present in `other`.
    fn sorted_filter<J>(self, other: J) -> SortedFilterIterator<Self, J, Self::Item>
    where
        Self: Sized,
        Self::Item: Ord,
        J: Iterator<Item = Self::Item>,
    {
        SortedFilterIterator::new(self, other)
    }
}

/// Default implementation of SortedFilter for all iterators.
impl<I: Iterator> SortedFilter for I {}

/// An iterator that yields items from a sorted iterator that do not appear in another sorted iterator.
pub struct SortedFilterIterator<I, J, T>
where
    I: Iterator<Item = T>,
    J: Iterator<Item = T>,
    T: Ord,
{
    left_iter: I,
    right_iter: J,
    right_head: Option<T>,
}

impl<I, J, T> SortedFilterIterator<I, J, T>
where
    I: Iterator<Item = T>,
    J: Iterator<Item = T>,
    T: Ord,
{
    /// Creates a new sorted filter iterator from two sorted iterators.
    pub fn new(left_iter: I, mut right_iter: J) -> Self {
        SortedFilterIterator {
            left_iter,
            right_head: right_iter.next(),
            right_iter,
        }
    }
}

impl<I, J, T> Iterator for SortedFilterIterator<I, J, T>
where
    I: Iterator<Item = T>,
    J: Iterator<Item = T>,
    T: Ord,
{
    type Item = T;

    fn next(&mut self) -> Option<T> {
        loop {
            let next_left = self.left_iter.next()?;
            loop {
                match &self.right_head {
                    None => return Some(next_left),
                    Some(right) => match next_left.cmp(right) {
                        std::cmp::Ordering::Less => return Some(next_left),
                        std::cmp::Ordering::Greater => self.right_head = self.right_iter.next(),
                        std::cmp::Ordering::Equal => break, // Skip the item since it appears in the right iterator.
                    },
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_left() {
        let left: Vec<i32> = vec![];
        let right = vec![1, 2, 3];
        let result: Vec<_> = left.into_iter().sorted_filter(right.into_iter()).collect();
        assert_eq!(result, vec![]);
    }

    #[test]
    fn test_empty_right() {
        let left = vec![1, 2, 3];
        let right: Vec<i32> = vec![];
        let result: Vec<_> = left.into_iter().sorted_filter(right.into_iter()).collect();
        assert_eq!(result, vec![1, 2, 3]);
    }

    #[test]
    fn test_no_overlap() {
        let left = vec![1, 3, 5, 7];
        let right = vec![2, 4, 6, 8];
        let result: Vec<_> = left.into_iter().sorted_filter(right.into_iter()).collect();
        assert_eq!(result, vec![1, 3, 5, 7]);
    }

    #[test]
    fn test_all_overlap() {
        let left = vec![1, 2, 3];
        let right = vec![1, 2, 3, 4, 5];
        let result: Vec<_> = left.into_iter().sorted_filter(right.into_iter()).collect();
        assert_eq!(result, vec![]);
    }

    #[test]
    fn test_partial_overlap() {
        let left = vec![1, 2, 3, 4, 5, 6];
        let right = vec![2, 4, 6, 8];
        let result: Vec<_> = left.into_iter().sorted_filter(right.into_iter()).collect();
        assert_eq!(result, vec![1, 3, 5]);
    }

    #[test]
    fn test_left_duplicates_filtered() {
        let left = vec![1, 1, 2, 2, 3, 3];
        let right = vec![2];
        let result: Vec<_> = left.into_iter().sorted_filter(right.into_iter()).collect();
        assert_eq!(result, vec![1, 1, 3, 3]);
    }

    #[test]
    fn test_right_duplicates() {
        let left = vec![1, 2, 3, 4, 5];
        let right = vec![2, 2, 2, 4, 4];
        let result: Vec<_> = left.into_iter().sorted_filter(right.into_iter()).collect();
        assert_eq!(result, vec![1, 3, 5]);
    }

    #[test]
    fn test_with_strings() {
        let left = vec!["a", "b", "c", "d"];
        let right = vec!["b", "d"];
        let result: Vec<_> = left.into_iter().sorted_filter(right.into_iter()).collect();
        assert_eq!(result, vec!["a", "c"]);
    }
}
