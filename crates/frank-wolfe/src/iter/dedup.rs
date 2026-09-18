/// A trait for deduplicating consecutive identical elements from an iterator.
pub trait Dedup: Iterator {
    /// Returns an iterator that yields only the first occurrence of each consecutive group of identical elements.
    fn dedup(mut self) -> DedupIterator<Self>
    where
        Self: Sized,
        Self::Item: Eq,
    {
        let next = self.next();
        DedupIterator {
            iter: self,
            head: next,
        }
    }
}

/// Default implementation of Dedup for all iterators.
impl<I: Iterator> Dedup for I {}

/// An iterator that deduplicates consecutive identical elements.
pub struct DedupIterator<I: Iterator> {
    iter: I,
    head: Option<I::Item>,
}

impl<I> Iterator for DedupIterator<I>
where
    I: Iterator,
    I::Item: Eq,
{
    type Item = I::Item;

    fn next(&mut self) -> Option<I::Item> {
        for item in self.iter.by_ref() {
            if self.head.as_ref() != Some(&item) {
                let next = self.head.replace(item);
                return next;
            }
        }
        self.head.take()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_dedup() {
        let vec = vec![1, 1, 2, 2, 2, 3, 3, 1];
        let result: Vec<_> = vec.into_iter().dedup().collect();
        assert_eq!(result, vec![1, 2, 3, 1]);
    }

    #[test]
    fn test_empty_iterator() {
        let vec: Vec<i32> = vec![];
        let result: Vec<_> = vec.into_iter().dedup().collect();
        assert_eq!(result, vec![]);
    }

    #[test]
    fn test_no_duplicates() {
        let vec = vec![1, 2, 3, 4, 5];
        let result: Vec<_> = vec.into_iter().dedup().collect();
        assert_eq!(result, vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_all_same() {
        let vec = vec![42, 42, 42, 42];
        let result: Vec<_> = vec.into_iter().dedup().collect();
        assert_eq!(result, vec![42]);
    }

    #[test]
    fn test_single_element() {
        let vec = vec![7];
        let result: Vec<_> = vec.into_iter().dedup().collect();
        assert_eq!(result, vec![7]);
    }

    #[test]
    fn test_alternating() {
        let vec = vec![1, 2, 1, 2, 1, 2];
        let result: Vec<_> = vec.into_iter().dedup().collect();
        assert_eq!(result, vec![1, 2, 1, 2, 1, 2]);
    }

    #[test]
    fn test_with_strings() {
        let vec = vec!["a", "a", "b", "b", "c", "a"];
        let result: Vec<_> = vec.into_iter().dedup().collect();
        assert_eq!(result, vec!["a", "b", "c", "a"]);
    }

    #[test]
    fn test_consecutive_pairs() {
        let vec = vec![1, 1, 2, 2, 3, 3];
        let result: Vec<_> = vec.into_iter().dedup().collect();
        assert_eq!(result, vec![1, 2, 3]);
    }
}
