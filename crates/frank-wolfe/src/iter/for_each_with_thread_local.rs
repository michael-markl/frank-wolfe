use std::cell::RefCell;

use rayon::iter::ParallelIterator;
use thread_local::ThreadLocal;

pub trait ForEachWithThreadLocal<T> {
    /// Operate on every item in the iterator with access to thread local data.
    /// Returns an iterator of all thread local data created.
    ///
    /// The `init` function initializes the thread local state.
    /// The `f` function operates on items of the iterator and a mutable borrow on
    /// the thread local state
    ///
    /// This function should be used, if the state is too large or complex to be
    /// created once per parallel task (otherwise, use rayon's `fold`).
    ///
    /// # PANICS
    /// Panics if reentrant processing during `f` attempts to borrow
    /// the already borrowed thread local state.
    fn for_each_with_thread_local<S: Send>(
        self,
        init: impl Fn() -> S + Sync,
        f: impl Fn(T, &mut S) + Sync + Send,
    ) -> impl Iterator<Item = S>;
}

impl<T, I: ParallelIterator<Item = T>> ForEachWithThreadLocal<T> for I {
    fn for_each_with_thread_local<S: Send>(
        self,
        init: impl Fn() -> S + Sync,
        f: impl Fn(T, &mut S) + Sync + Send,
    ) -> impl Iterator<Item = S> {
        let tl: ThreadLocal<RefCell<S>> = ThreadLocal::new();
        self.for_each(|item| {
            let cell = tl.get_or(|| RefCell::new(init()));

            // Borrow before initialization so reentry during either callback panics.
            let mut state = cell.borrow_mut();
            f(item, &mut state);
        });
        tl.into_iter().map(|it| it.into_inner())
    }
}

#[cfg(test)]
mod tests {
    use super::ForEachWithThreadLocal;
    use itertools::Itertools;
    use rayon::ThreadPoolBuilder;
    use rayon::iter::{IndexedParallelIterator, IntoParallelIterator};

    #[test]
    #[should_panic(expected = "RefCell already borrowed")]
    fn reentrant_callback_panics() {
        let pool = ThreadPoolBuilder::new().num_threads(1).build().unwrap();
        pool.install(|| {
            let _states = (0..2)
                .into_par_iter()
                .with_max_len(1)
                .for_each_with_thread_local(
                    || 0usize,
                    |_, state| {
                        rayon::yield_now();
                        *state += 1;
                    },
                );
        });
    }

    #[test]
    fn reentrant_init_does_not_panic() {
        let pool = ThreadPoolBuilder::new().num_threads(1).build().unwrap();
        pool.install(|| {
            // Separate jobs on one worker ensure yielding runs the other item
            // on the same thread while initialization still holds the borrow.
            let _states = (0..2)
                .into_par_iter()
                .with_max_len(1)
                .for_each_with_thread_local(
                    || {
                        rayon::yield_now();
                        0usize
                    },
                    |_, state| *state += 1,
                )
                .collect_vec();
            assert_eq!(_states, vec![2])
        });
    }

    #[test]
    fn sequential_callbacks_reuse_state() {
        let pool = ThreadPoolBuilder::new().num_threads(1).build().unwrap();
        let states = pool.install(|| {
            (0..8)
                .into_par_iter()
                .with_max_len(1)
                .for_each_with_thread_local(Vec::new, |item, state| state.push(item))
                .collect::<Vec<_>>()
        });

        assert_eq!(states.len(), 1);
        let mut items = states.into_iter().next().unwrap();
        items.sort_unstable();
        assert_eq!(items, (0..8).collect::<Vec<_>>());
    }
}
