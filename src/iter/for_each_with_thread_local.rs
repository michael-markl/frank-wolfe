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
    /// This function should be used, if the state is too large or complext to be
    /// created per parallel task (otherwise, use rayon's `fold`).
    ///
    /// # PANICS
    /// May panic, if during [init] or [f] the code yields to rayon and reentrant
    /// processing attempts try to borrow the already borrowed thread local state.
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

            // PANICS: Each thread gets its own cell, but if [init] or [f] yield to rayon,
            // another item might get processed, and the borrow might already be taken.
            let state: &mut S = &mut cell.borrow_mut();
            f(item, state);
        });
        tl.into_iter().map(|it| it.into_inner())
    }
}
