use std::cell::UnsafeCell;

use rayon::iter::ParallelIterator;
use thread_local::ThreadLocal;

pub trait ForEachWithThreadLocal<T> {
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
        let tl: ThreadLocal<UnsafeCell<S>> = ThreadLocal::new();
        self.for_each(|item| {
            let cell = tl.get_or(|| UnsafeCell::new(init()));

            // SAFETY: Each thread gets its own cell, and the cell is only accessed here, so the mutable pointer is not aliased.
            let state = unsafe { &mut *cell.get() };
            f(item, state);
        });
        tl.into_iter().map(|it| it.into_inner())
    }
}
