//! Data parallelism (P2): rayon on native targets, plain loops on wasm (the browser build has no
//! threads). Callers write one closure; the result order is the index order either way, so
//! parallel and sequential runs give identical results.

/// Threads data-parallel work spreads over (1 on wasm).
#[must_use]
pub fn threads() -> usize {
    #[cfg(not(target_arch = "wasm32"))]
    {
        rayon::current_num_threads()
    }
    #[cfg(target_arch = "wasm32")]
    {
        1
    }
}

/// Does this build run work on several threads? (Results never depend on it; it only tells
/// callers whether speculative work can pay off.)
#[must_use]
pub fn parallel() -> bool {
    threads() > 1
}

/// `f(0), f(1), …, f(n − 1)`, evaluated in parallel where threads exist.
pub fn map<T, F>(n: usize, f: F) -> Vec<T>
where
    T: Send,
    F: Fn(usize) -> T + Sync + Send,
{
    #[cfg(not(target_arch = "wasm32"))]
    {
        use rayon::prelude::*;
        (0..n).into_par_iter().map(f).collect()
    }
    #[cfg(target_arch = "wasm32")]
    {
        (0..n).map(f).collect()
    }
}

/// `f` applied to every item of `items`, in parallel where threads exist.
pub fn map_slice<S, T, F>(items: &[S], f: F) -> Vec<T>
where
    S: Sync,
    T: Send,
    F: Fn(&S) -> T + Sync + Send,
{
    #[cfg(not(target_arch = "wasm32"))]
    {
        use rayon::prelude::*;
        items.par_iter().map(f).collect()
    }
    #[cfg(target_arch = "wasm32")]
    {
        items.iter().map(f).collect()
    }
}

/// Runs both closures, in parallel where threads exist; the results are those of `(a(), b())`.
pub fn join<A, B, RA, RB>(a: A, b: B) -> (RA, RB)
where
    A: FnOnce() -> RA + Send,
    B: FnOnce() -> RB + Send,
    RA: Send,
    RB: Send,
{
    #[cfg(not(target_arch = "wasm32"))]
    {
        rayon::join(a, b)
    }
    #[cfg(target_arch = "wasm32")]
    {
        (a(), b())
    }
}

/// Sorts in parallel where threads exist (unstable; ties must not matter to the caller).
pub fn sort_unstable<T: Ord + Send>(v: &mut [T]) {
    #[cfg(not(target_arch = "wasm32"))]
    {
        use rayon::prelude::*;
        v.par_sort_unstable();
    }
    #[cfg(target_arch = "wasm32")]
    {
        v.sort_unstable();
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn order_is_kept() {
        let v = super::map(10_000, |i| i * 2);
        assert!(v.iter().enumerate().all(|(i, x)| *x == 2 * i));
        let w = super::map_slice(&v, |x| x + 1);
        assert_eq!(w[9_999], 19_999);
    }
}
