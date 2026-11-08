//! Data parallelism (P2): rayon on native targets, plain loops on wasm (the browser build has no
//! threads). Callers write one closure; the result order is the index order either way, so
//! parallel and sequential runs give identical results.

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
