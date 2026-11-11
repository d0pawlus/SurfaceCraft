//! The browser's thread pool: rayon's workers as web workers sharing the wasm memory. It exists
//! only in a bundle built with `+atomics` and on a page that is cross-origin isolated (the COOP
//! and COEP headers `cargo xtask web` writes into `vercel.json`); anywhere else the app runs on
//! the page's thread alone, as it did before.
//!
//! The page starts the workers (`pool.js`); each instantiates the same module on the shared
//! memory and then takes one of rayon's threads from a queue and runs it. The page's thread may
//! never block, so it only starts work on the pool (`surfacecraft_geom::par::spawn_background`)
//! and polls for the result.
//!
//! A panic in wasm aborts the worker it happens on. `pool.js` reports that, the pool is then
//! given up (`surfacecraft_geom::par::background_failed`): the command that was running is
//! reported as failed, and heavy commands run on the page's thread from then on, with the warning
//! a page without threads shows. Reloading the page brings the pool back.

/// A bundle without threads has no pool to start.
#[cfg(not(target_feature = "atomics"))]
pub async fn start() {}

#[cfg(target_feature = "atomics")]
pub use pool::start;

#[cfg(target_feature = "atomics")]
mod pool {
    use std::sync::mpsc::{Receiver, Sender, channel};
    use std::sync::{Mutex, OnceLock};
    use wasm_bindgen::prelude::*;

    /// Workers at most: each one reserves its own stack in the shared memory.
    const MAX_POOL_THREADS: usize = 16;
    /// Stack of a worker: solidcore asks for 8 MiB, as on the page's thread (`.cargo/config.toml`).
    const WORKER_STACK_BYTES: u32 = 8 * 1024 * 1024;
    /// The query parameter that sets the pool size (`?threads=4`; `0` runs without a pool).
    const THREADS_PARAM: &str = "threads=";

    /// rayon's threads waiting for a worker to run them.
    struct Queue {
        give: Sender<rayon::ThreadBuilder>,
        take: Mutex<Receiver<rayon::ThreadBuilder>>,
    }

    static QUEUE: OnceLock<Queue> = OnceLock::new();

    #[wasm_bindgen(module = "/src/pool.js")]
    extern "C" {
        #[wasm_bindgen(js_name = startWorkers)]
        fn start_workers(
            module: JsValue,
            memory: JsValue,
            main_js: js_sys::JsString,
            count: u32,
            stack: u32,
            on_dead: &Closure<dyn Fn(String)>,
        ) -> js_sys::Promise;
    }

    #[wasm_bindgen]
    extern "C" {
        /// The URL of the generated JavaScript module, which every worker imports.
        #[wasm_bindgen(thread_local_v2, js_namespace = ["import", "meta"], js_name = url)]
        static MAIN_JS: js_sys::JsString;
    }

    /// Runs one thread of the pool on the calling worker (called by `pool.js`, never by the page).
    #[wasm_bindgen]
    pub fn pool_worker_run() {
        let Some(queue) = QUEUE.get() else { return };
        let thread = match queue.take.lock() {
            Ok(take) => take.recv(),
            Err(poisoned) => poisoned.into_inner().recv(),
        };
        if let Ok(thread) = thread {
            thread.run();
        }
    }

    /// Starts the pool when the page allows one and tells the data parallelism about it.
    pub async fn start() {
        if !cross_origin_isolated() {
            warn(
                "SurfaceCraft: the page is not cross-origin isolated, so there are no threads: heavy tools run on the page's thread",
            );
            return;
        }
        let threads = pool_size();
        if threads == 0 {
            warn("SurfaceCraft: threads=0, running without a thread pool");
            return;
        }
        match build(threads).await {
            Ok(()) => {
                surfacecraft_geom::par::enable_background();
                web_sys::console::info_1(&format!("SurfaceCraft: thread pool of {threads} workers").into());
            }
            Err(e) => {
                warn("SurfaceCraft: the thread pool did not start; heavy tools run on the page's thread");
                web_sys::console::warn_1(&e);
            }
        }
    }

    /// Starts `threads` workers and builds rayon's global pool on them.
    async fn build(threads: usize) -> Result<(), JsValue> {
        let (give, take) = channel();
        if QUEUE.set(Queue { give, take: Mutex::new(take) }).is_err() {
            return Err("the thread pool was already started".into());
        }
        let main_js = MAIN_JS.with(Clone::clone);
        let on_dead = Closure::<dyn Fn(String)>::new(worker_died);
        let (module, memory) = (wasm_bindgen::module(), wasm_bindgen::memory());
        let workers = start_workers(module, memory, main_js, threads as u32, WORKER_STACK_BYTES, &on_dead);
        // the workers may report a death for as long as the page lives
        on_dead.forget();
        wasm_bindgen_futures::JsFuture::from(workers).await?;
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .spawn_handler(|thread| {
                let queue = QUEUE.get().ok_or_else(|| std::io::Error::other("no worker queue"))?;
                queue.give.send(thread).map_err(|_| std::io::Error::other("no worker takes the thread"))
            })
            .build_global()
            .map_err(|e| JsValue::from_str(&e.to_string()))
    }

    /// A worker of the pool died (a panic aborts in wasm): the command that ran on it will never
    /// finish and the pool may be stuck waiting for it, so it is not used again. The running
    /// command is reported as failed and later heavy commands run on the page's thread.
    fn worker_died(message: String) {
        surfacecraft_geom::par::background_failed();
        web_sys::console::error_1(
            &format!(
                "SurfaceCraft: a worker of the thread pool died ({message}); heavy tools now run on the page's thread"
            )
            .into(),
        );
    }

    fn warn(message: &str) {
        web_sys::console::warn_1(&message.into());
    }

    /// May this page share memory with workers?
    fn cross_origin_isolated() -> bool {
        js_sys::Reflect::get(&js_sys::global(), &"crossOriginIsolated".into())
            .ok()
            .and_then(|v| v.as_bool())
            .unwrap_or(false)
    }

    /// The `threads` query parameter, else one less than the logical cores (the page's own
    /// thread keeps painting while the pool is busy) and at least one; at most
    /// [`MAX_POOL_THREADS`].
    fn pool_size() -> usize {
        let Some(window) = web_sys::window() else { return 0 };
        let asked = window.location().search().ok().and_then(|q| threads_param(&q));
        let cores = window.navigator().hardware_concurrency();
        let cores = if cores.is_finite() && cores >= 1.0 { cores as usize } else { 1 };
        asked.unwrap_or_else(|| cores.saturating_sub(1).max(1)).min(MAX_POOL_THREADS)
    }

    /// The value of `threads=` in a query string such as `?threads=4&x=1`.
    fn threads_param(query: &str) -> Option<usize> {
        query.trim_start_matches('?').split('&').find_map(|kv| kv.strip_prefix(THREADS_PARAM)?.parse().ok())
    }
}
