// The workers of SurfaceCraft's thread pool in the browser. This file is both the module the
// page imports (startWorkers) and the script every worker runs: a worker waits for the compiled
// module and the shared memory, instantiates the same wasm on them with its own stack and then
// runs one thread of rayon's pool until the page goes away.
const INIT = 'surfacecraft_pool_init';
const READY = 'surfacecraft_pool_ready';
const FAILED = 'surfacecraft_pool_failed';

if (typeof WorkerGlobalScope !== 'undefined' && self instanceof WorkerGlobalScope) {
  self.addEventListener('message', async ({ data }) => {
    if (data?.type !== INIT) return;
    let pkg;
    try {
      pkg = await import(data.mainJs);
      await pkg.default({
        module_or_path: data.module,
        memory: data.memory,
        thread_stack_size: data.stackBytes
      });
    } catch (e) {
      postMessage({ type: FAILED, message: String(e) });
      return;
    }
    postMessage({ type: READY });
    // does not return: it waits for work from the pool
    pkg.pool_worker_run();
  });
}

// Kept so that the workers are never collected while the page lives (a worker that only shares
// the wasm memory with the page is not otherwise rooted in every browser).
let workers = [];

// Starts `count` workers on `module` and `memory`; resolves once every one is instantiated and
// rejects when one cannot start.
export async function startWorkers(module, memory, mainJs, count, stackBytes) {
  const start = () =>
    new Promise((resolve, reject) => {
      const worker = new Worker(import.meta.url, { type: 'module' });
      worker.addEventListener('message', ({ data }) => {
        if (data?.type === READY) resolve(worker);
        if (data?.type === FAILED) reject(new Error(data.message));
      });
      worker.addEventListener('error', e => reject(new Error(e.message || 'a pool worker failed to load')));
      worker.postMessage({ type: INIT, module, memory, mainJs, stackBytes });
    });
  workers = await Promise.all(Array.from({ length: count }, start));
}
