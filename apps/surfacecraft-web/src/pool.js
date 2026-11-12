// The workers of SurfaceCraft's thread pool in the browser. This file is both the module the
// page imports (startWorkers) and the script every worker runs: a worker waits for the compiled
// module and the shared memory, instantiates the same wasm on them with its own stack and then
// runs one thread of rayon's pool until the page goes away.
const INIT = 'surfacecraft_pool_init';
const READY = 'surfacecraft_pool_ready';
const FAILED = 'surfacecraft_pool_failed';
const DIED = 'surfacecraft_pool_died';

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
    try {
      // does not return while the pool lives: it waits for work
      pkg.pool_worker_run();
      postMessage({ type: DIED, message: 'the pool thread ended' });
    } catch (e) {
      // a panic in wasm is a trap: this thread of the pool is gone, and so is the work it had
      postMessage({ type: DIED, message: String(e) });
    }
  });
}

// Kept so that the workers are never collected while the page lives (a worker that only shares
// the wasm memory with the page is not otherwise rooted in every browser).
let workers = [];

// Starts `count` workers on `module` and `memory`; resolves once every one is instantiated and
// rejects when one cannot start. `onDead(message)` is called whenever a worker dies later.
export async function startWorkers(module, memory, mainJs, count, stackBytes, onDead) {
  const start = () =>
    new Promise((resolve, reject) => {
      const worker = new Worker(import.meta.url, { type: 'module' });
      let ready = false;
      const dead = message => (ready ? onDead(message) : reject(new Error(message)));
      worker.addEventListener('message', ({ data }) => {
        if (data?.type === READY) {
          ready = true;
          resolve(worker);
        }
        if (data?.type === FAILED || data?.type === DIED) dead(data.message);
      });
      worker.addEventListener('error', e => dead(e.message || 'a pool worker failed'));
      worker.postMessage({ type: INIT, module, memory, mainJs, stackBytes });
    });
  workers = await Promise.all(Array.from({ length: count }, start));
}
