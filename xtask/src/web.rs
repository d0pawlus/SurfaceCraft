//! `cargo xtask web`: the browser bundle in `apps/surfacecraft-web/dist` (needs `wasm-bindgen`
//! on PATH, at the version of Cargo.lock).
//!
//! The bundle has threads: the wasm is built with shared memory (`+atomics`), which needs a
//! standard library rebuilt with the same features (`-Z build-std`), hence one pinned nightly,
//! used for this build only; the rest of the workspace stays on stable, and `cargo xtask wasm`
//! keeps checking the crates for a browser without threads. Threads also need a cross-origin
//! isolated page, so the bundle carries a `vercel.json` with the COOP and COEP headers.
//! The bundle without threads is built too (on stable, as before) and the page picks one: a
//! browser that cannot share memory still gets the app. `--no-threads` builds only that one;
//! `--check` type-checks the threaded build without building it.
use crate::{cargo, root, run};
use std::path::Path;
use std::process::Command;

/// The nightly of the threaded build (with the `rust-src` component). `cargo xtask web-toolchain`
/// prints it for the workflows, so the pin lives here only. It is a nightly of the 1.99 cycle:
/// the compiler generation of the stable toolchain the rest of the workspace builds with.
pub(crate) const NIGHTLY: &str = "nightly-2026-08-01";

/// The page's stack (solidcore needs 8 MiB) and the browser backend of getrandom: what
/// `.cargo/config.toml` sets for wasm, repeated because RUSTFLAGS replaces that entry.
const BASE_RUSTFLAGS: &str = "-C link-arg=-zstack-size=8388608 --cfg getrandom_backend=\"wasm_js\"";

/// Shared memory and what it needs: the atomic instructions, a memory the page creates and hands
/// to every worker (imported, shared), and the thread-local storage symbols wasm-bindgen sets up
/// per thread. A shared memory has a fixed maximum: the bundle without threads could grow to the
/// 4 GiB of wasm32, and so can this one.
const THREAD_RUSTFLAGS: &str = "-C target-feature=+atomics,+bulk-memory,+mutable-globals \
    -C link-arg=--shared-memory -C link-arg=--import-memory -C link-arg=--max-memory=4294967296 \
    -C link-arg=--export=__wasm_init_tls -C link-arg=--export=__tls_size \
    -C link-arg=--export=__tls_align -C link-arg=--export=__tls_base";

/// The headers that make the page cross-origin isolated (`SharedArrayBuffer` and so threads need
/// it). Every resource of the page is same-origin, so `require-corp` blocks nothing.
const VERCEL_JSON: &str = r#"{
  "$schema": "https://openapi.vercel.sh/vercel.json",
  "headers": [
    {
      "source": "/(.*)",
      "headers": [
        { "key": "Cross-Origin-Opener-Policy", "value": "same-origin" },
        { "key": "Cross-Origin-Embedder-Policy", "value": "require-corp" },
        { "key": "Cross-Origin-Resource-Policy", "value": "same-origin" }
      ]
    }
  ]
}
"#;

/// Starts the app. The page gets the threaded bundle (`mt/`) only when the browser lets it share
/// memory with workers (a cross-origin isolated page that can create a shared memory) and
/// `?threads=0` was not asked for; everywhere else, and when the threaded bundle cannot start,
/// the bundle without threads (`st/`), which needs nothing special. The texts shown before the
/// app runs are English or Polish by the browser's language.
const START_SCRIPT: &str = "<script type=\"module\">
    const polish = (navigator.language || '').toLowerCase().startsWith('pl');
    const text = polish
      ? { loading: 'Wczytywanie SurfaceCraft… (pierwsza wizyta pobiera aplikację)',
          failed: 'SurfaceCraft nie uruchomił się w tej przeglądarce: ' }
      : { loading: 'Loading SurfaceCraft… (the first visit downloads the app)',
          failed: 'SurfaceCraft could not start in this browser: ' };
    const loading = document.getElementById('loading');
    if (loading) loading.textContent = text.loading;
    const sharesMemory = () => {
      try {
        const memory = new WebAssembly.Memory({ initial: 1, maximum: 1, shared: true });
        return typeof SharedArrayBuffer !== 'undefined' && memory.buffer instanceof SharedArrayBuffer;
      } catch {
        return false;
      }
    };
    const start = async bundle => (await import(`./${bundle}/surfacecraft-web.js`)).default();
    const oneThread = new URLSearchParams(location.search).get('threads') === '0';
    const threads = !oneThread && self.crossOriginIsolated === true && sharesMemory();
    try {
      try {
        await start(threads ? 'mt' : 'st');
      } catch (e) {
        if (!threads) throw e;
        console.warn('SurfaceCraft: the threaded bundle did not start; using the one without threads', e);
        await start('st');
      }
      loading?.remove();
    } catch (e) {
      console.error(e);
      if (loading) loading.textContent = text.failed + e;
    }
  </script>";

/// Builds the browser bundles into apps/surfacecraft-web/dist: `mt/` with threads (unless
/// `threads` is false) and `st/` without, and the page that picks one.
pub(crate) fn build(threads: bool) -> Result<(), String> {
    let dist = root().join("apps").join("surfacecraft-web").join("dist");
    // only what this task writes is cleared: a `.vercel` link made in dist by hand stays
    for stale in ["mt", "st", "snippets", "surfacecraft-web.js", "surfacecraft-web_bg.wasm"] {
        let path = dist.join(stale);
        let cleared = if path.is_dir() { std::fs::remove_dir_all(&path) } else { std::fs::remove_file(&path) };
        if let Err(e) = cleared
            && path.exists()
        {
            return Err(format!("clear dist/{stale}: {e}"));
        }
    }
    std::fs::create_dir_all(&dist).map_err(|e| format!("create dist: {e}"))?;
    if threads {
        bundle(&dist.join("mt"), true)?;
    }
    bundle(&dist.join("st"), false)?;
    let html = include_str!("../../apps/surfacecraft-web/index.html")
        .replace("<link data-trunk rel=\"rust\" data-bin=\"surfacecraft-web\" data-wasm-opt=\"z\" />", START_SCRIPT);
    std::fs::write(dist.join("index.html"), html).map_err(|e| format!("write index.html: {e}"))?;
    std::fs::write(dist.join("vercel.json"), VERCEL_JSON).map_err(|e| format!("write vercel.json: {e}"))?;
    // a public bundle carries its licences and the third-party notices
    for f in ["LICENSE-MIT", "LICENSE-APACHE", "NOTICE", "THIRD-PARTY-LICENSES.txt"] {
        std::fs::copy(root().join(f), dist.join(f)).map_err(|e| format!("copy {f}: {e}"))?;
    }
    eprintln!("==> web bundle in {} ({})", dist.display(), if threads { "mt and st" } else { "st only" });
    Ok(())
}

/// Compiles the web app with or without threads and puts its module and wasm into `out`.
fn bundle(out: &Path, threads: bool) -> Result<(), String> {
    compile(threads, false)?;
    std::fs::create_dir_all(out).map_err(|e| format!("create {}: {e}", out.display()))?;
    let target = std::env::var_os("CARGO_TARGET_DIR").map_or_else(|| root().join("target"), std::path::PathBuf::from);
    let wasm = target.join("wasm32-unknown-unknown").join("release").join("surfacecraft-web.wasm");
    let mut b = Command::new("wasm-bindgen");
    b.arg(&wasm).args(["--target", "web", "--no-typescript", "--out-dir"]).arg(out);
    run(b, if threads { "wasm-bindgen (threads)" } else { "wasm-bindgen (no threads)" })?;
    optimise_wasm(&out.join("surfacecraft-web_bg.wasm"), threads);
    Ok(())
}

/// Type-checks the threaded build (the code only it compiles: the pool, the allocator, the
/// `atomics` paths) the way [`build`] compiles it, without building it. CI runs this on every
/// pull request, so a break shows there and not at the deploy.
pub(crate) fn check() -> Result<(), String> {
    compile(true, true)
}

/// Compiles (or with `check_only` type-checks) the web app: with threads on the pinned nightly
/// and a rebuilt std, else on stable.
fn compile(threads: bool, check_only: bool) -> Result<(), String> {
    let (verb, profile) = if check_only { ("check", "ci") } else { ("build", "release") };
    let args = [verb, "--profile", profile, "--target", "wasm32-unknown-unknown", "-p", "surfacecraft-web"];
    if !threads {
        let mut c = cargo();
        c.args(args);
        return run(c, "web build (no threads)");
    }
    // rustup picks the toolchain; the variables cargo set for this xtask would pin stable again
    let mut c = Command::new("rustup");
    c.current_dir(root()).args(["run", NIGHTLY, "cargo"]).args(args).args(["-Z", "build-std=std,panic_abort"]);
    for var in ["RUSTUP_TOOLCHAIN", "CARGO", "RUSTC", "RUSTDOC"] {
        c.env_remove(var);
    }
    c.env("RUSTFLAGS", format!("{THREAD_RUSTFLAGS} {BASE_RUSTFLAGS}"));
    run(c, if check_only { "web check (threads)" } else { "web build (threads)" }).map_err(|e| {
        format!(
            "{e}\n  the threaded build needs: rustup toolchain install {NIGHTLY} --profile minimal --component rust-src"
        )
    })
}

/// Runs `wasm-opt` (`-O3` with threads, `-Oz` without) on the bundle when it is on PATH; otherwise (or on failure) keeps the
/// unoptimised file, so the build never depends on binaryen being installed.
fn optimise_wasm(path: &Path, threads: bool) {
    let size = |p: &Path| std::fs::metadata(p).map(|m| m.len()).unwrap_or(0);
    let available = Command::new("wasm-opt").arg("--version").output().is_ok_and(|o| o.status.success());
    let before = size(path);
    if !available {
        eprintln!(
            "==> wasm-opt not on PATH: bundle left unoptimised at {before} bytes (install binaryen to shrink it)"
        );
        return;
    }
    // the threaded bundle is the one that computes in parallel: optimised for speed, not size
    let level = if threads { "-O3" } else { "-Oz" };
    let tmp = path.with_extension("opt.wasm");
    let mut c = Command::new("wasm-opt");
    // The wasm features rustc enables by default for wasm32-unknown-unknown; without them an older
    // wasm-opt mangles the externref table wasm-bindgen emits and the bundle fails at start-up.
    c.arg(level).args([
        "--enable-reference-types",
        "--enable-bulk-memory",
        "--enable-mutable-globals",
        "--enable-sign-ext",
        "--enable-nontrapping-float-to-int",
        "--enable-multivalue",
    ]);
    if threads {
        // atomic instructions and the shared memory
        c.arg("--enable-threads");
    }
    c.arg(path).arg("-o").arg(&tmp);
    let ok = c.status().is_ok_and(|s| s.success());
    if ok && std::fs::rename(&tmp, path).is_ok() {
        eprintln!("==> wasm-opt {level}: {before} -> {} bytes", size(path));
    } else {
        let _ = std::fs::remove_file(&tmp);
        eprintln!("==> wasm-opt failed: bundle left unoptimised ({before} bytes)");
    }
}
