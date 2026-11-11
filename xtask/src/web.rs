//! `cargo xtask web`: the browser bundle in `apps/surfacecraft-web/dist` (needs `wasm-bindgen`
//! on PATH, at the version of Cargo.lock).
//!
//! The bundle has threads: the wasm is built with shared memory (`+atomics`), which needs a
//! standard library rebuilt with the same features (`-Z build-std`), hence one pinned nightly,
//! used for this build only; the rest of the workspace stays on stable, and `cargo xtask wasm`
//! keeps checking the crates for a browser without threads. Threads also need a cross-origin
//! isolated page, so the bundle carries a `vercel.json` with the COOP and COEP headers.
//! `--no-threads` builds the bundle as before, on stable.
use crate::{cargo, root, run};
use std::path::Path;
use std::process::Command;

/// The nightly of the threaded build (with the `rust-src` component). `cargo xtask web-toolchain`
/// prints it for the deploy workflow, so the pin lives here only.
pub(crate) const NIGHTLY: &str = "nightly-2026-11-01";

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

/// Starts the app; a browser that cannot run it says so instead of showing the loading text
/// forever.
const START_SCRIPT: &str = "<script type=\"module\">
    import init from './surfacecraft-web.js';
    const loading = document.getElementById('loading');
    init().then(
      () => loading?.remove(),
      e => {
        console.error(e);
        if (loading) loading.textContent = 'SurfaceCraft could not start in this browser: ' + e;
      }
    );
  </script>";

/// Builds the browser bundle into apps/surfacecraft-web/dist.
pub(crate) fn build(threads: bool) -> Result<(), String> {
    compile(threads)?;
    let dist = root().join("apps").join("surfacecraft-web").join("dist");
    // the snippets directory is named after a hash: one of an earlier build must not be deployed
    // (the rest of dist is overwritten; a `.vercel` link made there by hand stays)
    let snippets = dist.join("snippets");
    if snippets.exists() {
        std::fs::remove_dir_all(&snippets).map_err(|e| format!("clear dist/snippets: {e}"))?;
    }
    std::fs::create_dir_all(&dist).map_err(|e| format!("create dist: {e}"))?;
    let target = std::env::var_os("CARGO_TARGET_DIR").map_or_else(|| root().join("target"), std::path::PathBuf::from);
    let wasm = target.join("wasm32-unknown-unknown").join("release").join("surfacecraft-web.wasm");
    let mut b = Command::new("wasm-bindgen");
    b.arg(&wasm).args(["--target", "web", "--no-typescript", "--out-dir"]).arg(&dist);
    run(b, "wasm-bindgen")?;
    optimise_wasm(&dist.join("surfacecraft-web_bg.wasm"), threads);
    let html = include_str!("../../apps/surfacecraft-web/index.html")
        .replace("<link data-trunk rel=\"rust\" data-bin=\"surfacecraft-web\" data-wasm-opt=\"z\" />", START_SCRIPT);
    std::fs::write(dist.join("index.html"), html).map_err(|e| format!("write index.html: {e}"))?;
    std::fs::write(dist.join("vercel.json"), VERCEL_JSON).map_err(|e| format!("write vercel.json: {e}"))?;
    // a public bundle carries its licences and the third-party notices
    for f in ["LICENSE-MIT", "LICENSE-APACHE", "NOTICE", "THIRD-PARTY-LICENSES.txt"] {
        std::fs::copy(root().join(f), dist.join(f)).map_err(|e| format!("copy {f}: {e}"))?;
    }
    eprintln!("==> web bundle in {} ({})", dist.display(), if threads { "threads" } else { "no threads" });
    Ok(())
}

/// Compiles the web app: with threads on the pinned nightly and a rebuilt std, else on stable.
fn compile(threads: bool) -> Result<(), String> {
    let build = ["build", "--release", "--target", "wasm32-unknown-unknown", "-p", "surfacecraft-web"];
    if !threads {
        let mut c = cargo();
        c.args(build);
        return run(c, "web build (no threads)");
    }
    // rustup picks the toolchain; the variables cargo set for this xtask would pin stable again
    let mut c = Command::new("rustup");
    c.current_dir(root()).args(["run", NIGHTLY, "cargo"]).args(build).args(["-Z", "build-std=std,panic_abort"]);
    for var in ["RUSTUP_TOOLCHAIN", "CARGO", "RUSTC", "RUSTDOC"] {
        c.env_remove(var);
    }
    c.env("RUSTFLAGS", format!("{THREAD_RUSTFLAGS} {BASE_RUSTFLAGS}"));
    run(c, "web build (threads)").map_err(|e| {
        format!(
            "{e}\n  the threaded build needs: rustup toolchain install {NIGHTLY} --profile minimal --component rust-src"
        )
    })
}

/// Runs `wasm-opt -Oz` on the bundle when it is on PATH; otherwise (or on failure) keeps the
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
    let tmp = path.with_extension("opt.wasm");
    let mut c = Command::new("wasm-opt");
    // The wasm features rustc enables by default for wasm32-unknown-unknown; without them an older
    // wasm-opt mangles the externref table wasm-bindgen emits and the bundle fails at start-up.
    c.arg("-Oz").args([
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
        eprintln!("==> wasm-opt -Oz: {before} -> {} bytes", size(path));
    } else {
        let _ = std::fs::remove_file(&tmp);
        eprintln!("==> wasm-opt failed: bundle left unoptimised ({before} bytes)");
    }
}
