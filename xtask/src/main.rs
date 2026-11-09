//! `cargo xtask <task>`: the repository's checks.
//!
//! `ci` runs, in order: fmt → assets → licences → layers → app-geometry → clippy → wasm → test.
//! `dist` (not in ci) builds the host-target release binaries and `dist/<target>/SHA256SUMS.txt`.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod assets;
mod dist;
mod layers;
mod licences;

use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

/// Crates checked for wasm32 (every layer crate plus the web app).
const WASM_CRATES: &[&str] = &[
    "surfacecraft-geom",
    "surfacecraft-kernel",
    "surfacecraft-fit",
    "surfacecraft-align",
    "surfacecraft-sketch",
    "surfacecraft-deviation",
    "surfacecraft-doc",
    "surfacecraft-io",
    "surfacecraft-engine",
    "surfacecraft-ui-egui",
    "surfacecraft-mcp",
    "surfacecraft-ai",
    "surfacecraft-web",
];

pub(crate) fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).parent().map(Path::to_path_buf).unwrap_or_else(|| PathBuf::from("."))
}

fn cargo() -> Command {
    let mut c = Command::new(std::env::var("CARGO").unwrap_or_else(|_| "cargo".into()));
    c.current_dir(root());
    c
}

fn run(mut c: Command, what: &str) -> Result<(), String> {
    eprintln!("==> {what}");
    let st = c.status().map_err(|e| format!("{what}: cannot start: {e}"))?;
    if st.success() { Ok(()) } else { Err(format!("{what} failed")) }
}

fn fmt() -> Result<(), String> {
    let mut c = cargo();
    c.args(["fmt", "--all", "--", "--check"]);
    run(c, "fmt --check")
}

fn clippy() -> Result<(), String> {
    let mut c = cargo();
    c.args(["clippy", "--workspace", "--all-targets", "--profile", "ci", "--", "-D", "warnings"]);
    run(c, "clippy -D warnings")
}

fn wasm() -> Result<(), String> {
    let mut c = cargo();
    c.args(["check", "--target", "wasm32-unknown-unknown", "--profile", "ci"]);
    for k in WASM_CRATES {
        c.args(["-p", k]);
    }
    run(c, "wasm check")
}

/// Builds the browser bundle into apps/surfacecraft-web/dist (needs `wasm-bindgen` on PATH).
fn web() -> Result<(), String> {
    let mut c = cargo();
    c.args(["build", "--release", "--target", "wasm32-unknown-unknown", "-p", "surfacecraft-web"]);
    run(c, "web build")?;
    let dist = root().join("apps").join("surfacecraft-web").join("dist");
    std::fs::create_dir_all(&dist).map_err(|e| format!("create dist: {e}"))?;
    let wasm = root().join("target").join("wasm32-unknown-unknown").join("release").join("surfacecraft-web.wasm");
    let mut b = Command::new("wasm-bindgen");
    b.arg(&wasm).args(["--target", "web", "--no-typescript", "--out-dir"]).arg(&dist);
    run(b, "wasm-bindgen")?;
    optimise_wasm(&dist.join("surfacecraft-web_bg.wasm"));
    let html = include_str!("../../apps/surfacecraft-web/index.html").replace(
        "<link data-trunk rel=\"rust\" data-bin=\"surfacecraft-web\" data-wasm-opt=\"z\" />",
        "<script type=\"module\">import init from './surfacecraft-web.js'; init();</script>",
    );
    std::fs::write(dist.join("index.html"), html).map_err(|e| format!("write index.html: {e}"))?;
    eprintln!("==> web bundle in {}", dist.display());
    Ok(())
}

/// Runs `wasm-opt -Oz` on the bundle when it is on PATH; otherwise (or on failure) keeps the
/// unoptimised file, so the build never depends on binaryen being installed.
fn optimise_wasm(path: &Path) {
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
    c.arg("-Oz").arg(path).arg("-o").arg(&tmp);
    let ok = c.status().is_ok_and(|s| s.success());
    if ok && std::fs::rename(&tmp, path).is_ok() {
        eprintln!("==> wasm-opt -Oz: {before} -> {} bytes", size(path));
    } else {
        let _ = std::fs::remove_file(&tmp);
        eprintln!("==> wasm-opt failed: bundle left unoptimised ({before} bytes)");
    }
}

fn test() -> Result<(), String> {
    let has_nextest = cargo().args(["nextest", "--version"]).output().map(|o| o.status.success()).unwrap_or(false);
    if has_nextest {
        let mut c = cargo();
        c.args(["nextest", "run", "--workspace", "--profile", "ci", "--cargo-profile", "ci"]);
        run(c, "tests (nextest)")?;
        let mut d = cargo();
        d.args(["test", "--workspace", "--doc", "--profile", "ci"]);
        run(d, "doc tests")
    } else {
        let mut c = cargo();
        c.args(["test", "--workspace", "--profile", "ci"]);
        run(c, "tests")
    }
}

fn app_geometry() -> Result<(), String> {
    eprintln!("==> app-geometry");
    let list_path = root().join("xtask").join("app-geometry.txt");
    let text = std::fs::read_to_string(&list_path).map_err(|e| format!("read {}: {e}", list_path.display()))?;
    let listed: Vec<String> =
        text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')).map(str::to_string).collect();
    let mut marked = Vec::new();
    for dir in ["crates", "apps"] {
        walk(&root().join(dir), &mut |p| {
            if p.extension().is_some_and(|e| e == "rs")
                && let Ok(s) = std::fs::read_to_string(p)
                && s.contains("APP-GEOMETRY:")
            {
                marked.push(rel(p));
            }
        });
    }
    let mut errors = Vec::new();
    for m in &marked {
        if !listed.contains(m) {
            errors.push(format!("{m} computes app-side B-rep/NURBS geometry (APP-GEOMETRY marker) but is not listed in xtask/app-geometry.txt"));
        }
    }
    for l in &listed {
        if !marked.contains(l) {
            errors.push(format!("xtask/app-geometry.txt lists {l}, which has no APP-GEOMETRY marker: remove the entry (the list only shrinks)"));
        }
    }
    if errors.is_empty() { Ok(()) } else { Err(errors.join("\n")) }
}

pub(crate) fn walk(dir: &Path, f: &mut dyn FnMut(&Path)) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    for e in rd.flatten() {
        let p = e.path();
        let name = e.file_name();
        if name == "target" || name == ".git" {
            continue;
        }
        if p.is_dir() {
            walk(&p, f);
        } else {
            f(&p);
        }
    }
}

pub(crate) fn rel(p: &Path) -> String {
    p.strip_prefix(root()).unwrap_or(p).to_string_lossy().replace('\\', "/")
}

fn ci() -> Result<(), String> {
    fmt()?;
    assets::check()?;
    licences::run(true)?;
    layers::check()?;
    app_geometry()?;
    clippy()?;
    wasm()?;
    test()?;
    eprintln!("==> ci passed");
    Ok(())
}

/// The local pre-commit check: the cheap static steps, then clippy `-D warnings` (which type-checks)
/// of the packages named (`cargo xtask quick surfacecraft-fit surfacecraft-engine`); `--tests` also
/// runs their tests. The full `cargo xtask ci` runs on GitHub Actions for every pull request, and
/// local-only checks run in batched integration passes (docs/integration.md).
fn quick(packages: &[String], with_tests: bool) -> Result<(), String> {
    fmt()?;
    assets::check()?;
    licences::run(true)?;
    layers::check()?;
    app_geometry()?;
    if !packages.is_empty() {
        let mut c = cargo();
        c.args(["clippy", "--all-targets", "--profile", "ci"]);
        for p in packages {
            c.args(["-p", p]);
        }
        c.args(["--", "-D", "warnings"]);
        run(c, "clippy -D warnings (named packages)")?;
        if with_tests {
            let mut t = cargo();
            t.args(["test", "--profile", "ci"]);
            for p in packages {
                t.args(["-p", p]);
            }
            run(t, "tests (named packages)")?;
        }
    }
    eprintln!("==> quick passed (full ci runs on the pull request)");
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let task = args.first().map(String::as_str).unwrap_or("help");
    let check = args.iter().any(|a| a == "--check");
    let r =
        match task {
            "ci" => ci(),
            "quick" => {
                let rest: Vec<String> = args.iter().skip(1).filter(|a| *a != "--tests").cloned().collect();
                quick(&rest, args.iter().any(|a| a == "--tests"))
            }
            "fmt" => fmt(),
            "assets" => assets::check(),
            "licences" | "licenses" => licences::run(check),
            "layers" => layers::check(),
            "app-geometry" => app_geometry(),
            "clippy" => clippy(),
            "wasm" => wasm(),
            "test" => test(),
            "web" => web(),
            "dist" => dist::run(),
            _ => Err("tasks: ci, quick [--tests] [packages], fmt, assets, licences [--check], layers, app-geometry, clippy, wasm, test, web, dist"
                .into()),
        };
    match r {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
