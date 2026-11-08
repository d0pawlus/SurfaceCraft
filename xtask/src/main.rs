//! `cargo xtask <task>`: the repository's checks.
//!
//! `ci` runs, in order: fmt → assets → licences → layers → app-geometry → clippy → wasm → test.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod assets;
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

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let task = args.first().map(String::as_str).unwrap_or("help");
    let check = args.iter().any(|a| a == "--check");
    let r = match task {
        "ci" => ci(),
        "fmt" => fmt(),
        "assets" => assets::check(),
        "licences" | "licenses" => licences::run(check),
        "layers" => layers::check(),
        "app-geometry" => app_geometry(),
        "clippy" => clippy(),
        "wasm" => wasm(),
        "test" => test(),
        _ => Err("tasks: ci, fmt, assets, licences [--check], layers, app-geometry, clippy, wasm, test".into()),
    };
    match r {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
