//! `cargo xtask <task>`: the repository's checks.
//!
//! `ci` runs, in order: fmt → assets → licences → layers → ui-containers → app-geometry → clippy → wasm → test.
//! `web [--no-threads]` (not in ci) builds the browser bundles, `web --check` type-checks the threaded one
//! (its own CI job); `web-toolchain` prints the nightly they need.
//! `dist [--tag vX.Y.Z]` (not in ci) builds the host-target release packages in `dist/<target>/release/`.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod assets;
mod dist;
mod layers;
mod licences;
mod ui_containers;
mod web;

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

pub(crate) fn cargo() -> Command {
    let mut c = Command::new(std::env::var("CARGO").unwrap_or_else(|_| "cargo".into()));
    c.current_dir(root());
    c
}

pub(crate) fn run(mut c: Command, what: &str) -> Result<(), String> {
    eprintln!("==> {what}");
    let started = std::time::Instant::now();
    let st = c.status().map_err(|e| format!("{what}: cannot start: {e}"))?;
    eprintln!("==> {what}: {:.1} s", started.elapsed().as_secs_f64());
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

/// Runs every test of the workspace. With cargo-nextest installed (`cargo install cargo-nextest
/// --locked`; CI installs it) the test binaries run in parallel, one process per test, and the
/// doc tests, which nextest does not run, follow with `cargo test --doc`; without it this is a
/// plain `cargo test`. Both run the same tests and leave `#[ignore]` tests ignored.
fn test() -> Result<(), String> {
    if !has_nextest() {
        let mut c = cargo();
        c.args(["test", "--workspace", "--profile", "ci"]);
        return run(c, "tests (cargo test; install cargo-nextest to run them in parallel)");
    }
    let nextest = |extra: &[&str]| {
        let mut c = cargo();
        c.args(["nextest", "run", "--workspace", "--cargo-profile", "ci"]).args(extra);
        c
    };
    run(nextest(&["--no-run"]), "test build")?;
    // every failure is reported, not only the first one
    run(nextest(&["--no-fail-fast"]), "tests (nextest)")?;
    let mut d = cargo();
    d.args(["test", "--workspace", "--doc", "--profile", "ci"]);
    run(d, "doc tests")
}

fn has_nextest() -> bool {
    cargo().args(["nextest", "--version"]).output().is_ok_and(|o| o.status.success())
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
    ui_containers::check()?;
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
    ui_containers::check()?;
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
            "ui-containers" => ui_containers::check(),
            "app-geometry" => app_geometry(),
            "clippy" => clippy(),
            "wasm" => wasm(),
            "test" => test(),
            "web" if check => web::check(),
            "web" => web::build(!args.iter().any(|a| a == "--no-threads")),
            "web-toolchain" => {
                println!("{}", web::NIGHTLY);
                Ok(())
            }
            "dist" => dist::run(&args),
            _ => Err("tasks: ci, quick [--tests] [packages], fmt, assets, licences [--check], layers, ui-containers, app-geometry, clippy, wasm, test, web [--no-threads | --check], web-toolchain, dist [--tag vX.Y.Z]"
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
