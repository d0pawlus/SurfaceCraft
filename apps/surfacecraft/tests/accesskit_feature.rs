//! The desktop build must resolve eframe with its `accesskit` feature. eframe's defaults include it,
//! but the workspace turns the defaults off, so without the feature the native window exposes no
//! accessibility tree and a screen reader sees only the title bar.
//!
//! The check reads the features cargo resolves for one package with `cargo tree -p <package> -i
//! eframe -e features`. `cargo metadata` resolves the whole workspace at once, so the native app's
//! feature would show up for every member and could not tell the web app apart.
use std::path::Path;
use std::process::Command;

/// The line `cargo tree` prints for eframe's accesskit feature.
const ACCESSKIT: &str = "eframe feature \"accesskit\"";

/// The eframe feature tree that cargo resolves for `package`, run from the workspace root. A cargo
/// that cannot run, or a failing `cargo tree`, fails the test: it is not a reason to skip the check.
fn eframe_features(package: &str) -> String {
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_owned());
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("..").join("..");
    let output = Command::new(cargo)
        .current_dir(&root)
        .args(["tree", "--offline", "-p", package, "-i", "eframe", "-e", "features"])
        .output()
        .unwrap_or_else(|e| panic!("cargo tree could not run: {e}"));
    assert!(output.status.success(), "cargo tree -p {package} failed: {}", String::from_utf8_lossy(&output.stderr));
    String::from_utf8_lossy(&output.stdout).into_owned()
}

#[test]
fn the_native_app_resolves_eframe_with_accesskit() {
    let tree = eframe_features("surfacecraft");
    assert!(tree.contains(ACCESSKIT), "the native app must enable eframe's accesskit feature:\n{tree}");
}

#[test]
fn the_web_app_still_resolves_eframe_without_accesskit() {
    // a deliberate change for the web build means changing this test with it
    let tree = eframe_features("surfacecraft-web");
    assert!(!tree.contains(ACCESSKIT), "the web app does not enable eframe's accesskit feature:\n{tree}");
}
