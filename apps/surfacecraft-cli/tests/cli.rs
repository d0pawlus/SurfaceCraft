//! `surfacecraft-cli` end to end: the binary runs scripts and single commands.
use std::process::Command;

fn cli() -> Command {
    Command::new(env!("CARGO_BIN_EXE_surfacecraft-cli"))
}

fn root() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

#[test]
fn run_example_script_and_save_project() {
    let dir = std::env::temp_dir().join(format!("sc-cli-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let proj = dir.join("out.scproj");
    let out = cli()
        .args([
            "run",
            &root().join("examples/scripts/m0-smoke.json").to_string_lossy(),
            "--save",
            &proj.to_string_lossy(),
        ])
        .output()
        .unwrap();
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    let report: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert_eq!(report["ok"], true);
    let vol = report["steps"][4]["result"]["volume"].as_f64().unwrap();
    assert!(vol < 1000.0 && vol > 990.0, "filleted box volume {vol}");
    // the saved project opens in a new process
    let out = cli().args(["exec", "doc.list", "--open", &proj.to_string_lossy()]).output().unwrap();
    assert!(out.status.success());
    let list: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert_eq!(list["objects"].as_array().unwrap().len(), 2);
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn exec_reports_errors_with_exit_code() {
    let out = cli().args(["exec", "body.box", r#"{"size": [0, 1, 1]}"#]).output().unwrap();
    assert_eq!(out.status.code(), Some(1));
    let out = cli().args(["exec", "no.such"]).output().unwrap();
    assert_eq!(out.status.code(), Some(1));
    let out = cli().args(["bogus"]).output().unwrap();
    assert_eq!(out.status.code(), Some(2));
}

#[test]
fn commands_and_describe() {
    let out = cli().args(["commands", "mesh"]).output().unwrap();
    let v: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert!(v["commands"].as_array().unwrap().iter().all(|c| c["id"].as_str().unwrap().contains("mesh")));
    let out = cli().args(["describe", "mesh.import"]).output().unwrap();
    let v: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert!(v["input_schema"]["properties"]["path"].is_object());
}
