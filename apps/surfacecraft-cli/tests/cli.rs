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
    assert!(v["commands"].as_array().unwrap().iter().all(|c| {
        c["id"].as_str().unwrap().contains("mesh") || c["label"].as_str().unwrap().to_lowercase().contains("mesh")
    }));
    let out = cli().args(["describe", "mesh.import"]).output().unwrap();
    let v: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert!(v["input_schema"]["properties"]["path"].is_object());
}

/// `assist` talks to the configured provider program; here a tiny script stands in for an LLM.
#[test]
fn assist_runs_a_command_provider() {
    let dir = std::env::temp_dir().join(format!("sc-assist-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    // the stand-in answers once with align.center, then with a summary (state in a file)
    let reply1 = r#"{"text":"centring","calls":[{"name":"align.center","arguments":{"object":"scan"}}]}"#;
    let reply2 = r#"{"text":"centred","calls":[]}"#;
    let (prog, args): (String, Vec<String>) = if cfg!(windows) {
        let script = dir.join("llm.cmd");
        std::fs::write(
            &script,
            format!("@echo off\r\nmore > nul\r\nif exist \"%~dp0seen\" (echo {reply2}) else (echo x> \"%~dp0seen\" & echo {reply1})\r\n"),
        )
        .unwrap();
        ("cmd".into(), vec!["/C".into(), script.to_string_lossy().to_string()])
    } else {
        let script = dir.join("llm.sh");
        std::fs::write(
            &script,
            format!("#!/bin/sh\ncat > /dev/null\nd=$(dirname \"$0\")\nif [ -f \"$d/seen\" ]; then echo '{reply2}'; else touch \"$d/seen\"; echo '{reply1}'; fi\n"),
        )
        .unwrap();
        ("sh".into(), vec![script.to_string_lossy().to_string()])
    };
    let proj = dir.join("p.scproj");
    let mk = cli()
        .args([
            "exec",
            "mesh.synth",
            r#"{"shape":"box","center":[100,0,0],"name":"scan"}"#,
            "--save",
            &proj.to_string_lossy(),
        ])
        .output()
        .unwrap();
    assert!(mk.status.success());
    let out = cli()
        .env("SURFACECRAFT_AI_COMMAND", format!("{prog} {}", args.join(" ")))
        .args(["assist", "align", "center the scan", "--open", &proj.to_string_lossy()])
        .output()
        .unwrap();
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    let v: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert_eq!(v["summary"], "centred", "{v}");
    assert_eq!(v["steps"][0]["command"], "align.center");
    let none = cli().env_remove("SURFACECRAFT_AI_COMMAND").args(["assist", "align", "x"]).output().unwrap();
    assert_eq!(none.status.code(), Some(2));
    let _ = std::fs::remove_dir_all(dir);
}
