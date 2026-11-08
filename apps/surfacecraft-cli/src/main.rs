//! `surfacecraft-cli`: the same commands as the desktop app, from a terminal.
//!
//! ```text
//! surfacecraft-cli run <script.json> [--open <project>] [--save <project>]
//! surfacecraft-cli exec <command> [<params-json>] [--open <project>] [--save <project>]
//! surfacecraft-cli commands [<filter>]
//! surfacecraft-cli describe <command>
//! surfacecraft-cli licences
//! ```
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

use serde_json::{Value, json};
use std::process::ExitCode;
use surfacecraft_engine::Session;

const USAGE: &str = "usage:
  surfacecraft-cli run <script.json> [--open <project>] [--save <project>]
  surfacecraft-cli exec <command> [<params-json>] [--open <project>] [--save <project>]
  surfacecraft-cli commands [<filter>]
  surfacecraft-cli describe <command>
  surfacecraft-cli mcp [--local] [--allow-files]
  surfacecraft-cli assist <align|workflow> <goal> [--open <project>] [--save <project>]
  surfacecraft-cli licences

A script is {\"commands\": [{\"command\": \"mesh.import\", \"params\": {\"path\": \"scan.stl\"}}, ...]}.";

struct Opts {
    positional: Vec<String>,
    open: Option<String>,
    save: Option<String>,
}

fn parse(args: &[String]) -> Result<Opts, String> {
    let mut o = Opts { positional: Vec::new(), open: None, save: None };
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--open" => o.open = Some(it.next().ok_or("--open needs a path")?.clone()),
            "--save" => o.save = Some(it.next().ok_or("--save needs a path")?.clone()),
            _ => o.positional.push(a.clone()),
        }
    }
    Ok(o)
}

fn print(v: &Value) {
    println!("{}", serde_json::to_string_pretty(v).unwrap_or_else(|_| v.to_string()));
}

fn with_project(o: &Opts, s: &mut Session) -> Result<(), String> {
    if let Some(p) = &o.open {
        s.execute("doc.open", json!({"path": p})).map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn save_project(o: &Opts, s: &mut Session) -> Result<(), String> {
    if let Some(p) = &o.save {
        s.execute("doc.save", json!({"path": p})).map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn run(args: &[String]) -> Result<bool, String> {
    let (cmd, rest) = args.split_first().ok_or(USAGE)?;
    let o = parse(rest)?;
    let mut s = Session::new();
    match cmd.as_str() {
        "run" => {
            let path = o.positional.first().ok_or("run needs a script path")?;
            let text = std::fs::read_to_string(path).map_err(|e| format!("cannot read {path}: {e}"))?;
            let script: Value = serde_json::from_str(&text).map_err(|e| format!("{path} is not JSON: {e}"))?;
            with_project(&o, &mut s)?;
            let report = s.run_script(&script).map_err(|e| e.to_string())?;
            print(&serde_json::to_value(&report).unwrap_or(Value::Null));
            if report.ok {
                save_project(&o, &mut s)?;
            }
            Ok(report.ok)
        }
        "exec" => {
            let id = o.positional.first().ok_or("exec needs a command id")?;
            let params: Value = match o.positional.get(1) {
                Some(p) => serde_json::from_str(p).map_err(|e| format!("parameters are not JSON: {e}"))?,
                None => Value::Null,
            };
            with_project(&o, &mut s)?;
            match s.execute(id, params) {
                Ok(v) => {
                    print(&v);
                    save_project(&o, &mut s)?;
                    Ok(true)
                }
                Err(e) => {
                    print(&json!({"error": e}));
                    Ok(false)
                }
            }
        }
        "commands" => {
            let filter = o.positional.first().cloned();
            let v = s.execute("app.commands", json!({"filter": filter})).map_err(|e| e.to_string())?;
            print(&v);
            Ok(true)
        }
        "describe" => {
            let id = o.positional.first().ok_or("describe needs a command id")?;
            let v = s.execute("app.describe", json!({"id": id})).map_err(|e| e.to_string())?;
            print(&v);
            Ok(true)
        }
        "assist" => {
            let mode = match o.positional.first().map(String::as_str) {
                Some("align") => surfacecraft_ai::Mode::Align,
                Some("workflow") => surfacecraft_ai::Mode::Workflow,
                _ => return Err("assist needs a mode: align or workflow".into()),
            };
            let goal = o.positional.get(1).cloned().unwrap_or_default();
            let mut provider = surfacecraft_ai::CommandProvider::from_env()
                .ok_or("set SURFACECRAFT_AI_COMMAND to the program that answers the assistant (request JSON on stdin, response JSON on stdout)")?;
            with_project(&o, &mut s)?;
            let png = surfacecraft_ui_egui::shot::screenshot_png(&mut s, None, 512, 384);
            let out = surfacecraft_ai::run(&mut s, &mut provider, mode, &goal, png.as_deref(), 20)?;
            print(&serde_json::to_value(&out).unwrap_or(Value::Null));
            save_project(&o, &mut s)?;
            Ok(out.steps.iter().all(|st| st.ok))
        }
        "mcp" => {
            let local = o.positional.iter().any(|a| a == "--local");
            let allow_files = o.positional.iter().any(|a| a == "--allow-files");
            mcp_stdio(local, allow_files);
            Ok(true)
        }
        "licences" | "licenses" => {
            println!("{}", include_str!("../../../NOTICE"));
            println!("{}", include_str!("../../../THIRD-PARTY-LICENSES.txt"));
            Ok(true)
        }
        "help" | "--help" | "-h" => {
            println!("{USAGE}");
            Ok(true)
        }
        other => Err(format!("unknown subcommand `{other}`\n\n{USAGE}")),
    }
}

fn serve<B: surfacecraft_mcp::Backend>(mut server: surfacecraft_mcp::Server<B>) {
    use std::io::{BufRead, Write};
    let stdin = std::io::stdin();
    let mut out = std::io::stdout();
    for line in stdin.lock().lines() {
        let Ok(line) = line else { break };
        if let Some(reply) = server.handle_line(&line)
            && (writeln!(out, "{reply}").is_err() || out.flush().is_err())
        {
            break;
        }
    }
}

/// `surfacecraft-cli mcp`: bridged to a running desktop app when one answers, else headless.
fn mcp_stdio(local: bool, allow_files: bool) {
    if !local
        && let Some(b) =
            surfacecraft_mcp::bridge::BridgeBackend::connect(&surfacecraft_ui_egui::control::discovery_path())
    {
        eprintln!("surfacecraft mcp: bridged to the running SurfaceCraft app");
        serve(surfacecraft_mcp::Server::new(b));
        return;
    }
    eprintln!(
        "surfacecraft mcp: headless session (files {})",
        if allow_files { "allowed" } else { "off; pass data inline" }
    );
    let mut backend = surfacecraft_mcp::LocalBackend::new(allow_files);
    backend.render = Some(Box::new(|s, w, h| surfacecraft_ui_egui::shot::screenshot_png(s, None, w, h)));
    serve(surfacecraft_mcp::Server::new(backend));
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(e) => {
            eprintln!("{e}");
            ExitCode::from(2)
        }
    }
}
