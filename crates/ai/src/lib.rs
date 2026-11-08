//! SurfaceCraft AI assistant (layer L5), design doc §7/§8 M6: modes "align" and "workflow",
//! input = the user's goal and a picture of the viewport, output = engine commands.
//!
//! The assistant is provider-neutral: a [`Provider`] receives the conversation, the tools (every
//! engine command, with its JSON Schema) and an optional PNG, and answers with text and tool
//! calls. The loop executes the calls through the same [`Session::execute`] every front end
//! uses, feeds the results back and stops when the provider answers without calls (or after
//! `max_steps`). Every change it makes is an ordinary undoable command.
//!
//! Which LLM service backs it is the user's decision (not made in code). Built in:
//! [`ScriptedProvider`] (tests, demos) and, natively, [`CommandProvider`], which runs a program
//! the user configures (`SURFACECRAFT_AI_COMMAND`) with the request JSON on stdin and expects
//! the response JSON on stdout — any LLM can be plugged in through it.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use surfacecraft_engine::{Session, registry};

/// What the assistant is asked to do.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    /// Put the scan into a sensible coordinate system.
    Align,
    /// Plan and run a reconstruction workflow toward the goal.
    Workflow,
}

/// One message of the conversation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "role", rename_all = "snake_case")]
pub enum Message {
    /// The user's goal.
    User {
        /// Text.
        text: String,
    },
    /// The assistant's turn.
    Assistant {
        /// Text.
        text: String,
        /// Tool calls it made.
        calls: Vec<ToolCall>,
    },
    /// A tool result.
    Tool {
        /// Tool name.
        name: String,
        /// Result or error JSON.
        result: Value,
        /// Did it fail?
        is_error: bool,
    },
}

/// A tool call: a command id and its parameters.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ToolCall {
    /// Command id (`area.verb`).
    pub name: String,
    /// Parameters.
    #[serde(default)]
    pub arguments: Value,
}

/// What a provider is given.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Request {
    /// System prompt.
    pub system: String,
    /// Conversation so far.
    pub messages: Vec<Message>,
    /// Tools: `{name, description, input_schema}`.
    pub tools: Vec<Value>,
    /// PNG of the viewport, base64, when available.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub image_png_base64: Option<String>,
}

/// What a provider answers.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Response {
    /// Text for the user.
    #[serde(default)]
    pub text: String,
    /// Commands to run.
    #[serde(default)]
    pub calls: Vec<ToolCall>,
}

/// An LLM (or a stand-in).
pub trait Provider {
    /// One completion.
    fn complete(&mut self, request: &Request) -> Result<Response, String>;
}

/// Answers from a fixed list (tests and demos); records what it was asked.
#[derive(Clone, Debug, Default)]
pub struct ScriptedProvider {
    /// Answers, in order.
    pub answers: Vec<Response>,
    /// Requests seen.
    pub seen: Vec<Request>,
}

impl Provider for ScriptedProvider {
    fn complete(&mut self, request: &Request) -> Result<Response, String> {
        self.seen.push(request.clone());
        if self.answers.is_empty() {
            return Ok(Response { text: "done".into(), calls: Vec::new() });
        }
        Ok(self.answers.remove(0))
    }
}

/// Runs a user-configured program: request JSON on stdin, response JSON on stdout.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Clone, Debug)]
pub struct CommandProvider {
    /// Program.
    pub program: String,
    /// Arguments.
    pub args: Vec<String>,
}

#[cfg(not(target_arch = "wasm32"))]
impl CommandProvider {
    /// From `SURFACECRAFT_AI_COMMAND` (program and arguments separated by spaces).
    #[must_use]
    pub fn from_env() -> Option<Self> {
        let v = std::env::var("SURFACECRAFT_AI_COMMAND").ok()?;
        let mut parts = v.split_whitespace().map(str::to_string);
        let program = parts.next()?;
        Some(Self { program, args: parts.collect() })
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl Provider for CommandProvider {
    fn complete(&mut self, request: &Request) -> Result<Response, String> {
        use std::io::Write;
        use std::process::{Command, Stdio};
        let mut child = Command::new(&self.program)
            .args(&self.args)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .map_err(|e| format!("cannot start {}: {e}", self.program))?;
        let body = serde_json::to_vec(request).map_err(|e| e.to_string())?;
        if let Some(mut stdin) = child.stdin.take() {
            stdin.write_all(&body).map_err(|e| e.to_string())?;
        }
        let out = child.wait_with_output().map_err(|e| e.to_string())?;
        if !out.status.success() {
            return Err(format!("{} exited with {}", self.program, out.status));
        }
        serde_json::from_slice(&out.stdout).map_err(|e| format!("the provider's answer is not a response: {e}"))
    }
}

/// Commands offered per mode (the align mode keeps the model on alignment and inspection).
fn tools_for(mode: Mode) -> Vec<Value> {
    registry()
        .iter()
        .filter(|c| match mode {
            Mode::Align => {
                c.id.starts_with("align.")
                    || c.id.starts_with("primitive.")
                    || c.id.starts_with("region.")
                    || matches!(c.id, "doc.list" | "mesh.info" | "deviation.noise" | "edit.undo")
            }
            Mode::Workflow => !matches!(c.id, "doc.open" | "doc.save" | "doc.new"),
        })
        .map(|c| json!({"name": c.id, "description": c.summary, "input_schema": (c.input)()}))
        .collect()
}

fn system_prompt(mode: Mode) -> String {
    let common = "You drive SurfaceCraft, a scan-to-CAD application, only through the tools given; each tool is a command \
with a JSON Schema. Units are millimetres. Prefer measuring before changing (doc.list, mesh.info). Report deviations \
honestly; every change can be undone with edit.undo.";
    match mode {
        Mode::Align => format!(
            "{common} Mode: ALIGN. Put the scan into a coordinate system a designer expects: main planar faces on the \
principal planes, symmetry plane on a principal plane, axes of cylinders on coordinate axes. Typical tools: \
region.auto_segment, primitive.fit, align.datum (3-2-1), align.principal, align.icp. Look at the picture of the viewport."
        ),
        Mode::Workflow => format!(
            "{common} Mode: WORKFLOW. Reach the user's goal step by step: segment the scan into regions, fit primitives \
or surfaces, build sketches and features, check deviation, export."
        ),
    }
}

/// One step the assistant took.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Step {
    /// Command.
    pub command: String,
    /// Parameters.
    pub params: Value,
    /// Succeeded?
    pub ok: bool,
    /// Result or error.
    pub result: Value,
}

/// What a run did.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Outcome {
    /// Commands executed.
    pub steps: Vec<Step>,
    /// The assistant's final words.
    pub summary: String,
    /// Stopped at the step limit?
    pub truncated: bool,
}

/// Runs the assistant on `session` toward `goal`. `image` is a PNG of the viewport (optional).
pub fn run(
    session: &mut Session,
    provider: &mut dyn Provider,
    mode: Mode,
    goal: &str,
    image: Option<&[u8]>,
    max_steps: usize,
) -> Result<Outcome, String> {
    let tools = tools_for(mode);
    let allowed: Vec<String> =
        tools.iter().filter_map(|t| t.get("name").and_then(Value::as_str).map(str::to_string)).collect();
    let mut messages = vec![Message::User { text: goal.to_string() }];
    let image_png_base64 = image.map(b64);
    let mut steps = Vec::new();
    for _ in 0..max_steps.clamp(1, 200) {
        let req = Request {
            system: system_prompt(mode),
            messages: messages.clone(),
            tools: tools.clone(),
            image_png_base64: image_png_base64.clone(),
        };
        let resp = provider.complete(&req)?;
        if resp.calls.is_empty() {
            return Ok(Outcome { steps, summary: resp.text, truncated: false });
        }
        messages.push(Message::Assistant { text: resp.text.clone(), calls: resp.calls.clone() });
        for call in resp.calls {
            let (ok, result) = if allowed.contains(&call.name) {
                match session.execute(&call.name, call.arguments.clone()) {
                    Ok(v) => (true, v),
                    Err(e) => (false, serde_json::to_value(&e).unwrap_or(Value::Null)),
                }
            } else {
                (false, json!({"error": format!("`{}` is not available in this mode", call.name)}))
            };
            messages.push(Message::Tool { name: call.name.clone(), result: result.clone(), is_error: !ok });
            steps.push(Step { command: call.name, params: call.arguments, ok, result });
        }
    }
    Ok(Outcome { steps, summary: "stopped at the step limit".into(), truncated: true })
}

fn b64(data: &[u8]) -> String {
    const A: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for c in data.chunks(3) {
        let n = (u32::from(c.first().copied().unwrap_or(0)) << 16)
            | (u32::from(c.get(1).copied().unwrap_or(0)) << 8)
            | u32::from(c.get(2).copied().unwrap_or(0));
        for k in 0..4 {
            if k <= c.len() {
                out.push(char::from(A.get(((n >> (18 - 6 * k)) & 63) as usize).copied().unwrap_or(b'A')));
            } else {
                out.push('=');
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn call(name: &str, arguments: Value) -> ToolCall {
        ToolCall { name: name.into(), arguments }
    }

    /// Gate M6 (assistant, align mode): the loop gives the model the alignment tools and the
    /// picture, executes its calls through the engine and returns its summary.
    #[test]
    fn gate_m6_assistant_align_mode() {
        let mut s = Session::new();
        s.execute("mesh.synth", json!({"shape": "box", "size": [60, 30, 10], "center": [100, 50, 20], "name": "scan"}))
            .unwrap();
        s.execute("mesh.transform", json!({"object": "scan", "rotate": {"axis": [1, 1, 0], "degrees": 25}})).unwrap();
        let mut p = ScriptedProvider {
            answers: vec![
                Response { text: "Measuring.".into(), calls: vec![call("mesh.info", json!({"object": "scan"}))] },
                Response {
                    text: "Aligning to principal axes.".into(),
                    calls: vec![call("align.principal", json!({"object": "scan"}))],
                },
                Response { text: "The block is aligned: 60 × 30 × 10 along X/Y/Z.".into(), calls: vec![] },
            ],
            seen: Vec::new(),
        };
        let out = run(&mut s, &mut p, Mode::Align, "align this scan", Some(&[0x89, b'P', b'N', b'G']), 10).unwrap();
        assert_eq!(out.steps.len(), 2);
        assert!(out.steps.iter().all(|st| st.ok));
        assert!(out.summary.contains("aligned"));
        let b = s.doc.bbox().size();
        assert!((b.x - 60.0).abs() < 1e-6 && (b.y - 30.0).abs() < 1e-6 && (b.z - 10.0).abs() < 1e-6, "{b:?}");
        // the provider saw the picture, the align tools only, and the tool results
        let first = &p.seen[0];
        assert!(first.image_png_base64.is_some());
        let names: Vec<&str> = first.tools.iter().filter_map(|t| t["name"].as_str()).collect();
        assert!(names.contains(&"align.principal") && !names.contains(&"export.step"));
        assert!(matches!(p.seen[1].messages.last(), Some(Message::Tool { is_error: false, .. })));
        // undo removes the assistant's change like any other
        s.execute("edit.undo", Value::Null).unwrap();
        assert!((s.doc.bbox().size().x - 60.0).abs() > 1.0);
    }

    #[test]
    fn workflow_mode_reports_errors_and_limits() {
        let mut s = Session::new();
        let mut p = ScriptedProvider {
            answers: vec![
                Response {
                    text: String::new(),
                    calls: vec![call("mesh.synth", json!({"shape": "bracket", "noise": 0.05, "name": "scan"}))],
                },
                Response {
                    text: String::new(),
                    calls: vec![
                        call("region.auto_segment", json!({"mesh": "scan"})),
                        call("doc.save", json!({"path": "x"})),
                        call("no.such", json!({})),
                    ],
                },
            ],
            seen: Vec::new(),
        };
        let out = run(&mut s, &mut p, Mode::Workflow, "reconstruct the bracket", None, 2).unwrap();
        assert!(out.truncated);
        assert_eq!(out.steps.len(), 4);
        assert!(out.steps[1].ok);
        assert!(!out.steps[2].ok && !out.steps[3].ok, "doc.save is not offered and unknown tools fail");
        assert_eq!(s.doc.regions.len(), 8);
    }
}
