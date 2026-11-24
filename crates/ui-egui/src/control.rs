//! The desktop control channel (spike §3.5 item 18, "bridged" MCP): the app listens on
//! `127.0.0.1` (random port) and writes `{port, token, pid}` to a discovery file; clients send
//! JSON lines `{"token", "id", "method", "params"}` with methods `engine.execute`
//! (`{command, params}`) and `ui.screenshot` (`{width, height}`). Requests are executed on the
//! UI thread between frames, so the agent sees exactly what the user sees.
use serde_json::{Value, json};
use std::io::{BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, Sender, channel};
use surfacecraft_engine::json_lines::{LineError, read_line_bounded};

/// Longest request line read from a client that has sent the right token; requests name commands
/// and their parameters. A longer line ends the connection.
const MAX_REQUEST_BYTES: usize = 64 * 1024 * 1024;
/// Longest request line read before the client has sent the right token: any local process can
/// connect, and a client's first request (the bridge asks for `app.commands`) is short.
const MAX_UNVERIFIED_REQUEST_BYTES: usize = 64 * 1024;

/// A request waiting for the UI thread.
pub struct Request {
    /// Method.
    pub method: String,
    /// Parameters.
    pub params: Value,
    /// Where the reply goes.
    pub reply: Sender<Value>,
}

/// Path of the discovery file.
#[must_use]
pub fn discovery_path() -> PathBuf {
    std::env::temp_dir().join("surfacecraft-control.json")
}

fn token() -> String {
    let mut b = [0u8; 16];
    if getrandom::fill(&mut b).is_err() {
        // fall back to time and pid entropy; the channel only listens on localhost
        let t = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_nanos());
        b.copy_from_slice(&(t ^ u128::from(std::process::id())).to_le_bytes());
    }
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn serve(stream: TcpStream, token: &str, tx: &Sender<Request>, wake: &(dyn Fn() + Send + Sync)) {
    let Ok(read) = stream.try_clone() else { return };
    let mut out = stream;
    let mut reader = BufReader::new(read);
    let mut verified = false;
    loop {
        let max = if verified { MAX_REQUEST_BYTES } else { MAX_UNVERIFIED_REQUEST_BYTES };
        let line = match read_line_bounded(&mut reader, max) {
            Ok(Some(line)) => line,
            // a last request without its newline is not answered: clients end every request with one
            Ok(None) | Err(LineError::Unterminated | LineError::Io(_)) => break,
            Err(e) => {
                // the line cannot be answered by its id; the connection ends (a too long line is
                // still partly unread)
                let _ = writeln!(out, "{}", json!({"id": Value::Null, "error": e.to_string()}));
                break;
            }
        };
        let msg: Value = serde_json::from_str(&line).unwrap_or(Value::Null);
        let id = msg.get("id").cloned().unwrap_or(Value::Null);
        let reply = if msg.get("token").and_then(Value::as_str) != Some(token) {
            json!({"id": id, "error": "bad token"})
        } else {
            verified = true;
            let (rtx, rrx) = channel();
            let req = Request {
                method: msg.get("method").and_then(Value::as_str).unwrap_or("").to_string(),
                params: msg.get("params").cloned().unwrap_or(Value::Null),
                reply: rtx,
            };
            if tx.send(req).is_err() {
                break;
            }
            wake();
            match rrx.recv_timeout(std::time::Duration::from_secs(600)) {
                Ok(v) => json!({"id": id, "result": v}),
                Err(_) => json!({"id": id, "error": "the app did not answer"}),
            }
        };
        if writeln!(out, "{reply}").is_err() {
            break;
        }
    }
}

/// Starts listening; returns the request receiver and the port. `wake` asks the UI for a frame.
pub fn start(wake: impl Fn() + Send + Sync + 'static) -> Option<(Receiver<Request>, u16)> {
    let listener = TcpListener::bind(("127.0.0.1", 0)).ok()?;
    let port = listener.local_addr().ok()?.port();
    let tok = token();
    let info = json!({"port": port, "token": tok, "pid": std::process::id()});
    std::fs::write(discovery_path(), info.to_string()).ok()?;
    let (tx, rx) = channel::<Request>();
    let wake = std::sync::Arc::new(wake);
    std::thread::spawn(move || {
        for s in listener.incoming().flatten() {
            let (tx, tok, wake) = (tx.clone(), tok.clone(), wake.clone());
            std::thread::spawn(move || serve(s, &tok, &tx, wake.as_ref()));
        }
    });
    Some((rx, port))
}
