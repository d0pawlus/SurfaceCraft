//! Bridged mode: commands go to a running desktop app over its localhost control channel.
use crate::Backend;
use serde_json::{Value, json};
use std::io::{BufReader, Write};
use std::net::{Shutdown, TcpStream};
use surfacecraft_engine::json_lines::{LineError, read_line_bounded};

/// Longest reply line read from the app. The largest reply is a screenshot: the bridge clamps it
/// to 4096 x 4096, whose PNG is at most about 64 MiB (RGBA that does not compress) and about
/// 86 MiB in base64; 128 MiB leaves room for that and for large command results.
const MAX_REPLY_BYTES: usize = 128 * 1024 * 1024;

/// A connection to a running app.
pub struct BridgeBackend {
    stream: TcpStream,
    reader: BufReader<TcpStream>,
    token: String,
    next: u64,
}

impl BridgeBackend {
    /// Connects using a discovery file `{port, token}`; `None` when no app answers.
    #[must_use]
    pub fn connect(discovery: &std::path::Path) -> Option<Self> {
        let info: Value = serde_json::from_str(&std::fs::read_to_string(discovery).ok()?).ok()?;
        let port = u16::try_from(info.get("port")?.as_u64()?).ok()?;
        let token = info.get("token")?.as_str()?.to_string();
        let addr = std::net::SocketAddr::from(([127, 0, 0, 1], port));
        let stream = TcpStream::connect_timeout(&addr, std::time::Duration::from_millis(500)).ok()?;
        let reader = BufReader::new(stream.try_clone().ok()?);
        let mut b = Self { stream, reader, token, next: 1 };
        // a live app answers a harmless command
        b.request("engine.execute", json!({"command": "app.commands", "params": {"filter": "app."}})).ok()?;
        Some(b)
    }

    fn request(&mut self, method: &str, params: Value) -> Result<Value, Value> {
        let id = self.next;
        self.next += 1;
        let msg = json!({"token": self.token, "id": id, "method": method, "params": params});
        writeln!(self.stream, "{msg}").map_err(|e| json!({"kind": "failed", "message": e.to_string()}))?;
        let line = match read_line_bounded(&mut self.reader, MAX_REPLY_BYTES) {
            Ok(Some(line)) => line,
            Ok(None) => return Err(json!({"kind": "failed", "message": "the app closed the connection"})),
            Err(e) => {
                if matches!(e, LineError::TooLong(_)) {
                    // the rest of that reply is still unread: later replies would be misread
                    let _ = self.stream.shutdown(Shutdown::Both);
                }
                return Err(json!({"kind": "failed", "message": format!("reply from the app: {e}")}));
            }
        };
        // a plain JSON value (no typed deserialisation), from a line of bounded length
        let v: Value = serde_json::from_str(&line).map_err(|e| json!({"kind": "failed", "message": e.to_string()}))?;
        match (v.get("result"), v.get("error")) {
            (Some(r), _) => Ok(r.clone()),
            (None, Some(e)) => Err(json!({"kind": "failed", "message": e})),
            _ => Err(json!({"kind": "failed", "message": "malformed reply"})),
        }
    }
}

impl Backend for BridgeBackend {
    fn execute(&mut self, id: &str, params: Value) -> Result<Value, Value> {
        let r = self.request("engine.execute", json!({"command": id, "params": params}))?;
        match (r.get("ok"), r.get("err")) {
            (Some(v), _) => Ok(v.clone()),
            (None, Some(e)) => Err(e.clone()),
            _ => Err(json!({"kind": "failed", "message": "malformed reply"})),
        }
    }
    fn screenshot(&mut self, w: u32, h: u32) -> Option<Vec<u8>> {
        let r = self.request("ui.screenshot", json!({"width": w, "height": h})).ok()?;
        let s = r.get("png")?.as_str()?;
        decode_b64(s)
    }
}

fn decode_b64(s: &str) -> Option<Vec<u8>> {
    let val = |c: u8| -> Option<u32> {
        Some(match c {
            b'A'..=b'Z' => u32::from(c - b'A'),
            b'a'..=b'z' => u32::from(c - b'a') + 26,
            b'0'..=b'9' => u32::from(c - b'0') + 52,
            b'+' => 62,
            b'/' => 63,
            _ => return None,
        })
    };
    let bytes: Vec<u8> = s.bytes().filter(|b| *b != b'=' && !b.is_ascii_whitespace()).collect();
    let mut out = Vec::with_capacity(bytes.len() * 3 / 4);
    for chunk in bytes.chunks(4) {
        let mut n = 0u32;
        for (k, &c) in chunk.iter().enumerate() {
            n |= val(c)? << (18 - 6 * k as u32);
        }
        let full = [(n >> 16) as u8, (n >> 8) as u8, n as u8];
        out.extend_from_slice(full.get(..chunk.len().saturating_sub(1))?);
    }
    Some(out)
}
