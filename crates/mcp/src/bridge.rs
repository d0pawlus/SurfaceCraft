//! Bridged mode: commands go to a running desktop app over its localhost control channel.
use crate::Backend;
use serde_json::{Value, json};
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;

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
        let mut line = String::new();
        self.reader.read_line(&mut line).map_err(|e| json!({"kind": "failed", "message": e.to_string()}))?;
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
