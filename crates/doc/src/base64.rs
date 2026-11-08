//! Standard base64 (RFC 4648, with padding). Decoding hostile input never panics.

const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Encodes bytes.
#[must_use]
pub fn encode(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for chunk in data.chunks(3) {
        let b0 = chunk.first().copied().unwrap_or(0);
        let b1 = chunk.get(1).copied().unwrap_or(0);
        let b2 = chunk.get(2).copied().unwrap_or(0);
        let n = (u32::from(b0) << 16) | (u32::from(b1) << 8) | u32::from(b2);
        let idx = [(n >> 18) & 63, (n >> 12) & 63, (n >> 6) & 63, n & 63];
        for (k, &i) in idx.iter().enumerate() {
            if k <= chunk.len() {
                out.push(char::from(ALPHABET.get(i as usize).copied().unwrap_or(b'A')));
            } else {
                out.push('=');
            }
        }
    }
    out
}

fn value(c: u8) -> Option<u32> {
    match c {
        b'A'..=b'Z' => Some(u32::from(c - b'A')),
        b'a'..=b'z' => Some(u32::from(c - b'a') + 26),
        b'0'..=b'9' => Some(u32::from(c - b'0') + 52),
        b'+' | b'-' => Some(62),
        b'/' | b'_' => Some(63),
        _ => None,
    }
}

/// Decodes base64 (standard or URL-safe alphabet; whitespace ignored; padding optional).
pub fn decode(text: &str) -> Result<Vec<u8>, String> {
    let bytes: Vec<u8> = text.bytes().filter(|b| !b.is_ascii_whitespace() && *b != b'=').collect();
    if bytes.len() % 4 == 1 {
        return Err("base64 length is invalid".into());
    }
    let mut out = Vec::with_capacity(bytes.len() / 4 * 3 + 3);
    for chunk in bytes.chunks(4) {
        let mut n = 0u32;
        for (k, &c) in chunk.iter().enumerate() {
            let v = value(c).ok_or_else(|| format!("invalid base64 character {:?}", char::from(c)))?;
            n |= v << (18 - 6 * k as u32);
        }
        let full = [(n >> 16) as u8, (n >> 8) as u8, n as u8];
        let take = chunk.len().saturating_sub(1);
        out.extend_from_slice(full.get(..take).unwrap_or(&[]));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        for n in 0..40 {
            let data: Vec<u8> = (0..n).map(|i| (i * 37 + 11) as u8).collect();
            assert_eq!(decode(&encode(&data)).unwrap(), data);
        }
        assert_eq!(encode(b"Man"), "TWFu");
        assert_eq!(encode(b"Ma"), "TWE=");
        assert!(decode("T").is_err());
        assert!(decode("T!==").is_err());
    }
}
