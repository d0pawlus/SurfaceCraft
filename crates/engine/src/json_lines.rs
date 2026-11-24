//! Reading one line of the desktop control channel (JSON lines over localhost TCP), with a bound
//! on its length. Both ends read what the other end sends: the app reads requests from any local
//! process that connects, the MCP bridge reads the app's replies. Neither may hold an unbounded
//! line in memory, so the bound applies while reading, not after.

use std::io::{BufRead, Read};

/// Why a line could not be read.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum LineError {
    /// More than the limit arrived without a newline. The rest of that line is still unread, so
    /// the connection cannot be trusted any more and should be closed.
    #[error("a line longer than {0} bytes")]
    TooLong(usize),
    /// The other end closed the connection in the middle of a line.
    #[error("the connection closed in the middle of a line")]
    Unterminated,
    /// The line is not UTF-8.
    #[error("a line that is not UTF-8")]
    NotUtf8,
    /// Reading failed.
    #[error("reading failed: {0}")]
    Io(String),
}

/// Reads the next line (without its `\n` or `\r\n`), holding at most `max` bytes of it plus the
/// `\n` (a `\r` before it counts towards `max`). `Ok(None)` when the connection closed between
/// lines; a last line without its `\n` is [`LineError::Unterminated`].
///
/// # Errors
/// [`LineError::TooLong`] when more than `max` bytes come before a newline, [`LineError::Unterminated`]
/// when the connection closes in the middle of a line, [`LineError::NotUtf8`] and [`LineError::Io`].
pub fn read_line_bounded<R: BufRead>(reader: &mut R, max: usize) -> Result<Option<String>, LineError> {
    let mut buf = Vec::new();
    let limit = u64::try_from(max).unwrap_or(u64::MAX).saturating_add(1);
    let n = reader.by_ref().take(limit).read_until(b'\n', &mut buf).map_err(|e| LineError::Io(e.to_string()))?;
    if n == 0 {
        return Ok(None);
    }
    if buf.last() != Some(&b'\n') {
        return Err(if buf.len() > max { LineError::TooLong(max) } else { LineError::Unterminated });
    }
    buf.pop();
    if buf.last() == Some(&b'\r') {
        buf.pop();
    }
    String::from_utf8(buf).map(Some).map_err(|_| LineError::NotUtf8)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufReader, Cursor};

    fn read_all(bytes: &[u8], max: usize) -> Vec<Result<Option<String>, LineError>> {
        let mut r = BufReader::with_capacity(7, Cursor::new(bytes.to_vec()));
        let mut out = Vec::new();
        loop {
            let next = read_line_bounded(&mut r, max);
            let stop = !matches!(next, Ok(Some(_)));
            out.push(next);
            if stop {
                return out;
            }
        }
    }

    #[test]
    fn lines_are_read_one_by_one_without_their_newline() {
        let got = read_all(b"{\"a\":1}\r\n{\"b\":2}\n", 64);
        assert_eq!(got, vec![Ok(Some("{\"a\":1}".into())), Ok(Some("{\"b\":2}".into())), Ok(None)]);
    }

    #[test]
    fn a_line_of_exactly_the_limit_passes_and_one_byte_more_does_not() {
        let max = 10;
        let exact = format!("{}\n", "x".repeat(max));
        assert_eq!(read_all(exact.as_bytes(), max)[0], Ok(Some("x".repeat(max))));
        let over = format!("{}\n", "x".repeat(max + 1));
        assert_eq!(read_all(over.as_bytes(), max)[0], Err(LineError::TooLong(max)));
    }

    #[test]
    fn an_endless_line_stops_at_the_limit_instead_of_filling_memory() {
        // a reader that never ends and never sends a newline
        let mut endless = BufReader::new(std::io::repeat(b'x'));
        assert_eq!(read_line_bounded(&mut endless, 1 << 20), Err(LineError::TooLong(1 << 20)));
    }

    #[test]
    fn a_line_cut_off_by_the_end_of_the_connection_is_an_error() {
        assert_eq!(read_all(b"{\"id\":1", 64), vec![Err(LineError::Unterminated)]);
        assert_eq!(read_all(b"ok\n{\"id\"", 64), vec![Ok(Some("ok".into())), Err(LineError::Unterminated)]);
    }

    #[test]
    fn bytes_that_are_not_utf8_are_an_error() {
        assert_eq!(read_all(b"\xff\xfe\n", 64)[0], Err(LineError::NotUtf8));
    }

    #[test]
    fn hostile_bytes_never_panic() {
        // deterministic pseudo-random streams of bytes, newlines and limits, including 0
        let mut state: u64 = 0x9e37_79b9_7f4a_7c15;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        for _ in 0..500 {
            let len = (next() % 300) as usize;
            let bytes: Vec<u8> = (0..len).map(|_| if next() % 9 == 0 { b'\n' } else { (next() % 256) as u8 }).collect();
            let max = (next() % 40) as usize;
            for line in read_all(&bytes, max).into_iter().flatten().flatten() {
                assert!(line.len() <= max, "a line of {} bytes passed the limit {max}", line.len());
            }
        }
    }
}
