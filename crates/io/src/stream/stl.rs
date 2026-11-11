//! Streaming STL: binary (50-byte records) and ASCII (line by line, bounded lines).
use super::{Batcher, io_err};
use crate::{IoError, bad};
use std::io::{BufRead, BufReader, Read, Seek, SeekFrom};
use surfacecraft_geom::Point3;

const HEADER_BYTES: u64 = 84;
const RECORD_BYTES: u64 = 50;
const NORMAL_BYTES: usize = 12;
const COUNT_AT: usize = 80;
/// Bytes of the head inspected for the `solid` / `facet` keywords (as the in-memory reader does).
const ASCII_PROBE_BYTES: usize = 256;
const READ_BUFFER: usize = 1 << 20;
/// Longest ASCII line accepted.
const MAX_LINE_BYTES: u64 = 4096;

fn le_u32(b: &[u8], at: usize) -> Option<u32> {
    let a = <[u8; 4]>::try_from(b.get(at..at.checked_add(4)?)?).ok()?;
    Some(u32::from_le_bytes(a))
}

fn le_f32(b: &[u8], at: usize) -> Option<f32> {
    le_u32(b, at).map(f32::from_bits)
}

/// Is a file with these leading bytes and this length binary STL? (The rule of the in-memory
/// reader: the declared count fits the length exactly, or the head does not look like ASCII.)
pub(crate) fn is_binary(head: &[u8], len: u64) -> bool {
    let declared = le_u32(head, COUNT_AT).map(u64::from);
    let fits =
        declared.and_then(|n| n.checked_mul(RECORD_BYTES)).and_then(|s| s.checked_add(HEADER_BYTES)) == Some(len);
    let looks_ascii = head.get(..head.len().min(ASCII_PROBE_BYTES)).is_some_and(|h| {
        let t = String::from_utf8_lossy(h);
        t.trim_start().starts_with("solid") && t.contains("facet")
    });
    fits || !looks_ascii
}

/// Reads binary STL from the start of `r`.
pub(crate) fn read_binary<R: Read + Seek>(mut r: R, out: &mut Batcher<'_>) -> Result<(), IoError> {
    let len = r.seek(SeekFrom::End(0)).map_err(|e| io_err(&e))?;
    r.seek(SeekFrom::Start(0)).map_err(|e| io_err(&e))?;
    let mut rd = BufReader::with_capacity(READ_BUFFER, r);
    let mut head = [0u8; HEADER_BYTES as usize];
    rd.read_exact(&mut head).map_err(|_| bad("STL", "file shorter than the 84-byte header"))?;
    let n = u64::from(le_u32(&head, COUNT_AT).ok_or_else(|| bad("STL", "file shorter than the 84-byte header"))?);
    let available = len.saturating_sub(HEADER_BYTES) / RECORD_BYTES;
    if n > available {
        return Err(bad("STL", format!("header declares {n} triangles but the file holds {available}")));
    }
    let mut rec = [0u8; RECORD_BYTES as usize];
    for t in 0..n {
        rd.read_exact(&mut rec).map_err(|_| bad("STL", "truncated triangle"))?;
        let mut corners = [Point3::origin(); 3];
        for (k, c) in corners.iter_mut().enumerate() {
            let at = NORMAL_BYTES + k * 12;
            let (Some(x), Some(y), Some(z)) = (le_f32(&rec, at), le_f32(&rec, at + 4), le_f32(&rec, at + 8)) else {
                return Err(bad("STL", "truncated triangle"));
            };
            *c = Point3::new(f64::from(x), f64::from(y), f64::from(z));
        }
        if !corners.iter().all(surfacecraft_geom::finite) {
            return Err(bad("STL", format!("triangle {t} has a non-finite coordinate")));
        }
        out.push(corners)?;
    }
    Ok(())
}

fn parse3<'a>(it: &mut impl Iterator<Item = &'a str>) -> Option<Point3> {
    let p = Point3::new(it.next()?.parse().ok()?, it.next()?.parse().ok()?, it.next()?.parse().ok()?);
    surfacecraft_geom::finite(&p).then_some(p)
}

/// Reads ASCII STL, one bounded line at a time.
pub(crate) fn read_ascii<R: Read>(r: R, out: &mut Batcher<'_>) -> Result<(), IoError> {
    let mut rd = BufReader::with_capacity(READ_BUFFER, r);
    let mut line = Vec::new();
    let mut corners = [Point3::origin(); 3];
    let (mut corner, mut ln, mut facets) = (0usize, 0usize, 0usize);
    loop {
        line.clear();
        let n = (&mut rd).take(MAX_LINE_BYTES + 1).read_until(b'\n', &mut line).map_err(|e| io_err(&e))?;
        if n == 0 {
            break;
        }
        ln += 1;
        if line.len() as u64 > MAX_LINE_BYTES {
            return Err(bad("STL", format!("line {ln} is longer than {MAX_LINE_BYTES} bytes")));
        }
        let text = std::str::from_utf8(&line).map_err(|_| bad("STL", "ASCII STL is not valid UTF-8"))?;
        let mut it = text.split_whitespace();
        match it.next() {
            Some("vertex") => {
                let p = parse3(&mut it).ok_or_else(|| bad("STL", format!("line {ln}: bad vertex")))?;
                if let Some(slot) = corners.get_mut(corner) {
                    *slot = p;
                }
                corner += 1;
                if corner == 3 {
                    out.push(corners)?;
                    facets += 1;
                    corner = 0;
                }
            }
            Some("endloop") if corner != 0 => {
                return Err(bad("STL", format!("line {ln}: facet without 3 vertices")));
            }
            _ => {}
        }
    }
    if facets == 0 {
        return Err(bad("STL", "no facets"));
    }
    Ok(())
}
