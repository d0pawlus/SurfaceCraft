//! The layout of the IGES file: the solids and sheets of a record and the sections of the whole file.

use super::format::*;
use super::writer::*;
use super::*;

/// Solids (outer shell, void shells) and sheets (open face shells) of a record.
pub(super) struct Layout {
    pub(super) solids: Vec<(usize, Vec<usize>)>,
    pub(super) sheets: Vec<usize>,
}

pub(super) fn layout_of(bi: usize, b: &Brep) -> Result<Layout, IoError> {
    let to = |s: u32| usize::try_from(s).map_err(|_| bad("shell index"));
    let mut used = vec![false; b.shells.len()];
    let mark = |s: usize, used: &mut Vec<bool>| -> Result<(), IoError> {
        *used.get_mut(s).ok_or_else(|| bad(format!("body {bi}: shell {s} missing")))? = true;
        Ok(())
    };
    // Shells that bound a bounded region (its outer shell).
    let mut bounding = vec![false; b.shells.len()];
    for reg in b.regions.iter().skip(1) {
        if let Some(&s) = reg.shells.first()
            && let Some(x) = bounding.get_mut(to(s)?)
        {
            *x = true;
        }
    }
    let mut out = Layout { solids: Vec::new(), sheets: Vec::new() };
    for (ri, reg) in b.regions.iter().enumerate().skip(1) {
        if reg.void {
            continue;
        }
        let Some(&outer) = reg.shells.first() else { continue };
        let outer = to(outer)?;
        if !matches!(b.shells.get(outer), Some(Shell::Faces { .. })) {
            return Err(bad(format!("body {bi}: region {ri}'s outer shell is not a face shell")));
        }
        mark(outer, &mut used)?;
        let mut voids = Vec::new();
        for &s in reg.shells.iter().skip(1) {
            let s = to(s)?;
            if bounding.get(s).copied().unwrap_or(false) {
                mark(s, &mut used)?;
                voids.push(s);
            }
        }
        out.solids.push((outer, voids));
    }
    for (si, sh) in b.shells.iter().enumerate() {
        if used.get(si).copied().unwrap_or(true) {
            continue;
        }
        match sh {
            Shell::Faces { .. } => out.sheets.push(si),
            Shell::Wire { .. } => return Err(bad(format!("body {bi}: wire shell {si} is not written"))),
            Shell::Vertex { .. } => return Err(bad(format!("body {bi}: vertex shell {si} is not written"))),
        }
    }
    Ok(out)
}

/// One 80-column line: `data` padded to 72 columns, the section letter and the sequence number.
fn line(out: &mut String, data: &str, sect: char, seq: usize) {
    let _ = writeln!(out, "{data:<72}{sect}{seq:>7}");
}

/// The fields of the global section.
fn global_fields(w: &W, name: &str) -> Vec<String> {
    let stamp = holl("20260101.000000");
    vec![
        "1H,".to_string(),
        "1H;".to_string(),
        holl(name),
        holl(&format!("{name}.igs")),
        holl("SurfaceCraft"),
        holl("SurfaceCraft IGES 5.3"),
        "32".into(),
        "38".into(),
        "6".into(),
        "308".into(),
        "15".into(),
        holl(name),
        real(1.0),
        "2".into(),
        holl("MM"),
        "1".into(),
        real(1.0),
        stamp.clone(),
        real(1e-6),
        real(w.max_coord),
        holl(""),
        holl(""),
        "11".into(),
        "0".into(),
        stamp,
    ]
}

/// Writes the global section; the number of its lines.
fn write_global(out: &mut String, w: &W, name: &str) -> usize {
    let g = global_fields(w, name);
    let mut gseq = 0;
    for chunk in wrap(&g.iter().map(String::as_str).collect::<Vec<_>>(), ',', ';', 72) {
        gseq += 1;
        line(out, &chunk, 'G', gseq);
    }
    gseq
}

/// The parameter lines of each entity (its type, then its parameters; 64 columns).
fn parameter_lines(w: &W) -> Result<Vec<Vec<String>>, IoError> {
    let mut pdata: Vec<Vec<String>> = Vec::with_capacity(w.ents.len());
    for e in &w.ents {
        let mut fields = vec![e.typ.to_string()];
        fields.extend(e.params.iter().cloned());
        pdata.push(wrap(&fields.iter().map(String::as_str).collect::<Vec<_>>(), ',', ';', 64));
    }
    let total: usize = pdata.iter().map(Vec::len).sum();
    if total > MAX_SEQUENCE {
        return Err(bad("too many parameter lines"));
    }
    Ok(pdata)
}

/// A number right-aligned in a field of 8 columns.
fn f8(x: i64) -> String {
    format!("{x:>8}")
}

/// The first directory line of entity `e`, whose parameters start at parameter line `next`.
fn directory_first(e: &Ent, next: usize) -> String {
    let status = if e.dependent { "00010000" } else { "00000000" };
    format!("{}{}{}{}{}{}{}{}{status}", f8(e.typ), f8(next as i64), f8(0), f8(0), f8(0), f8(0), f8(e.xform), f8(0))
}

/// The second directory line of entity `e`, whose parameters take `lines` parameter lines.
fn directory_second(e: &Ent, lines: usize) -> String {
    format!(
        "{}{}{}{}{}{}{}{:>8}{}",
        f8(e.typ),
        f8(0),
        f8(0),
        f8(lines as i64),
        f8(e.form),
        f8(0),
        f8(0),
        e.label,
        f8(0)
    )
}

/// Writes the directory section: two lines per entity.
fn write_directory(out: &mut String, w: &W, pdata: &[Vec<String>]) {
    let mut next = 1usize;
    for (k, (e, p)) in w.ents.iter().zip(pdata).enumerate() {
        line(out, &directory_first(e, next), 'D', 2 * k + 1);
        line(out, &directory_second(e, p.len()), 'D', 2 * k + 2);
        next += p.len();
    }
}

/// Writes the parameter section; the number of its lines.
fn write_parameters(out: &mut String, pdata: &[Vec<String>]) -> usize {
    let mut pseq = 0;
    for (k, p) in pdata.iter().enumerate() {
        for l in p {
            pseq += 1;
            line(out, &format!("{l:<64}{:>8}", 2 * k + 1), 'P', pseq);
        }
    }
    pseq
}

/// The file: start, global, directory and parameter sections and terminator, 80 columns.
pub(super) fn layout(w: &W, name: &str) -> Result<String, IoError> {
    if 2 * w.ents.len() > MAX_SEQUENCE {
        return Err(bad("too many entities for the directory section"));
    }
    let mut out = String::new();
    line(&mut out, "SurfaceCraft IGES export", 'S', 1);
    let gseq = write_global(&mut out, w, name);
    let pdata = parameter_lines(w)?;
    write_directory(&mut out, w, &pdata);
    let pseq = write_parameters(&mut out, &pdata);
    line(&mut out, &format!("S{:>7}G{:>7}D{:>7}P{:>7}", 1, gseq, 2 * w.ents.len(), pseq), 'T', 1);
    Ok(out)
}
