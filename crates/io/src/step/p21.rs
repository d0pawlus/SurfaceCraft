// Adapted from SolidCraft (https://github.com/tangency-labs/solidcraft) at fcd3186,
// crates/step/src/write.rs (Part 21 writing: reals, strings, entity instances, the exchange file).
// Copyright (c) 2026 SolidCraft contributors. Licensed under MIT OR Apache-2.0.
// Modified for SurfaceCraft (Apache-2.0 §4): reduced to the parameter kinds the shell writer uses,
// own `Param` type, SurfaceCraft header; no geometry types from solidcraft-geom.
// Technical debt (design doc §9/§10): replace with solidcore's STEP writer once it writes shells.

//! ISO 10303-21 writing: a DATA section built entity by entity. Identical geometric entities
//! are written once and shared; topological ones are distinct ([`Writer::add_distinct`]).

use std::collections::HashMap;
use std::fmt::Write as _;

/// An entity parameter.
#[derive(Clone, Debug, PartialEq)]
pub enum Param {
    /// Integer.
    Int(i64),
    /// Real.
    Real(f64),
    /// String literal.
    Str(String),
    /// Reference `#id`.
    Ref(u64),
    /// Enumeration `.NAME.`.
    Enum(String),
    /// Aggregate `( … )`.
    List(Vec<Param>),
    /// Derived value `*`.
    Derived,
    /// Unset value `$`.
    Unset,
    /// Typed parameter `NAME(value)`.
    Typed(String, Box<Param>),
}

/// A real as Part 21 writes it: always with a decimal point, exponent as `E`.
#[must_use]
pub fn real(x: f64) -> String {
    if !x.is_finite() {
        return "0.".into();
    }
    let s = format!("{x:?}");
    match s.split_once('e') {
        Some((m, e)) => {
            let m = if m.contains('.') { m.to_string() } else { format!("{m}.") };
            format!("{m}E{e}")
        }
        None if s.contains('.') => s,
        None => format!("{s}."),
    }
}

/// A string literal: quotes doubled, non-ASCII as `\X2\` UTF-16 runs.
#[must_use]
pub fn string(s: &str) -> String {
    let mut o = String::from("'");
    for c in s.chars() {
        match c {
            '\'' => o.push_str("''"),
            '\\' => o.push_str("\\\\"),
            c if c.is_ascii() && !c.is_ascii_control() => o.push(c),
            c if c.is_ascii_control() => {}
            c => {
                o.push_str("\\X2\\");
                let mut buf = [0u16; 2];
                for u in c.encode_utf16(&mut buf) {
                    let _ = write!(o, "{u:04X}");
                }
                o.push_str("\\X0\\");
            }
        }
    }
    o.push('\'');
    o
}

fn param(p: &Param, o: &mut String) {
    match p {
        Param::Int(i) => {
            let _ = write!(o, "{i}");
        }
        Param::Real(x) => o.push_str(&real(*x)),
        Param::Str(s) => o.push_str(&string(s)),
        Param::Ref(id) => {
            let _ = write!(o, "#{id}");
        }
        Param::Enum(e) => {
            let _ = write!(o, ".{e}.");
        }
        Param::List(v) => {
            o.push('(');
            params(v, o);
            o.push(')');
        }
        Param::Derived => o.push('*'),
        Param::Unset => o.push('$'),
        Param::Typed(name, v) => {
            o.push_str(name);
            o.push('(');
            param(v, o);
            o.push(')');
        }
    }
}

fn params(v: &[Param], o: &mut String) {
    for (i, p) in v.iter().enumerate() {
        if i > 0 {
            o.push(',');
        }
        param(p, o);
    }
}

/// The DATA section under construction.
#[derive(Debug, Default)]
pub struct Writer {
    lines: Vec<String>,
    shared: HashMap<String, u64>,
}

impl Writer {
    /// An empty writer.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    fn push(&mut self, body: String) -> u64 {
        if let Some(id) = self.shared.get(&body) {
            return *id;
        }
        let id = self.push_distinct(&body);
        self.shared.insert(body, id);
        id
    }

    fn push_distinct(&mut self, body: &str) -> u64 {
        let id = self.lines.len() as u64 + 1;
        self.lines.push(format!("#{id}={body};"));
        id
    }

    fn simple(name: &str, p: &[Param]) -> String {
        let mut body = format!("{name}(");
        params(p, &mut body);
        body.push(')');
        body
    }

    /// A simple entity instance, shared with an identical earlier one.
    pub fn add(&mut self, name: &str, p: &[Param]) -> u64 {
        self.push(Self::simple(name, p))
    }

    /// A simple entity instance of its own, even when an identical one exists.
    pub fn add_distinct(&mut self, name: &str, p: &[Param]) -> u64 {
        self.push_distinct(&Self::simple(name, p))
    }

    /// A complex entity instance; its records are written in alphabetical order, as Part 21 asks.
    pub fn add_complex(&mut self, records: &[(&str, Vec<Param>)]) -> u64 {
        let mut sorted: Vec<&(&str, Vec<Param>)> = records.iter().collect();
        sorted.sort_by(|a, b| a.0.cmp(b.0));
        let mut body = String::from("(");
        for (name, p) in sorted {
            body.push_str(name);
            body.push('(');
            params(p, &mut body);
            body.push(')');
        }
        body.push(')');
        self.push(body)
    }

    /// The whole exchange file around the entities written so far.
    #[must_use]
    pub fn file(&self, name: &str, schema: &str) -> String {
        let mut data = String::new();
        for l in &self.lines {
            data.push_str(l);
            data.push('\n');
        }
        format!(
            "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME({},'',(''),(''),'SurfaceCraft','SurfaceCraft','');\nFILE_SCHEMA(({}));\nENDSEC;\nDATA;\n{}ENDSEC;\nEND-ISO-10303-21;\n",
            string(name),
            string(schema),
            data
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reals_strings_and_sharing() {
        assert_eq!(real(1.0), "1.0");
        assert_eq!(real(3e20), "3.E20");
        assert_eq!(real(1.5e-12), "1.5E-12");
        assert_eq!(real(f64::NAN), "0.");
        assert_eq!(string("it's"), "'it''s'");
        let mut w = Writer::new();
        let a = w.add("CARTESIAN_POINT", &[Param::Str(String::new()), Param::List(vec![Param::Real(1.0)])]);
        let b = w.add("CARTESIAN_POINT", &[Param::Str(String::new()), Param::List(vec![Param::Real(1.0)])]);
        assert_eq!(a, b);
        let c = w.add_distinct("VERTEX_POINT", &[Param::Str(String::new()), Param::Ref(a)]);
        assert_ne!(c, a);
        assert!(w.file("x", "AUTOMOTIVE_DESIGN").contains("#2=VERTEX_POINT('',#1);"));
    }
}
