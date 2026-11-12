//! Arithmetic expressions over named values: the language of parameters and of dimensions that
//! are given as a formula (`w / 2`, `sin(a) * r`).
//!
//! The language is unitless; the callers decide what a number means (millimetres, or degrees for
//! angles). Grammar, loosest binding first:
//!
//! ```text
//! sum     = product { ("+" | "-") product }
//! product = unary { ("*" | "/") unary }
//! unary   = ("-" | "+") unary | power
//! power   = atom [ "^" unary ]
//! atom    = number | name | function "(" sum { "," sum } ")" | "(" sum ")"
//! ```
//!
//! `^` is right-associative and binds tighter than a unary minus on its left, so `2^3^2` is 512,
//! `-2^2` is -4 and `2^-2` is 0.25 (the exponent may carry a sign). `+ - * /` are
//! left-associative. There is no implicit multiplication. Angles of the trigonometric functions
//! are in degrees, and so are the results of the inverse ones.
//!
//! Every failure is a value: an unknown name, a division by zero, a domain error, a result that
//! is not finite, a text that is too long or nested too deeply. Evaluation never panics.
use std::collections::BTreeSet;
use std::fmt;

/// Longest expression text accepted (bytes).
pub const MAX_LEN: usize = 1024;
/// Deepest nesting of parentheses, signs and exponents accepted.
pub const MAX_DEPTH: usize = 64;
/// Longest name (bytes).
pub const MAX_NAME_LEN: usize = 64;

/// Why an expression could not be parsed or evaluated.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExprError(pub String);

impl fmt::Display for ExprError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ExprError {}

fn err<T>(m: impl Into<String>) -> Result<T, ExprError> {
    Err(ExprError(m.into()))
}

/// A binary operator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    /// `+`.
    Add,
    /// `-`.
    Sub,
    /// `*`.
    Mul,
    /// `/`.
    Div,
    /// `^`.
    Pow,
}

/// A built-in function.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Func {
    /// Sine of degrees.
    Sin,
    /// Cosine of degrees.
    Cos,
    /// Tangent of degrees.
    Tan,
    /// Arc sine, in degrees.
    Asin,
    /// Arc cosine, in degrees.
    Acos,
    /// Arc tangent, in degrees.
    Atan,
    /// Arc tangent of y / x, in degrees.
    Atan2,
    /// Square root.
    Sqrt,
    /// Absolute value.
    Abs,
    /// Smallest of the arguments.
    Min,
    /// Largest of the arguments.
    Max,
    /// Largest integer not above.
    Floor,
    /// Smallest integer not below.
    Ceil,
    /// Nearest integer, halves away from zero.
    Round,
}

const FUNCS: [(&str, Func); 14] = [
    ("sin", Func::Sin),
    ("cos", Func::Cos),
    ("tan", Func::Tan),
    ("asin", Func::Asin),
    ("acos", Func::Acos),
    ("atan", Func::Atan),
    ("atan2", Func::Atan2),
    ("sqrt", Func::Sqrt),
    ("abs", Func::Abs),
    ("min", Func::Min),
    ("max", Func::Max),
    ("floor", Func::Floor),
    ("ceil", Func::Ceil),
    ("round", Func::Round),
];

/// The only constant.
const PI_NAME: &str = "pi";
/// Most arguments of `min` and `max`.
const MAX_ARGS: usize = 16;

impl Func {
    fn from_name(n: &str) -> Option<Func> {
        FUNCS.iter().find(|(name, _)| *name == n).map(|(_, f)| *f)
    }

    /// The name in expressions.
    #[must_use]
    pub fn name(self) -> &'static str {
        FUNCS.iter().find(|(_, f)| *f == self).map_or("?", |(n, _)| n)
    }

    fn arity_ok(self, n: usize) -> bool {
        match self {
            Func::Atan2 => n == 2,
            Func::Min | Func::Max => (2..=MAX_ARGS).contains(&n),
            _ => n == 1,
        }
    }
}

/// Is `name` a function or constant of the language (so not usable as a parameter name)?
#[must_use]
pub fn is_reserved(name: &str) -> bool {
    name == PI_NAME || Func::from_name(name).is_some()
}

/// Checks a parameter name: a letter or underscore, then letters, digits and underscores (ASCII,
/// case-sensitive), at most [`MAX_NAME_LEN`] bytes, not a function or constant of the language.
pub fn check_name(name: &str) -> Result<(), ExprError> {
    let mut chars = name.chars();
    let first_ok = chars.next().is_some_and(|c| c.is_ascii_alphabetic() || c == '_');
    if !first_ok || !chars.all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return err(format!(
            "`{}` is not a valid name: use a letter or underscore, then letters, digits and underscores",
            name.chars().take(40).collect::<String>()
        ));
    }
    if name.len() > MAX_NAME_LEN {
        return err(format!("a name is at most {MAX_NAME_LEN} characters"));
    }
    if is_reserved(name) {
        return err(format!("`{name}` is a function or constant of the expression language"));
    }
    Ok(())
}

/// A syntax tree.
#[derive(Clone, Debug, PartialEq)]
pub enum Node {
    /// A literal.
    Num(f64),
    /// A name (a parameter).
    Var(String),
    /// Unary minus.
    Neg(Box<Node>),
    /// A binary operation.
    Bin(Op, Box<Node>, Box<Node>),
    /// A function call.
    Call(Func, Vec<Node>),
}

/// A parsed expression.
#[derive(Clone, Debug, PartialEq)]
pub struct Expr {
    root: Node,
}

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Num(f64),
    Ident(String),
    Plus,
    Minus,
    Star,
    Slash,
    Caret,
    LParen,
    RParen,
    Comma,
}

/// A token and the byte range of the text it came from.
struct Lexed {
    tok: Tok,
    start: usize,
    end: usize,
}

fn lex_number(text: &str, start: usize) -> Result<(f64, usize), ExprError> {
    let b = text.as_bytes();
    let at = |i: usize| b.get(i).copied().unwrap_or(0);
    let mut i = start;
    while at(i).is_ascii_digit() {
        i += 1;
    }
    if at(i) == b'.' {
        i += 1;
        while at(i).is_ascii_digit() {
            i += 1;
        }
    }
    let signed = matches!(at(i + 1), b'+' | b'-') && at(i + 2).is_ascii_digit();
    if matches!(at(i), b'e' | b'E') && (at(i + 1).is_ascii_digit() || signed) {
        i += 2;
        while at(i).is_ascii_digit() {
            i += 1;
        }
    }
    if at(i).is_ascii_alphanumeric() || at(i) == b'_' || at(i) == b'.' {
        return err(format!("malformed number near position {}", start + 1));
    }
    let lit = text.get(start..i).unwrap_or("");
    match lit.parse::<f64>() {
        Ok(v) if v.is_finite() => Ok((v, i)),
        Ok(_) => err(format!("the number {lit} is out of range")),
        Err(_) => err(format!("malformed number {lit}")),
    }
}

fn lex(text: &str) -> Result<Vec<Lexed>, ExprError> {
    if text.len() > MAX_LEN {
        return err(format!("an expression is at most {MAX_LEN} characters"));
    }
    let b = text.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while let Some(&c) = b.get(i) {
        let single = match c {
            b' ' | b'\t' | b'\n' | b'\r' => {
                i += 1;
                continue;
            }
            b'+' => Some(Tok::Plus),
            b'-' => Some(Tok::Minus),
            b'*' => Some(Tok::Star),
            b'/' => Some(Tok::Slash),
            b'^' => Some(Tok::Caret),
            b'(' => Some(Tok::LParen),
            b')' => Some(Tok::RParen),
            b',' => Some(Tok::Comma),
            _ => None,
        };
        if let Some(tok) = single {
            out.push(Lexed { tok, start: i, end: i + 1 });
            i += 1;
        } else if c.is_ascii_digit() || (c == b'.' && b.get(i + 1).is_some_and(u8::is_ascii_digit)) {
            let (v, end) = lex_number(text, i)?;
            out.push(Lexed { tok: Tok::Num(v), start: i, end });
            i = end;
        } else if c.is_ascii_alphabetic() || c == b'_' {
            let mut end = i;
            while b.get(end).is_some_and(|d| d.is_ascii_alphanumeric() || *d == b'_') {
                end += 1;
            }
            let name = text.get(i..end).unwrap_or("").to_string();
            out.push(Lexed { tok: Tok::Ident(name), start: i, end });
            i = end;
        } else {
            return err(format!("unexpected character at position {}", i + 1));
        }
    }
    Ok(out)
}

struct Parser {
    toks: Vec<Tok>,
    pos: usize,
    depth: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.pos)
    }

    fn eat(&mut self, t: &Tok) -> bool {
        if self.peek() == Some(t) {
            self.pos += 1;
            true
        } else {
            false
        }
    }

    fn enter(&mut self) -> Result<(), ExprError> {
        self.depth += 1;
        if self.depth > MAX_DEPTH {
            return err(format!("the expression is nested more than {MAX_DEPTH} levels deep"));
        }
        Ok(())
    }

    fn sum(&mut self) -> Result<Node, ExprError> {
        self.enter()?;
        let mut left = self.product()?;
        loop {
            let op = match self.peek() {
                Some(Tok::Plus) => Op::Add,
                Some(Tok::Minus) => Op::Sub,
                _ => break,
            };
            self.pos += 1;
            let right = self.product()?;
            left = Node::Bin(op, Box::new(left), Box::new(right));
        }
        self.depth -= 1;
        Ok(left)
    }

    fn product(&mut self) -> Result<Node, ExprError> {
        let mut left = self.unary()?;
        loop {
            let op = match self.peek() {
                Some(Tok::Star) => Op::Mul,
                Some(Tok::Slash) => Op::Div,
                _ => break,
            };
            self.pos += 1;
            let right = self.unary()?;
            left = Node::Bin(op, Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    fn unary(&mut self) -> Result<Node, ExprError> {
        if self.eat(&Tok::Minus) {
            self.enter()?;
            let inner = self.unary()?;
            self.depth -= 1;
            return Ok(Node::Neg(Box::new(inner)));
        }
        if self.eat(&Tok::Plus) {
            self.enter()?;
            let inner = self.unary()?;
            self.depth -= 1;
            return Ok(inner);
        }
        self.power()
    }

    fn power(&mut self) -> Result<Node, ExprError> {
        let base = self.atom()?;
        if self.eat(&Tok::Caret) {
            self.enter()?;
            let exp = self.unary()?;
            self.depth -= 1;
            return Ok(Node::Bin(Op::Pow, Box::new(base), Box::new(exp)));
        }
        Ok(base)
    }

    fn atom(&mut self) -> Result<Node, ExprError> {
        let Some(t) = self.peek().cloned() else {
            return err("the expression ends where a value was expected");
        };
        self.pos += 1;
        match t {
            Tok::Num(v) => Ok(Node::Num(v)),
            Tok::LParen => {
                let inner = self.sum()?;
                if !self.eat(&Tok::RParen) {
                    return err("a parenthesis is not closed");
                }
                Ok(inner)
            }
            Tok::Ident(name) => self.name_or_call(name),
            Tok::RParen => err("unexpected `)`"),
            Tok::Comma => err("unexpected `,`"),
            Tok::Plus | Tok::Minus | Tok::Star | Tok::Slash | Tok::Caret => err("an operator is missing its operand"),
        }
    }

    fn name_or_call(&mut self, name: String) -> Result<Node, ExprError> {
        let Some(f) = Func::from_name(&name) else {
            if self.peek() == Some(&Tok::LParen) {
                return err(format!("`{name}` is not a function"));
            }
            return Ok(if name == PI_NAME { Node::Num(std::f64::consts::PI) } else { Node::Var(name) });
        };
        if !self.eat(&Tok::LParen) {
            return err(format!("`{name}` is a function: write {name}(…)"));
        }
        let mut args = vec![self.sum()?];
        while self.eat(&Tok::Comma) {
            args.push(self.sum()?);
        }
        if !self.eat(&Tok::RParen) {
            return err("a parenthesis is not closed");
        }
        if !f.arity_ok(args.len()) {
            return err(format!("`{name}` does not take {} argument(s)", args.len()));
        }
        Ok(Node::Call(f, args))
    }
}

impl Expr {
    /// Parses `text`.
    pub fn parse(text: &str) -> Result<Expr, ExprError> {
        let toks: Vec<Tok> = lex(text)?.into_iter().map(|l| l.tok).collect();
        if toks.is_empty() {
            return err("the expression is empty");
        }
        let mut p = Parser { toks, pos: 0, depth: 0 };
        let root = p.sum()?;
        if p.pos < p.toks.len() {
            return err("unexpected text after the expression: an operator is missing");
        }
        Ok(Expr { root })
    }

    /// The syntax tree.
    #[must_use]
    pub fn root(&self) -> &Node {
        &self.root
    }

    /// The names the expression uses, sorted.
    #[must_use]
    pub fn names(&self) -> BTreeSet<String> {
        let mut out = BTreeSet::new();
        collect_names(&self.root, &mut out);
        out
    }

    /// Evaluates with `lookup` supplying the value of each name.
    pub fn eval(&self, lookup: &dyn Fn(&str) -> Result<f64, ExprError>) -> Result<f64, ExprError> {
        eval_node(&self.root, lookup)
    }
}

fn collect_names(n: &Node, out: &mut BTreeSet<String>) {
    match n {
        Node::Num(_) => {}
        Node::Var(v) => {
            out.insert(v.clone());
        }
        Node::Neg(a) => collect_names(a, out),
        Node::Bin(_, a, b) => {
            collect_names(a, out);
            collect_names(b, out);
        }
        Node::Call(_, args) => args.iter().for_each(|a| collect_names(a, out)),
    }
}

fn finite(v: f64, what: &str) -> Result<f64, ExprError> {
    if v.is_finite() { Ok(v) } else { err(format!("{what} is not a finite number")) }
}

fn eval_node(n: &Node, lookup: &dyn Fn(&str) -> Result<f64, ExprError>) -> Result<f64, ExprError> {
    match n {
        Node::Num(v) => Ok(*v),
        Node::Var(name) => finite(lookup(name)?, &format!("`{name}`")),
        Node::Neg(a) => Ok(-eval_node(a, lookup)?),
        Node::Bin(op, a, b) => {
            let (x, y) = (eval_node(a, lookup)?, eval_node(b, lookup)?);
            binary(*op, x, y)
        }
        Node::Call(f, args) => {
            let vals = args.iter().map(|a| eval_node(a, lookup)).collect::<Result<Vec<_>, _>>()?;
            call(*f, &vals)
        }
    }
}

fn binary(op: Op, x: f64, y: f64) -> Result<f64, ExprError> {
    let v = match op {
        Op::Add => x + y,
        Op::Sub => x - y,
        Op::Mul => x * y,
        Op::Div => {
            if y == 0.0 {
                return err("division by zero");
            }
            x / y
        }
        Op::Pow => {
            if x == 0.0 && y < 0.0 {
                return err("zero to a negative power");
            }
            if x < 0.0 && y.fract() != 0.0 {
                return err("a negative number to a fractional power");
            }
            x.powf(y)
        }
    };
    finite(v, "the result")
}

/// Degrees reduced to `[0, 360)`.
fn turn(deg: f64) -> f64 {
    deg.rem_euclid(360.0)
}

fn sin_deg(x: f64) -> f64 {
    match turn(x) {
        0.0 | 180.0 => 0.0,
        90.0 => 1.0,
        270.0 => -1.0,
        r => r.to_radians().sin(),
    }
}

fn cos_deg(x: f64) -> f64 {
    match turn(x) {
        90.0 | 270.0 => 0.0,
        0.0 => 1.0,
        180.0 => -1.0,
        r => r.to_radians().cos(),
    }
}

fn call(f: Func, a: &[f64]) -> Result<f64, ExprError> {
    let x = a.first().copied().unwrap_or(0.0);
    let v = match f {
        Func::Sin => sin_deg(x),
        Func::Cos => cos_deg(x),
        Func::Tan => {
            let r = turn(x);
            if r == 90.0 || r == 270.0 {
                return err("tan is undefined at 90 degrees");
            }
            if r == 0.0 || r == 180.0 { 0.0 } else { r.to_radians().tan() }
        }
        Func::Asin | Func::Acos if !(-1.0..=1.0).contains(&x) => {
            return err(format!("{}({x}): the argument must be within -1 and 1", f.name()));
        }
        Func::Asin => x.asin().to_degrees(),
        Func::Acos => x.acos().to_degrees(),
        Func::Atan => x.atan().to_degrees(),
        Func::Atan2 => {
            let y2 = a.get(1).copied().unwrap_or(0.0);
            if x == 0.0 && y2 == 0.0 {
                return err("atan2(0, 0) is undefined");
            }
            x.atan2(y2).to_degrees()
        }
        Func::Sqrt => {
            if x < 0.0 {
                return err("the square root of a negative number");
            }
            x.sqrt()
        }
        Func::Abs => x.abs(),
        Func::Min => a.iter().copied().fold(f64::INFINITY, f64::min),
        Func::Max => a.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        Func::Floor => x.floor(),
        Func::Ceil => x.ceil(),
        Func::Round => x.round(),
    };
    finite(v, f.name())
}

/// Rewrites every use of the name `old` in `text` to `new` (whole names only: `w` inside `width`
/// stays), keeping the rest of the text as it was.
pub fn rename_in_text(text: &str, old: &str, new: &str) -> Result<String, ExprError> {
    let toks = lex(text)?;
    let mut out = String::with_capacity(text.len());
    let mut at = 0;
    for t in &toks {
        if matches!(&t.tok, Tok::Ident(n) if n == old) {
            out.push_str(text.get(at..t.start).unwrap_or(""));
            out.push_str(new);
            at = t.end;
        }
    }
    out.push_str(text.get(at..).unwrap_or(""));
    Ok(out)
}

fn prec(n: &Node) -> u8 {
    match n {
        Node::Bin(Op::Add | Op::Sub, ..) => 1,
        Node::Bin(Op::Mul | Op::Div, ..) => 2,
        Node::Neg(_) => 3,
        Node::Bin(Op::Pow, ..) => 4,
        Node::Num(_) | Node::Var(_) | Node::Call(..) => 5,
    }
}

fn number_text(v: f64) -> String {
    let a = v.abs();
    if a == 0.0 || (1e-5..1e15).contains(&a) { format!("{v}") } else { format!("{v:e}") }
}

fn write_child(f: &mut fmt::Formatter<'_>, n: &Node, parens: bool) -> fmt::Result {
    if parens {
        write!(f, "(")?;
        write_node(f, n)?;
        write!(f, ")")
    } else {
        write_node(f, n)
    }
}

fn write_node(f: &mut fmt::Formatter<'_>, n: &Node) -> fmt::Result {
    match n {
        Node::Num(v) => f.write_str(&number_text(*v)),
        Node::Var(v) => f.write_str(v),
        Node::Neg(a) => {
            write!(f, "-")?;
            write_child(f, a, prec(a) < 3)
        }
        Node::Bin(op, a, b) => {
            let (sym, p) = match op {
                Op::Add => (" + ", 1),
                Op::Sub => (" - ", 1),
                Op::Mul => (" * ", 2),
                Op::Div => (" / ", 2),
                Op::Pow => ("^", 4),
            };
            // `^` is right-associative and its base cannot be a signed or powered value unparenthesised;
            // the others are left-associative, so an equal-precedence right operand needs them
            let (left_parens, right_parens) =
                if *op == Op::Pow { (prec(a) <= 4, prec(b) < 3) } else { (prec(a) < p, prec(b) <= p) };
            write_child(f, a, left_parens)?;
            f.write_str(sym)?;
            write_child(f, b, right_parens)
        }
        Node::Call(func, args) => {
            write!(f, "{}(", func.name())?;
            for (k, a) in args.iter().enumerate() {
                if k > 0 {
                    f.write_str(", ")?;
                }
                write_node(f, a)?;
            }
            write!(f, ")")
        }
    }
}

impl fmt::Display for Expr {
    /// The expression in its canonical text: parsing it again gives the same tree.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write_node(f, &self.root)
    }
}

#[cfg(test)]
mod tests;
