//! Forms generated from a command's input JSON Schema.
//!
//! The form edits a `serde_json::Value` in place and reports whether it changed: components return
//! edit events, they never execute anything. Every colour is a theme token, every text a catalogue
//! key (parameter names and descriptions come from the schema, with optional catalogue overrides
//! `param.<command>.<name>` / `param.<name>`). Numbers use the locale's decimal separator and show
//! their unit and limits; optional parameters have a "set" check box; object parameters are
//! pickers over the document's objects; invalid values get a message from [`validate`].
use crate::fieldpick::PickRequest;
use crate::theme::Tokens;
use serde_json::Value;
use std::cell::RefCell;
use std::collections::HashMap;
use surfacecraft_doc::ObjectKind;

mod collections;
mod names;
mod rows;
mod schema;
mod validate;
mod widgets;

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests;

pub use names::{accepted_kinds, humanize, preferred_object, unit_of};
pub use rows::{described_default, form};
pub use schema::{Variant, classify, default_value, resolve, variants};
pub use validate::{Issue, validate};

/// An object the pickers offer.
#[derive(Clone, Debug, PartialEq)]
pub struct ObjInfo {
    /// Id.
    pub id: u64,
    /// Kind.
    pub kind: ObjectKind,
    /// Name.
    pub name: String,
}

/// What a form knows about the document and the viewport picks.
pub struct FormCtx<'a> {
    /// Command the form is for (catalogue overrides of parameter names).
    pub command: &'a str,
    /// Objects of the document.
    pub objects: &'a [ObjInfo],
    /// Object selected in the object list.
    pub selected: Option<u64>,
    /// Objects of the viewport picks, in pick order.
    pub pick_objects: &'a [u64],
    /// Points picked in the viewport.
    pub pick_points: &'a [[f64; 3]],
    /// Triangles picked in the viewport.
    pub pick_triangles: &'a [u32],
    /// The parameter the viewport is picking into right now, with the hint to show next to it.
    pub picking: Option<(&'a str, &'a str)>,
    /// Where a "pick in viewport" button leaves its request (the caller acts on it after the
    /// form: the form itself never starts anything).
    pub pick_request: &'a RefCell<Option<PickRequest>>,
    /// Theme tokens (error colour).
    pub tokens: &'a Tokens,
    /// Validation messages by path (see [`validate`]).
    pub issues: &'a HashMap<String, String>,
}

/// What a schema node is, for the widget to use.
#[derive(Clone, Debug, PartialEq)]
pub enum Kind {
    /// `true`/`false`.
    Bool,
    /// Whole number.
    Int,
    /// Real number.
    Num,
    /// Text.
    Str,
    /// One of these values.
    Enum(Vec<Value>),
    /// Number (id) or string (name) of an object.
    ObjRef,
    /// List.
    Array,
    /// Named fields.
    Object,
    /// Anything else: edited as JSON text.
    Any,
}
