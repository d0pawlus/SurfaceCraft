//! Hole patches that keep sharp edges.
//!
//! A hole that straddles a sharp edge (a spike removed on the edge of a part, missing data on a
//! corner) is first filled and faired like any other, which rounds the edge off into a chamfer.
//! [`sharpen_patch`] then looks at the surface the hole's border runs through: when it falls
//! into two or three smooth, connected sides that meet at sharp edges, each side is extended over
//! the hole (a plane, or a quadric when the side is curved) and the patch vertices are moved
//! onto the extended sides, the ones next to an edge onto the edge itself. A hole in a smooth
//! surface has one side and is left as the fairing made it, and so is any patch the move would
//! turn over or squash.

use crate::{Mesh, Point3, Vec3, tri};
use nalgebra::{SMatrix, SVector};
use std::collections::{HashMap, HashSet};

/// Two parts of the surface around a hole are different sides when their normals differ by more
/// than this (degrees), and an edge between them is sharp when it turns by more than this.
const FEATURE_DEG: f64 = 35.0;
/// Most sides a hole may touch (three meet at a corner); more is left to the fairing.
const MAX_SIDES: usize = 3;
/// Most groups of normals looked at before the surface is taken to be too rough to read.
const MAX_GROUPS: usize = 8;
/// Smallest share of the area next to the hole's border that makes a group of triangles a side.
const MIN_SIDE_SHARE: f64 = 0.1;
/// Rings of triangles around the hole's border a side is fitted to (the first decides the sides).
const SUPPORT_RINGS: usize = 2;
/// Fewest points a quadric (six terms) is fitted to; below it the side is a plane. With fewer,
/// a thin band of a flat side now and then bends by noise alone and the quadric, carried over
/// the hole, curls away from the side.
const MIN_QUADRIC_POINTS: usize = 20;
/// A quadric replaces the plane only when it leaves less than this share of the plane's RMS,
/// each RMS taken over the points left after the terms fitted (so three more terms that only
/// follow the noise gain nothing).
const QUADRIC_GAIN: f64 = 0.8;
/// Triangles of one normal further apart along it than this (in mean loop edge lengths) are on
/// two sides.
const SAME_SIDE_STEP: f64 = 0.5;
/// A side whose fit leaves more than this RMS (in mean loop edge lengths) is not one surface.
const MAX_SIDE_RMS: f64 = 0.25;
/// Sides closer to each other's middle than this (in mean loop edge lengths) make no wedge.
const MIN_WEDGE: f64 = 1e-6;
/// A patch vertex this close to a second side (in mean loop edge lengths) goes onto the edge.
const EDGE_SNAP: f64 = 0.5;
/// Projection rounds for one vertex.
const PROJECTIONS: usize = 12;
/// A point is on a side within this many mean loop edge lengths.
const ON_SIDE: f64 = 1e-12;
/// Three sides meet in a corner when a point is on all of them within this share of the hole's
/// size.
const CORNER_TOLERANCE: f64 = 1e-6;
/// Smallest area of a patch triangle after its vertices move, in squared mean loop edge lengths
/// (an equilateral triangle of that edge has 0.43).
const MIN_AREA: f64 = 0.02;
/// The moved patch is kept only when its thinnest triangle has at least this share of the area
/// of the thinnest one before (or [`MIN_AREA`]).
const MIN_AREA_KEPT: f64 = 0.5;
/// A patch triangle faces the way a side does when the cosine between their normals is above
/// this.
const MIN_FACING: f64 = 0.1;
/// Rounds of untangling a patch whose triangles the move turned over.
const UNTANGLE_ROUNDS: usize = 20;
/// A patch edge is flipped when the other diagonal is at most this share as far from the wedge.
const FLIP_GAIN: f64 = 0.5;
/// Passes of edge flips over a patch.
const FLIP_PASSES: usize = 8;

/// Terms of the height polynomial of a side: 1, x, y, x², xy, y².
const TERMS: usize = 6;
/// Terms of a plane.
const PLANE_TERMS: usize = 3;
mod hole;
mod near;
mod sharpen;
mod sides;
#[cfg(test)]
mod tests;
mod untangle;

#[cfg(test)]
use sharpen::LAST_REASON;
pub(crate) use sharpen::sharpen_patch;
pub(crate) use sides::Around;
#[cfg(test)]
use sides::bends;
