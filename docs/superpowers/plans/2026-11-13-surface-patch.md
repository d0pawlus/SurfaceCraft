# `surface.patch` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** one trimmed bicubic B-spline patch over a closed n-sided boundary, with G0/G1/G2 to
neighbouring faces, interior points/curves and scan data, as the command `surface.patch`, and as
an option of `surface.from_curves`.

**Architecture:** a dense variational solver in `crates/kernel/src/vpatch.rs` (bending energy,
hard linear rows by the null-space method, soft rows and data); a driver in
`crates/kernel/src/patch_fit.rs` (start parameters, rounds, refinement, measurement, trimming with
the existing `nurbs::trimmed_sheet`); mesh flattening in `crates/fit/src/param.rs`; the command in
`crates/engine/src/cmd/surface_patch.rs`.

**Tech Stack:** Rust, `nalgebra` (dense), solidcore through `crates/kernel` only.

**Spec:** `docs/superpowers/specs/2026-11-13-surface-patch-design.md` (read the "Aktualizacja"
block at its top first; where it disagrees with the body, the block wins).

## Global Constraints

- Read `AGENTS.md` and the skills it routes to before writing code. English in code, comments, commits.
- No `unwrap/expect/panic/todo/unimplemented/unreachable/unsafe` in non-test code; every input is hostile
  (NaN, infinities, empty or huge arrays, indices out of range) and must end in `Err`, never a panic.
- Only `crates/kernel` names solidcore. `crates/fit` is mesh computation only.
- `crates/kernel/src/vpatch.rs` is the **one** new app-geometry file: header line
  `//! APP-GEOMETRY: stands in for a solidcore constrained variational surface fit (n-sided boundary, G0/G1/G2 sides, interior points and curves).`
  and one new line in `xtask/app-geometry.txt` with the comment
  `# surface.patch (exception accepted by the owner 2026-11-12): constrained variational B-spline fit is not in solidcore; tangency-labs/solidcraft#759.`
- No new workspace dependency. `nalgebra.workspace = true` is added to `crates/kernel/Cargo.toml`.
- No `HashMap`/`HashSet` in numeric paths (determinism): `BTreeMap`, `Vec`.
- Implementation sources: the literature of spec §10 and solidcore's public API. Nothing from any
  closed product; do not open or cite anything outside this repository and the solidcore checkout.
- Thresholds of design doc §11.1 are never loosened (freeform max deviation ≤ 0.2 mm).
- **Subagents do not compile**: no `cargo build/check/clippy/test/xtask`; only `cargo fmt`. The
  orchestrator builds once per wave and sends compile errors back.
- Subagents do not commit, push, or touch files outside their task's **Files** list.
- Limits: control net 4..=24 per direction; sides ≤ 64; side points ≤ 10 000; data points ≤ 50 000
  (thinned deterministically by stride).

## Review Focus

1. Boundary side given as a 2-point segment or with repeated points → no zero-length division, a
   clean `Err` or a valid patch (Task 2 test `degenerate_sides`).
2. All data points in one corner of the outline (rest empty) → the patch stays smooth, report warns
   `no data` instead of oscillating (Task 1 test `empty_area_is_filled_by_energy`).
3. Outline whose plane projection self-intersects → `Err` naming the reason, not a folded patch
   (Task 2 test `refuses_self_crossing_projection`).
4. Hard rows more numerous than unknowns / duplicated rows → dropped rows counted, finite result
   (Task 1 test `redundant_rows_are_dropped`).
5. A body edge used as a side whose adjacent face is ambiguous for G1 → `Err` asking for `face`
   (Task 3 test `g1_side_needs_reference`).

---

## File structure

| File | Responsibility |
|---|---|
| `crates/kernel/src/vpatch.rs` (new) | basis rows (through solidcore), energy, assembly, null-space solve. Pure numbers in, `Surface` out |
| `crates/kernel/src/patch_fit.rs` (new) | driver: loop check, start parameters, rounds, knot refinement by re-solve on a larger net, measurement, trim; reference-face queries |
| `crates/fit/src/param.rs` (new) | flattening of a mesh region to the plane (harmonic + ARAP) |
| `crates/engine/src/cmd/surface_patch.rs` (new) | command `surface.patch`: schemas, side resolution, loop assembly, document object |
| `crates/engine/src/cmd/curves.rs`, `crates/fit/src/features.rs`, `features_ngon.rs` | option `ngon: "patch"` |

---

### Task 1: solver core `kernel::vpatch`  (Sonnet)

**Files:** Create `crates/kernel/src/vpatch.rs`; modify `crates/kernel/src/lib.rs` (`pub mod vpatch;`),
`crates/kernel/Cargo.toml` (`nalgebra.workspace = true`), `xtask/app-geometry.txt`.

**Interfaces — Produces:**

```rust
pub const MIN_CONTROL: usize = 4;
pub const MAX_CONTROL: usize = 24;
pub const DEGREE: usize = 3;

/// A scan/data sample with its parameters.
pub struct DataPoint { pub uv: [f64; 2], pub at: [f64; 3], pub weight: f64 }

/// One linear condition on the patch `S`.
pub enum Row {
    /// `S(uv) = at` (3 scalar rows).
    Position { uv: [f64; 2], at: [f64; 3] },
    /// `S_d(uv) · normal = 0`, `S_d = dir[0]·S_u + dir[1]·S_v` (1 scalar row).
    Tangent { uv: [f64; 2], dir: [f64; 2], normal: [f64; 3] },
    /// `S_dd(uv) · normal = value`, `S_dd = dir0²·S_uu + 2·dir0·dir1·S_uv + dir1²·S_vv` (1 scalar row).
    Curvature { uv: [f64; 2], dir: [f64; 2], normal: [f64; 3], value: f64 },
}

pub struct Problem {
    pub domain: [[f64; 2]; 2],        // [[u0, u1], [v0, v1]], u0 < u1, v0 < v1
    pub control: [usize; 2],          // control points per direction
    pub hard: Vec<Row>,
    pub soft: Vec<(Row, f64)>,        // weight > 0
    pub data: Vec<DataPoint>,
    /// Closed polygon in (u, v): bending energy is integrated over knot cells that touch or lie
    /// inside it plus one ring of cells; other cells get only `REGULARIZE`. Empty = whole domain.
    pub outline: Vec<[f64; 2]>,
    /// Relative stiffness (> 0): energy weight = smoothing · trace(data normal matrix) / trace(K);
    /// with no data and no soft rows the energy weight is 1.
    pub smoothing: f64,
}

pub struct Solution {
    pub surface: crate::record::brep::Surface, // Surface::Spline, clamped uniform knots on `domain`
    pub rank: usize,           // numerical rank of the hard rows
    pub dropped: usize,        // hard scalar rows found dependent
    pub hard_residual: f64,    // max |row·x − rhs| over hard rows after the solve (mm or mm-derivative units)
}

pub fn solve(p: &Problem) -> crate::Result<Solution>;
/// `S`, `S_u`, `S_v`, `S_uu`, `S_uv`, `S_vv` at `uv` (used by the driver for measurements).
pub fn derivatives(s: &crate::record::brep::Surface, uv: [f64; 2]) -> crate::Result<[[f64; 3]; 6]>;
```

Decisions the implementer cannot make alone:
- Unknowns: all control points, ordering `x[3·(i·m + j) + c]`, `i` along u. Knots: clamped, uniform
  interior knots on `domain`.
- Basis values and first/second derivatives come from solidcore (find the B-spline basis-derivative
  routine under `solidcore::nurbs`, e.g. `KnotVector::basis_eval` and its derivative sibling; do not
  write a Cox–de Boor of your own unless solidcore has no derivative routine — say so in the report).
- Energy `K`: `∫∫ |S_uu|² + 2|S_uv|² + |S_vv|²`, 4×4 Gauss points per active knot cell. Inactive
  cells: `REGULARIZE = 1e-8 ·` (mean diagonal of the active energy) on the diagonal of their
  control points only. Always add `1e-12 · mean diag` to the whole diagonal.
- Scaling: before assembly map positions to a unit box (centre and diagonal of all `at`); undo after.
- Solve: stack hard rows into `C x = d` (dense `nalgebra::DMatrix`), SVD of `C` (rank tolerance
  `1e-9 · σ_max`), particular solution by pseudo-inverse, null-space basis `Z`, then minimise
  `½xᵀHx − bᵀx` over `x = x_p + Z y` by Cholesky of `ZᵀHZ` (fall back to SVD solve if Cholesky
  fails). When no row couples coordinates (only `Position` rows and data), solve the scalar system
  once with three right-hand sides.
- Refuse (`KernelError::Argument`): non-finite input, `control` outside 4..=24, empty/inverted
  domain, `uv` outside the domain by more than 1e-9 of its span (clamp inside that), weights ≤ 0,
  zero `dir` or `normal`, more than 50 000 data points, more than 20 000 rows.

- [ ] **Step 1: tests** (in `vpatch.rs`, `#[cfg(test)]`), exact assertions:
  - `recovers_known_surface`: sample a known 8×8 bicubic `Surface::Spline` (control z = 3·sin(x/12)·cos(y/9) on a 60×40 grid of x, y) at 40×40 uv points as data, `control: [8, 8]`, hard `Position` rows on a 12-per-side boundary ring, `smoothing: 1e-9` → every control point within `1e-6 · 72.1` of the original.
  - `boundary_only_plane`: 5-gon boundary of 40 `Position` rows on the plane `z = 0.2x − 0.1y + 3`, no data → `max |z − plane| ≤ 1e-9` over a 30×30 grid of the domain.
  - `tangent_rows_hold`: boundary as above plus `Tangent` rows along one side with normal `(0, 0.6, 0.8)` → `|S_d·n| ≤ 1e-9` at those uv.
  - `curvature_rows_hold`: one `Curvature` row with `value: 0.02` → met to `1e-9`.
  - `redundant_rows_are_dropped`: every hard row given twice → `dropped` equals the number of duplicated scalar rows, same surface as without duplicates to `1e-9`.
  - `noise_stays_smooth`: 1600 data points of the wave above with deterministic pseudo-noise σ = 0.03 (LCG, seed 7), `control: [10, 10]`, `smoothing: 1e-3` → rms of residuals within 0.03 ± 20 %.
  - `empty_area_is_filled_by_energy`: data only in the quarter `u, v < 0.5` of the unit domain → all control points finite, `max |S|` over the domain ≤ 5 × the data's bounding-box diagonal.
  - `nonconvex_outline_decouples_arms`: U-shaped `outline`, data on arm A only vs data on both arms → the surface over arm A differs by ≤ 10 % of arm B's data amplitude.
  - `hostile_input_is_refused`: NaN uv, `control: [3, 9]`, `[25, 4]`, inverted domain, zero normal, zero weight → `Err`, no panic.
- [ ] **Step 2: implement** `solve` and `derivatives`.
- [ ] **Step 3:** `cargo fmt`; report files changed, the solidcore routines used, and anything you could not verify without compiling.

### Task 2: driver `kernel::patch_fit`  (Sonnet) — after Task 1 builds

**Files:** Create `crates/kernel/src/patch_fit.rs`; modify `crates/kernel/src/lib.rs` (`pub mod patch_fit;`).

**Interfaces — Consumes:** `vpatch::{Problem, Row, DataPoint, solve, derivatives}`,
`nurbs::{SurfaceProbe, trimmed_sheet, domain}`. **Produces:**

```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Continuity { G0, G1, G2 }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Conflicts { Relax, Fail }

/// One side of the loop, running from its first point to its last (which is the next side's first).
pub struct Side {
    pub points: Vec<[f64; 3]>,                 // ≥ 2, dense polyline of the side
    pub continuity: Continuity,
    /// The face the patch must continue (G1/G2): its surface, as stored in the body record.
    pub reference: Option<crate::record::brep::Face>,
    /// Parameters of `points` when the caller has a flattening (same length as `points`).
    pub uv: Option<Vec<[f64; 2]>>,
}

pub struct Request {
    pub sides: Vec<Side>,                      // ordered, closed (ends within 1e-9 after the caller's snap)
    pub data: Vec<[f64; 3]>,                   // soft
    pub data_uv: Option<Vec<[f64; 2]>>,        // with a flattening
    pub hard_points: Vec<[f64; 3]>,
    pub soft_points: Vec<([f64; 3], f64)>,
    pub curves: Vec<Vec<[f64; 3]>>,            // interior, hard G0
    pub tolerance: f64,                        // data deviation target, mm
    pub smoothing: Option<f64>,
    pub control: Option<[usize; 2]>,
    pub max_control: usize,                    // ≤ vpatch::MAX_CONTROL
    pub conflicts: Conflicts,
}

pub struct SideReport { pub continuity: Continuity, pub gap_max: f64, pub angle_max_deg: Option<f64>,
                        pub curvature_error_max: Option<f64>, pub samples: usize, pub folded: bool }
pub struct CornerReport { pub at: [f64; 3], pub mismatch_deg: f64, pub relaxed_length: f64 }
pub struct Report { pub control: [usize; 2], pub rounds: u32, pub rms: f64, pub p99: f64, pub max: f64,
                    pub points: usize, pub within_tolerance: bool, pub within_targets: bool,
                    pub sides: Vec<SideReport>, pub corners: Vec<CornerReport>, pub dropped: usize,
                    pub warnings: Vec<String> }
pub struct Patch { pub surface: crate::record::brep::Surface, pub sheet: crate::record::brep::Brep, pub report: Report }

pub const G0_TARGET: f64 = 1e-3;          // mm, measured densely
pub const G1_TARGET_DEG: f64 = 0.05;
pub const G2_TARGET_REL: f64 = 0.05;
pub const CORNER_MISMATCH_DEG: f64 = 0.5;

pub fn fit_patch(req: &Request) -> crate::Result<Patch>;
/// Unit normal of `face` at the point of its surface closest to `p`, and its normal curvature in
/// the tangent direction `t` there (through solidcore evaluation; `Err` for a face it cannot evaluate).
pub fn face_normal_curvature(face: &crate::record::brep::Face, p: [f64; 3], t: [f64; 3]) -> crate::Result<([f64; 3], f64)>;
```

Decisions (spec §4.2–4.5; follow them, these are the pins):
- Start parameters without `uv`: project sides, curves, points and data on the best-fit plane of
  the side points (PCA); `Err("the outline is not a height field: give a mesh region")` when the
  projected loop self-intersects or a G1/G2 reference normal is more than 75° off the plane normal.
  Orient the loop counter-clockwise; the patch normal `S_u × S_v` then agrees with the plane normal.
- Domain: the loop's uv bounding box grown by one control cell on each side.
- Start net: `control` if given, else `max(6, min(max_control, round(sqrt(n_data / 12))))` per
  direction, at least 6.
- Hard rows only at collocation points: one per knot interval crossed by a side, plus both ends;
  `Position` always; `Tangent` (dir = in-plane normal of the side's uv image, pointing into the
  patch) for G1/G2; `Curvature` for G2 with `value = κ · |S_d|²` from the previous round (first
  round: G1 only). Every dense side sample not used for collocation goes to `soft` with weight
  `1e4 ×` the mean data weight (`Position`; `Tangent`/`Curvature` likewise).
- Corner check before solving: at each corner compare the tangent planes of the two sides'
  references (and the tangent of a G0 side against a G1 neighbour's plane). Mismatch above
  `CORNER_MISMATCH_DEG`: `Conflicts::Fail` → `Err` listing corner position and angle;
  `Conflicts::Relax` → `Tangent`/`Curvature` rows of both sides within 15 % of each side's length
  from the corner move to `soft` with a weight falling linearly to 0 at the corner; recorded in
  `CornerReport`.
- Rounds: solve → re-project data and interior points with `SurfaceProbe::closest_uv` → measure.
  If a target is missed and the net is below `max_control`, grow the direction with the larger
  miss by `ceil(1.5×)` (capped) and repeat. Stop when all targets hold, when the worst measure
  improves by < 5 %, or after 6 rounds.
- Measurement, dense (≥ 20 samples per knot interval along each side): gap = distance of side
  samples to the patch (`closest`); angle = between reference normal and patch normal, signed by
  the fold test; fold = `S_d` at the side points into the reference face's interior (dot with the
  in-face direction away from the edge > 0) → `folded: true`, and `within_targets: false`.
- Trim: `nurbs::trimmed_sheet(surface, &outer_uv, &[])` with `outer_uv` = `closest_uv` of the dense
  side samples.
- Warnings (exact strings): `"no data: the interior is shaped by the boundary and the bending energy"`,
  `"the control net reached max_control"`, `"too few data points for the control net"` (fewer than
  3 data points per control point).

- [ ] **Step 1: tests** (`#[cfg(test)]` in `patch_fit.rs`):
  - `flat_pentagon`: 5 sides on `z = 0`, G0, no data → `max |z| ≤ 1e-9` on the sheet's tessellation vertices, one face, 5·k edges, `within_targets`.
  - `sphere_cap_boundary_only`: 5 sides on a sphere R = 50 (outline radius 15), G0, no data → gap ≤ `G0_TARGET`; `p99/median` of |mean curvature| over a 20×20 interior grid ≤ 5.
  - `wave_with_data`: 6 sides on `z = 3·sin(x/12)·cos(y/9)` (outline radius 15 about (30, 20)), 2000 data points with σ = 0.03 → `max ≤ 0.2`, `within_tolerance`.
  - `g1_to_cylinder`, `g2_to_cylinder`: 5-sided hole (radius 6 in the unrolled plane) in a cylinder R = 20; every side's `reference` is the cylinder face → `angle_max_deg ≤ 0.05`, G2: `curvature_error_max ≤ 0.05`, `folded == false`.
  - `incompatible_corner`: two G1 sides whose reference planes differ by 30° at the corner → `Relax`: `Ok`, one `CornerReport` with `mismatch_deg` in 29..31, `within_targets == false`; `Fail`: `Err` containing `"corner"`.
  - `refuses_self_crossing_projection`: a figure-eight loop → `Err` containing `"height field"`.
  - `degenerate_sides`: a side with two identical points, a side of 2 points, 65 sides → `Err` or valid patch, never a panic.
  - `hostile_request`: NaN points, empty sides, tolerance 0, `max_control` 99 → `Err`.
- [ ] **Step 2: implement.**  - [ ] **Step 3:** `cargo fmt`; report as in Task 1.

### Task 3: flattening `fit::param`  (Sonnet) — parallel with Task 1

**Files:** Create `crates/fit/src/param.rs`; modify `crates/fit/src/lib.rs` (`pub mod param;`).

**Produces:**

```rust
pub struct Flattening {
    /// One (u, v) per vertex of the mesh; `None` for vertices not used by `triangles`.
    pub uv: Vec<Option<[f64; 2]>>,
    /// The boundary loop (vertex ids, counter-clockwise in (u, v)).
    pub boundary: Vec<u32>,
    /// Triangles whose orientation is reversed in (u, v) (0 for a valid flattening).
    pub flipped: usize,
    /// Largest and mean edge-length distortion |l_uv / l_3d − 1| after uniform scaling.
    pub distortion_max: f64, pub distortion_mean: f64,
}
/// Flattens the triangles `triangles` of `mesh` (one disc: one boundary loop, no holes).
pub fn flatten(mesh: &surfacecraft_geom::Mesh, triangles: &[u32], arap_iterations: usize) -> Result<Flattening, String>;
```

Decisions: boundary mapped by chord length onto a convex polygon whose corners are the boundary
vertices with turning angle > 30° (a circle when there are fewer than 3); interior by the linear
system of mean-value weights (Floater); then `arap_iterations` local/global ARAP steps with the
boundary free except two pinned vertices; if ARAP produces flipped triangles keep the harmonic
result. Uniform scale so the uv area equals the 3-D area. `Err` for: no triangles, more than one
boundary loop, a non-manifold edge, a closed surface (no boundary), non-finite vertices, more than
400 000 triangles. Dense solves are not acceptable here: use a conjugate-gradient or Gauss–Seidel
solver written on the CSR/adjacency lists (no new dependency).

- [ ] **Step 1: tests:** `plane_is_isometric` (synth plane 40×30: `distortion_max ≤ 1e-6`, `flipped == 0`);
  `cylinder_patch_unrolls` (120° of a cylinder R = 20, h = 30: `distortion_mean ≤ 0.02`, `flipped == 0`);
  `sphere_cap` (cap of half-angle 40°: `flipped == 0`, `distortion_mean ≤ 0.08`);
  `l_shape_is_bijective` (concave planar L: `flipped == 0`, `distortion_max ≤ 1e-6`);
  `refuses_two_loops`, `refuses_closed_surface`, `refuses_non_finite`, `empty_triangles`.
- [ ] **Step 2: implement.**  - [ ] **Step 3:** `cargo fmt`; report.

### Task 4: command `surface.patch`  (Sonnet) — after Task 2 builds

**Files:** Create `crates/engine/src/cmd/surface_patch.rs`, `crates/engine/tests/surface_patch.rs`;
modify `crates/engine/src/cmd/mod.rs` (module, command list, heavy-command list if the file has one).
Read `skills/surfacecraft-commands/SKILL.md` and `crates/engine/src/cmd/surface_fit.rs` first and
follow their patterns (schemas with `deny_unknown_fields`, `ObjRef`, `SurfaceObject`, display mesh).

Input schema (field names are fixed):
`sides: [{ body?, edge?, points?, continuity?: "g0"|"g1"|"g2", face?: { body, face }, side_tolerance? }]`,
`points: [{ at, weight?, hard? }]`, `curves: [[[x,y,z],…]]`, `region?`, `mesh?`, `triangles?`,
`tolerance?` (0.2), `smoothing?`, `control?`, `max_control?` (16, ≤ 24), `gap_tolerance?` (0.01),
`conflicts?: "relax"|"fail"` (`relax`), `name?`.
Output: `surface` (id), `control`, `degree`, `rounds`, `rms`, `p99`, `max`, `points`,
`within_tolerance`, `within_targets`, `sides: [{continuity, gap_max_mm, angle_max_deg?, curvature_error_max?, samples, folded}]`,
`corners: [{at, mismatch_deg, relaxed_length_mm}]`, `dropped`, `warnings`.

Decisions: a body-edge side is sampled from the kernel's edge polyline of the body
(`tess::tessellate_topo(..).edges[edge]`, densified to ≥ 33 points); its reference face for G1/G2 is
`face` when given, else the single face using that edge, else `Err("side N: give `face` (the edge has K faces)")`.
A polyline side with more than 2 points is smoothed by a cubic B-spline through the kernel
(`side_tolerance`, default 0.001) and resampled. Sides are chained greedily by nearest free end;
gaps > `gap_tolerance` → `Err` with the gap position and size; smaller gaps are closed by moving
polyline ends onto the common corner. With `region`/`mesh`: data = the region's vertices, and
`fit::param::flatten` gives `data_uv`/side `uv` when the sides are the region's boundary (every side
point within `gap_tolerance` of a boundary vertex); otherwise the plane projection of the driver.
`SurfaceObject { parent: None, degree: 3, control, rms, max, .. }`.

- [ ] **Step 1: tests** (`crates/engine/tests/surface_patch.rs`): `patch_flat_pentagon_from_polylines`
  (5 polyline sides on z = 0 → surface object, `max` 0, `within_targets`);
  `patch_fills_hole_in_wavy_sheet_g1` (a `surface.fit` sheet, trimmed 5-gon hole built with
  `kernel::nurbs::trimmed_sheet` and added as a body; sides = the hole's edges, `continuity: g1` →
  `angle_max_deg ≤ 0.05` on every side);
  `g1_side_needs_reference` (polyline side with `g1` and no `face` → error naming the side);
  `open_loop_is_refused` (gap 1 mm → error containing the gap size);
  `patch_with_region_data` (region of a noisy synth wave bounded by 4 polylines → `max ≤ 0.2`);
  `hostile_params_never_panic` additions for the command (follow the existing hostile test file).
- [ ] **Step 2: implement.**  - [ ] **Step 3:** `cargo fmt`; report.

### Task 5: joining measurement  (Sonnet) — after Task 4 builds

**Files:** Create `crates/kernel/tests/patch_join.rs`. One `#[ignore]`d test `measure_patch_join`
that prints, for a box whose top face is replaced by (a) a flat and (b) a domed pentagon-outlined
`fit_patch` result inside a 4-patch Bézier frame: the gap of every shared side, and whether
stitching at tolerance = gap gives a closed shell, a volume, a STEP round trip, and a boolean cut by
a small box. No assertions beyond "does not panic". The orchestrator runs it and records the numbers
in the spec and `docs/STATUS.md`; the result decides whether Task 6 may report a solid.

### Task 6: `surface.from_curves` option `ngon`  (Sonnet) — after Tasks 3–5

**Files:** modify `crates/fit/src/features.rs`, `crates/fit/src/features_ngon.rs`,
`crates/engine/src/cmd/curves.rs`, `crates/engine/tests/organic.rs` (or the file holding the
`from_curves` tests). New input `ngon: "split" | "patch"` (default `"split"`, today's behaviour,
byte-identical results). With `"patch"`: four-cornered regions stay Bézier patches, every other
one-outline region goes to `kernel::patch_fit::fit_patch` (sides = its feature curves sampled, G0;
data = region vertices; uv from `fit::param::flatten`). Output gains `trimmed_patches` (count),
`side_gap_max_mm`, and `closed`/`solid` follow Task 5's finding (a solid is reported only if the
stitched shell is accepted; otherwise `closed: false` and a warning with the gap).
Tests: `from_curves_ngon_patch_fills_triangle_and_pentagon` (σ = 0.03; `skipped` empty, `max ≤ 0.2`,
`trimmed_patches == 2`), `from_curves_default_unchanged` (existing gates untouched), hostile params.

### Task 7: documentation  (Haiku) — last

`docs/STATUS.md` (new row with measured numbers, debt entry for `vpatch.rs`, corrected stale line
about four-cornered `from_curves`), `docs/guides/fit-surfaces.md` and `build-solid.md` (a section on
`surface.patch`), `docs/upstream/` draft comment for tangency-labs/solidcraft#759 (not sent).
