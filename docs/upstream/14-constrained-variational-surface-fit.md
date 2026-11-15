# Request: a constrained variational surface fit, one B-spline patch (draft)

Status: draft, not filed. Checked against solidcore head `f02d13e9ce38f45a8cd4b5a794fa09ae81d2221e` (2026-11-15). Searched the head for a fit that minimises a surface energy under linear conditions on position, tangent plane and curvature: found none as a public primitive. Nearest, none of them this (kernel paths): `crates/solidcore/src/nurbs/srf_match.rs` (moves the rows next to one boundary to meet another surface at G0/G1/G2 at stations; no energy, no data); `crates/solidcore/src/nurbs/srf_fit.rs` (`lst_sq_approx`: least squares on a regular grid, no conditions); `crates/solidcore/src/nurbs/n_sided.rs` and `crates/solidcore-ops/src/surface.rs` (`boundary`, `n_sided`: G0 certified, G1 across the inner curves "the fill's aim, not certified"); `crates/solidcore/src/nurbs/gordon.rs`; `crates/solidcore/src/blend/blend_setback.rs` + `blend_setback/qp.rs` + `blend_setback/curved.rs` (`net_fairness_rows`: a thin-plate energy under linear conditions, solved by a dense dual active-set QP, but `pub(super)`, for the setback corner patches only); `crates/solidcore/src/nurbs/crv_fit/p2.rs` (constrained least squares, curves only). Duplicates (`gh issue list --repo tangency-labs/solidcraft --state all`, read only): none asks for the primitive. Related, open: tangency-labs/solidcraft#759 (patch on edges, G1/G2, N-sided loops; carries draft 10 as a comment), #14374 (a multi-patch network fitted with G1 seams, whose "required capabilities" include a constrained fitting capability), #772 (surface modelling tracker). #763 (boundary surface, G0/G1/G2 per edge) is closed.
Pin: `solidcore` rev `4f2da85c26f7d608808d94cffcf06ebe867ef61d`.

Draft 10 (the comment on #759) says what was learned building this outside solidcore; this text asks for the primitive and does not repeat it. Motivation, one line: fitting one smooth patch to a boundary loop with continuity to the neighbouring faces needs a solver that no kernel entry provides.

## The primitive

One B-spline surface patch (non-rational, clamped, degree and knots given or built from control counts), whose control points minimise

    E = w2 · E2 + w3 · E3 + Σ weighted data residuals  subject to hard linear rows, with soft linear rows in the sum,

- `E2`, thin-plate bending energy: the integral of `|S_uu|² + 2|S_uv|² + |S_vv|²`, summed over the knot cells of a chosen region (cells outside it get only a small regularisation).
- `E3`, curvature variation: the integral of the squared third derivatives (`|S_uuu|² + 3|S_uuv|² + 3|S_uvv|² + |S_vvv|²`) over the same cells, so that one primitive covers a thin-plate fit and a fit that minimises the variation of curvature with a small `w2` kept for regularity.
- `w2` and `w3` are chosen by the caller (each `>= 0`, not both zero); the primitive normalises them against the data terms in one documented way.

Conditions are linear in the control points, each hard (met exactly, dependent rows reported and dropped) or soft (weight):

- position `S(u, v) = q`;
- tangent plane `S_d(u, v) · n = 0`, `S_d` a directional derivative in the parameter plane;
- normal curvature `S_dd(u, v) · n = c`.
- A general row would serve all three and the planned form of the curvature row: coefficients on `S`, `S_d` and `S_dd` at `(u, v)` (each a vector the result is dotted with) and a right-hand side. The G2 row that follows from linearising `S_dd · n = κ |S_d|²` by a Newton step is `S_dd · n − 2κ S_d⁰ · S_d = −κ |S_d⁰|²`, `S_d⁰` the first directional derivative of the previous iterate: a first- and a second-derivative term in one row, still linear in the control points.

Data: points with parameters and weights (the parameters are the caller's).

## What must come back

Not just the surface. The caller drives refinement from these, so they are part of the request:

- the rank of the hard rows, the number of rows dropped as dependent, and which;
- the residual of every hard row after the solve (largest, and per row), so a contradiction between conditions is visible as a number;
- the residual of the data (rms, largest) and the value of each energy term;
- the measured gap, angle and normal-curvature error of a side against a reference face are the kernel's `solidcore::query::continuity` measures (`continuity_along`); the primitive need not duplicate them, but should say which parameters the hard rows were imposed at, so that a caller can measure between them.

Refusals, naming why: more rows than unknowns can fit and hard rows that contradict beyond a tolerance, a non-finite number, a problem past the size the solver is documented for (decided from counts, never from the clock).

## Sizes (why dense algebra stops)

The application's stand-in is dense: the hard rows reduced by QR and decomposed by SVD, the energy minimised on their null space by a Cholesky factorisation. With G1/G2 rows the three coordinates are coupled, so a `n × m` net has `3 n m` unknowns, and several dense `c × c` matrices are alive at once. Measured by the application (aarch64, `ci` profile, one thread): a 16 × 16 net (768 unknowns) 0.8 to 1.3 s and 72 to 105 MB; 24 × 24 (1728 unknowns) 4 to 5 s with about 700 hard rows and 278 to 433 MB; 66 s with 27 000 hard rows. The cost is about `c³` for the products and the factorisation and about `h c²` for `h` hard rows; the application therefore limits the net to 24 per direction and refuses a larger problem before allocating. The matrices are not dense in nature: a cubic basis function overlaps 7 × 7 neighbours, so the energy matrix is banded, and the position, tangent and curvature rows touch 16 control points each. A primitive in the kernel can be sparse (or banded) and should not carry the 24 × 24 limit. What net sizes matter to a caller is open; the application has not measured any above 24 and does not claim a target. A question for the maintainers: which size should the primitive be designed for?

## Not asked

Rational patches, trimming, building the sheet, choosing the net or the parameters of the data, continuity between several fitted patches (that is a network problem, draft 15).

---
Notes for the owner (not part of the text to post):

1. What the application has today, from the code: `crates/kernel/src/vpatch.rs` and `vpatch/{assemble,solve,cells,cost}.rs`. `Problem { domain, control, hard, soft, data, outline, smoothing }`; `Row::{Position, Tangent, Curvature}`; `Solution { surface, rank, dropped, hard_residual }`; `MAX_CONTROL = 24`, `MIN_CONTROL = 4`, cost bounds `MAX_SOLVE_SECONDS = 8.0` and `MAX_SOLVE_BYTES` 512 MB native (256 MB wasm32). The figures in the sizes section are copied from the comments in `vpatch/cost.rs`.
2. Implemented today: only term E2 (bending energy, 4 × 4 Gauss points per cell). Term E3 and the weights `w2`, `w3` are PLANNED in the application, not implemented and not measured; the text says so only by asking for them. If you prefer, reword the post to say the application has E2 only.
3. The G2 row today is `S_dd · n = κ |S_d|²` with the right-hand side computed from the previous round's `S_d` (`crates/kernel/src/patch_fit/rows.rs`, `curvature_row`): a fixed-point linearisation whose left side holds only the second derivative. The Newton form `S_dd · n − 2κ S_d⁰ · S_d = −κ |S_d⁰|²` is PLANNED, not in the code.
4. The report fields (side gap, angle, curvature error, corners relaxed, `dropped`, `hard_residual`) are the application's `patch_fit::Report`; the text asks the kernel only for what a solver can know (ranks, residuals, energies).
5. Check before posting: that `blend_setback/qp.rs` is still `pub(super)` at the head you post against, and whether the maintainers want the primitive under `nurbs` (next to `srf_match`) or in `solidcore-ops`.
