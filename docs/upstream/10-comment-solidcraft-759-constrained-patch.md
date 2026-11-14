# Draft 10: comment for tangency-labs/solidcraft#759 (constrained variational patch: what we learned)

Status: DRAFT — not sent. Posting needs the owner's consent. Post it as one comment on #759
("Patch on edges: G1/G2, N-sided loops, minimum-energy fit, guide points"). It covers the kernel side
only and contains no application code.

**solidcore commit the numbers come from:** `4f2da85c26f7d608808d94cffcf06ebe867ef61d`. Check the
current head before posting.

## What was built on top of solidcore

- One bicubic B-spline patch, degree 3, clamped uniform knots on a rectangular parameter domain. The
  control points are the unknowns; the net is 4 to 24 points per direction (1728 scalar unknowns for
  24 × 24).
- Bending energy over the knot cells the outline touches: the integral of |S_uu|² + 2|S_uv|² +
  |S_vv|², with 4 × 4 Gauss points per cell. Cells away from the outline get a small regularisation
  only, so an area without data is filled by the energy.
- Weighted least-squares residuals of the data points.
- Conditions that are linear in the control points: a position row for G0 (and for interior points),
  a tangent row S_d · n = 0 for G1, and a curvature row S_dd · n = κ |S_d|² for G2, with κ from the
  previous round.
- Solve, dense: the hard rows are decomposed with an SVD for rank and a particular solution; the
  energy is minimised on the null space of the hard rows (Cholesky of the reduced matrix, SVD as the
  fallback).
- Basis functions and their derivatives come from solidcore (`basis_derivs`, `Surface::derivs`);
  reference faces are evaluated with solidcore.

## What we learned

1. **Hard collocation rows over-determine the system when sampled densely.** A side can be imposed
   exactly only at as many points as the free degrees of freedom along the boundary allow. We use one
   set of rows per knot interval a side crosses, plus the corners. Sampling the hard rows more
   densely makes them nearly dependent, and the solve then reports conflicts that are not in the
   geometry. Between the collocation points we measure the gap, the angle and the curvature error on
   a dense sample (at least 20 per knot interval), and the refinement is driven by those numbers.
2. **Energy scale.** The energy weight has to be normalised by the trace of the data normal matrix
   only. Soft rows (numerous, and heavy) must stay out of that trace: when they were in it, they
   stiffened the fit. We trace them separately, and a test pins the behaviour.
3. **G1 rows never conflict algebraically.** A G1 row has a zero right-hand side. Two different
   normals at a corner therefore give a degenerate patch without any error from the solver. We compare
   the tangent planes at every corner before solving (threshold 0.5 degrees) and then relax the
   tangent conditions near the corner, or refuse.
4. **G1 rows do not see the sign of the normal.** A patch folded back over its reference face
   satisfies the row as well as a smooth continuation does. After the solve we check that S_d points
   away from the interior of the reference face, and report a fold as an error of the side.
5. **Hard G0 on noisy sides.** When the sides come from a scan (noisy, or rounded at a crease), a
   hard G0 pulls the patch onto the noise: the fit was 2 to 4 times worse than the noise. Treating
   those sides as soft (the patch follows the data and reports its distance from the side) avoids it.

## Measured G1 / G2 numbers (synthetic data, pin above)

- Five-sided hole in a cylinder of R = 20 mm, boundary conditions only:
  - G1: net 6 × 6, gap ≤ 0.0006 mm, angle ≤ 0.047°, 91 ms.
  - G2: net 9 × 9, gap ≤ 0.0004 mm, angle ≤ 0.016°, normal-curvature error ≤ 2.0 %, 187 ms.
  - G0: gap ≤ 0.001 mm.
- Wave z = 3·sin(x/12)·cos(y/9) over a 40 × 24 mm region, 2150 points, noise σ = 0.03 mm, four hard
  sides: net 13 × 13, one round, rms 0.030 mm, max 0.100 mm, side gap 0.00018 mm.

## Joining the patch to its block

Separate patch sheets did not sew with `nurbs::stitch` at any tolerance we tried, and the decode of
such a record is refused. That is a separate report (draft 11). The patch's own surface worked in
`hybrid::replace_face` and in `hybrid::cut_by_spline`, which gave a closed solid.

## What we are not asking for

Nothing yet. Until solidcore has this primitive, the solver stays in the application, marked as
app-side geometry; it can be removed when solidcore provides the fit.
