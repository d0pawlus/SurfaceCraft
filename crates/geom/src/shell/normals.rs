//! Vertex normals for the offset: angle-weighted (Thürmer and Wüthrich, "Computing vertex normals
//! from polygonal facets", Journal of Graphics Tools 3(1), 1998, so the normal does not depend on how
//! a fan of triangles around a vertex is cut), then averaged over a ball of half the wall so that
//! scanner noise does not fold the offset.
use crate::{KdTree, Point3, Vec3, normalize, par};
use nalgebra::{Matrix3, Vector3};

/// Neighbours at most that average a normal (the nearest ones when a ball holds more: a fine scan).
const MAX_BALL: usize = 256;
/// Length of the sum of the unit directions of the two boundary edges at a rim vertex (2 for a straight
/// rim) from which the rim counts as straight: at a sharper corner the neighbours are a wedge, not
/// a half plane, and the one-sided average is kept.
const MIN_STRAIGHT_RIM: f64 = 1.7;
/// Neighbours at least that fit a plane to the normals at the rim.
const MIN_FIT_NEIGHBOURS: usize = 5;
/// Ridge on the slopes of the fit, as a share of the spread of the neighbours (keeps the fit stable
/// where they are almost collinear).
const RIDGE: f64 = 0.01;
/// The extrapolated normal may differ from the mean of the neighbours by at most this angle (rad).
const MAX_EXTRAPOLATION: f64 = 0.35;
/// Normals that make a larger angle than this cosine with the vertex's own are not averaged in (the
/// other side of a thin wall, the far side of a crease).
const MIN_AGREEMENT_COS: f64 = 0.5;

/// Angle-weighted normals of the vertices (zero for a vertex whose triangles have no area).
pub(super) fn angle_weighted(positions: &[Point3], triangles: &[[u32; 3]]) -> Vec<Vec3> {
    let mut n = vec![Vec3::zeros(); positions.len()];
    for t in triangles {
        let (Some(a), Some(b), Some(c)) =
            (positions.get(t[0] as usize), positions.get(t[1] as usize), positions.get(t[2] as usize))
        else {
            continue;
        };
        let Some(face) = normalize((b - a).cross(&(c - a))) else { continue };
        for (k, o, p, q) in [(0usize, a, b, c), (1, b, c, a), (2, c, a, b)] {
            let (u, v) = (p - o, q - o);
            let angle = u.cross(&v).norm().atan2(u.dot(&v));
            if let Some(slot) = n.get_mut(t[k] as usize) {
                *slot += face * angle;
            }
        }
    }
    n.into_iter().map(|v| normalize(v).unwrap_or_else(Vec3::zeros)).collect()
}

/// Unit direction of the rim at every vertex of a boundary loop (the sum of the directions of the two
/// boundary edges that meet there), zero elsewhere.
fn rim_tangents(positions: &[Point3], boundary: &[(u32, u32)]) -> Vec<Vec3> {
    let mut t = vec![Vec3::zeros(); positions.len()];
    for &(a, b) in boundary {
        let (Some(p), Some(q)) = (positions.get(a as usize), positions.get(b as usize)) else { continue };
        let Some(d) = normalize(q - p) else { continue };
        for v in [a, b] {
            if let Some(slot) = t.get_mut(v as usize) {
                *slot += d;
            }
        }
    }
    t
}

/// The normal field of the neighbours `ids` extrapolated to the rim vertex `i`: every component of
/// the normals is fitted by a plane over the position in the surface (weighted least squares, with a
/// small ridge on the slopes), and the value of the fit at the vertex is returned. A one-sided average
/// belongs to the middle of the neighbours, which lie on one side of the rim, and leans towards the
/// sheet by the slope of the field times the distance to that middle; the extrapolation does not.
/// `None` when the neighbours do not determine a fit, or when it moves the normal by more than
/// [`MAX_EXTRAPOLATION`] (a slope that large is noise, which the extrapolation would amplify).
fn extrapolated(raw: &[Vec3], positions: &[Point3], i: usize, ids: &[&usize], tangent: &Vec3) -> Option<Vec3> {
    let p = positions.get(i)?;
    let mean: Vec3 = ids.iter().filter_map(|&&j| raw.get(j)).sum();
    let n0 = normalize(mean)?;
    let u = normalize(*tangent - n0 * tangent.dot(&n0))?;
    let w = n0.cross(&u);
    let (mut m, mut rhs) = (Matrix3::zeros(), Matrix3::zeros());
    for &&j in ids {
        let (q, v) = (positions.get(j)?, raw.get(j)?);
        let d = q - p;
        let x = Vector3::new(1.0, d.dot(&u), d.dot(&w));
        m += x * x.transpose();
        rhs += x * v.transpose();
    }
    let count = ids.len() as f64;
    let spread = (m[(1, 1)] + m[(2, 2)]) / count;
    m[(1, 1)] += RIDGE * count * spread;
    m[(2, 2)] += RIDGE * count * spread;
    let fit = m.try_inverse()? * rhs;
    let at_rim = Vec3::new(fit[(0, 0)], fit[(0, 1)], fit[(0, 2)]);
    let moved = normalize(at_rim)?.dot(&n0).clamp(-1.0, 1.0).acos();
    (moved <= MAX_EXTRAPOLATION).then_some(at_rim)
}

/// The sum of the normals `raw` of `ids` that agree with `own`; for a rim vertex the field is
/// extrapolated to the rim instead ([`extrapolated`]).
fn average(raw: &[Vec3], positions: &[Point3], i: usize, ids: &[usize], tangent: &Vec3) -> Vec3 {
    let own = raw.get(i).copied().unwrap_or_else(Vec3::zeros);
    let agree: Vec<&usize> =
        ids.iter().filter(|&&j| raw.get(j).is_some_and(|v| v.dot(&own) > MIN_AGREEMENT_COS)).collect();
    let sum: Vec3 = agree.iter().filter_map(|&&j| raw.get(j)).sum();
    if tangent.norm() >= MIN_STRAIGHT_RIM && agree.len() >= MIN_FIT_NEIGHBOURS {
        return extrapolated(raw, positions, i, &agree, tangent).unwrap_or(sum);
    }
    sum
}

/// The normals averaged over the vertices within `radius` (a distance, mm) that face about the same
/// way, then renormalised; `None` when a vertex ends up without a normal. Noise finer than the
/// radius cannot fold an offset of that size, so the radius is half the wall (but at least one edge).
/// At the rim the ball covers one side only and the average would lean towards the sheet (tilting the
/// rim and the strip along it): there the normal field is extrapolated to the rim ([`extrapolated`]).
pub(super) fn smoothed(
    positions: &[Point3],
    triangles: &[[u32; 3]],
    boundary: &[(u32, u32)],
    radius: f64,
) -> Option<Vec<Vec3>> {
    let raw = angle_weighted(positions, triangles);
    let tree = KdTree::build(positions);
    let tangents = rim_tangents(positions, boundary);
    let n = par::map(positions.len(), |i| {
        let own = raw.get(i).copied().unwrap_or_else(Vec3::zeros);
        let Some(p) = positions.get(i) else { return own };
        let mut ids = tree.within(p, radius);
        if ids.len() > MAX_BALL {
            ids = tree.knn(p, MAX_BALL).into_iter().map(|(j, _)| j).collect();
        }
        ids.sort_unstable();
        let tangent = tangents.get(i).copied().unwrap_or_else(Vec3::zeros);
        normalize(average(&raw, positions, i, &ids, &tangent)).unwrap_or(own)
    });
    n.iter().all(|v| v.norm() > 0.5).then_some(n)
}

#[cfg(test)]
mod tests {
    use super::super::topology;
    use super::*;
    use crate::{Mesh, synth};

    fn rim_of(m: &Mesh) -> Vec<(u32, u32)> {
        topology::boundary_edges(&topology::directed_edges(&m.triangles))
    }

    /// Angle (rad) between the normal of rim vertex `v` and `expected`.
    fn error(n: &[Vec3], v: u32, expected: &Vec3) -> f64 {
        n.get(v as usize).map_or(f64::MAX, |n| n.dot(expected).clamp(-1.0, 1.0).acos())
    }

    #[test]
    fn rim_normals_of_a_flat_panel_are_exactly_the_panel_normal() {
        // Arrange: a tilted flat panel, the radius a few cells
        let panel =
            synth::plane(Point3::origin(), Vec3::new(1.0, 0.0, 0.3), Vec3::new(0.0, 1.0, 0.2), 10.0, 6.0, 25, 15);
        let expected = panel.vertex_normals()[0];
        let rim = rim_of(&panel);

        // Act
        let n = smoothed(&panel.positions, &panel.triangles, &rim, 1.0).expect("normals");

        // Assert
        assert!(!rim.is_empty());
        for &(a, _) in &rim {
            assert!(error(&n, a, &expected) < 1e-9, "vertex {a}");
        }
    }

    #[test]
    fn rim_normals_of_a_cylinder_patch_are_radial_where_the_one_sided_average_leans_inwards() {
        // Arrange: 60 degrees of a cylinder of radius 10 mm about the y axis, 0.5 mm cells, the average
        // taken over 1 mm
        let (radius, span, cells_u) = (10.0, 60.0f64.to_radians(), 63usize);
        let mut patch = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), span, 8.0, cells_u, 16);
        for p in &mut patch.positions {
            let angle = p.x - span / 2.0;
            *p = Point3::new(radius * angle.sin(), p.y, radius * angle.cos());
        }
        let rim = rim_of(&patch);
        let radial = |p: &Point3| Vec3::new(p.x, 0.0, p.z).normalize();
        let worst = |n: &[Vec3]| {
            rim.iter()
                .filter(|&&(a, _)| {
                    // the rim edges along the axis (the straight sides), not the corners
                    patch.positions.get(a as usize).is_some_and(|p| p.y > 0.6 && p.y < 7.4)
                })
                .map(|&(a, _)| error(n, a, &radial(&patch.positions[a as usize])))
                .fold(0.0, f64::max)
        };

        // Act
        let unbiased = smoothed(&patch.positions, &patch.triangles, &rim, 1.0).expect("normals");
        let one_sided = smoothed(&patch.positions, &patch.triangles, &[], 1.0).expect("normals");

        // Assert: the sides of the patch (a straight rim) are radial within half a degree
        assert!(worst(&unbiased) < 0.5f64.to_radians(), "{}", worst(&unbiased).to_degrees());
        assert!(worst(&one_sided) > 4.0 * worst(&unbiased), "{} {}", worst(&one_sided), worst(&unbiased));
    }

    #[test]
    fn rim_normals_around_a_small_hole_stay_close_to_the_surface_normal() {
        // Arrange: a spherical cap of radius 30 mm with a hole about 1.4 mm wide, cells of 0.8 mm
        let radius = 30.0;
        let mut cap = synth::plane(Point3::new(-18.0, -18.0, 0.0), Vec3::x(), Vec3::y(), 36.0, 36.0, 45, 45);
        for p in &mut cap.positions {
            p.z = (radius * radius - p.x * p.x - p.y * p.y).sqrt();
        }
        let hole = Point3::new(3.0, 4.0, 0.0);
        let kept: Vec<u32> = (0..cap.triangle_count() as u32)
            .filter(|&t| cap.face_centroid(t as usize).is_some_and(|c| (c.coords.xy() - hole.coords.xy()).norm() > 1.0))
            .collect();
        let cap = cap.submesh(&kept);
        let rim = rim_of(&cap);

        // Act
        let n = smoothed(&cap.positions, &cap.triangles, &rim, 1.0).expect("normals");

        // Assert: the rim of the hole within a degree of the sphere's normal, the outer rim (corners, 0.8 mm
        // facets on a steep part of the sphere) within two and a half
        for &(a, _) in &rim {
            let p = cap.positions[a as usize];
            let limit: f64 = if (p.coords.xy() - hole.coords.xy()).norm() < 3.0 { 1.0 } else { 2.5 };
            assert!(error(&n, a, &p.coords.normalize()) < limit.to_radians(), "vertex {a} at {p:?}");
        }
    }
}
