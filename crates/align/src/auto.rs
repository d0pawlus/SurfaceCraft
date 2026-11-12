//! Automatic pose (`align.auto`): a coordinate system for a scan from the scan alone.
//!
//! The surface orientations vote: an area-weighted histogram of the triangle normals seeds the
//! dominant directions, each seed is refined to an exact planar face ([`plane_face`]), and open
//! flat rims count as the faces they stand for. The part is laid on its largest face that it can
//! stand on (nothing of the part beyond the face): that plane becomes z = 0 with the part above
//! it, the longer side of the footprint lies along x ([`crate::frame`]), and the origin is the
//! centre of the base face. A turned part (rotational symmetry) gets its axis on z with an end
//! face on z = 0 instead. Without any dominant flat face the principal frame is the fallback, at
//! low confidence.
//!
//! Ties (faces within 3 % of the largest area, equal signs) resolve deterministically: an open
//! rim beats a closed face, otherwise the pose needing the smaller rotation from the current one
//! wins; for a part symmetric under such a choice every answer is equally right.
use crate::faces::{FaceSet, PlaneFace, dominant_directions, material_side, plane_face};
use crate::frame::{Context, InPlane, least_rotation, level_all_faces, sample_vertices, side_frame};
use crate::{AlignError, principal_frame, rim, turned};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Mat3, Mesh, Transform, Vec3};

/// Proposals with a frame agreement (share of the area on the frame's axes) below this are
/// reported as not confident.
pub const CONFIDENCE_THRESHOLD: f64 = 0.5;
/// A base face has at least this share of the total area.
const MIN_FACE_SHARE: f64 = 0.02;
/// A face a part can stand on has at most this share of the area on its minority side.
const SUPPORT_LIMIT: f64 = 0.02;
/// Faces with at least this share of the largest area are tied.
const AREA_TIE: f64 = 0.97;
const SEED_COUNT: usize = 10;
/// Normals within 10 degrees of a frame axis agree with the frame (and take part in
/// `all_faces` levelling).
const NEAR_AXIS_COS: f64 = 0.984_807_753_012_208;
/// Sine of 10 degrees.
const NEAR_EQUATOR: f64 = 0.173_648_177_666_930_3;
/// A rim stands for an end face of a turned part within 15 degrees of its axis.
const END_FACE_COS: f64 = 0.965_925_826_289_068_3;
/// Two seeds that found the same face: normals within 5 degrees, planes closer than this many
/// slab half widths.
const DUPLICATE_COS: f64 = 0.996_194_698_091_745_5;

/// How the base is levelled.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum LevelMode {
    /// The base face lies exactly on z = 0.
    #[default]
    LargestFace,
    /// A least-squares compromise over all near-axis-aligned faces, weighted by area: the base may tilt a little.
    AllFaces,
}

/// What the pose is based on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum AutoMethod {
    /// The largest planar face the part can stand on.
    PlanarFace,
    /// An open flat rim standing for the missing face.
    OpenRim,
    /// The axis of a turned part, one end face on the table.
    Turned,
    /// No dominant flat face: the principal axes, at low confidence.
    PrincipalFallback,
}

/// The face the part lies on.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct BaseFace {
    /// Area of the face (an open rim: the area it encloses), mm².
    pub area: f64,
    /// Share of the surface area.
    pub share: f64,
    /// Normal pointing into the part, in the scan's frame.
    pub normal: [f64; 3],
    /// Centre of the face (the new origin's footprint), in the scan's frame.
    pub point: [f64; 3],
    /// Is it an open rim rather than triangles?
    pub rim: bool,
    /// Share of the surface on the wrong side of the face (0 for a part that stands on it).
    pub overhang: f64,
}

/// A proposed pose.
#[derive(Clone, Debug)]
pub struct AutoPose {
    /// The motion that puts the scan into the frame (model → aligned).
    pub transform: Transform,
    pub method: AutoMethod,
    pub mode: LevelMode,
    /// Share of the surface area within 10 degrees of the frame's axes (of its axis and
    /// equator for a turned part): 1 for a box, near 0 for a blob.
    pub confidence: f64,
    pub base: Option<BaseFace>,
    /// Axis of a turned part, in the scan's frame.
    pub axis: Option<[f64; 3]>,
    pub in_plane: InPlane,
}

impl AutoPose {
    /// Is the frame agreement at least [`CONFIDENCE_THRESHOLD`]?
    #[must_use]
    pub fn is_confident(&self) -> bool {
        self.confidence >= CONFIDENCE_THRESHOLD
    }
}

struct Cand {
    face: PlaneFace,
    up: Vec3,
    overhang: f64,
}

fn cand_of(set: &FaceSet, face: PlaneFace) -> Cand {
    let (overhang, up) = material_side(set, &face);
    Cand { face, up, overhang }
}

fn same_face(a: &PlaneFace, b: &PlaneFace) -> bool {
    a.normal.dot(&b.normal).abs() > DUPLICATE_COS
        && (b.point - a.point).dot(&a.normal).abs() < 2.0 * (a.half_width + b.half_width)
}

/// The planar faces of the part (large enough, distinct) and its open rims.
fn candidates(set: &FaceSet, rims: &[PlaneFace]) -> Vec<Cand> {
    let mut faces: Vec<PlaneFace> = Vec::new();
    for seed in dominant_directions(set, SEED_COUNT) {
        let Some(f) = plane_face(set, &seed) else { continue };
        if f.area >= MIN_FACE_SHARE * set.total && !faces.iter().any(|g| same_face(g, &f)) {
            faces.push(f);
        }
    }
    faces.extend(rims.iter().cloned());
    faces.into_iter().map(|f| cand_of(set, f)).collect()
}

/// The candidates the pose is chosen among: those a part can stand on (all when none can), the
/// ones within the area tie of the largest, narrowed to the largest rim when one is among them.
fn tied(cands: &[Cand]) -> Vec<&Cand> {
    let standing: Vec<&Cand> = cands.iter().filter(|c| c.overhang <= SUPPORT_LIMIT).collect();
    let pool: Vec<&Cand> = if standing.is_empty() { cands.iter().collect() } else { standing };
    let best = pool.iter().map(|c| c.face.area).fold(0.0, f64::max);
    let near: Vec<&Cand> = pool.into_iter().filter(|c| c.face.area >= AREA_TIE * best).collect();
    let largest_rim = near.iter().filter(|c| c.face.rim).max_by(|a, b| a.face.area.total_cmp(&b.face.area));
    match largest_rim {
        Some(r) => vec![*r],
        None => near,
    }
}

fn base_of(set: &FaceSet, c: &Cand) -> BaseFace {
    BaseFace {
        area: c.face.area,
        share: c.face.area / set.total,
        normal: c.up.into(),
        point: c.face.point.into(),
        rim: c.face.rim,
        overhang: c.overhang,
    }
}

/// Share of the area whose normal is within 10 degrees of an axis of the rotation `r`.
fn frame_confidence(set: &FaceSet, r: &Mat3) -> f64 {
    let agree: f64 = set.faces.iter().filter(|f| (r * f.n).iter().any(|c| c.abs() > NEAR_AXIS_COS)).map(|f| f.w).sum();
    agree / set.total
}

/// Share of the area on the poles or the equator of axis `z` (a turned part).
fn axis_confidence(set: &FaceSet, z: &Vec3) -> f64 {
    let agree: f64 = set
        .faces
        .iter()
        .filter(|f| {
            let c = f.n.dot(z).abs();
            c > NEAR_AXIS_COS || c < NEAR_EQUATOR
        })
        .map(|f| f.w)
        .sum();
    agree / set.total
}

fn method_of(base: &Cand) -> AutoMethod {
    if base.face.rim { AutoMethod::OpenRim } else { AutoMethod::PlanarFace }
}

/// The pose that lays the part on `base`.
fn prismatic_pose(ctx: &Context, base: &Cand, mode: LevelMode) -> AutoPose {
    let (r0, in_plane) = side_frame(ctx, &base.up, &base.face.point);
    let r = if mode == LevelMode::AllFaces { level_all_faces(ctx.set, &r0, NEAR_AXIS_COS) } else { r0 };
    AutoPose {
        transform: Transform::from_parts(&r, -(r * base.face.point.coords)),
        method: method_of(base),
        mode,
        confidence: frame_confidence(ctx.set, &r),
        base: Some(base_of(ctx.set, base)),
        axis: None,
        in_plane,
    }
}

fn least_motion(poses: Vec<AutoPose>) -> Option<AutoPose> {
    poses.into_iter().min_by(|a, b| a.transform.angle().total_cmp(&b.transform.angle()))
}

fn prismatic(ctx: &Context, rims: &[PlaneFace], mode: LevelMode) -> Option<AutoPose> {
    let cands = candidates(ctx.set, rims);
    least_motion(tied(&cands).into_iter().map(|c| prismatic_pose(ctx, c, mode)).collect())
}

/// The pose of a turned part: the end face's normal is the axis (exact where the end face is
/// large), z = axis, the least rotation puts it there. `None` without a flat end face (a smooth
/// body of revolution has no better frame than the principal one).
fn turned_pose(ctx: &Context, rims: &[PlaneFace], axis: &Vec3, mode: LevelMode) -> Option<AutoPose> {
    let mut ends: Vec<PlaneFace> = [*axis, -*axis]
        .iter()
        .filter_map(|s| plane_face(ctx.set, s))
        .filter(|f| f.area >= MIN_FACE_SHARE * ctx.set.total)
        .collect();
    ends.extend(rims.iter().filter(|r| r.normal.dot(axis).abs() > END_FACE_COS).cloned());
    let cands: Vec<Cand> = ends.into_iter().map(|f| cand_of(ctx.set, f)).collect();
    let poses: Vec<AutoPose> = tied(&cands)
        .into_iter()
        .map(|c| {
            let r = least_rotation(&c.up);
            AutoPose {
                transform: Transform::from_parts(&r, -(r * c.face.point.coords)),
                method: AutoMethod::Turned,
                mode,
                confidence: axis_confidence(ctx.set, &c.up),
                base: Some(base_of(ctx.set, c)),
                axis: Some(c.up.into()),
                in_plane: InPlane::Free,
            }
        })
        .collect();
    least_motion(poses)
}

fn fallback(mesh: &Mesh, set: &FaceSet, mode: LevelMode) -> Result<AutoPose, AlignError> {
    let t = principal_frame(mesh)?;
    Ok(AutoPose {
        confidence: frame_confidence(set, t.rotation.matrix()),
        transform: t,
        method: AutoMethod::PrincipalFallback,
        mode,
        base: None,
        axis: None,
        in_plane: InPlane::Free,
    })
}

/// Proposes a pose for `mesh` from its geometry alone.
///
/// # Errors
/// `NotEnough` when the mesh has no triangle with an area.
pub fn auto_pose(mesh: &Mesh, mode: LevelMode) -> Result<AutoPose, AlignError> {
    let set = FaceSet::new(mesh).ok_or_else(|| AlignError::NotEnough("the mesh has no area".into()))?;
    let rims = rim::rim_faces(mesh, &set);
    let sample = sample_vertices(&mesh.positions);
    let ctx = Context { set: &set, rims: &rims, sample: &sample };
    let turned = turned::detect(&set, &sample).and_then(|t| turned_pose(&ctx, &rims, &t.axis, mode));
    match turned.or_else(|| prismatic(&ctx, &rims, mode)) {
        Some(p) => Ok(p),
        None => fallback(mesh, &set, mode),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Point3, Rng, synth};

    fn bracket(noise: f64) -> Mesh {
        let mut m = synth::bracket();
        synth::add_noise(&mut m, noise, &mut Rng::new(3));
        m
    }

    #[test]
    fn a_bracket_lies_on_its_bottom_with_the_long_side_on_x() {
        let t = Transform::rotation_axis_angle(&Vec3::new(0.3, -0.8, 0.5), 0.7)
            .then_after(&Transform::translation(Vec3::new(50.0, -20.0, 9.0)));
        let moved = bracket(0.03).transformed(&t);
        let pose = auto_pose(&moved, LevelMode::LargestFace).unwrap();
        assert_eq!(pose.method, AutoMethod::PlanarFace);
        let net = pose.transform.then_after(&t);
        let down = net.apply_vec(&-Vec3::z());
        assert!(down.dot(&-Vec3::z()) > 0.999_99, "{down:?}");
        assert!(net.apply(&Point3::origin()).z.abs() < 0.03);
        assert!(net.apply_vec(&Vec3::x()).dot(&Vec3::x()).abs() > 0.999_99);
        assert!(pose.is_confident(), "{}", pose.confidence);
    }

    #[test]
    fn a_blob_falls_back_to_the_principal_frame() {
        let mut m = synth::sphere(Point3::origin(), 20.0, 40, 20);
        for p in &mut m.positions {
            *p = Point3::new(p.x * 1.6, p.y, p.z * 0.7);
        }
        let pose = auto_pose(&m, LevelMode::LargestFace).unwrap();
        assert_eq!(pose.method, AutoMethod::PrincipalFallback);
        assert!(!pose.is_confident(), "{}", pose.confidence);
    }

    #[test]
    fn an_empty_mesh_is_an_error() {
        assert!(auto_pose(&Mesh::default(), LevelMode::LargestFace).is_err());
    }
}
