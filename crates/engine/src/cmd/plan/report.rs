//! Reports of the plan: the serialisable output types and the report built from a plan.

use super::*;

// ---------------------------------------------------------------------------------------------
// reports

/// One region's role, for `body.plan_regions`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PlannedRegion {
    /// The primitive object.
    pub object: ObjectId,
    /// Surface kind.
    pub surface: String,
    /// `global`, `blend`, `local-subtract`, `local-add` or `skipped`.
    pub role: String,
    /// Why.
    pub reason: String,
    /// Tools (indices into `tools`) this region belongs to.
    pub tools: Vec<usize>,
    /// Local additions (indices into `adds`) this region belongs to.
    #[serde(default)]
    pub adds: Vec<usize>,
}

/// One local tool, for `body.plan_regions`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PlannedToolOut {
    /// Index.
    pub index: usize,
    /// `blind_hole`, `pocket` (walls round a floor, with the islands it leaves), `step` (walls with a
    /// floor that run out of the part), `rib` (a face cut down round raised islands) or `through_slot`.
    pub kind: String,
    /// The primitive objects that form it.
    pub members: Vec<ObjectId>,
    /// Numbers.
    pub detail: String,
}

/// One contact of neighbouring regions, for `body.plan_regions`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ContactOut {
    /// First primitive object.
    pub a: ObjectId,
    /// Second primitive object.
    pub b: ObjectId,
    /// `convex`, `concave` or `tangent`.
    pub kind: String,
    /// Angle between the normals at the contact, degrees.
    pub angle_deg: f64,
    /// Distance between the regions, mm.
    pub distance: f64,
}

/// One plane squared to a model axis (or left alone), for `body.plan_regions`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SquaredOut {
    /// The primitive object.
    pub object: ObjectId,
    /// Angle between the fitted and the squared normal, degrees.
    pub angle_deg: f64,
    /// Largest distance the squaring moves the plane over its region, mm.
    pub max_shift_mm: f64,
    /// The largest shift allowed, mm.
    pub limit_mm: f64,
    /// Whether the plane was squared (false: the shift exceeds the limit).
    pub applied: bool,
}

/// The report of a plan.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PlanOut {
    /// Planes squared to a model axis, or left as fitted (see `applied`).
    #[serde(default)]
    pub squared_planes: Vec<SquaredOut>,
    /// Role of every input region.
    pub regions: Vec<PlannedRegion>,
    /// Local tools (blind holes, pockets, through slots) the regions form.
    pub tools: Vec<PlannedToolOut>,
    /// Local additions (bosses, domes) the regions form.
    #[serde(default)]
    pub adds: Vec<PlannedToolOut>,
    /// Neighbour graph.
    pub contacts: Vec<ContactOut>,
}

/// The report of `plan` for `items`.
#[must_use]
pub fn report(items: &[Item], plan: &Plan) -> PlanOut {
    let id = |i: usize| items.get(i).map_or(ObjectId(0), |it| it.id);
    PlanOut {
        squared_planes: plan
            .squared
            .iter()
            .map(|q| SquaredOut {
                object: id(q.item),
                angle_deg: q.angle_deg,
                max_shift_mm: q.max_shift,
                limit_mm: q.limit,
                applied: q.applied,
            })
            .collect(),
        regions: items
            .iter()
            .zip(&plan.roles)
            .map(|(it, r)| PlannedRegion {
                object: it.id,
                surface: it.kind.to_string(),
                role: r.kind.name().to_string(),
                reason: r.how.clone(),
                tools: r.tools.clone(),
                adds: r.adds.clone(),
            })
            .collect(),
        adds: plan
            .adds
            .iter()
            .enumerate()
            .map(|(index, t)| PlannedToolOut {
                index,
                kind: t.kind.to_string(),
                members: t.members.iter().map(|&m| id(m)).collect(),
                detail: t.detail.clone(),
            })
            .collect(),
        tools: plan
            .tools
            .iter()
            .enumerate()
            .map(|(index, t)| PlannedToolOut {
                index,
                kind: t.kind.to_string(),
                members: t.members.iter().map(|&m| id(m)).collect(),
                detail: t.detail.clone(),
            })
            .collect(),
        contacts: plan
            .contacts
            .iter()
            .map(|c| ContactOut {
                a: id(c.a),
                b: id(c.b),
                kind: c.kind.to_string(),
                angle_deg: c.angle_deg,
                distance: c.distance,
            })
            .collect(),
    }
}
