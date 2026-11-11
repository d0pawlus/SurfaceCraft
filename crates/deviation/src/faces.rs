//! Deviation of a scan per B-rep face (P8-E2 `body.compare`): each scan point is attributed to the
//! face of its nearest body triangle, then measured with the same rules as the whole-scan statistics
//! ([`stats_of`](crate::stats_of)).
use crate::Sample;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// Deviation of the scan points nearest to one B-rep face.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct FaceDeviation {
    /// Face index (as in the body record).
    pub face: usize,
    /// Counted scan points nearest to the face; the other values are zero when this is zero.
    pub count: usize,
    /// Largest |deviation| (mm).
    pub max_abs: f64,
    /// RMS deviation (mm).
    pub rms: f64,
    /// Mean signed deviation (mm); positive when the scan lies outside the face.
    pub mean: f64,
    /// Share of the counted points within the tolerance (0..1).
    pub within: f64,
}

/// One row per face `0..face_count`, from samples each tagged with the face of its nearest
/// reference triangle (`None` when the reference has no face map). Points without a face, and
/// faces past `face_count`, are left out.
#[must_use]
pub fn per_face(tagged: &[(Sample, Option<usize>)], face_count: usize, tolerance: f64) -> Vec<FaceDeviation> {
    let mut groups: Vec<Vec<Sample>> = vec![Vec::new(); face_count];
    for (sample, face) in tagged {
        if let Some(group) = face.and_then(|f| groups.get_mut(f)) {
            group.push(*sample);
        }
    }
    groups
        .iter()
        .enumerate()
        .map(|(face, group)| {
            let st = crate::stats_of(group, tolerance);
            FaceDeviation { face, count: st.count, max_abs: st.max_abs, rms: st.rms, mean: st.mean, within: st.within }
        })
        .collect()
}

/// Sorts rows worst first: faces with points before empty ones, then by RMS (largest first), then by
/// face index.
pub fn worst_first(rows: &mut [FaceDeviation]) {
    rows.sort_by(|a, b| (a.count == 0).cmp(&(b.count == 0)).then(b.rms.total_cmp(&a.rms)).then(a.face.cmp(&b.face)));
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(d: f64) -> Sample {
        Sample { d, facing: true, reference: [0.0; 3] }
    }

    #[test]
    fn points_are_grouped_by_their_nearest_face() {
        let tagged = vec![
            (at(0.2), Some(0)),
            (at(0.2), Some(0)),
            (at(-0.1), Some(1)),
            (Sample::UNMEASURED, Some(1)),
            (at(0.5), None),
        ];
        let rows = per_face(&tagged, 3, 0.1);
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0].count, 2, "{rows:?}");
        assert!((rows[0].mean - 0.2).abs() < 1e-12 && (rows[0].rms - 0.2).abs() < 1e-12, "{rows:?}");
        assert!(rows[0].within < 1e-12, "{rows:?}");
        assert_eq!(rows[1].count, 1, "{rows:?}");
        assert!((rows[1].max_abs - 0.1).abs() < 1e-12 && (rows[1].within - 1.0).abs() < 1e-12, "{rows:?}");
        assert_eq!((rows[2].count, rows[2].max_abs), (0, 0.0), "{rows:?}");
    }

    #[test]
    fn a_face_index_past_the_count_is_ignored() {
        let rows = per_face(&[(at(0.2), Some(9))], 3, 0.1);
        assert!(rows.iter().all(|r| r.count == 0), "{rows:?}");
    }

    #[test]
    fn worst_first_puts_empty_faces_last_then_orders_by_rms() {
        let row = |face, count, rms| FaceDeviation { face, count, max_abs: rms, rms, mean: 0.0, within: 0.0 };
        let mut rows = vec![row(0, 0, 0.0), row(1, 5, 0.1), row(2, 5, 0.3), row(3, 5, 0.1)];
        worst_first(&mut rows);
        let order: Vec<usize> = rows.iter().map(|r| r.face).collect();
        assert_eq!(order, vec![2, 1, 3, 0]);
    }
}
