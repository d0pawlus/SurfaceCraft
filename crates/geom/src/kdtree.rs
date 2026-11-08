//! A static k-d tree over 3-D points: nearest, k-nearest and radius queries.
//!
//! Built once (O(n log n)), queried many times. Indices refer to the input slice. Non-finite
//! points are left out of the tree.
use crate::Point3;

const LEAF: usize = 16;

#[derive(Clone, Debug)]
enum Node {
    Leaf { start: usize, end: usize },
    Split { axis: usize, value: f64, left: usize, right: usize },
}

/// Static k-d tree.
#[derive(Clone, Debug, Default)]
pub struct KdTree {
    points: Vec<Point3>,
    order: Vec<u32>,
    nodes: Vec<Node>,
}

impl KdTree {
    /// Builds a tree over `points`.
    #[must_use]
    pub fn build(points: &[Point3]) -> Self {
        let order: Vec<u32> =
            (0..points.len()).filter(|&i| points.get(i).is_some_and(crate::finite)).map(|i| i as u32).collect();
        let mut t = KdTree { points: points.to_vec(), order, nodes: Vec::new() };
        if !t.order.is_empty() {
            let n = t.order.len();
            t.build_node(0, n, 0);
        }
        t
    }

    /// Number of indexed points.
    #[must_use]
    pub fn len(&self) -> usize {
        self.order.len()
    }

    /// Is the tree empty?
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    /// The indexed point `i` (input index).
    #[must_use]
    pub fn point(&self, i: usize) -> Option<&Point3> {
        self.points.get(i)
    }

    fn coord(&self, oi: usize, axis: usize) -> f64 {
        self.order.get(oi).and_then(|&i| self.points.get(i as usize)).map_or(0.0, |p| p[axis])
    }

    fn build_node(&mut self, start: usize, end: usize, depth: usize) -> usize {
        let id = self.nodes.len();
        if end - start <= LEAF || depth > 64 {
            self.nodes.push(Node::Leaf { start, end });
            return id;
        }
        // split on the axis of largest extent
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for oi in start..end {
            for a in 0..3 {
                let v = self.coord(oi, a);
                lo[a] = lo[a].min(v);
                hi[a] = hi[a].max(v);
            }
        }
        let axis = (0..3).max_by(|&a, &b| (hi[a] - lo[a]).total_cmp(&(hi[b] - lo[b]))).unwrap_or(0);
        let mid = start + (end - start) / 2;
        let pts = &self.points;
        if let Some(slice) = self.order.get_mut(start..end) {
            slice.select_nth_unstable_by(mid - start, |&x, &y| {
                let vx = pts.get(x as usize).map_or(0.0, |p| p[axis]);
                let vy = pts.get(y as usize).map_or(0.0, |p| p[axis]);
                vx.total_cmp(&vy)
            });
        }
        let value = self.coord(mid, axis);
        self.nodes.push(Node::Leaf { start, end });
        let left = self.build_node(start, mid, depth + 1);
        let right = self.build_node(mid, end, depth + 1);
        if let Some(n) = self.nodes.get_mut(id) {
            *n = Node::Split { axis, value, left, right };
        }
        id
    }

    /// Nearest point to `q`: `(input index, squared distance)`.
    #[must_use]
    pub fn nearest(&self, q: &Point3) -> Option<(usize, f64)> {
        self.nearest_within(q, f64::INFINITY)
    }

    /// Nearest point to `q` closer than `sqrt(max_d2)`.
    #[must_use]
    pub fn nearest_within(&self, q: &Point3, max_d2: f64) -> Option<(usize, f64)> {
        if self.nodes.is_empty() || !crate::finite(q) {
            return None;
        }
        let mut best: Option<(usize, f64)> = None;
        let mut best_d2 = max_d2;
        let mut stack: Vec<(usize, f64)> = vec![(0, 0.0)];
        while let Some((ni, bound)) = stack.pop() {
            if bound >= best_d2 {
                continue;
            }
            match self.nodes.get(ni) {
                Some(Node::Leaf { start, end }) => {
                    for &i in self.order.get(*start..*end).unwrap_or(&[]) {
                        if let Some(p) = self.points.get(i as usize) {
                            let d2 = (p - q).norm_squared();
                            if d2 < best_d2 {
                                best_d2 = d2;
                                best = Some((i as usize, d2));
                            }
                        }
                    }
                }
                Some(Node::Split { axis, value, left, right }) => {
                    let diff = q[*axis] - value;
                    let (near, far) = if diff < 0.0 { (*left, *right) } else { (*right, *left) };
                    stack.push((far, diff * diff));
                    stack.push((near, 0.0));
                }
                None => {}
            }
        }
        best
    }

    /// The `k` nearest points to `q`, sorted by distance: `(input index, squared distance)`.
    #[must_use]
    pub fn knn(&self, q: &Point3, k: usize) -> Vec<(usize, f64)> {
        let mut best: Vec<(usize, f64)> = Vec::with_capacity(k + 1);
        if k == 0 || self.nodes.is_empty() || !crate::finite(q) {
            return best;
        }
        let mut stack: Vec<(usize, f64)> = vec![(0, 0.0)];
        while let Some((ni, bound)) = stack.pop() {
            let worst = if best.len() == k { best.last().map_or(f64::INFINITY, |b| b.1) } else { f64::INFINITY };
            if bound >= worst {
                continue;
            }
            match self.nodes.get(ni) {
                Some(Node::Leaf { start, end }) => {
                    for &i in self.order.get(*start..*end).unwrap_or(&[]) {
                        if let Some(p) = self.points.get(i as usize) {
                            let d2 = (p - q).norm_squared();
                            let worst = if best.len() == k {
                                best.last().map_or(f64::INFINITY, |b| b.1)
                            } else {
                                f64::INFINITY
                            };
                            if d2 < worst {
                                let pos = best.partition_point(|b| b.1 <= d2);
                                best.insert(pos, (i as usize, d2));
                                best.truncate(k);
                            }
                        }
                    }
                }
                Some(Node::Split { axis, value, left, right }) => {
                    let diff = q[*axis] - value;
                    let (near, far) = if diff < 0.0 { (*left, *right) } else { (*right, *left) };
                    stack.push((far, diff * diff));
                    stack.push((near, 0.0));
                }
                None => {}
            }
        }
        best
    }

    /// All points within `radius` of `q` (unsorted input indices).
    #[must_use]
    pub fn within(&self, q: &Point3, radius: f64) -> Vec<usize> {
        let mut out = Vec::new();
        if self.nodes.is_empty() || !crate::finite(q) || !(radius >= 0.0) {
            return out;
        }
        let r2 = radius * radius;
        let mut stack = vec![0usize];
        while let Some(ni) = stack.pop() {
            match self.nodes.get(ni) {
                Some(Node::Leaf { start, end }) => {
                    for &i in self.order.get(*start..*end).unwrap_or(&[]) {
                        if self.points.get(i as usize).is_some_and(|p| (p - q).norm_squared() <= r2) {
                            out.push(i as usize);
                        }
                    }
                }
                Some(Node::Split { axis, value, left, right }) => {
                    let diff = q[*axis] - value;
                    if diff <= radius {
                        stack.push(*left);
                    }
                    if diff >= -radius {
                        stack.push(*right);
                    }
                }
                None => {}
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Rng;

    #[test]
    fn matches_brute_force() {
        let mut rng = Rng::new(3);
        let pts: Vec<Point3> =
            (0..2000).map(|_| Point3::new(rng.range(-5.0, 5.0), rng.range(-5.0, 5.0), rng.range(-5.0, 5.0))).collect();
        let tree = KdTree::build(&pts);
        for _ in 0..200 {
            let q = Point3::new(rng.range(-6.0, 6.0), rng.range(-6.0, 6.0), rng.range(-6.0, 6.0));
            let brute = pts
                .iter()
                .enumerate()
                .map(|(i, p)| (i, (p - q).norm_squared()))
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .unwrap();
            let got = tree.nearest(&q).unwrap();
            assert_eq!(got.0, brute.0);
            let k = tree.knn(&q, 8);
            assert_eq!(k.len(), 8);
            assert_eq!(k[0].0, brute.0);
            let mut all: Vec<f64> = pts.iter().map(|p| (p - q).norm_squared()).collect();
            all.sort_by(f64::total_cmp);
            assert!((k[7].1 - all[7]).abs() < 1e-12);
            let r = tree.within(&q, 1.0);
            let br = pts.iter().filter(|p| (*p - q).norm() <= 1.0).count();
            assert_eq!(r.len(), br);
        }
    }

    #[test]
    fn empty_and_nan_are_safe() {
        let t = KdTree::build(&[]);
        assert!(t.nearest(&Point3::origin()).is_none());
        let t = KdTree::build(&[Point3::new(f64::NAN, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
        assert_eq!(t.len(), 1);
        assert_eq!(t.nearest(&Point3::origin()).unwrap().0, 1);
        assert!(t.nearest(&Point3::new(f64::NAN, 0.0, 0.0)).is_none());
        assert!(t.within(&Point3::origin(), f64::NAN).is_empty());
    }
}
