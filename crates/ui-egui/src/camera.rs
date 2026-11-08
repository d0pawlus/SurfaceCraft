//! The one orbit camera of the viewport.
use surfacecraft_geom::{Aabb, Mat4, Point3, Vec3};

/// Orbit camera around a target point (Z up).
#[derive(Clone, Debug, PartialEq)]
pub struct Camera {
    /// Point orbited.
    pub target: Point3,
    /// Distance from target to eye.
    pub distance: f64,
    /// Azimuth (radians, around Z).
    pub yaw: f64,
    /// Elevation (radians).
    pub pitch: f64,
    /// Vertical field of view (radians).
    pub fov: f64,
}

impl Default for Camera {
    fn default() -> Self {
        Self { target: Point3::origin(), distance: 150.0, yaw: -0.8, pitch: 0.5, fov: 0.7 }
    }
}

impl Camera {
    /// Eye position.
    #[must_use]
    pub fn eye(&self) -> Point3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        self.target + Vec3::new(cp * cy, cp * sy, sp) * self.distance
    }

    /// Unit view direction (eye → target).
    #[must_use]
    pub fn forward(&self) -> Vec3 {
        surfacecraft_geom::normalize(self.target - self.eye()).unwrap_or_else(|| -Vec3::z())
    }

    /// View matrix (world → camera), relative to `origin` (shifted for f32 precision).
    #[must_use]
    pub fn view(&self, origin: &Point3) -> Mat4 {
        let eye = self.eye() - origin.coords;
        let target = self.target - origin.coords;
        Mat4::look_at_rh(&eye, &target, &Vec3::z())
    }

    /// Projection for `aspect` = width / height.
    #[must_use]
    pub fn projection(&self, aspect: f64) -> Mat4 {
        let near = (self.distance * 0.002).max(1e-4);
        let far = self.distance * 50.0 + 10.0;
        // wgpu clip space z in [0, 1]
        let f = 1.0 / (self.fov / 2.0).tan();
        let a = if aspect.is_finite() && aspect > 0.0 { aspect } else { 1.0 };
        Mat4::new(
            f / a,
            0.0,
            0.0,
            0.0,
            0.0,
            f,
            0.0,
            0.0,
            0.0,
            0.0,
            far / (near - far),
            far * near / (near - far),
            0.0,
            0.0,
            -1.0,
            0.0,
        )
    }

    /// Frames `bbox` (no change when it is empty).
    pub fn fit(&mut self, bbox: &Aabb) {
        if bbox.is_empty() {
            return;
        }
        self.target = bbox.center();
        let r = (bbox.diagonal() * 0.5).max(1e-3);
        self.distance = r / (self.fov / 2.0).sin() * 1.1;
    }

    /// Orbits by a screen drag (pixels).
    pub fn orbit(&mut self, dx: f64, dy: f64) {
        self.yaw -= dx * 0.008;
        self.pitch = (self.pitch + dy * 0.008).clamp(-1.55, 1.55);
    }

    /// Pans by a screen drag (pixels) for a viewport `height` px tall.
    pub fn pan(&mut self, dx: f64, dy: f64, height: f64) {
        let f = self.forward();
        let right = surfacecraft_geom::normalize(f.cross(&Vec3::z())).unwrap_or_else(Vec3::x);
        let up = right.cross(&f);
        let scale = 2.0 * self.distance * (self.fov / 2.0).tan() / height.max(1.0);
        self.target += (-right * dx + up * dy) * scale;
    }

    /// Zooms by scroll `amount` (positive = closer).
    pub fn zoom(&mut self, amount: f64) {
        self.distance = (self.distance * (-amount * 0.002).exp()).clamp(1e-3, 1e7);
    }

    /// The world-space ray through viewport pixel `(x, y)` of a `w × h` viewport.
    #[must_use]
    pub fn ray(&self, x: f64, y: f64, w: f64, h: f64) -> (Point3, Vec3) {
        let ndc_x = 2.0 * x / w.max(1.0) - 1.0;
        let ndc_y = 1.0 - 2.0 * y / h.max(1.0);
        let t = (self.fov / 2.0).tan();
        let f = self.forward();
        let right = surfacecraft_geom::normalize(f.cross(&Vec3::z())).unwrap_or_else(Vec3::x);
        let up = right.cross(&f);
        let dir = f + right * (ndc_x * t * w / h.max(1.0)) + up * (ndc_y * t);
        (self.eye(), surfacecraft_geom::normalize(dir).unwrap_or(f))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centre_ray_hits_target_and_projection_maps_target_to_centre() {
        let mut c = Camera::default();
        c.fit(&Aabb::from_points(&[Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 20.0, 30.0)]));
        let (o, d) = c.ray(50.0, 40.0, 100.0, 80.0);
        let to_target = (c.target - o).normalize();
        assert!((d - to_target).norm() < 1e-9);
        let origin = Point3::new(1.0, 1.0, 1.0);
        let m = c.projection(1.25) * c.view(&origin);
        let p = m * (c.target - origin.coords).to_homogeneous();
        assert!((p.x / p.w).abs() < 1e-9 && (p.y / p.w).abs() < 1e-9);
        assert!((0.0..1.0).contains(&(p.z / p.w)));
    }
}
