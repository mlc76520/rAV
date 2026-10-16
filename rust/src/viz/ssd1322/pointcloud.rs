//! 3D point cloud: points scattered in a sphere, linked by random edges kept
//! only between 3D-near neighbours. The cloud rotates continuously (speed
//! follows overall energy); each point belongs to one of the 7 bands and
//! breathes outward with it; links light up with the mean level of their
//! endpoints (invisible in silence); a bass transient flashes and expands.
//! Panels share geometry and rotation but spin mirror-image, each on its
//! own channel.

use crate::viz::{Beat, Frame, Rng, Viz, norm7};
use crate::display::Canvas;
use crate::font::Fonts;
use crate::snapshot::FrameSnapshot;
use crate::ui::Titles;
use std::f32::consts::TAU;

const N_POINTS: usize = 56;
const MAX_EDGES: usize = 64;

struct Point3 {
    x: f32,
    y: f32,
    z: f32,
    r: f32, // unit direction + rest radius
    band: usize,
}

pub struct PointCloud {
    pts: Vec<Point3>,
    edges: Vec<(usize, usize)>,
    yaw: f32,
    pitch: f32,
    beat: [Beat; 2],
    titles: Titles,
}

impl PointCloud {
    pub fn new() -> Self {
        let mut rng = Rng::new(0x9E37_79B9);
        // Random directions (rejection-sampled in the unit ball), radii in
        // [0.45, 1.0] so the cloud has volume instead of being a shell.
        let pts: Vec<Point3> = (0..N_POINTS)
            .map(|i| loop {
                let (x, y, z) = (2.0 * rng.next() - 1.0, 2.0 * rng.next() - 1.0, 2.0 * rng.next() - 1.0);
                let d2 = x * x + y * y + z * z;
                if (0.04..=1.0).contains(&d2) {
                    let inv = 1.0 / d2.sqrt();
                    break Point3 { x: x * inv, y: y * inv, z: z * inv, r: 0.45 + 0.55 * rng.next(), band: i % 7 };
                }
            })
            .collect();
        // Random candidate pairs, kept when their rest positions are close,
        // so links read as local structure rather than clutter.
        let mut edges: Vec<(usize, usize)> = Vec::new();
        for _ in 0..4000 {
            if edges.len() >= MAX_EDGES {
                break;
            }
            let (a, b) = ((rng.next() * N_POINTS as f32) as usize, (rng.next() * N_POINTS as f32) as usize);
            if a == b {
                continue;
            }
            let (pa, pb) = (&pts[a], &pts[b]);
            let (dx, dy, dz) = (pa.x * pa.r - pb.x * pb.r, pa.y * pa.r - pb.y * pb.r, pa.z * pa.r - pb.z * pb.r);
            if dx * dx + dy * dy + dz * dz > 0.75 * 0.75 {
                continue;
            }
            if !edges.iter().any(|&(x, y)| (x, y) == (a, b) || (x, y) == (b, a)) {
                edges.push((a, b));
            }
        }
        Self { pts, edges, yaw: 0.0, pitch: 0.0, beat: [Beat::default(); 2], titles: Titles::default() }
    }

    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, spec: &[i32; 7], left: bool, snap: &FrameSnapshot) {
        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);

        let band = norm7(spec);
        let beat = &mut self.beat[!left as usize];
        beat.update(band[0].max(band[1]), snap.dt, 4.0);
        let flash = beat.level;

        let (cyw, syw, cp, sp) = (self.yaw.cos(), self.yaw.sin(), self.pitch.cos(), self.pitch.sin());
        let dir = if left { 1.0 } else { -1.0 }; // panels spin mirror-image

        let mut sx = [0i32; N_POINTS];
        let mut sy = [0i32; N_POINTS];
        let mut near = [0f32; N_POINTS];
        let mut amp = [0f32; N_POINTS];
        for (i, p) in self.pts.iter().enumerate() {
            let a = band[p.band];
            amp[i] = a;
            let rr = p.r * (0.72 + 0.45 * a + 0.18 * flash);
            let (x, y, z) = (p.x * rr, p.y * rr, p.z * rr);
            let x1 = x * cyw + z * syw; // rotate around Y...
            let z1 = -x * syw + z * cyw;
            let y1 = y * cp - z1 * sp; // ...then around X
            let z2 = y * sp + z1 * cp;
            let zcam = 3.0 - z2; // z2 = +1 -> nearest
            let s = 170.0 / zcam;
            sx[i] = (120.0 + dir * x1 * s * 1.4) as i32;
            sy[i] = (74.0 + y1 * s * 0.75) as i32;
            near[i] = ((4.0 - zcam) * 0.5).clamp(0.0, 1.0);
        }

        // Links first (under the points): brightness = mean level of the two
        // endpoint bands, so silence -> no links.
        for &(a, b) in &self.edges {
            let br = 0.5 * (amp[a] + amp[b]);
            let lvl = (1.0 + 9.0 * br + 3.0 * flash * br + 2.0 * br * (near[a] + near[b])) as i32;
            if lvl < 2 || (sx[a] < -40 && sx[b] < -40) || (sx[a] > 280 && sx[b] > 280) {
                continue;
            }
            cv.line(sx[a], sy[a], sx[b], sy[b], lvl.min(12) as u8);
        }

        // Points: nearer and louder -> brighter and bigger.
        for i in 0..N_POINTS {
            let lvl = ((3.0 + 8.0 * near[i] + 5.0 * amp[i] + 3.0 * flash) as i32).clamp(2, 15) as u8;
            let (x, y) = (sx[i], sy[i]);
            cv.pixel(x, y, lvl);
            if near[i] > 0.5 {
                cv.pixel(x + 1, y, lvl);
                cv.pixel(x, y + 1, lvl);
                cv.pixel(x + 1, y + 1, lvl);
            }
            if near[i] > 0.75 && amp[i] > 0.4 {
                // glow on near, loud points
                let halo = (lvl as i32 - 8).max(2) as u8;
                cv.pixel(x - 1, y, halo);
                cv.pixel(x + 2, y, halo);
                cv.pixel(x, y - 1, halo);
                cv.pixel(x, y + 2, halo);
            }
        }
    }
}

impl Viz for PointCloud {
    fn name(&self) -> &'static str {
        "Point Cloud"
    }

    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        // Rotation speed follows overall energy; never stops entirely.
        let e = (snap.vu_l + snap.vu_r) as f32 * (0.5 / 255.0);
        self.yaw += snap.dt * (0.25 + 1.5 * e);
        self.pitch += snap.dt * (0.17 + 0.8 * e);
        if self.yaw > TAU {
            self.yaw -= TAU;
        }
        if self.pitch > TAU {
            self.pitch -= TAU;
        }
        self.draw(f.left, f.fonts, &snap.spectrum_l, true, snap);
        self.draw(f.right, f.fonts, &snap.spectrum_r, false, snap);
    }
}
