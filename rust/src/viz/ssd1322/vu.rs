//! VU meters. A real VU dial is spaced by VOLTAGE, not by dB: position =
//! 10^(dB/20), normalised so +3 dB is full scale. 0 dB lands at ~71 % and
//! -3..+3 spread over the right half, so every mark gets room. The meter
//! value (0-255) maps linearly onto that position.

use crate::viz::{Frame, Viz};
use super::{SCREEN_HEIGHT, SCREEN_WIDTH};
use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::snapshot::FrameSnapshot;

fn pos(db: f32) -> f32 {
    10f32.powf(db / 20.0) / 10f32.powf(3.0 / 20.0)
}

const MARKS: [f32; 11] = [-20.0, -10.0, -7.0, -5.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0];

fn unsigned_label(db: f32) -> String {
    (db as i32).abs().to_string()
}

// ---- A. Classic: straight scale across the top, needle from below ----

pub struct VuMeter;

impl VuMeter {
    fn scale_x(p: f32) -> i32 {
        8 + (p * 218.0).round() as i32
    }

    fn draw(cv: &mut Canvas, fonts: &Fonts, val: i32, label: &str) {
        cv.clear();
        let (x0db, xmax) = (Self::scale_x(pos(0.0)), Self::scale_x(1.0));

        // dB scale: marks, labels, and the bold 0..+3 zone.
        for db in MARKS {
            let x = Self::scale_x(pos(db));
            fonts.draw_centered(cv, x, 11, &unsigned_label(db), Size::Regular, if db > 0.0 { 15 } else { 12 });
            cv.line(x, 13, x, 18, 15);
        }
        cv.line(Self::scale_x(pos(-20.0)), 18, xmax, 18, 15);
        cv.rect(x0db, 15, xmax - x0db + 1, 3, true, 15);

        // 0-100 % scale, which ends at 0 dB as on real meters.
        cv.line(Self::scale_x(0.0), 22, x0db, 22, 8);
        for p in (0..=100).step_by(20) {
            let x = Self::scale_x(pos(0.0) * p as f32 / 100.0);
            cv.line(x, 22, x, 25, 8);
            fonts.draw_centered(cv, x, 38, &p.to_string(), Size::Regular, 7);
        }

        fonts.draw(cv, 0, 52, "-", Size::Regular, 15);
        fonts.draw(cv, 230, 52, "+", Size::Regular, 15);
        fonts.draw(cv, 0, 127, label, Size::Regular, 10);
        fonts.draw(cv, 220, 127, "VU", Size::Regular, 10);

        // Needle: tip along the scale, base slightly offset (arc illusion).
        let tip = Self::scale_x(val as f32 / 255.0);
        let base = 120 - (232 - tip) / 16;
        let tip_y = 20 - tip * (239 - tip) / 400;
        cv.line(base, 127, tip, tip_y, 15);
        cv.line(base + 1, 127, tip, tip_y, 10);
        cv.line(base - 1, 127, tip, tip_y, 10);
    }
}

impl Viz for VuMeter {
    fn name(&self) -> &'static str {
        "VU Meter"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        Self::draw(f.left, f.fonts, snap.vu_l, "LEFT");
        Self::draw(f.right, f.fonts, snap.vu_r, "RIGHT");
    }
}

// ---- B. Vintage: arc dial, needle pivoting out of a housing ----
// The dial is static: drawn once and cached; each frame only restores it
// and draws the needle + housing.

pub struct VintageVu {
    dial: Option<Vec<u8>>,
}

impl VintageVu {
    const PX: f32 = 120.0;
    const PY: f32 = 182.0; // pivot (below the screen)
    const R: f32 = 148.0;
    const TH: f32 = 0.73; // half-sweep, radians

    pub fn new() -> Self {
        Self { dial: None }
    }

    fn angle(p: f32) -> f32 {
        -Self::TH + p * 2.0 * Self::TH
    }

    fn point(r: f32, a: f32) -> (i32, i32) {
        ((Self::PX + r * a.sin()).round() as i32, (Self::PY - r * a.cos()).round() as i32)
    }

    fn arc(cv: &mut Canvas, r: f32, p0: f32, p1: f32, level: u8) {
        let mut prev = Self::point(r, Self::angle(p0));
        for i in 1..=120 {
            let cur = Self::point(r, Self::angle(p0 + (p1 - p0) * i as f32 / 120.0));
            cv.line(prev.0, prev.1, cur.0, cur.1, level);
            prev = cur;
        }
    }

    fn draw_dial(cv: &mut Canvas, fonts: &Fonts) {
        cv.clear();
        Self::arc(cv, Self::R, 0.0, 1.0, 15);
        for k in 1..=3 {
            Self::arc(cv, Self::R + k as f32, pos(0.0), 1.0, 15); // bold 0..+3
        }
        let cap = fonts.cap_height(Size::Regular);
        for db in MARKS {
            let a = Self::angle(pos(db));
            let ((x0, y0), (x1, y1)) = (Self::point(Self::R, a), Self::point(Self::R + 7.0, a));
            cv.line(x0, y0, x1, y1, 15);
            let (lx, ly) = Self::point(Self::R + 16.0, a);
            fonts.draw_centered(cv, lx, ly + cap / 2, &unsigned_label(db), Size::Regular, if db > 0.0 { 15 } else { 12 });
        }
        Self::arc(cv, Self::R - 10.0, 0.0, pos(0.0), 6); // 0-100 % arc
        let scap = fonts.cap_height(Size::Small);
        for p in (0..=100).step_by(20) {
            let a = Self::angle(pos(0.0) * p as f32 / 100.0);
            let ((x0, y0), (x1, y1)) = (Self::point(Self::R - 10.0, a), Self::point(Self::R - 14.0, a));
            cv.line(x0, y0, x1, y1, 6);
            let (lx, ly) = Self::point(Self::R - 22.0, a);
            fonts.draw_centered(cv, lx, ly + scap / 2, &p.to_string(), Size::Small, 7);
        }
        fonts.draw_centered(cv, 120, 92, "VU", Size::Large, 5);
    }

    fn draw(cv: &mut Canvas, fonts: &Fonts, dial: &[u8], val: i32, label: &str) {
        cv.buf.copy_from_slice(dial);
        // Needle: 3 px wide (bright core, dimmer edges) from the pivot.
        let a = Self::angle(val as f32 / 255.0);
        let (nx, ny) = (a.sin(), -a.cos());
        for o in -1..=1 {
            let o = o as f32;
            let b = ((Self::PX + 40.0 * nx - o * ny).round() as i32, (Self::PY + 40.0 * ny + o * nx).round() as i32);
            let t = ((Self::PX + (Self::R + 4.0) * nx - o * ny).round() as i32,
                     (Self::PY + (Self::R + 4.0) * ny + o * nx).round() as i32);
            cv.line(b.0, b.1, t.0, t.1, if o != 0.0 { 9 } else { 15 });
        }
        // Housing over the needle's root: disc centred on the pivot, large
        // enough that even at full sweep the needle emerges from it.
        const HR: i32 = 74;
        let (px, py) = (Self::PX as i32, Self::PY as i32);
        for y in (py - HR)..SCREEN_HEIGHT as i32 {
            for x in (px - HR)..=(px + HR) {
                let r2 = (x - px).pow(2) + (y - py).pow(2);
                if r2 <= HR * HR {
                    cv.pixel(x, y, if r2 > (HR - 2) * (HR - 2) { 8 } else { 1 });
                }
            }
        }
        fonts.draw_centered(cv, 120, 124, label, Size::Small, 10);
    }
}

impl Viz for VintageVu {
    fn name(&self) -> &'static str {
        "VU Vintage"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        let dial = self.dial.get_or_insert_with(|| {
            let mut cv = Canvas::new(SCREEN_WIDTH, SCREEN_HEIGHT);
            Self::draw_dial(&mut cv, f.fonts);
            cv.buf
        });
        Self::draw(f.left, f.fonts, dial, snap.vu_l, "LEFT");
        Self::draw(f.right, f.fonts, dial, snap.vu_r, "RIGHT");
    }
}

// ---- C. Studio: segmented bargraph with a peak-hold segment ----

#[derive(Default, Clone, Copy)]
struct Peak {
    pos: f32,
    hold: f32,
}

#[derive(Default)]
pub struct StudioVu {
    peaks: [Peak; 2],
}

impl StudioVu {
    const X0: i32 = 6;
    const W: i32 = 218;
    const Y: i32 = 34;
    const H: i32 = 40;
    const SEG: i32 = 36;
    const PEAK_HOLD_S: f32 = 1.0;
    const PEAK_FALL_PER_S: f32 = 0.5; // of full scale

    fn draw(cv: &mut Canvas, fonts: &Fonts, val: i32, peak: &mut Peak, label: &str, dt: f32) {
        cv.clear();
        fonts.draw(cv, 2, 14, label, Size::Regular, 12);
        fonts.draw(cv, 208, 14, "VU", Size::Regular, 8);

        let (level, hot_from) = (val as f32 / 255.0, pos(0.0));
        if level >= peak.pos {
            *peak = Peak { pos: level, hold: Self::PEAK_HOLD_S };
        } else {
            peak.hold -= dt;
            if peak.hold <= 0.0 {
                peak.pos = level.max(peak.pos - Self::PEAK_FALL_PER_S * dt);
            }
        }
        let peak_seg = ((peak.pos * Self::SEG as f32) as i32).min(Self::SEG - 1);

        for s in 0..Self::SEG {
            let (x, w) = (Self::X0 + s * Self::W / Self::SEG, Self::W / Self::SEG - 2);
            let p = (s as f32 + 0.5) / Self::SEG as f32;
            let (on, hot) = (p <= level, p >= hot_from);
            let mut lvl = match (on, hot) {
                (true, true) => 15,
                (true, false) => (6.0 + 6.0 * p) as u8,
                (false, true) => 3,
                (false, false) => 2,
            };
            if s == peak_seg && peak.pos > 0.02 && !on {
                lvl = if hot { 15 } else { 11 }; // peak hold
            }
            cv.rect(x, Self::Y, w, Self::H, true, lvl);
            if lvl == 15 && hot {
                cv.rect(x, Self::Y, w, 2, true, 8); // "hot" cap
            }
        }

        // Scale under the bar, signed above 0 dB.
        let x0db = Self::X0 + (hot_from * Self::W as f32) as i32;
        cv.line(Self::X0, Self::Y + Self::H + 4, Self::X0 + Self::W - 1, Self::Y + Self::H + 4, 8);
        cv.rect(x0db, Self::Y + Self::H + 3, Self::X0 + Self::W - x0db, 3, true, 15);
        for db in MARKS {
            let x = Self::X0 + (pos(db) * Self::W as f32).round() as i32;
            let text = if db > 0.0 { format!("+{}", db as i32) } else { unsigned_label(db) };
            cv.line(x, Self::Y + Self::H + 4, x, Self::Y + Self::H + 8, 12);
            fonts.draw_centered(cv, x, Self::Y + Self::H + 22, &text, Size::Regular, if db >= 0.0 { 15 } else { 11 });
        }
        fonts.draw_centered(cv, 120, 112, "dB", Size::Small, 7);
    }
}

impl Viz for StudioVu {
    fn name(&self) -> &'static str {
        "VU Studio"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        let [pl, pr] = &mut self.peaks;
        Self::draw(f.left, f.fonts, snap.vu_l, pl, "LEFT", snap.dt);
        Self::draw(f.right, f.fonts, snap.vu_r, pr, "RIGHT", snap.dt);
    }
}
