//! Analog-style VU meter: a needle per channel over a dB / power scale.

use crate::viz::{Frame, Viz};
use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::snapshot::FrameSnapshot;

const DB_VALUES: [i32; 11] = [-20, -10, -7, -5, -3, -2, -1, 0, 1, 2, 3];
const DB_TEXTS: [&str; 11] = ["20", "10", "7", "5", "3", "2", "1", "0", "1", "2", "3"];
const POWER_SCALE: [&str; 6] = ["0", "20", "40", "60", "80", "100"];

pub struct VuMeter;

impl VuMeter {
    fn meter(cv: &mut Canvas, fonts: &Fonts, val: i32, label: &str) {
        cv.clear();
        // log10(10^(dB/20)) == dB/20, so the scale is linear in dB over [-20, +3]
        for (db, text) in DB_VALUES.iter().zip(DB_TEXTS) {
            let x = (db + 20) * 125 / 23;
            fonts.draw(cv, x, 5, text, Size::Small, 15);
            cv.line(x, 7, x, 9, 15);
        }
        cv.line(108, 8, 127, 8, 15);
        cv.line(0, 9, 127, 9, 15);
        cv.line(0, 11, 110, 11, 15);
        cv.line(0, 6, 0, 8, 15);
        cv.line(0, 11, 0, 13, 15);
        cv.line(127, 6, 127, 8, 15);
        for (i, text) in POWER_SCALE.iter().enumerate() {
            let x = i as i32 * 22;
            fonts.draw(cv, x, 22, text, Size::Small, 15);
            cv.line(x, 11, x, 13, 15);
        }
        fonts.draw(cv, 0, 28, "-", Size::Small, 15);
        fonts.draw(cv, 124, 28, "+", Size::Small, 15);
        fonts.draw(cv, 0, 64, label, Size::Small, 15);
        fonts.draw(cv, 120, 64, "dB", Size::Small, 15);

        let pos = val * 127 / 255;
        let start_x = 71 - (127 - pos) / 8;
        let end_y = 20 - pos * (127 - pos) / 200;
        cv.line(start_x, 63, pos, end_y, 15);
    }
}

impl Viz for VuMeter {
    fn name(&self) -> &'static str {
        "VU Meter"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        Self::meter(f.left, f.fonts, snap.vu_l, "LEFT");
        Self::meter(f.right, f.fonts, snap.vu_r, "RIGHT");
    }
}
