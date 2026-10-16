//! 7-band spectrum analyzers: filled bars with peak-hold, or outlines.

use super::Peaks;
use crate::viz::{Frame, Viz};
use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::snapshot::{BANDS_SMALL, FrameSnapshot};
use crate::ui::Titles;

const LABELS: [&str; BANDS_SMALL] = ["63", "160", "400", "1K", "2.5K", "6.3K", "16K"];
const FLOOR: i32 = 57;
const MAX_H: i32 = 49;

fn bar_x(i: usize) -> i32 {
    1 + i as i32 * 19
}

fn labels(cv: &mut Canvas, fonts: &Fonts) {
    for (i, l) in LABELS.iter().enumerate() {
        fonts.draw(cv, bar_x(i), 64, l, Size::Small, 15);
    }
}

pub struct Spectrum {
    titles: Titles,
    peaks: [Peaks<BANDS_SMALL>; 2],
}

impl Default for Spectrum {
    fn default() -> Self {
        Self { titles: Titles::small(), peaks: [Peaks::new((FLOOR - 1) as f32), Peaks::new((FLOOR - 1) as f32)] }
    }
}

impl Spectrum {
    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, left: bool, data: &[i32; BANDS_SMALL], snap: &FrameSnapshot) {
        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);
        let peaks = &mut self.peaks[!left as usize];
        for (i, v) in data.iter().enumerate() {
            let (x, h) = (bar_x(i), v * MAX_H / 255);
            let y = FLOOR - h;
            if h > 0 {
                cv.rect(x, y, 12, h, true, 15);
            }
            let p = peaks.update(i, y);
            cv.line(x, p, x + 11, p, 15);
        }
        labels(cv, fonts);
    }
}

impl Viz for Spectrum {
    fn name(&self) -> &'static str {
        "Spectrum"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.draw(f.left, f.fonts, true, &snap.spectrum_l, snap);
        self.draw(f.right, f.fonts, false, &snap.spectrum_r, snap);
    }
}

pub struct EmptySpectrum {
    titles: Titles,
}

impl Default for EmptySpectrum {
    fn default() -> Self {
        Self { titles: Titles::small() }
    }
}

impl EmptySpectrum {
    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, left: bool, data: &[i32; BANDS_SMALL], snap: &FrameSnapshot) {
        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);
        for (i, v) in data.iter().enumerate() {
            let (x, h, w) = (bar_x(i), v * MAX_H / 255, 12);
            let y = FLOOR - h;
            if h > 0 {
                cv.line(x, FLOOR, x, y, 15);
                cv.line(x + w, FLOOR, x + w, y, 15);
                cv.line(x, y, x + w, y, 15);
            }
        }
        labels(cv, fonts);
    }
}

impl Viz for EmptySpectrum {
    fn name(&self) -> &'static str {
        "Empty Spectrum"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.draw(f.left, f.fonts, true, &snap.spectrum_l, snap);
        self.draw(f.right, f.fonts, false, &snap.spectrum_r, snap);
    }
}
