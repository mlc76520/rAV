//! Oscilloscope trace of the 240 samples around the audible instant.

use crate::viz::{Frame, Viz};
use crate::display::Canvas;
use crate::font::Fonts;
use crate::snapshot::FrameSnapshot;
use crate::ui::Titles;

#[derive(Default)]
pub struct Waveform {
    titles: Titles,
}

impl Waveform {
    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, samples: &[f32], left: bool, snap: &FrameSnapshot) {
        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);
        const CY: i32 = 64;
        cv.line(0, CY, 239, CY, 10);
        for i in 0..239 {
            let y1 = (CY - (samples[i] * 47.0) as i32).clamp(12, 127);
            let y2 = (CY - (samples[i + 1] * 47.0) as i32).clamp(12, 127);
            cv.line(i as i32, y1, i as i32 + 1, y2, 15);
        }
    }
}

impl Viz for Waveform {
    fn name(&self) -> &'static str {
        "Waveform"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.draw(f.left, f.fonts, &snap.waveform_l, true, snap);
        self.draw(f.right, f.fonts, &snap.waveform_r, false, snap);
    }
}
