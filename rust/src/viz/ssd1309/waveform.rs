//! Oscilloscope: the last 128 samples of each channel.

use crate::viz::{Frame, Viz};
use crate::display::Canvas;
use crate::font::Fonts;
use crate::snapshot::FrameSnapshot;
use crate::ui::Titles;

const CENTER_Y: i32 = 37;
const GAIN: f32 = 25.0;

pub struct Waveform {
    titles: Titles,
}

impl Default for Waveform {
    fn default() -> Self {
        Self { titles: Titles::small() }
    }
}

impl Waveform {
    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, left: bool, samples: &[f32], snap: &FrameSnapshot) {
        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);
        cv.line(0, CENTER_Y, 127, CENTER_Y, 15);
        let y = |s: f32| (CENTER_Y - (s * GAIN) as i32).clamp(12, 63);
        for (i, pair) in samples.windows(2).enumerate() {
            cv.line(i as i32, y(pair[0]), i as i32 + 1, y(pair[1]), 15);
        }
    }
}

impl Viz for Waveform {
    fn name(&self) -> &'static str {
        "Waveform"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.draw(f.left, f.fonts, true, &snap.waveform_l, snap);
        self.draw(f.right, f.fonts, false, &snap.waveform_r, snap);
    }
}
