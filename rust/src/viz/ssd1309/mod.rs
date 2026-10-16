//! The SSD1309 family: 128x64 monochrome, the original rAV screens. Every
//! shape is drawn at level 15 (lit) or 0.

mod spectrum;
mod trackinfo;
mod vu;
mod waveform;

use super::Viz;
use super::editor::{Layout, PlaylistEditor};
use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::mpd::MpdClient;
use std::sync::Arc;

pub fn all(mpd: Arc<MpdClient>) -> Vec<Box<dyn Viz>> {
    vec![
        Box::new(vu::VuMeter),
        Box::new(spectrum::Spectrum::default()),
        Box::new(spectrum::EmptySpectrum::default()),
        Box::new(waveform::Waveform::default()),
        Box::new(trackinfo::TrackInfo::new()),
        Box::new(PlaylistEditor::new(mpd, &EDITOR)),
    ]
}

static EDITOR: Layout = Layout {
    size: Size::Small,
    visible_lines: 6,
    line_h: 9,
    list_y0: 18,
    header_y: 7,
    row_text_w: 120,
    selection: (7, 8, 126),
    scrollbar_top_dy: 7,
    scrollbar_w: 2,
    scrollbar_min: 4,
    loading_at: (25, 35),
    empty_at: (10, 35),
    feedback_box: (8, 22, 112, 20),
    feedback_inset: 1,
    feedback_baseline: 35,
    info_lines: (7, 16),
    info_rule_y: 18,
    spectrum_axis_y: 40,
    spectrum_gap: 0,
    spectrum_h: 20,
    help_y: 64,
};

/// Peak-hold markers that fall back toward `floor` by 0.5 px per frame.
/// They start resting on the floor instead of falling from the top row.
pub struct Peaks<const N: usize> {
    y: [f32; N],
    floor: f32,
}

impl<const N: usize> Peaks<N> {
    pub fn new(floor: f32) -> Self {
        Self { y: [floor; N], floor }
    }

    /// Feeds the bar top `top` of band i; returns the marker row.
    pub fn update(&mut self, i: usize, top: i32) -> i32 {
        let p = &mut self.y[i];
        *p = if (top as f32) < *p { top as f32 } else { (*p + 0.5).min(self.floor) };
        *p as i32
    }
}

/// Volume bar in a cleared box across the middle of the panel.
pub fn volume_overlay(cv: &mut Canvas, fonts: &Fonts, vol: i32) {
    let vol = vol.clamp(0, 100);
    cv.rect(15, 25, 98, 16, true, 0);
    cv.rect(14, 24, 100, 18, false, 15);
    cv.rect(18, 28, 88, 8, false, 15);
    let bar_w = vol * 88 / 100;
    if bar_w > 0 {
        cv.rect(18, 28, bar_w, 8, true, 15);
    }
    let text = format!("{vol}%");
    fonts.draw(cv, (cv.width() - fonts.width(&text, Size::Small)) / 2, 23, &text, Size::Small, 15);
}
