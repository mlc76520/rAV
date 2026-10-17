//! The SSD1322 family: 240x128 in 16 grey levels.

mod pointcloud;
mod spectrum;
mod terrain;
mod trackinfo;
mod vu;
mod waveform;
mod xxl;

use super::Viz;
use super::editor::{Layout, PlaylistEditor};
use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::mpd::MpdClient;
use crate::ui::VolumeBar;
use std::sync::Arc;

pub const SCREEN_WIDTH: usize = 240;
pub const SCREEN_HEIGHT: usize = 128;

pub fn all(mpd: Arc<MpdClient>) -> Vec<Box<dyn Viz>> {
    vec![
        Box::new(vu::VuMeter),
        Box::new(vu::VintageVu::new()),
        Box::new(vu::StudioVu::default()),
        Box::new(spectrum::Spectrum::default()),
        Box::new(spectrum::SpectrumLed::new()),
        Box::new(spectrum::SpectrumHd::new()),
        Box::new(spectrum::EmptySpectrum::default()),
        Box::new(waveform::Waveform::default()),
        Box::new(pointcloud::PointCloud::new()),
        Box::new(terrain::Terrain::new()),
        Box::new(xxl::SpectrumXxl::new()),
        Box::new(xxl::Poitrine::default()),
        Box::new(trackinfo::TrackInfo::new()),
        Box::new(PlaylistEditor::new(mpd, &EDITOR)),
    ]
}

static EDITOR: Layout = Layout {
    size: Size::Regular,
    visible_lines: 8,
    line_h: 12,
    list_y0: 24,
    header_y: 10,
    row_text_w: 230,
    selection: (11, 12, 236),
    scrollbar_top_dy: 9,
    scrollbar_w: 3,
    scrollbar_min: 6,
    loading_at: (50, 64),
    empty_at: (20, 64),
    feedback_box: (30, 40, 180, 30),
    feedback_inset: 2,
    feedback_baseline: 60,
    info_lines: (10, 24),
    info_rule_y: 28,
    spectrum_axis_y: 70,
    spectrum_gap: 1,
    spectrum_h: 40,
    help_y: 127,
};

/// The 7 analysis bands, in sound-engineer notation so labels never touch.
pub const BAND7_LABELS: [&str; 7] = ["63", "160", "400", "1k", "2k5", "6k3", "16k"];

/// Sphere-lit ellipse: highlight at (hx, hy) in normalised coords, limb
/// darkening toward the rim, specular glint at the highlight.
pub fn shaded_ellipse(cv: &mut Canvas, cx: i32, cy: i32, rx: f32, ry: f32, hx: f32, hy: f32) {
    let r = rx.max(ry) as i32 + 1;
    let (ix, iy) = (1.0 / (rx * rx), 1.0 / (ry * ry));
    for dy in -r..=r {
        for dx in -r..=r {
            let q = (dx * dx) as f32 * ix + (dy * dy) as f32 * iy;
            if q > 1.0 {
                continue;
            }
            let (nx, ny) = (dx as f32 / rx - hx, dy as f32 / ry - hy);
            let h = ((nx * nx + ny * ny) * 0.55).min(1.0);
            let mut lvl = 2 + (11.0 * (1.0 - h) * (1.0 - 0.55 * q)) as i32;
            if h < 0.04 {
                lvl = 15; // specular glint
            }
            cv.pixel(cx + dx, cy + dy, lvl.clamp(1, 15) as u8);
        }
    }
}

/// Flat disc with a distinct rim ring.
pub fn disc(cv: &mut Canvas, cx: i32, cy: i32, r: f32, inner: u8, rim: u8) {
    let ri = r as i32 + 1;
    let inv = 1.0 / (r * r);
    for dy in -ri..=ri {
        for dx in -ri..=ri {
            let q = (dx * dx + dy * dy) as f32 * inv;
            if q <= 1.0 {
                cv.pixel(cx + dx, cy + dy, if q > 0.55 { rim } else { inner });
            }
        }
    }
}

/// Volume bar with a dithered gradient fill.
pub fn volume_bar(cv: &mut Canvas, fonts: &Fonts, vol: i32) {
    const BAR: VolumeBar = VolumeBar { w: 200, h: 26, size: Size::Regular, gradient: true, tick_level: 4 };
    BAR.draw(cv, fonts, vol);
}
