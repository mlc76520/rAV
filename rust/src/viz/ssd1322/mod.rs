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
use crate::ui::dither;
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

/// Rounded bar centred on the panel: dithered gradient fill, quarter ticks
/// on the empty track, percentage in the middle drawn black over the fill
/// and white over the track, split pixel-exactly at the fill edge.
pub fn volume_bar(cv: &mut Canvas, fonts: &Fonts, vol: i32) {
    const W: i32 = 200;
    const H: i32 = 26;
    const X: i32 = (SCREEN_WIDTH as i32 - W) / 2;
    const Y: i32 = (SCREEN_HEIGHT as i32 - H) / 2;
    let vol = vol.clamp(0, 100);

    cv.rect(X - 4, Y - 4, W + 8, H + 8, true, 0); // backdrop
    // Frame with cut corners (reads as rounded at this size).
    cv.line(X + 2, Y, X + W - 3, Y, 15);
    cv.line(X + 2, Y + H - 1, X + W - 3, Y + H - 1, 15);
    cv.line(X, Y + 2, X, Y + H - 3, 15);
    cv.line(X + W - 1, Y + 2, X + W - 1, Y + H - 3, 15);
    for (px, py) in [(X + 1, Y + 1), (X + W - 2, Y + 1), (X + 1, Y + H - 2), (X + W - 2, Y + H - 2)] {
        cv.pixel(px, py, 15);
    }

    // Track, 1 px inside the frame.
    const TX: i32 = X + 2;
    const TY: i32 = Y + 2;
    const TW: i32 = W - 4;
    const TH: i32 = H - 4;
    let fill = vol * TW / 100;
    for i in 0..fill {
        let lv = 4.0 + 11.0 * i as f32 / (TW - 1) as f32;
        for r in 0..TH {
            cv.pixel(TX + i, TY + r, dither(lv, TX + i, TY + r));
        }
    }
    for q in 1..4 {
        let tx = TX + TW * q / 4;
        if tx >= TX + fill {
            cv.line(tx, TY, tx, TY + 2, 4);
            cv.line(tx, TY + TH - 3, tx, TY + TH - 1, 4);
        }
    }

    let text = format!("{vol}%");
    let text_x = X + (W - fonts.width(&text, Size::Regular)) / 2;
    let base = TY + (TH + fonts.cap_height(Size::Regular)) / 2;
    if fill > 0 {
        fonts.render(cv, &text, text_x, base, Size::Regular, true, 15, Some((TX, fill)));
    }
    if fill < TW {
        fonts.render(cv, &text, text_x, base, Size::Regular, false, 15, Some((TX + fill, TW - fill)));
    }
}
