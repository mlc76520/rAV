//! Spectrum bars. All share the band area: title on top, bars between TOP
//! and BOT, labels underneath. Peaks step once per frame: hold, then fall.

use crate::viz::{Frame, Viz};
use super::SCREEN_WIDTH;
use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::snapshot::FrameSnapshot;
use super::BAND7_LABELS;
use crate::ui::{Titles, dither};

/// Peak marker: `h` in the viz's own unit (pixels or segments).
#[derive(Default, Clone, Copy)]
struct Peak {
    h: i32,
    hold: i32,
    fall: i32,
}

impl Peak {
    /// Hold `hold_frames`, then drop one unit every `fall_frames` frames.
    fn update(&mut self, h: i32, hold_frames: i32, fall_frames: i32) {
        if h >= self.h {
            *self = Peak { h, hold: hold_frames, fall: 0 };
        } else if self.hold > 0 {
            self.hold -= 1;
        } else {
            self.fall += 1;
            if self.fall >= fall_frames {
                self.fall = 0;
                self.h -= 1;
            }
        }
    }
}

// ---- Smooth gradient bars (dithered), bright cap, floating peak ----

#[derive(Default)]
pub struct Spectrum {
    peaks: [[Peak; 7]; 2],
    titles: Titles,
}

impl Spectrum {
    const TOP: i32 = 26;
    const BOT: i32 = 112;
    const HMAX: i32 = Self::BOT - Self::TOP;
    const W: i32 = 24;
    const PITCH: i32 = 34;

    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, data: &[i32; 7], left: bool, snap: &FrameSnapshot) {
        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);
        for (i, peak) in self.peaks[!left as usize].iter_mut().enumerate() {
            let x = 4 + i as i32 * Self::PITCH;
            let h = data[i].clamp(0, 255) * Self::HMAX / 255;
            let y_top = Self::BOT - h + 1;
            for y in y_top..=Self::BOT {
                let lv = 3.0 + 12.0 * (Self::BOT - y) as f32 / Self::HMAX as f32; // brightness tracks height
                for c in 0..Self::W {
                    if y == y_top && (c == 0 || c == Self::W - 1) {
                        continue; // softened corners
                    }
                    cv.pixel(x + c, y, dither(lv, x + c, y));
                }
            }
            if h > 1 {
                cv.line(x + 1, y_top, x + Self::W - 2, y_top, 15);
            }
            peak.update(h, 15, 1);
            if peak.h > 0 {
                cv.rect(x, Self::BOT - peak.h - 4, Self::W, 2, true, 12);
            }
            fonts.draw_centered(cv, x + Self::W / 2, 127, BAND7_LABELS[i], Size::Regular, 10);
        }
        cv.line(0, Self::BOT + 1, SCREEN_WIDTH as i32 - 1, Self::BOT + 1, 5);
    }
}

impl Viz for Spectrum {
    fn name(&self) -> &'static str {
        "Spectrum"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.draw(f.left, f.fonts, &snap.spectrum_l, true, snap);
        self.draw(f.right, f.fonts, &snap.spectrum_r, false, snap);
    }
}

// ---- 80s hi-fi graphic EQ: LED segment columns, unlit ones as ghosts ----

pub struct SpectrumLed {
    peaks: [[Peak; 7]; 2],
    titles: Titles,
}

impl SpectrumLed {
    const TOP: i32 = 26;
    const BOT: i32 = 113;
    const SEG_PITCH: i32 = 4;
    const N: i32 = (Self::BOT - Self::TOP + 1) / Self::SEG_PITCH;
    const W: i32 = 26;
    const PITCH: i32 = 34;

    pub fn new() -> Self {
        let p = Peak { h: -1, hold: 0, fall: 0 }; // h = top lit segment
        Self { peaks: [[p; 7]; 2], titles: Titles::default() }
    }

    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, data: &[i32; 7], left: bool, snap: &FrameSnapshot) {
        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);
        for (i, peak) in self.peaks[!left as usize].iter_mut().enumerate() {
            let x = 3 + i as i32 * Self::PITCH;
            let lit = data[i].clamp(0, 255) * Self::N / 255;
            peak.update(lit - 1, 40, 3);
            for s in 0..Self::N {
                let lvl = if s < lit {
                    (4 + 11 * s / (Self::N - 1)) as u8 // brighter up the column
                } else if s == peak.h {
                    15 // peak
                } else {
                    1 // ghost
                };
                cv.rect(x, Self::BOT - (s + 1) * Self::SEG_PITCH + 1, Self::W, Self::SEG_PITCH - 1, true, lvl);
            }
            fonts.draw_centered(cv, x + Self::W / 2, 127, BAND7_LABELS[i], Size::Regular, 10);
        }
    }
}

impl Viz for SpectrumLed {
    fn name(&self) -> &'static str {
        "Spectrum LED"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.draw(f.left, f.fonts, &snap.spectrum_l, true, snap);
        self.draw(f.right, f.fonts, &snap.spectrum_r, false, snap);
    }
}

// ---- Studio analyser: 30 bars from the 120 log bands, grid, freq axis ----

const HD_BARS: usize = 30;

pub struct SpectrumHd {
    peaks: [[Peak; HD_BARS]; 2],
    titles: Titles,
}

impl SpectrumHd {
    const TOP: i32 = 26;
    const BOT: i32 = 112;
    const HMAX: i32 = Self::BOT - Self::TOP;
    const PITCH: i32 = 8;
    // The 120 bands span 8 octaves (63 Hz-16 kHz), log-spaced: a tick every
    // 2 octaves falls every quarter of the width. (tick x, label x, text)
    const AXIS: [(i32, i32, &'static str); 5] =
        [(1, 8, "63"), (60, 60, "250"), (120, 120, "1k"), (180, 180, "4k"), (239, 230, "16k")];

    pub fn new() -> Self {
        Self { peaks: [[Peak::default(); HD_BARS]; 2], titles: Titles::default() }
    }

    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, data: &[i32], left: bool, snap: &FrameSnapshot) {
        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);
        for g in 0..=4 {
            for x in (0..SCREEN_WIDTH as i32).step_by(3) {
                cv.pixel(x, Self::BOT - g * Self::HMAX / 4, 2); // dotted grid
            }
        }
        for (b, peak) in self.peaks[!left as usize].iter_mut().enumerate() {
            let sum: i32 = data[b * 4..b * 4 + 4].iter().map(|v| v.clamp(&0, &255)).sum();
            let (h, x) = ((sum / 4) * Self::HMAX / 255, b as i32 * Self::PITCH + 1);
            if h > 0 {
                cv.rect(x, Self::BOT - h + 1, Self::PITCH - 2, h, true, 10);
                cv.line(x, Self::BOT - h + 1, x + Self::PITCH - 3, Self::BOT - h + 1, 15);
            }
            peak.update(h, 20, 1);
            if peak.h > 0 {
                let y = Self::BOT - peak.h - 2;
                cv.line(x, y, x + Self::PITCH - 3, y, 15);
            }
        }
        for (tick, lx, text) in Self::AXIS {
            cv.line(tick, Self::BOT + 1, tick, Self::BOT + 3, 8);
            fonts.draw_centered(cv, lx, 125, text, Size::Small, 9);
        }
    }
}

impl Viz for SpectrumHd {
    fn name(&self) -> &'static str {
        "Spectrum HD"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.draw(f.left, f.fonts, &snap.spectrum_large_l, true, snap);
        self.draw(f.right, f.fonts, &snap.spectrum_large_r, false, snap);
    }
}

// ---- Outline bars ----

#[derive(Default)]
pub struct EmptySpectrum {
    titles: Titles,
}

impl EmptySpectrum {
    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, data: &[i32; 7], left: bool, snap: &FrameSnapshot) {
        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);
        for (i, &v) in data.iter().enumerate() {
            let (x, w) = (4 + i as i32 * 34, 22);
            let h = v * 95 / 255;
            let y = 114 - h;
            if h > 0 {
                cv.line(x, 114, x, y, 15);
                cv.line(x + w, 114, x + w, y, 15);
                cv.line(x, y, x + w, y, 15);
            }
            fonts.draw_centered(cv, x + w / 2, 127, BAND7_LABELS[i], Size::Regular, 15);
        }
    }
}

impl Viz for EmptySpectrum {
    fn name(&self) -> &'static str {
        "Empty Spectrum"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.draw(f.left, f.fonts, &snap.spectrum_l, true, snap);
        self.draw(f.right, f.fonts, &snap.spectrum_r, false, snap);
    }
}
