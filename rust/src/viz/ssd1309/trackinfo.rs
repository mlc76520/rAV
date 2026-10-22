//! Track details with a mini 7-band spectrum (left), 64-band mirrored
//! spectrum (right).

use super::Peaks;
use crate::viz::{Frame, Viz};
use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::snapshot::{BANDS_SMALL, FrameSnapshot};
use crate::ui::{self, TextScroller};

const MINI_FLOOR: i32 = 43;

pub struct TrackInfo {
    scroller: TextScroller,
    peaks: [Peaks<BANDS_SMALL>; 2],
    hatch_phase: f32, // s, scrolls the stream hatching
}

impl TrackInfo {
    pub fn new() -> Self {
        Self {
            scroller: TextScroller::new(Size::Small),
            peaks: [Peaks::new(MINI_FLOOR as f32), Peaks::new(MINI_FLOOR as f32)],
            hatch_phase: 0.0,
        }
    }

    fn time_and_progress(&self, cv: &mut Canvas, fonts: &Fonts, y: i32, elapsed: u32, total: u32) {
        fonts.draw(cv, 1, y, &ui::time_text(elapsed, total), Size::Small, 15);
        ui::progress_bar(cv, (1, y + 5, 126, 5), 0, elapsed, total, self.hatch_phase);
    }

    fn details(&mut self, cv: &mut Canvas, fonts: &Fonts, snap: &FrameSnapshot) {
        let t = &snap.track;
        let mut y = 5;
        if snap.bt_streaming {
            self.scroller.set_text(&t.formatted_text);
            self.scroller.render(cv, fonts, 1, y, 128, snap.dt);
            y += 10;
            fonts.draw(cv, 1, y, "Source: Bluetooth", Size::Small, 15);
            y += 10;
            fonts.draw(cv, 1, y, "A2DP", Size::Small, 15);
            y += 20;
            if snap.total_s > 0 {
                self.time_and_progress(cv, fonts, y, snap.elapsed_s, snap.total_s);
            }
        } else {
            let mut line = format!("{}. {}", t.track_number, t.title);
            for part in [&t.artist, &t.album] {
                if !part.is_empty() {
                    line += &format!(" - {part}");
                }
            }
            if !t.date.is_empty() {
                line += &format!(" ({})", t.date);
            }
            self.scroller.set_text(&line);
            self.scroller.render(cv, fonts, 1, y, 126, snap.dt);
            y += 10;
            fonts.draw(cv, 1, y, &format!("Bitrate: {}", t.bitrate), Size::Small, 15);
            y += 10;
            fonts.draw(cv, 1, y, &t.format, Size::Small, 15);
            if snap.total_s == 0 {
                ui::radio_icon(cv, 1, y + 2, false); // a stream: web radio
            }
            y += 20;
            self.hatch_phase += snap.dt;
            self.time_and_progress(cv, fonts, y, snap.elapsed_s, snap.total_s);
        }
    }

    fn mini_spectrum(&mut self, cv: &mut Canvas, snap: &FrameSnapshot) {
        for (ch, (data, x0)) in [(&snap.spectrum_l, 80), (&snap.spectrum_r, 103)].into_iter().enumerate() {
            for (i, v) in data.iter().enumerate() {
                let (x, h) = (x0 + i as i32 * 3, v * 32 / 255);
                if h > 0 {
                    cv.rect(x, MINI_FLOOR - h, 2, h, true, 15);
                }
                let p = self.peaks[ch].update(i, MINI_FLOOR - h);
                cv.line(x, p, x + 1, p, 15);
            }
        }
        cv.line(77, 13, 77, 45, 15);
        cv.line(77, 45, 125, 45, 15);
    }
}

impl Viz for TrackInfo {
    fn name(&self) -> &'static str {
        "Track Info"
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        f.left.clear();
        self.details(f.left, f.fonts, snap);
        self.mini_spectrum(f.left, snap);

        f.right.clear();
        for (i, (l, r)) in snap.spectrum_large_l.iter().zip(&snap.spectrum_large_r).enumerate() {
            let x = i as i32 * 2;
            let (hl, hr) = (1 + l * 31 / 255, 1 + r * 31 / 255);
            f.right.rect(x, 32 - hl, 1, hl, true, 15);
            f.right.rect(x, 32, 1, hr, true, 15);
        }
    }
}
