//! Track details on the left panel (source, bitrate/format, time + progress,
//! mini 7-band spectrum), 120-band mirrored spectrum on the right.

use crate::viz::{Frame, Viz};
use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::snapshot::FrameSnapshot;
use crate::ui::{self, Titles};

pub struct TrackInfo {
    // Peak lines of the mini spectrum (y), starting at the floor.
    peaks: [[f32; 7]; 2],
    titles: Titles,
    hatch_phase: f32, // s, scrolls the stream hatching
}

impl TrackInfo {
    pub fn new() -> Self {
        Self { peaks: [[109.0; 7]; 2], titles: Titles::default(), hatch_phase: 0.0 }
    }

    fn progress(&self, cv: &mut Canvas, fonts: &Fonts, y: i32, elapsed: u32, total: u32) {
        fonts.draw(cv, 1, y, &ui::time_text(elapsed, total), Size::Regular, 15);
        ui::progress_bar(cv, (1, y + 5, 236, 8), 1, elapsed, total, self.hatch_phase);
    }
}

impl Viz for TrackInfo {
    fn name(&self) -> &'static str {
        "Track Info"
    }

    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        let (cv, fonts) = (&mut *f.left, f.fonts);
        cv.clear();
        self.titles.draw(cv, fonts, true, 5, snap);
        let mut y = 35;
        if snap.bt_streaming {
            fonts.draw(cv, 1, y, "Source: Bluetooth", Size::Regular, 15);
            y += 20;
            fonts.draw(cv, 1, y, "A2DP", Size::Regular, 15);
            y += 60;
            if snap.total_s > 0 {
                self.progress(cv, fonts, y, snap.elapsed_s, snap.total_s);
            }
        } else {
            fonts.draw(cv, 1, y, &format!("Bitrate: {}", snap.track.bitrate), Size::Regular, 15);
            y += 20;
            fonts.draw(cv, 1, y, &snap.track.format, Size::Regular, 15);
            if snap.total_s == 0 {
                ui::radio_icon(cv, 1, y + 5, true); // a stream: web radio
            }
            y += 60;
            self.hatch_phase += snap.dt;
            self.progress(cv, fonts, y, snap.elapsed_s, snap.total_s);
        }

        // Mini spectrum, both channels, with falling peak lines.
        for (c, (spec, x0)) in [(&snap.spectrum_l, 150), (&snap.spectrum_r, 195)].into_iter().enumerate() {
            for i in 0..7 {
                let (x, h) = (x0 + i as i32 * 5, spec[i] * 50 / 255);
                if h > 0 {
                    cv.rect(x, 109 - h, 3, h, true, 15);
                }
                let peak = &mut self.peaks[c][i];
                *peak = if ((109 - h) as f32) < *peak { (109 - h) as f32 } else { (*peak + 1.0).min(109.0) };
                cv.line(x, *peak as i32, x + 2, *peak as i32, 15);
            }
        }
        cv.line(145, 53, 145, 110, 15);
        cv.line(145, 110, 235, 110, 15);

        // Right panel: 120-band spectrum, left channel up, right down.
        let cv = &mut *f.right;
        cv.clear();
        for i in 0..120 {
            let x = i as i32 * 2;
            let hl = 1 + snap.spectrum_large_l[i] * 63 / 255;
            cv.rect(x, 64 - hl, 1, hl, true, 15);
            let hr = 1 + snap.spectrum_large_r[i] * 63 / 255;
            cv.rect(x, 64, 1, hr, true, 15);
        }
    }
}
