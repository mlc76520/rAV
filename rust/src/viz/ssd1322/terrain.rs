//! 3D spectral terrain: each new row is the live 120-band spectrum (bass
//! left, treble right); one row is pushed per rendered frame, so the panel
//! shows the last ~0.4 s of music as a mountain landscape. Hidden-line
//! removal uses the classic per-column skyline: rows are drawn front to back
//! and a farther pixel shows only where it pokes above every nearer ridge.
//! Depth fades brightness, peaks glow, a bass transient flashes the front.

use crate::viz::{Beat, Frame, Viz};
use super::SCREEN_WIDTH;
use crate::display::Canvas;
use crate::font::Fonts;
use crate::snapshot::FrameSnapshot;
use crate::ui::Titles;

const ROWS: usize = 26;
const BANDS: usize = 120;

struct Channel {
    hist: [[u8; BANDS]; ROWS], // ring buffer, newest row at head
    head: usize,
    beat: Beat,
}

pub struct Terrain {
    ch: [Channel; 2],
    titles: Titles,
}

impl Terrain {
    pub fn new() -> Self {
        let ch = || Channel { hist: [[0; BANDS]; ROWS], head: 0, beat: Beat::default() };
        Self { ch: [ch(), ch()], titles: Titles::default() }
    }

    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, spec: &[i32], left: bool, snap: &FrameSnapshot) {
        let ch = &mut self.ch[!left as usize];
        // One new row per frame: rows advance exactly one depth step each
        // frame, so no fractional smooth-scroll interpolation is needed.
        ch.head = (ch.head + ROWS - 1) % ROWS;
        for (h, &v) in ch.hist[ch.head].iter_mut().zip(spec) {
            *h = v.clamp(0, 255) as u8;
        }
        let bass = spec[..8].iter().map(|&v| v as f32 / 255.0).fold(0.0, f32::max);
        ch.beat.update(bass, snap.dt, 4.0);
        let flash = ch.beat.level;

        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);

        let mut miny = [127i32; SCREEN_WIDTH];
        for r in 0..ROWS {
            let row = &ch.hist[(ch.head + r) % ROWS];
            let t = r as f32 / ROWS as f32; // 0 = front ... 1 = horizon
            let s = 1.0 / (1.0 + 3.0 * t); // perspective scale
            let y_base = 42.0 + 82.0 * s; // row baseline rises with depth
            let half = 118.0 * s; // row half-width shrinks
            let hmax = 62.0 * s; // peak height shrinks too
            let (xl, xr) = ((120.0 - half) as i32, (120.0 + half) as i32);
            let base_lvl = (2.0 + 13.0 * s + if r < 4 { flash * 3.0 } else { 0.0 }) as i32;

            let ustep = (BANDS - 1) as f32 / (xr - xl) as f32;
            let mut prev_y: Option<i32> = None;
            for x in xl..=xr {
                let u = (x - xl) as f32 * ustep;
                let (b0, fu) = (u as usize, u - (u as usize) as f32);
                let v = row[b0] as f32 * (1.0 - fu) + row[(b0 + 1).min(BANDS - 1)] as f32 * fu;
                let y = ((y_base - v / 255.0 * hmax) as i32).max(0);
                let lvl = (base_lvl + (v / 255.0 * 3.0) as i32).min(15) as u8; // peaks glow

                // Vertical run to the previous column keeps the ridge
                // connected; only the part above the skyline is visible.
                let (y0, y1) = prev_y.map_or((y, y), |p| (p.min(y), p.max(y)));
                let col = &mut miny[x as usize];
                for yy in y0..=y1 {
                    if yy >= *col {
                        break;
                    }
                    cv.pixel(x, yy, lvl);
                }
                *col = (*col).min(y);
                prev_y = Some(y);
            }
        }
    }
}

impl Viz for Terrain {
    fn name(&self) -> &'static str {
        "Terrain"
    }

    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.draw(f.left, f.fonts, &snap.spectrum_large_l, true, snap);
        self.draw(f.right, f.fonts, &snap.spectrum_large_r, false, snap);
    }
}
