//! "Matrix rain" transition played when switching visualizations, sized
//! to the canvas: each column's rain wipes down over the last frame of the
//! old visualization and uncovers the new one, running live, behind its
//! trail (on the mono panel the fading tails simply drop out once below
//! the lit threshold).

use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::viz::Rng;
use std::time::{Duration, Instant};

const CHAR_W: i32 = 8;
const CHAR_H: i32 = 10; // SMALL font + line spacing
const ROW_BUF: usize = 18; // char ring: rows on a 128-px panel + 6
const DURATION: Duration = Duration::from_millis(750);
const CHARSET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%&*<>=+-/\\|?{}[]";

#[derive(Clone, Copy, Default)]
struct Column {
    start_y: f32, // head baseline at the start, above the screen
    speed: f32,   // px/s, so that the trail has left the screen in time
    trail: i32,   // length in chars
    scramble: i32,
    chars: [u8; ROW_BUF],
}

impl Column {
    fn head_y(&self, t: f32) -> i32 {
        (self.start_y + self.speed * t) as i32
    }
}

pub struct Transition {
    cols: [Vec<Column>; 2],
    /// Last frame of the old visualization, per panel.
    old: [Canvas; 2],
    rng: Rng,
    start: Option<Instant>,
}

impl Transition {
    pub fn new() -> Self {
        Self { cols: [Vec::new(), Vec::new()], old: [Canvas::new(0, 0), Canvas::new(0, 0)], rng: Rng::new(0x9E37_79B9), start: None }
    }

    fn rand_u32(&mut self, n: u32) -> u32 {
        (self.rng.next() * n as f32) as u32 % n
    }

    fn random_char(&mut self) -> u8 {
        CHARSET[self.rand_u32(CHARSET.len() as u32) as usize]
    }

    fn init_column(&mut self, height: i32) -> Column {
        let trail = 3 + self.rand_u32(5) as i32; // 3-7 chars
        let start_y = -(self.rand_u32(height as u32 / 2 + 1) as f32);
        // Done after 55-100 % of the transition: ragged, but always complete.
        let done = DURATION.as_secs_f32() * (0.55 + 0.45 * self.rng.next());
        let travel = height as f32 + ((trail + 1) * CHAR_H) as f32 - start_y;
        let mut c = Column { start_y, speed: travel / done, trail, scramble: 0, chars: [0; ROW_BUF] };
        for ch in c.chars.iter_mut() {
            *ch = CHARSET[(self.rng.next() * CHARSET.len() as f32) as usize % CHARSET.len()];
        }
        c
    }

    /// Starts the rain over the frames now on the panels.
    pub fn start(&mut self, left: &Canvas, right: &Canvas) {
        self.start = Some(Instant::now());
        for (p, cv) in [left, right].into_iter().enumerate() {
            self.old[p].clone_from(cv);
            let n = ((cv.width() + CHAR_W - 1) / CHAR_W) as usize;
            self.cols[p] = (0..n).map(|_| self.init_column(cv.height())).collect();
        }
    }

    /// Composes one panel: the old frame below each column's head, black
    /// under its trail, the new visualization (already in `cv`) above.
    fn render_columns(&mut self, panel: usize, cv: &mut Canvas, fonts: &Fonts, t: f32) {
        let (width, height) = (cv.width(), cv.height());
        for i in 0..self.cols[panel].len() {
            // Occasional character scrambling for that "decoding" feel.
            self.cols[panel][i].scramble += 1;
            if self.cols[panel][i].scramble >= 3 {
                self.cols[panel][i].scramble = 0;
                let (slot, ch) = (self.rand_u32(ROW_BUF as u32) as usize, self.random_char());
                self.cols[panel][i].chars[slot] = ch;
            }
            let c = self.cols[panel][i];
            let (x0, head) = (i as i32 * CHAR_W, c.head_y(t));
            let x1 = (x0 + CHAR_W).min(width);
            let top = head - c.trail * CHAR_H;
            for y in top.max(0)..height {
                let row = y as usize * width as usize;
                for x in x0..x1 {
                    let idx = row + x as usize;
                    cv.buf[idx] = if y > head { self.old[panel].buf[idx] } else { 0 };
                }
            }
            for k in 0..c.trail {
                let y = head - k * CHAR_H;
                if y < 0 || y >= height + CHAR_H {
                    continue;
                }
                // Stable char per row position so it doesn't flicker.
                let row = (y / CHAR_H).rem_euclid(ROW_BUF as i32) as usize;
                let lvl = match k {
                    0 => 15, // bright head
                    1 => 12,
                    2 => 9,
                    _ => (7 - (k - 2)).max(1) as u8, // fading tail
                };
                let s = [c.chars[row]];
                fonts.draw(cv, x0, y, std::str::from_utf8(&s).unwrap_or(" "), Size::Small, lvl);
            }
        }
    }

    /// Composes the transition over the new visualization's frame on both
    /// canvases. Returns false once the transition is over.
    pub fn update(&mut self, left: &mut Canvas, right: &mut Canvas, fonts: &Fonts) -> bool {
        let Some(start) = self.start else { return false };
        let elapsed = start.elapsed();
        if elapsed >= DURATION {
            self.start = None;
            return false;
        }
        let t = elapsed.as_secs_f32();
        self.render_columns(0, left, fonts, t);
        self.render_columns(1, right, fonts, t);
        true
    }
}
