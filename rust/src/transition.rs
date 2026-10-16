//! "Matrix rain" transition played when switching visualizations, sized
//! to the canvas (on the mono panel the fading tails simply drop out once
//! below the lit threshold).

use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::viz::Rng;
use std::time::{Duration, Instant};

const CHAR_W: i32 = 8;
const CHAR_H: i32 = 10; // SMALL font + line spacing
const ROW_BUF: usize = 18; // char ring: rows on a 128-px panel + 6
const DURATION: Duration = Duration::from_millis(1600);
const FADE: Duration = Duration::from_millis(400); // last part fades to black
const CHARSET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%&*<>=+-/\\|?{}[]";

#[derive(Clone, Copy, Default)]
struct Column {
    head_y: f32, // pixel y of the head (baseline)
    speed: f32,  // px/s
    trail: i32,  // length in chars
    scramble: i32,
    chars: [u8; ROW_BUF],
}

pub struct Transition {
    cols: [Vec<Column>; 2],
    rng: Rng,
    start: Option<Instant>,
    last: Instant,
}

impl Transition {
    pub fn new() -> Self {
        Self { cols: [Vec::new(), Vec::new()], rng: Rng::new(0x9E37_79B9), start: None, last: Instant::now() }
    }

    fn rand_u32(&mut self, n: u32) -> u32 {
        (self.rng.next() * n as f32) as u32 % n
    }

    fn random_char(&mut self) -> u8 {
        CHARSET[self.rand_u32(CHARSET.len() as u32) as usize]
    }

    fn init_column(&mut self) -> Column {
        let mut c = Column {
            head_y: -(self.rand_u32(120) as f32), // start above the screen
            speed: 60.0 + self.rand_u32(140) as f32, // 60-200 px/s
            trail: 5 + self.rand_u32(10) as i32,   // 5-14 chars
            scramble: 0,
            chars: [0; ROW_BUF],
        };
        for ch in c.chars.iter_mut() {
            *ch = CHARSET[(self.rng.next() * CHARSET.len() as f32) as usize % CHARSET.len()];
        }
        c
    }

    /// Starts the rain on panels `width` pixels wide.
    pub fn start(&mut self, width: i32) {
        let now = Instant::now();
        self.start = Some(now);
        self.last = now;
        let n = (width / CHAR_W) as usize;
        for p in 0..2 {
            self.cols[p] = (0..n).map(|_| self.init_column()).collect();
        }
    }

    fn render_columns(&mut self, panel: usize, cv: &mut Canvas, fonts: &Fonts, dt: f32) {
        cv.clear();
        let height = cv.height();
        for i in 0..self.cols[panel].len() {
            // Occasional character scrambling for that "decoding" feel.
            self.cols[panel][i].scramble += 1;
            if self.cols[panel][i].scramble >= 3 {
                self.cols[panel][i].scramble = 0;
                let (slot, ch) = (self.rand_u32(ROW_BUF as u32) as usize, self.random_char());
                self.cols[panel][i].chars[slot] = ch;
            }
            let c = &mut self.cols[panel][i];
            let (x, head) = (i as i32 * CHAR_W, c.head_y as i32);
            for t in 0..c.trail {
                let y = head - t * CHAR_H;
                if y < CHAR_H - 2 || y >= height + CHAR_H {
                    continue;
                }
                // Stable char per row position so it doesn't flicker.
                let row = (y / CHAR_H).rem_euclid(ROW_BUF as i32) as usize;
                let lvl = match t {
                    0 => 15, // bright head
                    1 => 12,
                    2 => 9,
                    _ => (7 - (t - 2)).max(1) as u8, // fading tail
                };
                let s = [c.chars[row]];
                fonts.draw(cv, x, y, std::str::from_utf8(&s).unwrap_or(" "), Size::Small, lvl);
            }
            c.head_y += c.speed * dt;
            if c.head_y - (c.trail * CHAR_H) as f32 > (height + CHAR_H) as f32 {
                self.cols[panel][i] = self.init_column(); // recycle once off screen
            }
        }
    }

    /// 16-entry LUT fade (k in 0..1).
    fn fade(cv: &mut Canvas, k: f32) {
        if k >= 1.0 {
            return;
        }
        if k <= 0.0 {
            cv.clear();
            return;
        }
        let q = (k * 16.0) as u32;
        let lut: [u8; 16] = std::array::from_fn(|v| (v as u32 * q / 16) as u8);
        for p in cv.buf.iter_mut() {
            *p = lut[(*p & 0x0F) as usize];
        }
    }

    /// Draws one transition frame on both canvases. Returns false once the
    /// transition is over (the caller then renders the visualization).
    pub fn update(&mut self, left: &mut Canvas, right: &mut Canvas, fonts: &Fonts) -> bool {
        let Some(start) = self.start else { return false };
        let now = Instant::now();
        let elapsed = now - start;
        if elapsed >= DURATION {
            self.start = None;
            return false;
        }
        let dt = (now - self.last).as_secs_f32().min(0.1);
        self.last = now;
        self.render_columns(0, left, fonts, dt);
        self.render_columns(1, right, fonts, dt);
        let fade_start = DURATION - FADE;
        if elapsed > fade_start {
            let k = 1.0 - (elapsed - fade_start).as_secs_f32() / FADE.as_secs_f32();
            Self::fade(left, k);
            Self::fade(right, k);
        }
        true
    }
}
