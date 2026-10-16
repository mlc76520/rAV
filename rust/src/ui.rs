//! Drawing helpers shared by both panel families: ordered dithering and
//! the scrolling track title.

use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::snapshot::FrameSnapshot;

/// Ordered (Bayer 4x4) dither thresholds: smooth gradients with 16 grey
/// levels (and readable shapes once thresholded on the mono panel).
const BAYER: [[u8; 4]; 4] = [[0, 8, 2, 10], [12, 4, 14, 6], [3, 11, 1, 9], [15, 7, 13, 5]];

/// Grey level for a fractional level lv (0..15) at pixel (x, y).
#[inline]
pub fn dither(lv: f32, x: i32, y: i32) -> u8 {
    let lo = lv as i32;
    let frac16 = ((lv - lo as f32) * 16.0) as i32;
    (lo + (frac16 > BAYER[(y & 3) as usize][(x & 3) as usize] as i32) as i32).min(15) as u8
}

#[derive(PartialEq, Clone, Copy)]
enum ScrollState {
    WaitStart,
    ScrollRight,
    WaitEnd,
    ScrollLeft,
}

/// Ping-pong text scroller: waits, scrolls to the end, waits, scrolls back.
/// Text that fits is centred instead.
pub struct TextScroller {
    size: Size,
    text: String,
    width: Option<i32>,
    pos: f32,
    wait: f32,
    state: ScrollState,
}

impl TextScroller {
    const SPEED: f32 = 25.0; // px/s
    const WAIT: f32 = 2.0; // s

    pub fn new(size: Size) -> Self {
        Self { size, text: String::new(), width: None, pos: 0.0, wait: 0.0, state: ScrollState::WaitStart }
    }

    pub fn set_text(&mut self, text: &str) {
        if text != self.text {
            *self = Self { text: text.to_string(), ..Self::new(self.size) };
        }
    }

    /// Draws the text with its baseline at y, clipped to [x, x + w).
    pub fn render(&mut self, cv: &mut Canvas, fonts: &Fonts, x: i32, baseline: i32, w: i32, dt: f32) {
        if self.text.is_empty() {
            return;
        }
        let dt = dt.clamp(0.0, 0.1);
        let tw = *self.width.get_or_insert_with(|| fonts.width(&self.text, self.size));
        let render_x = if tw <= w {
            self.state = ScrollState::WaitStart;
            self.pos = 0.0;
            self.wait = 0.0;
            x + (w - tw) / 2
        } else {
            let max_scroll = (tw - w) as f32;
            match self.state {
                ScrollState::WaitStart => {
                    self.pos = 0.0;
                    self.wait += dt;
                    if self.wait >= Self::WAIT {
                        self.wait = 0.0;
                        self.state = ScrollState::ScrollRight;
                    }
                }
                ScrollState::ScrollRight => {
                    self.pos += Self::SPEED * dt;
                    if self.pos >= max_scroll {
                        self.pos = max_scroll;
                        self.state = ScrollState::WaitEnd;
                    }
                }
                ScrollState::WaitEnd => {
                    self.wait += dt;
                    if self.wait >= Self::WAIT {
                        self.wait = 0.0;
                        self.state = ScrollState::ScrollLeft;
                    }
                }
                ScrollState::ScrollLeft => {
                    self.pos -= Self::SPEED * dt;
                    if self.pos <= 0.0 {
                        self.pos = 0.0;
                        self.state = ScrollState::WaitStart;
                    }
                }
            }
            x - self.pos as i32
        };
        fonts.render(cv, &self.text, render_x, baseline, self.size, false, 15, Some((x, w)));
    }
}

/// The track title scrolling across the top of each panel.
pub struct Titles {
    left: TextScroller,
    right: TextScroller,
    /// y is the top of the text row (SSD1322 layouts) rather than its
    /// baseline (SSD1309 layouts).
    top_aligned: bool,
}

impl Default for Titles {
    /// SSD1322 layout: Regular size, y = top of the row.
    fn default() -> Self {
        Self { left: TextScroller::new(Size::Regular), right: TextScroller::new(Size::Regular), top_aligned: true }
    }
}

impl Titles {
    /// SSD1309 layout: Small size, y = baseline.
    pub fn small() -> Self {
        Self { left: TextScroller::new(Size::Small), right: TextScroller::new(Size::Small), top_aligned: false }
    }

    pub fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, left: bool, y: i32, snap: &FrameSnapshot) {
        let text = &snap.track.formatted_text;
        if text.is_empty() {
            return;
        }
        let s = if left { &mut self.left } else { &mut self.right };
        let baseline = if self.top_aligned { y + fonts.ascent(s.size) } else { y };
        s.set_text(text);
        s.render(cv, fonts, 0, baseline, cv.width(), snap.dt);
    }
}
