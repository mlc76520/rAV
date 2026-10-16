//! FreeType text rendering with a glyph cache. The charset on these screens
//! is tiny, so after warm-up FreeType is never called on the hot path.
//! Single-threaded by design (render thread only): the cache is a RefCell.

use crate::display::Canvas;
use anyhow::{Context, Result};
use freetype::face::LoadFlag;
use freetype::{Face, Library};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Size {
    Small,
    Regular,
    Large,
}

struct Glyph {
    bits: Vec<u8>, // packed mono bitmap, MSB first, `pitch` bytes per row
    width: i32,
    rows: i32,
    pitch: i32,
    left: i32,
    top: i32,
    advance: i32,
}

pub struct Fonts {
    _lib: Library,
    regular: Face,
    small: Option<Face>,
    large: Option<Face>,
    cache: RefCell<HashMap<(char, Size), Option<Rc<Glyph>>>>,
}

impl Fonts {
    /// `px`: pixel heights of the Small, Regular and Large sizes.
    pub fn load(path: &str, px: [u32; 3]) -> Result<Self> {
        let lib = Library::init().context("FreeType init")?;
        let face = |px| -> Result<Face> {
            let f = lib.new_face(path, 0).with_context(|| format!("loading font {path}"))?;
            f.set_pixel_sizes(0, px)?;
            Ok(f)
        };
        let regular = face(px[1])?;
        let small = face(px[0]).ok();
        let large = face(px[2]).ok();
        Ok(Self { _lib: lib, regular, small, large, cache: RefCell::new(HashMap::new()) })
    }

    fn face(&self, size: Size) -> &Face {
        match size {
            Size::Small => self.small.as_ref().unwrap_or(&self.regular),
            Size::Large => self.large.as_ref().unwrap_or(&self.regular),
            Size::Regular => &self.regular,
        }
    }

    fn glyph(&self, c: char, size: Size) -> Option<Rc<Glyph>> {
        if let Some(g) = self.cache.borrow().get(&(c, size)) {
            return g.clone();
        }
        let face = self.face(size);
        let g = face
            .load_char(c as usize, LoadFlag::RENDER | LoadFlag::TARGET_MONO)
            .ok()
            .map(|_| {
                let slot = face.glyph();
                let bm = slot.bitmap();
                let (rows, pitch) = (bm.rows(), bm.pitch());
                let bits = if rows > 0 && pitch > 0 {
                    bm.buffer()[..(rows * pitch) as usize].to_vec()
                } else {
                    Vec::new()
                };
                Rc::new(Glyph {
                    bits,
                    width: bm.width(),
                    rows,
                    pitch,
                    left: slot.bitmap_left(),
                    top: slot.bitmap_top(),
                    advance: (slot.advance().x >> 6) as i32,
                })
            });
        self.cache.borrow_mut().insert((c, size), g.clone());
        g
    }

    /// Draws `text` with its baseline at y. Pixels only ever get brighter
    /// (max with what is there); `invert` instead darkens by `level`.
    /// `clip` = Some((x, w)) restricts drawing to that column range.
    pub fn render(&self, cv: &mut Canvas, text: &str, x: i32, y: i32, size: Size,
                  invert: bool, level: u8, clip: Option<(i32, i32)>) {
        let (lo, hi) = match clip {
            Some((cx, cw)) => (cx.max(0), (cx + cw).min(cv.width())),
            None => (0, cv.width()),
        };
        let lvl = level & 0x0F;
        let mut cursor = x;
        for c in text.chars() {
            let Some(g) = self.glyph(c, size) else { continue };
            let (sx, sy) = (cursor + g.left, y - g.top);
            for r in 0..g.rows {
                let py = sy + r;
                if py < 0 || py >= cv.height() {
                    continue;
                }
                let row = &g.bits[(r * g.pitch) as usize..];
                for col in 0..g.width {
                    let px = sx + col;
                    if px < lo || px >= hi || row[(col >> 3) as usize] & (0x80 >> (col & 7)) == 0 {
                        continue;
                    }
                    let cur = cv.get(px, py);
                    let v = if invert { cur.saturating_sub(lvl) } else { cur.max(lvl) };
                    cv.pixel(px, py, v);
                }
            }
            cursor += g.advance;
        }
    }

    pub fn draw(&self, cv: &mut Canvas, x: i32, y: i32, text: &str, size: Size, level: u8) {
        self.render(cv, text, x, y, size, false, level, None);
    }

    /// Draws `text` centred on cx.
    pub fn draw_centered(&self, cv: &mut Canvas, cx: i32, y: i32, text: &str, size: Size, level: u8) {
        self.draw(cv, cx - self.width(text, size) / 2, y, text, size, level);
    }

    pub fn width(&self, text: &str, size: Size) -> i32 {
        text.chars().filter_map(|c| self.glyph(c, size)).map(|g| g.advance).sum()
    }

    /// Longest prefix that fits in max_px with ".." appended (single pass),
    /// or the text itself if it already fits.
    pub fn truncate(&self, text: &str, max_px: i32, size: Size) -> String {
        let ell = self.width("..", size);
        let (mut width, mut keep) = (0, 0);
        for (i, c) in text.char_indices() {
            if width + ell <= max_px {
                keep = i;
            }
            if let Some(g) = self.glyph(c, size) {
                width += g.advance;
            }
        }
        if width <= max_px {
            text.to_string()
        } else {
            format!("{}..", &text[..keep])
        }
    }

    pub fn ascent(&self, size: Size) -> i32 {
        self.face(size).size_metrics().map(|m| (m.ascender >> 6) as i32).unwrap_or(0)
    }

    pub fn descent(&self, size: Size) -> i32 {
        self.face(size).size_metrics().map(|m| ((-m.descender) >> 6) as i32).unwrap_or(0)
    }

    /// Cap height approximation used to centre digits vertically.
    pub fn cap_height(&self, size: Size) -> i32 {
        self.ascent(size) - self.descent(size)
    }
}
