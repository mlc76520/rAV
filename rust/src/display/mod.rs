//! The panels. Visualizations draw on a Canvas (one byte per pixel, grey
//! level 0-15, sized by the profile); each panel driver converts it to its
//! controller's format when the frame is presented:
//!  - SSD1309: 1 bit per pixel in 8-row pages, lit from level 8 up;
//!  - SSD1322: 4 bits per pixel, sent by a writer thread.

mod ssd1309;
mod ssd1322;

use crate::hw::OutputLine;
use crate::profile::{PanelKind, Profile};
use anyhow::Result;

/// Grey-level drawing surface.
#[derive(Clone)]
pub struct Canvas {
    width: usize,
    height: usize,
    pub buf: Vec<u8>,
}

impl Canvas {
    pub fn new(width: usize, height: usize) -> Self {
        Self { width, height, buf: vec![0; width * height] }
    }

    pub fn width(&self) -> i32 {
        self.width as i32
    }

    pub fn height(&self) -> i32 {
        self.height as i32
    }

    pub fn clear(&mut self) {
        self.buf.fill(0);
    }

    #[inline]
    fn index(&self, x: i32, y: i32) -> Option<usize> {
        (x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height)
            .then(|| y as usize * self.width + x as usize)
    }

    /// Sets one pixel (off-screen is ignored); level 0 clears it.
    #[inline]
    pub fn pixel(&mut self, x: i32, y: i32, level: u8) {
        if let Some(i) = self.index(x, y) {
            self.buf[i] = level & 0x0F;
        }
    }

    #[inline]
    pub fn get(&self, x: i32, y: i32) -> u8 {
        self.index(x, y).map_or(0, |i| self.buf[i])
    }

    /// Bresenham line, both ends included.
    pub fn line(&mut self, mut x0: i32, mut y0: i32, x1: i32, y1: i32, level: u8) {
        let (dx, dy) = ((x1 - x0).abs(), (y1 - y0).abs());
        let (sx, sy) = (if x0 < x1 { 1 } else { -1 }, if y0 < y1 { 1 } else { -1 });
        let mut err = dx - dy;
        loop {
            self.pixel(x0, y0, level);
            if x0 == x1 && y0 == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                x0 += sx;
            }
            if e2 < dx {
                err += dx;
                y0 += sy;
            }
        }
    }

    /// Outline, or clipped fill (level 0 = clear the area).
    pub fn rect(&mut self, x: i32, y: i32, w: i32, h: i32, filled: bool, level: u8) {
        let lvl = level & 0x0F;
        if filled {
            let (xs, ys) = (x.max(0), y.max(0));
            let xe = (x + w).min(self.width as i32);
            let ye = (y + h).min(self.height as i32);
            if xe <= xs {
                return;
            }
            for py in ys..ye {
                let row = py as usize * self.width;
                self.buf[row + xs as usize..row + xe as usize].fill(lvl);
            }
        } else {
            self.line(x, y, x + w - 1, y, lvl);
            self.line(x + w - 1, y, x + w - 1, y + h - 1, lvl);
            self.line(x + w - 1, y + h - 1, x, y + h - 1, lvl);
            self.line(x, y + h - 1, x, y, lvl);
        }
    }

    /// Zeroes whole rows [from, to).
    pub fn blank_rows(&mut self, from: i32, to: i32) {
        let (from, to) = (from.max(0) as usize, to.min(self.height as i32).max(0) as usize);
        if from < to {
            self.buf[from * self.width..to * self.width].fill(0);
        }
    }
}

/// A panel controller.
trait Panel {
    /// Converts and sends the canvas, unless it is unchanged.
    fn present(&mut self, cv: &Canvas);
    fn set_on(&mut self, on: bool);
    /// Makes the next present() send even an unchanged frame.
    fn invalidate(&mut self);
    /// Flushes everything sent so far (stops a writer thread).
    fn finish(&mut self) {}
}

/// A panel and its canvas.
pub struct Display {
    pub canvas: Canvas,
    panel: Box<dyn Panel>,
}

impl Display {
    /// Resets the panel and sends its init sequence.
    pub fn new(profile: &Profile, spi_path: &str, dc: OutputLine, rst: &OutputLine) -> Result<Self> {
        rst.set(false);
        std::thread::sleep(std::time::Duration::from_millis(10));
        rst.set(true);
        std::thread::sleep(std::time::Duration::from_millis(10));
        let spi = crate::hw::open_spi(spi_path, profile.spi_hz)?;
        let panel: Box<dyn Panel> = match profile.panel {
            PanelKind::Ssd1309 => Box::new(ssd1309::Ssd1309::new(spi, dc)),
            PanelKind::Ssd1322 => Box::new(ssd1322::Ssd1322::new(spi, dc)?),
        };
        Ok(Self { canvas: Canvas::new(profile.width, profile.height), panel })
    }

    pub fn present(&mut self) {
        self.panel.present(&self.canvas);
    }

    pub fn sleep(&mut self) {
        self.panel.set_on(false);
    }

    pub fn wake(&mut self) {
        self.panel.set_on(true);
    }

    /// Display off, then a black frame so the panel wakes up blank.
    pub fn shutdown(&mut self) {
        self.panel.set_on(false);
        self.canvas.clear();
        self.panel.invalidate();
        self.present();
        self.panel.finish();
    }
}
