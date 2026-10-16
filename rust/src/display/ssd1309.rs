//! SSD1309, 128x64 monochrome: the canvas is thresholded into the
//! controller's 8-row pages and written synchronously (1 KB, ~1.3 ms at
//! 6.25 MHz).

use super::{Canvas, Panel};
use crate::hw::OutputLine;
use spidev::Spidev;
use std::io::Write;

/// Grey levels from here up light the pixel.
const THRESHOLD: u8 = 8;

pub struct Ssd1309 {
    spi: Spidev,
    dc: OutputLine,
    frame: Vec<u8>,
    last: Option<Vec<u8>>,
}

impl Ssd1309 {
    pub fn new(spi: Spidev, dc: OutputLine) -> Self {
        let mut p = Self { spi, dc, frame: Vec::new(), last: None };
        p.commands(&[
            0xAE,             // display off
            0x20, 0x00,       // horizontal addressing: one write fills all pages
            0xB0, 0xC8,       // page 0, COM scan remapped
            0x00, 0x10, 0x40, // column 0, start line 0
            0x81, 0xFF,       // contrast
            0xA1, 0xA6,       // segment remap, normal (not inverted)
            0xA8, 0x3F,       // multiplex 64
            0xA4,             // display follows RAM
            0xD3, 0x00,       // no offset
            0xD5, 0x70,       // clock
            0xD9, 0x22,       // precharge
            0xDA, 0x12,       // COM pins
            0xDB, 0x34,       // VCOMH
            0x8D, 0x14,       // charge pump
            0xAF,             // display on
        ]);
        p
    }

    fn commands(&mut self, cmds: &[u8]) {
        self.dc.set(false);
        let _ = self.spi.write_all(cmds);
    }

    /// Byte (page * width + x), bit (y % 8).
    fn pack(cv: &Canvas, out: &mut Vec<u8>) {
        let (w, h) = (cv.width() as usize, cv.height() as usize);
        out.clear();
        out.resize(w * h / 8, 0);
        for y in 0..h {
            let (page, bit) = ((y / 8) * w, 1u8 << (y % 8));
            for (x, &lv) in cv.buf[y * w..(y + 1) * w].iter().enumerate() {
                if lv >= THRESHOLD {
                    out[page + x] |= bit;
                }
            }
        }
    }
}

impl Panel for Ssd1309 {
    fn present(&mut self, cv: &Canvas) {
        let mut frame = std::mem::take(&mut self.frame);
        Self::pack(cv, &mut frame);
        if self.last.as_ref() != Some(&frame) {
            self.commands(&[0xB0, 0x00, 0x10]); // back to page 0, column 0
            self.dc.set(true);
            let _ = self.spi.write_all(&frame);
            self.frame = self.last.replace(frame).unwrap_or_default();
        } else {
            self.frame = frame;
        }
    }

    fn set_on(&mut self, on: bool) {
        self.commands(&[if on { 0xAF } else { 0xAE }]);
    }

    fn invalidate(&mut self) {
        self.last = None;
    }
}
