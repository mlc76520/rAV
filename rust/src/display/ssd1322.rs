//! SSD1322, 240x128 in 16 grey levels: present() packs the canvas to 4 bits
//! per pixel on the caller's thread, skips the transfer if the frame is
//! unchanged, and otherwise hands it to a writer thread through a
//! rendezvous channel: it blocks only while the previous frame is still on
//! the wire, so rendering frame N+1 overlaps the SPI transfer of frame N.
//! Commands travel on the same channel, so they are always ordered with the
//! frames. A process-wide bus mutex keeps each panel's command/data
//! sequence contiguous on the shared SPI bus.

use super::{Canvas, Panel};
use crate::hw::{self, OutputLine};
use anyhow::Result;
use spidev::Spidev;
use std::io::Write;
use std::sync::Mutex;
use std::sync::mpsc::{self, Receiver, Sender, SyncSender};
use std::thread::{self, JoinHandle};

static SPI_BUS: Mutex<()> = Mutex::new(());

enum Msg {
    Command { cmd: u8, data: Vec<u8> },
    Frame(Vec<u8>),
}

pub struct Ssd1322 {
    tx: Option<SyncSender<Msg>>,
    spare_rx: Receiver<Vec<u8>>,
    last: Option<Vec<u8>>,
    writer: Option<JoinHandle<()>>,
}

impl Ssd1322 {
    pub fn new(spi: Spidev, dc: OutputLine) -> Result<Self> {
        let (tx, rx) = mpsc::sync_channel::<Msg>(0); // rendezvous
        let (spare_tx, spare_rx) = mpsc::channel();
        let writer = thread::Builder::new()
            .name("spi-writer".into())
            .spawn(move || writer_loop(spi, dc, rx, spare_tx))?;
        let p = Self { tx: Some(tx), spare_rx, last: None, writer: Some(writer) };
        // U8g2 "TopWin" SSD1322 240x128 init sequence.
        for (cmd, data) in [
            (0xFD, &[0x12][..]),
            (0xAE, &[]),
            (0xB3, &[0xF1]),
            (0xCA, &[0x7F]),          // 128 rows
            (0xA2, &[0x00]),
            (0xA1, &[0x00]),
            (0xA0, &[0x36, 0x01]),
            (0xAB, &[0x01]),
            (0xB4, &[0xA2, 0xFD]),
            (0xC1, &[0xFF]),
            (0xC7, &[0x0F]),
            (0xB1, &[0xF7]),
            (0xD1, &[0xA2, 0x20]),
            (0xBB, &[0x1F]),
            (0xB6, &[0x0F]),
            (0xBE, &[0x07]),
            (0xA6, &[]),
            (0xA9, &[]),               // exit partial display
            (0xAF, &[]),
        ] {
            p.command(cmd, data);
        }
        Ok(p)
    }

    fn command(&self, cmd: u8, data: &[u8]) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(Msg::Command { cmd, data: data.to_vec() });
        }
    }
}

impl Panel for Ssd1322 {
    fn present(&mut self, cv: &Canvas) {
        let mut packed = self.spare_rx.try_recv().unwrap_or_default();
        packed.clear();
        packed.extend(cv.buf.chunks_exact(2).map(|px| ((px[0] & 0x0F) << 4) | (px[1] & 0x0F)));
        // Frame unchanged: skip the ~6 ms SPI transfer entirely.
        if self.last.as_ref() == Some(&packed) {
            return;
        }
        match &mut self.last {
            Some(last) => last.clone_from(&packed),
            None => self.last = Some(packed.clone()),
        }
        if let Some(tx) = &self.tx {
            let _ = tx.send(Msg::Frame(packed));
        }
    }

    fn set_on(&mut self, on: bool) {
        self.command(if on { 0xAF } else { 0xAE }, &[]);
    }

    fn invalidate(&mut self) {
        self.last = None;
    }

    fn finish(&mut self) {
        self.tx = None; // writer drains and exits
        if let Some(w) = self.writer.take() {
            let _ = w.join();
        }
    }
}

impl Drop for Ssd1322 {
    fn drop(&mut self) {
        self.finish();
    }
}

fn writer_loop(mut spi: Spidev, dc: OutputLine, rx: Receiver<Msg>, spare_tx: Sender<Vec<u8>>) {
    let chunk = hw::spi_chunk_size();
    for msg in rx {
        let _bus = SPI_BUS.lock().unwrap_or_else(|e| e.into_inner());
        match msg {
            Msg::Command { cmd, data } => send_command(&mut spi, &dc, cmd, &data),
            Msg::Frame(frame) => {
                send_command(&mut spi, &dc, 0x15, &[0x18, 0x53]); // columns
                send_command(&mut spi, &dc, 0x75, &[0x00, 0x7F]); // rows
                send_command(&mut spi, &dc, 0x5C, &[]);           // write RAM
                dc.set(true);
                for part in frame.chunks(chunk) {
                    let _ = spi.write_all(part);
                }
                let _ = spare_tx.send(frame); // recycle the buffer
            }
        }
    }
}

fn send_command(spi: &mut Spidev, dc: &OutputLine, cmd: u8, data: &[u8]) {
    dc.set(false);
    let _ = spi.write_all(&[cmd]);
    if !data.is_empty() {
        dc.set(true);
        let _ = spi.write_all(data);
    }
}
