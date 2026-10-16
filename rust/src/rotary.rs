//! Rotary encoder (quadrature) + its push button, from kernel edge events.
//!
//! Each event carries the line and the edge, so the decoder always sees the
//! exact level sequence, even when events arrive in bursts on fast spins.

use crate::profile::Pins;
use anyhow::{Context, Result};
use gpiocdev::Request;
use gpiocdev::line::{Bias, EdgeDetection, EdgeKind, Value, Values};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

// Quadrature transition table: index = (previous state << 2) | new state.
const TRANS: [i8; 16] = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0];
const DETENT: i32 = 4; // transitions per click
const BUTTON_DEBOUNCE_NS: u64 = 50_000_000;

pub struct Rotary {
    rotation: Arc<AtomicI32>,
    pressed: Arc<AtomicBool>,
    running: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Rotary {
    pub fn start(chip: &PathBuf, pins: &Pins) -> Result<Self> {
        let (clk, dt, sw) = (pins.rot_clk, pins.rot_dt, pins.rot_sw);
        let req = Request::builder()
            .on_chip(chip)
            .with_consumer("rav-encoder")
            .with_lines(&[clk, dt])
            .as_input()
            .with_bias(Bias::PullUp)
            .with_edge_detection(EdgeDetection::BothEdges)
            .with_debounce_period(Duration::from_micros(5))
            .with_line(sw)
            .as_input()
            .with_bias(Bias::PullUp)
            .with_edge_detection(EdgeDetection::BothEdges)
            .with_debounce_period(Duration::from_millis(5))
            .request()
            .context("requesting encoder lines")?;

        let mut values = Values::from_offsets(&[clk, dt]);
        req.values(&mut values).context("reading encoder")?;
        let level = |o| (values.get(o) == Some(Value::Active)) as u8;
        let mut state = (level(clk) << 1) | level(dt);

        let (rotation, pressed, running) =
            (Arc::new(AtomicI32::new(0)), Arc::new(AtomicBool::new(false)), Arc::new(AtomicBool::new(true)));
        let (rot, btn, run) = (Arc::clone(&rotation), Arc::clone(&pressed), Arc::clone(&running));
        let thread = thread::Builder::new().name("encoder".into()).spawn(move || {
            let mut acc = 0i32;
            let mut last_press_ns = 0u64;
            while run.load(Ordering::Relaxed) {
                if !req.wait_edge_event(Duration::from_millis(100)).unwrap_or(false) {
                    continue;
                }
                let Ok(ev) = req.read_edge_event() else { continue };
                let high = ev.kind == EdgeKind::Rising;
                if ev.offset == sw {
                    // Pressed = falling edge; software debounce on top.
                    if !high && ev.timestamp_ns.saturating_sub(last_press_ns) > BUTTON_DEBOUNCE_NS {
                        last_press_ns = ev.timestamp_ns;
                        btn.store(true, Ordering::Relaxed);
                    }
                    continue;
                }
                let cur = if ev.offset == clk {
                    (state & 0b01) | ((high as u8) << 1)
                } else {
                    (state & 0b10) | high as u8
                };
                acc += TRANS[((state << 2) | cur) as usize] as i32;
                state = cur;
                if acc >= DETENT {
                    acc -= DETENT;
                    rot.fetch_add(1, Ordering::Relaxed);
                } else if acc <= -DETENT {
                    acc += DETENT;
                    rot.fetch_sub(1, Ordering::Relaxed);
                }
            }
        })?;
        Ok(Self { rotation, pressed, running, thread: Some(thread) })
    }

    /// Detents since the last call. Which sign is clockwise depends on the
    /// wiring: see Profile::knob_up.
    pub fn take_rotation(&self) -> i32 {
        self.rotation.swap(0, Ordering::Relaxed)
    }

    pub fn take_press(&self) -> bool {
        self.pressed.swap(false, Ordering::Relaxed)
    }
}

impl Drop for Rotary {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}
