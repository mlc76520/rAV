//! Hardware access: GPIO through the kernel character device (gpiocdev),
//! SPI through spidev. Both work the same on the Pi 3 and the Pi 5, so there
//! is no per-board backend.

use crate::profile::{GPIO_CHIP_LABELS, Pins};
use anyhow::{Context, Result, bail};
use gpiocdev::Request;
use gpiocdev::line::{Bias, Value};
use spidev::{SpiModeFlags, Spidev, SpidevOptions};
use std::path::PathBuf;

/// The header GPIO chip, looked up by label: its number differs between
/// boards and has changed across kernel updates.
pub fn find_gpio_chip() -> Result<(PathBuf, &'static str)> {
    for path in gpiocdev::chip::chips().context("listing GPIO chips")? {
        let Ok(chip) = gpiocdev::Chip::from_path(&path) else { continue };
        let Ok(info) = chip.info() else { continue };
        if let Some(label) = GPIO_CHIP_LABELS.iter().find(|l| info.label == **l) {
            return Ok((path, label));
        }
    }
    bail!("no GPIO chip labelled {:?}", GPIO_CHIP_LABELS)
}

/// One output line.
pub struct OutputLine {
    req: Request,
    offset: u32,
}

impl OutputLine {
    pub fn new(chip: &PathBuf, offset: u32, high: bool) -> Result<Self> {
        let req = Request::builder()
            .on_chip(chip)
            .with_consumer("rav")
            .with_line(offset)
            .as_output(if high { Value::Active } else { Value::Inactive })
            .request()
            .with_context(|| format!("requesting GPIO {offset} as output"))?;
        Ok(Self { req, offset })
    }

    pub fn set(&self, high: bool) {
        let _ = self.req.set_value(self.offset, if high { Value::Active } else { Value::Inactive });
    }
}

/// The four front-panel push buttons (pull-up, pressed = low). Each line is
/// requested on its own, so a pin the kernel keeps for something else
/// (e.g. GPIO 9, SPI0 MISO) only disables that button instead of the whole
/// program.
pub struct Buttons {
    lines: Vec<(u32, Request)>,
    pins: [u32; 4], // viz, play, fw, rw
}

#[derive(Clone, Copy, Default)]
pub struct ButtonState {
    pub viz: bool,
    pub play: bool,
    pub fw: bool,
    pub rw: bool,
}

impl Buttons {
    pub fn new(chip: &PathBuf, pins: &Pins) -> Self {
        let pins = [pins.viz, pins.play, pins.fw, pins.rw];
        let lines = pins
            .into_iter()
            .filter_map(|offset| {
                Request::builder()
                    .on_chip(chip)
                    .with_consumer("rav")
                    .with_line(offset)
                    .as_input()
                    .with_bias(Bias::PullUp)
                    .request()
                    .map_err(|e| println!("Button on GPIO {offset} unavailable: {e}"))
                    .ok()
                    .map(|req| (offset, req))
            })
            .collect();
        Self { lines, pins }
    }

    pub fn read(&self) -> ButtonState {
        let pressed = |o| {
            self.lines.iter().find(|(l, _)| *l == o)
                .is_some_and(|(l, req)| req.value(*l).is_ok_and(|v| v == Value::Inactive))
        };
        let [viz, play, fw, rw] = self.pins;
        ButtonState { viz: pressed(viz), play: pressed(play), fw: pressed(fw), rw: pressed(rw) }
    }
}

pub fn open_spi(path: &str, speed_hz: u32) -> Result<Spidev> {
    let mut spi = Spidev::open(path).with_context(|| format!("opening {path}"))?;
    spi.configure(
        &SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(speed_hz)
            .mode(SpiModeFlags::SPI_MODE_0)
            .build(),
    )
    .with_context(|| format!("configuring {path}"))?;
    Ok(spi)
}

/// Max bytes per spidev write, from the kernel module parameter (4096 by
/// default; spidev.bufsiz=65536 on the cmdline sends a frame in one go).
pub fn spi_chunk_size() -> usize {
    std::fs::read_to_string("/sys/module/spidev/parameters/bufsiz")
        .ok()
        .and_then(|s| s.trim().parse::<usize>().ok())
        .filter(|&v| v >= 4096)
        .unwrap_or(4096)
}
