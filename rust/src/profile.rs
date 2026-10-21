//! Display profiles: everything that differs between the two builds of the
//! front panel (SSD1309 128x64 mono, e.g. on a Pi 3; SSD1322 240x128
//! greyscale, e.g. on a Pi 5). The rest of the program is shared.
//!
//! The profile is chosen at startup: RAV_DISPLAY in the environment, else
//! `display = ...` in /etc/rav-audio/visualizer.conf, else SSD1309.

use crate::bands::{
    BANDS64, BANDS64_BASS_END, BANDS64_MID_END, BANDS120, BANDS120_BASS_END, BANDS120_MID_END, FreqBand,
};
use crate::display::Canvas;
use crate::font::Fonts;
use crate::mpd::MpdClient;
use crate::viz::{self, Viz};
use anyhow::{Result, bail};
use std::sync::Arc;

pub const CONFIG_PATH: &str = "/etc/rav-audio/visualizer.conf";

// Chip-select is handled by spidev: left panel on CE0 (GPIO 8), right on CE1
// (GPIO 7).
pub const LEFT_SPI: &str = "/dev/spidev0.0";
pub const RIGHT_SPI: &str = "/dev/spidev0.1";

/// The 40-pin header's GPIO controller, found by label rather than by chip
/// number: Pi 3 (bcm2835), Pi 4 (bcm2711), Pi 5 (rp1).
pub const GPIO_CHIP_LABELS: [&str; 3] = ["pinctrl-bcm2835", "pinctrl-bcm2711", "pinctrl-rp1"];

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PanelKind {
    Ssd1309,
    Ssd1322,
}

/// BCM GPIO numbers.
pub struct Pins {
    pub left_dc: u32,
    pub left_rst: u32,
    pub right_dc: u32,
    pub right_rst: u32,
    pub viz: u32,
    pub play: u32,
    pub fw: u32,
    pub rw: u32,
    pub led: u32,
    pub rot_clk: u32,
    pub rot_dt: u32,
    pub rot_sw: u32,
}

pub struct Features {
    /// "Matrix rain" when switching visualizations.
    pub transition: bool,
    /// Holding PLAY opens the settings menu (network, Bluetooth, restart,
    /// power off); PLAY then acts on release.
    pub settings: bool,
    /// Holding VIZ tours the hidden visualizations (VIZ then acts on
    /// release instead of press).
    pub hidden_tier: bool,
}

pub struct Profile {
    pub name: &'static str,
    pub panel: PanelKind,
    pub width: usize,
    pub height: usize,
    pub spi_hz: u32,
    pub pins: Pins,
    /// Tried in order; relative paths are looked up in the working directory.
    pub font_paths: &'static [&'static str],
    /// Pixel heights of the Small, Regular and Large font sizes.
    pub font_px: [u32; 3],
    /// Bass FFT size: 8192 for fine bass bins, 4096 for half the latency.
    pub fft_bass: usize,
    /// The large spectrum's bands, split by FFT: [0, bass_end) from the
    /// bass FFT, [bass_end, mid_end) mid, the rest treble.
    pub bands: &'static [FreqBand],
    pub bands_bass_end: usize,
    pub bands_mid_end: usize,
    /// Detent sign that turns the volume up (encoder wiring differs).
    pub knob_up: i32,
    pub features: Features,
    pub vizs: fn(Arc<MpdClient>) -> Vec<Box<dyn Viz>>,
    pub volume_overlay: fn(&mut Canvas, &Fonts, i32),
}

pub static SSD1309: Profile = Profile {
    name: "ssd1309",
    panel: PanelKind::Ssd1309,
    width: 128,
    height: 64,
    // What the bcm2835 build ran at (250 MHz / 40), stable on this wiring
    // without a decoupling capacitor. A frame is 1 KB, ~1.3 ms per panel.
    spi_hz: 6_250_000,
    pins: Pins {
        left_dc: 25,
        left_rst: 24,
        right_dc: 23,
        right_rst: 22,
        viz: 27,
        play: 26,
        // Was GPIO 9, which is SPI0 MISO and kept by the kernel for the
        // display bus: rewired to GPIO 4 (physical pin 7) like the Pi 5 box.
        fw: 4,
        rw: 6,
        led: 16,
        rot_clk: 17,
        rot_dt: 5,
        rot_sw: 13,
    },
    font_paths: &[
        "trixel-square.ttf",
        "/usr/share/rav-audio/trixel-square.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ],
    font_px: [8, 10, 14],
    fft_bass: 4096,
    bands: &BANDS64,
    bands_bass_end: BANDS64_BASS_END,
    bands_mid_end: BANDS64_MID_END,
    knob_up: 1,
    features: Features { transition: true, settings: true, hidden_tier: false },
    vizs: viz::ssd1309::all,
    volume_overlay: viz::ssd1309::volume_overlay,
};

pub static SSD1322: Profile = Profile {
    name: "ssd1322",
    panel: PanelKind::Ssd1322,
    width: 240,
    height: 128,
    spi_hz: 20_000_000,
    pins: Pins {
        left_dc: 25,
        left_rst: 24,
        right_dc: 23,
        right_rst: 22,
        viz: 27,
        play: 26,
        // GPIO 9 is SPI0 MISO, claimed by the RP1 pinctrl since kernel
        // 6.18: the FW button moved to GPIO 4 (physical pin 7).
        fw: 4,
        rw: 6,
        led: 16,
        rot_clk: 17,
        rot_dt: 5,
        rot_sw: 13,
    },
    font_paths: &[
        "FreePixel.ttf",
        // Not shipped in the package (licence): installed by hand.
        "/usr/local/share/rav-audio/FreePixel.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ],
    font_px: [8, 16, 24],
    fft_bass: 8192,
    bands: &BANDS120,
    bands_bass_end: BANDS120_BASS_END,
    bands_mid_end: BANDS120_MID_END,
    knob_up: -1, // clockwise counts negative on this encoder
    features: Features { transition: true, settings: true, hidden_tier: true },
    vizs: viz::ssd1322::all,
    volume_overlay: viz::ssd1322::volume_bar,
};

pub fn by_name(name: &str) -> Result<&'static Profile> {
    match name.trim().to_ascii_lowercase().as_str() {
        "ssd1309" => Ok(&SSD1309),
        "ssd1322" => Ok(&SSD1322),
        other => bail!("unknown display {other:?} (expected ssd1309 or ssd1322)"),
    }
}

/// `display = ...` from the config file, ignoring comments and other keys.
fn configured_display() -> Option<String> {
    let text = std::fs::read_to_string(CONFIG_PATH).ok()?;
    text.lines()
        .map(|l| l.split('#').next().unwrap_or(""))
        .filter_map(|l| l.split_once('='))
        .find(|(k, _)| k.trim() == "display")
        .map(|(_, v)| v.trim().to_string())
}

pub fn load() -> Result<&'static Profile> {
    match std::env::var("RAV_DISPLAY").ok().or_else(configured_display) {
        Some(name) => by_name(&name),
        None => Ok(&SSD1309),
    }
}
