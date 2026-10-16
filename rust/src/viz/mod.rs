//! Visualizations: the shared trait and helpers, then one family per panel
//! (each drawn for its own resolution, chosen by the profile).

use crate::display::Canvas;
use crate::font::Fonts;
use crate::snapshot::FrameSnapshot;

pub mod editor;
pub mod ssd1309;
pub mod ssd1322;

/// The two panels and the fonts, for one frame.
pub struct Frame<'a> {
    pub left: &'a mut Canvas,
    pub right: &'a mut Canvas,
    pub fonts: &'a Fonts,
}

/// Where a visualization sits in the VIZ button cycle.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Tier {
    Normal,
    /// Reached only by holding VIZ; skipped by the cycle and auto-cycle.
    Hidden,
    /// The playlist editor: opened with the encoder button.
    Editor,
}

pub trait Viz {
    fn name(&self) -> &'static str;
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot);
    fn tier(&self) -> Tier {
        Tier::Normal
    }
    /// Only the playlist editor returns Some.
    fn as_editor(&mut self) -> Option<&mut editor::PlaylistEditor> {
        None
    }
}

/// Bass-transient detector shared by several visualizations: fires when
/// the bass jumps well above its own smoothed average, then `level` decays
/// from 1 to 0 at `decay` per second (no re-trigger above 0.3).
#[derive(Clone, Copy)]
pub struct Beat {
    env: f32,
    pub level: f32,
}

impl Default for Beat {
    fn default() -> Self {
        Self { env: 0.05, level: 0.0 }
    }
}

impl Beat {
    pub fn update(&mut self, bass: f32, dt: f32, decay: f32) -> bool {
        self.env += (bass - self.env) * (dt * 1.5).min(1.0);
        let fired = bass > 0.22 && bass > self.env * 1.45 && self.level < 0.3;
        if fired {
            self.level = 1.0;
        }
        self.level = (self.level - dt * decay).max(0.0);
        fired
    }
}

/// Spectrum levels as 0..1 floats.
pub fn norm7(spec: &[i32; 7]) -> [f32; 7] {
    spec.map(|v| v as f32 / 255.0)
}

/// Small xorshift32 RNG -> [0, 1).
pub struct Rng(u32);

impl Rng {
    pub fn new(seed: u32) -> Self {
        Self(seed)
    }
    pub fn next(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        (self.0 >> 8) as f32 * (1.0 / 16_777_216.0)
    }
}
