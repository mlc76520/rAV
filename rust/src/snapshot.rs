//! Data shared with the render loop.

use std::sync::{Arc, Mutex};
use std::time::Instant;

pub const BANDS_SMALL: usize = 7;

/// Immutable "now playing" state. Producers (MPD / Bluetooth threads) build
/// a new one on change and publish it; the render thread only clones the
/// Arc, so it never waits on a producer's I/O.
#[derive(Clone)]
pub struct NowPlaying {
    pub formatted_text: String,
    pub track_number: String, // queue position + 1 (MPD)
    pub title: String,
    pub artist: String, // empty when unknown
    pub album: String,  // empty when unknown
    pub date: String,   // year, or empty
    pub bitrate: String,
    pub format: String,
    pub elapsed_ms: u32, // as of status_ts
    pub total_s: u32,
    pub is_playing: bool,
    pub status_ts: Instant,
}

impl Default for NowPlaying {
    fn default() -> Self {
        Self {
            formatted_text: "Waiting for MPD...".into(),
            track_number: String::new(),
            title: "No song playing".into(),
            artist: String::new(),
            album: String::new(),
            date: String::new(),
            bitrate: String::new(),
            format: String::new(),
            elapsed_ms: 0,
            total_s: 0,
            is_playing: false,
            status_ts: Instant::now(),
        }
    }
}

impl NowPlaying {
    /// Elapsed seconds extrapolated to `now` from the last status sample,
    /// so the clock never drifts and needs no per-frame bookkeeping.
    pub fn elapsed_at(&self, now: Instant) -> u32 {
        let mut ms = self.elapsed_ms as u64;
        if self.is_playing {
            ms += now.saturating_duration_since(self.status_ts).as_millis() as u64;
        }
        let s = (ms / 1000) as u32;
        if self.total_s > 0 { s.min(self.total_s) } else { s }
    }
}

/// Lock-light publication slot (the lock is held only to swap/clone an Arc).
#[derive(Default)]
pub struct Published(Mutex<Arc<NowPlaying>>);

impl Published {
    pub fn get(&self) -> Arc<NowPlaying> {
        self.0.lock().unwrap_or_else(|e| e.into_inner()).clone()
    }
    pub fn set(&self, np: NowPlaying) {
        *self.0.lock().unwrap_or_else(|e| e.into_inner()) = Arc::new(np);
    }
}

/// Everything a visualization needs for one frame, sampled once.
pub struct FrameSnapshot {
    pub now: Instant,
    pub dt: f32, // seconds since previous frame, clamped to [0, 0.1]
    /// A phone is streaming over A2DP: its track replaces MPD's.
    pub bt_streaming: bool,
    pub track: Arc<NowPlaying>,
    pub elapsed_s: u32,
    pub total_s: u32,
    /// One sample per screen column.
    pub waveform_l: Vec<f32>,
    pub waveform_r: Vec<f32>,
    pub spectrum_l: [i32; BANDS_SMALL],
    pub spectrum_r: [i32; BANDS_SMALL],
    /// The profile's large band table (64 or 120 bands).
    pub spectrum_large_l: Vec<i32>,
    pub spectrum_large_r: Vec<i32>,
    pub vu_l: i32,
    pub vu_r: i32,
}

impl FrameSnapshot {
    pub fn new(waveform_len: usize, large_bands: usize) -> Self {
        Self {
            now: Instant::now(),
            dt: 0.016,
            bt_streaming: false,
            track: Arc::new(NowPlaying::default()),
            elapsed_s: 0,
            total_s: 0,
            waveform_l: vec![0.0; waveform_len],
            waveform_r: vec![0.0; waveform_len],
            spectrum_l: [0; BANDS_SMALL],
            spectrum_r: [0; BANDS_SMALL],
            spectrum_large_l: vec![0; large_bands],
            spectrum_large_r: vec![0; large_bands],
            vu_l: 0,
            vu_r: 0,
        }
    }
}
