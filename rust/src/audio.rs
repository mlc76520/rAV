//! Audio capture and analysis.
//!
//! Capture: a PipeWire stream on the default sink's monitor (passive, so
//! the output can still suspend). The monitor delivers each sample when the
//! graph hands it to the DAC, which then holds it for its device buffer
//! before it is audible. That look-ahead, measured live from /proc/asound,
//! lets the analysis windows be CENTRED on the sample that will be heard
//! when the frame lights up. With a short device buffer (the rav-audio
//! default) there is little or none, and the windows simply end at the
//! newest audio; a larger api.alsa.headroom buys exact sync.
//!
//! Analysis: left+right packed into ONE complex FFT per size (real-pair
//! trick), per-bin power shared by the 7- and 64-band views, FFTs rerun
//! only when new audio arrived or the audible point moved.

use crate::bands::{BANDS7, BANDS7_BASS_END, BANDS7_MID_END, FreqBand};
use crate::profile::Profile;
use crate::snapshot::{BANDS_SMALL, FrameSnapshot};
use pipewire as pw;
use pw::spa;
use pw::spa::pod::Pod;
use rustfft::num_complex::Complex32;
use rustfft::{Fft, FftPlanner};
use std::cell::Cell;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const SAMPLE_RATE: usize = 44100; // requested from PipeWire, which converts
// The bass FFT size comes from the profile (Profile::fft_bass).
const FFT_MID: usize = 2048;
const FFT_TREBLE: usize = 512;
const SILENCE_THRESHOLD: f32 = 0.001;
const SLEEP_TIMEOUT: Duration = Duration::from_secs(10);
// Holds the look-ahead (~10k samples) plus a full bass window.
const RING_SIZE: usize = 32768; // power of two (~743 ms)
const RING_MASK: usize = RING_SIZE - 1;
// Time from render to photons: rest of this frame + SPI transfer.
const DISPLAY_LATENCY_S: f32 = 0.020;
const NOMINAL_FPS: f32 = 60.0; // smoothing runs per frame

struct Ring {
    left: Vec<f32>,
    right: Vec<f32>,
    write_pos: usize,
    // When the newest sample arrived and how big its chunk was: the time
    // reference that maps ring positions to the instants they are audible.
    last_chunk_time: Instant,
    last_chunk_frames: usize,
}

/// State shared between the PipeWire thread and the render thread.
struct Shared {
    ring: Mutex<Ring>,
    running: AtomicBool,
    out_latency: AtomicU32, // f32 bits: DAC delay in s, < 0 = unknown
    epoch: Instant,
    last_audio_ms: AtomicU64, // since epoch
}

impl Shared {
    fn latency(&self) -> f32 {
        f32::from_bits(self.out_latency.load(Ordering::Relaxed))
    }
    fn mark_audio(&self) {
        self.last_audio_ms.store(self.epoch.elapsed().as_millis() as u64, Ordering::Relaxed);
    }
}

pub struct Audio {
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,

    fft_bass: Arc<dyn Fft<f32>>,
    fft_mid: Arc<dyn Fft<f32>>,
    fft_treble: Arc<dyn Fft<f32>>,
    fft_buf: Vec<Complex32>,
    scratch: Vec<Complex32>,
    window_bass: Vec<f32>,
    window_mid: Vec<f32>,
    window_treble: Vec<f32>,
    temp_l: Vec<f32>,
    temp_r: Vec<f32>,
    // Per-bin |L|^2 / |R|^2, shared by both views.
    power: [(Vec<f32>, Vec<f32>); 3], // bass, mid, treble

    n_bass: usize,
    large: &'static [FreqBand], // the profile's large band table
    large_bass_end: usize,
    large_mid_end: usize,

    last_fft_pos: usize,
    last_fft_lead: usize,
    bands7: Option<(Vec<f32>, Vec<f32>)>,
    bands_large: Option<(Vec<f32>, Vec<f32>)>,
    prev7: ([f32; BANDS_SMALL], [f32; BANDS_SMALL]),
    prev_large: (Vec<f32>, Vec<f32>),

    integral: f32,
    gravity: f32,
    scale: f32,
}

fn hann(n: usize) -> Vec<f32> {
    (0..n).map(|i| 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / (n - 1) as f32).cos())).collect()
}

impl Audio {
    pub fn new(profile: &Profile) -> Self {
        let n_bass = profile.fft_bass;
        let n_large = profile.bands.len();
        let mut planner = FftPlanner::<f32>::new();
        let (fb, fm, ft) = (planner.plan_fft_forward(n_bass), planner.plan_fft_forward(FFT_MID),
                            planner.plan_fft_forward(FFT_TREBLE));
        let scratch_len = [&fb, &fm, &ft].iter().map(|f| f.get_inplace_scratch_len()).max().unwrap_or(0);
        let nr: f32 = 0.77; // noise reduction 77 %
        let sensitivity: f32 = 1.0;
        let epoch = Instant::now();
        Self {
            shared: Arc::new(Shared {
                ring: Mutex::new(Ring {
                    left: vec![0.0; RING_SIZE],
                    right: vec![0.0; RING_SIZE],
                    write_pos: 0,
                    last_chunk_time: epoch,
                    last_chunk_frames: 0,
                }),
                running: AtomicBool::new(false),
                out_latency: AtomicU32::new((-1.0f32).to_bits()),
                epoch,
                last_audio_ms: AtomicU64::new(0),
            }),
            thread: None,
            fft_bass: fb,
            fft_mid: fm,
            fft_treble: ft,
            fft_buf: vec![Complex32::default(); n_bass],
            scratch: vec![Complex32::default(); scratch_len],
            window_bass: hann(n_bass),
            window_mid: hann(FFT_MID),
            window_treble: hann(FFT_TREBLE),
            temp_l: vec![0.0; n_bass],
            temp_r: vec![0.0; n_bass],
            power: [
                (vec![0.0; n_bass / 2 + 1], vec![0.0; n_bass / 2 + 1]),
                (vec![0.0; FFT_MID / 2 + 1], vec![0.0; FFT_MID / 2 + 1]),
                (vec![0.0; FFT_TREBLE / 2 + 1], vec![0.0; FFT_TREBLE / 2 + 1]),
            ],
            n_bass,
            large: profile.bands,
            large_bass_end: profile.bands_bass_end,
            large_mid_end: profile.bands_mid_end,
            last_fft_pos: usize::MAX,
            last_fft_lead: usize::MAX,
            bands7: None,
            bands_large: None,
            prev7: ([0.0; BANDS_SMALL], [0.0; BANDS_SMALL]),
            prev_large: (vec![0.0; n_large], vec![0.0; n_large]),
            integral: nr * 0.95,
            gravity: (1.0 - nr * 0.8).max(0.2),
            scale: sensitivity * 1.8,
        }
    }

    /// Never fails: the capture thread (re)connects to PipeWire by itself,
    /// so the visualizer may start before the daemon.
    pub fn start(&mut self) {
        if self.shared.running.swap(true, Ordering::SeqCst) {
            return;
        }
        let shared = Arc::clone(&self.shared);
        self.thread = Some(thread::Builder::new().name("pipewire".into())
            .spawn(move || capture_thread(shared)).expect("spawn capture thread"));
    }

    pub fn stop(&mut self) {
        self.shared.running.store(false, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }

    pub fn reset_sleep_timer(&self) {
        self.shared.mark_audio();
    }

    pub fn has_audio(&self) -> bool {
        let now = self.shared.epoch.elapsed().as_millis() as u64;
        now.saturating_sub(self.shared.last_audio_ms.load(Ordering::Relaxed)) < SLEEP_TIMEOUT.as_millis() as u64
    }

    // ---- analysis (render thread) ----

    /// How many samples behind the newest one lies the sample that will be
    /// audible when this frame reaches the panel, moved further ahead by the
    /// smoothing's mean lag so the smoothed bars land on the beat too.
    /// 0 when no look-ahead is available.
    fn audible_lead(&self, ring: &Ring, now: Instant) -> usize {
        let latency = self.shared.latency();
        if latency < 0.0 || ring.last_chunk_frames == 0 {
            return 0;
        }
        // The newest sample closes the quantum just handed to the DAC: it
        // plays after the measured (mean) delay + half a quantum.
        let newest_audible_in = -now.saturating_duration_since(ring.last_chunk_time).as_secs_f32()
            + latency + 0.5 * ring.last_chunk_frames as f32 / SAMPLE_RATE as f32;
        let smooth_lag = (self.integral / (1.0 - self.integral) / NOMINAL_FPS).min(0.1);
        let lead_s = newest_audible_in - DISPLAY_LATENCY_S - smooth_lag;
        if lead_s <= 0.0 {
            return 0;
        }
        ((lead_s * SAMPLE_RATE as f32) as usize).min(RING_SIZE - self.n_bass - 8192) // writer headroom
    }

    /// n samples per channel whose newest lies `end_behind` samples behind
    /// the newest written one.
    fn copy_window(ring: &Ring, end_behind: usize, out_l: &mut [f32], out_r: &mut [f32]) {
        let n = out_l.len();
        let start = (ring.write_pos + 2 * RING_SIZE - end_behind - n) & RING_MASK;
        let first = n.min(RING_SIZE - start);
        out_l[..first].copy_from_slice(&ring.left[start..start + first]);
        out_r[..first].copy_from_slice(&ring.right[start..start + first]);
        out_l[first..].copy_from_slice(&ring.left[..n - first]);
        out_r[first..].copy_from_slice(&ring.right[..n - first]);
    }

    /// One complex FFT of (L + iR); per-channel power via hermitian symmetry:
    /// L[k] = (X[k]+X*[N-k])/2, R[k] = (X[k]-X*[N-k])/2i.
    fn pair_fft(&mut self, which: usize, offset: usize) {
        let (fft, window) = match which {
            0 => (&self.fft_bass, &self.window_bass),
            1 => (&self.fft_mid, &self.window_mid),
            _ => (&self.fft_treble, &self.window_treble),
        };
        let n = window.len();
        let buf = &mut self.fft_buf[..n];
        for i in 0..n {
            buf[i] = Complex32::new(self.temp_l[offset + i] * window[i], self.temp_r[offset + i] * window[i]);
        }
        fft.process_with_scratch(buf, &mut self.scratch);
        let (pl, pr) = &mut self.power[which];
        pl[0] = buf[0].re * buf[0].re;
        pr[0] = buf[0].im * buf[0].im;
        for j in 1..=n / 2 {
            let (a, b) = (buf[j], buf[n - j]);
            let (lr, li) = (0.5 * (a.re + b.re), 0.5 * (a.im - b.im));
            let (rr, ri) = (0.5 * (a.im + b.im), 0.5 * (b.re - a.re));
            pl[j] = lr * lr + li * li;
            pr[j] = rr * rr + ri * ri;
        }
    }

    /// temp_* holds the bass window centred on the audible sample (clamped
    /// to the newest audio when the look-ahead is short); each FFT takes the
    /// sub-window of its own size centred there, clamped the same way.
    fn run_ffts(&mut self, lead: usize) {
        let end_behind = |n: usize| lead.saturating_sub(n / 2);
        let n_bass = self.n_bass;
        let bass_end = end_behind(n_bass);
        let offset = |n: usize| n_bass - n - (end_behind(n) - bass_end);
        self.pair_fft(0, offset(n_bass));
        self.pair_fft(1, offset(FFT_MID));
        self.pair_fft(2, offset(FFT_TREBLE));
        self.bands7 = None;
        self.bands_large = None;
    }

    fn aggregate(&self, table: &[FreqBand], bass_end: usize, mid_end: usize) -> (Vec<f32>, Vec<f32>) {
        // The correction tables were tuned with an 8192-point bass FFT. A
        // shorter window shrinks steady tones (x N) more than transients, so
        // sqrt of the ratio keeps both close to their calibrated heights.
        let bass_gain = (8192.0 / self.n_bass as f32).sqrt();
        let (mut out_l, mut out_r) = (vec![0.0; table.len()], vec![0.0; table.len()]);
        for (i, band) in table.iter().enumerate() {
            let (which, n) = if i < bass_end { (0, self.n_bass) } else if i < mid_end { (1, FFT_MID) } else { (2, FFT_TREBLE) };
            let (pl, pr) = &self.power[which];
            let lo = band.low as usize * n / SAMPLE_RATE;
            let mut hi = band.high as usize * n / SAMPLE_RATE;
            if hi <= lo {
                hi = lo + 1;
            }
            let (mut sl, mut sr) = (0.0, 0.0);
            for j in lo..hi.min(n / 2) {
                sl += pl[j];
                sr += pr[j];
            }
            let k = self.scale * band.correction * if which == 0 { bass_gain } else { 1.0 };
            out_l[i] = (sl / (hi - lo) as f32).sqrt() * k;
            out_r[i] = (sr / (hi - lo) as f32).sqrt() * k;
        }
        (out_l, out_r)
    }

    fn smooth(integral: f32, gravity: f32, prev: &mut [f32], bands: &[f32], out: &mut [i32]) {
        for i in 0..out.len() {
            let smoothed = integral * prev[i] + (1.0 - integral) * bands[i];
            if smoothed < prev[i] {
                prev[i] = (prev[i] - (prev[i] - smoothed) * gravity).max(smoothed);
            } else {
                prev[i] = smoothed;
            }
            out[i] = (prev[i] as i32).clamp(0, 255);
        }
    }

    /// Fills the per-frame snapshot from the audio that will be AUDIBLE
    /// when this frame lights up. Waveform and FFT input are copied under
    /// one lock with one time reference; smoothing advances once per frame.
    pub fn fill_snapshot(&mut self, s: &mut FrameSnapshot) {
        let (lead, refresh) = {
            let ring = self.shared.ring.lock().unwrap();
            let lead = self.audible_lead(&ring, s.now);
            let half = s.waveform_l.len() / 2;
            Self::copy_window(&ring, lead.saturating_sub(half), &mut s.waveform_l, &mut s.waveform_r);
            let refresh = ring.write_pos != self.last_fft_pos || lead.abs_diff(self.last_fft_lead) >= 64;
            if refresh {
                self.last_fft_pos = ring.write_pos;
                self.last_fft_lead = lead;
                Self::copy_window(&ring, lead.saturating_sub(self.n_bass / 2), &mut self.temp_l, &mut self.temp_r);
            }
            (lead, refresh)
        };
        if refresh {
            self.run_ffts(lead); // outside the lock
        }
        if self.bands7.is_none() {
            self.bands7 = Some(self.aggregate(&BANDS7, BANDS7_BASS_END, BANDS7_MID_END));
        }
        if self.bands_large.is_none() {
            self.bands_large = Some(self.aggregate(self.large, self.large_bass_end, self.large_mid_end));
        }
        let (a, g) = (self.integral, self.gravity);
        let (b7l, b7r) = self.bands7.as_ref().unwrap();
        Self::smooth(a, g, &mut self.prev7.0, b7l, &mut s.spectrum_l);
        Self::smooth(a, g, &mut self.prev7.1, b7r, &mut s.spectrum_r);
        let (bll, blr) = self.bands_large.as_ref().unwrap();
        Self::smooth(a, g, &mut self.prev_large.0, bll, &mut s.spectrum_large_l);
        Self::smooth(a, g, &mut self.prev_large.1, blr, &mut s.spectrum_large_r);
        s.vu_l = s.spectrum_l.iter().sum::<i32>() / BANDS_SMALL as i32;
        s.vu_r = s.spectrum_r.iter().sum::<i32>() / BANDS_SMALL as i32;
    }
}

impl Drop for Audio {
    fn drop(&mut self) {
        self.stop();
    }
}

// ---- PipeWire capture (runs on its own thread) ----

fn push_interleaved(shared: &Shared, bytes: &[u8], conv: &mut (Vec<f32>, Vec<f32>)) {
    let frames = (bytes.len() / 8).min(RING_SIZE);
    if conv.0.len() < frames {
        conv.0.resize(frames, 0.0);
        conv.1.resize(frames, 0.0);
    }
    // Deinterleave + peak scan outside the lock.
    let mut peak = 0.0f32;
    for (i, f) in bytes.chunks_exact(8).take(frames).enumerate() {
        let l = f32::from_le_bytes([f[0], f[1], f[2], f[3]]);
        let r = f32::from_le_bytes([f[4], f[5], f[6], f[7]]);
        conv.0[i] = l;
        conv.1[i] = r;
        peak = peak.max(l.abs()).max(r.abs());
    }
    {
        let mut ring = shared.ring.lock().unwrap();
        let pos = ring.write_pos;
        let first = frames.min(RING_SIZE - pos);
        ring.left[pos..pos + first].copy_from_slice(&conv.0[..first]);
        ring.right[pos..pos + first].copy_from_slice(&conv.1[..first]);
        ring.left[..frames - first].copy_from_slice(&conv.0[first..frames]);
        ring.right[..frames - first].copy_from_slice(&conv.1[first..frames]);
        ring.write_pos = (pos + frames) & RING_MASK;
        ring.last_chunk_time = Instant::now();
        ring.last_chunk_frames = frames;
    }
    if peak > SILENCE_THRESHOLD {
        shared.mark_audio();
    }
}

/// Live delay of the running DAC in seconds: frames queued in its kernel
/// ring buffer / its current rate (which follows the track: 44.1k, 96k...).
/// Skips snd-aloop. None when nothing is playing.
fn read_dac_delay() -> Option<f32> {
    for card in std::fs::read_dir("/proc/asound").ok()?.flatten() {
        let card = card.path();
        let is_card = card.file_name().and_then(|n| n.to_str()).is_some_and(|n| n.starts_with("card"));
        if !is_card || std::fs::read_to_string(card.join("id")).is_ok_and(|id| id.starts_with("Loopback")) {
            continue;
        }
        for pcm in std::fs::read_dir(&card).into_iter().flatten().flatten() {
            let pcm = pcm.path();
            if !pcm.file_name().and_then(|n| n.to_str()).is_some_and(|n| n.starts_with("pcm") && n.ends_with('p')) {
                continue;
            }
            for sub in std::fs::read_dir(&pcm).into_iter().flatten().flatten() {
                let status = std::fs::read_to_string(sub.path().join("status")).unwrap_or_default();
                if !status.lines().any(|l| l.starts_with("state:") && l.contains("RUNNING")) {
                    continue;
                }
                let delay = status.lines().find(|l| l.starts_with("delay"))
                    .and_then(|l| l.split(':').nth(1)).and_then(|v| v.trim().parse::<f32>().ok());
                let rate = std::fs::read_to_string(sub.path().join("hw_params")).ok().and_then(|hp| {
                    hp.lines().find(|l| l.starts_with("rate:"))
                        .and_then(|l| l.split_whitespace().nth(1)).and_then(|v| v.parse::<f32>().ok())
                });
                if let (Some(d), Some(r)) = (delay, rate) {
                    if r > 0.0 {
                        return Some(d / r);
                    }
                }
            }
        }
    }
    None
}

fn capture_thread(shared: Arc<Shared>) {
    pw::init();
    let mut warned = false;
    while shared.running.load(Ordering::Relaxed) {
        if let Err(e) = session(&shared, &mut warned) {
            if !warned {
                println!("PipeWire: {e}, retrying...");
                warned = true;
            }
        }
        for _ in 0..10 {
            if !shared.running.load(Ordering::Relaxed) {
                break;
            }
            thread::sleep(Duration::from_millis(100)); // retry after 1 s
        }
    }
}

/// One connection to the PipeWire daemon; returns when it fails (daemon
/// down/restarted) or on stop().
fn session(shared: &Arc<Shared>, warned: &mut bool) -> Result<(), pw::Error> {
    let mainloop = pw::main_loop::MainLoopRc::new(None)?;
    let context = pw::context::ContextRc::new(&mainloop, None)?;
    let core = context.connect_rc(None)?;
    *warned = false;

    let failed = Rc::new(Cell::new(false));
    let _core_listener = core
        .add_listener_local()
        .error({
            let failed = Rc::clone(&failed);
            move |id, _seq, res, message| {
                if id == pw::core::PW_ID_CORE {   // e.g. -EPIPE: daemon restarted
                    println!("PipeWire core error: {message} ({res})");
                    failed.set(true);
                }
            }
        })
        .register();

    let props = pw::properties::properties! {
        *pw::keys::MEDIA_TYPE => "Audio",
        *pw::keys::MEDIA_CATEGORY => "Capture",
        *pw::keys::MEDIA_ROLE => "Music",
        *pw::keys::STREAM_CAPTURE_SINK => "true", // the default sink's monitor
        "node.passive" => "true",                // never keeps the DAC awake
        "node.latency" => "512/44100",
    };
    let stream = pw::stream::StreamBox::new(&core, "rav-visualizer", props)?;
    let _listener = stream
        .add_local_listener_with_user_data((Vec::<f32>::new(), Vec::<f32>::new()))
        .state_changed({
            let failed = Rc::clone(&failed);
            move |_, _, _, new| {
                if let pw::stream::StreamState::Error(e) = new {
                    println!("PipeWire stream error: {e}");
                    failed.set(true);
                }
            }
        })
        .process({
            let shared = Arc::clone(shared);
            move |stream, conv| {
                let Some(mut buffer) = stream.dequeue_buffer() else { return };
                let datas = buffer.datas_mut();
                let Some(d) = datas.first_mut() else { return };
                let (offset, size) = (d.chunk().offset() as usize, d.chunk().size() as usize);
                if let Some(bytes) = d.data() {
                    if offset + size <= bytes.len() {
                        push_interleaved(&shared, &bytes[offset..offset + size], conv);
                    }
                }
            }
        })
        .register()?;

    // F32 stereo at 44.1 kHz: PipeWire converts whatever the graph runs at.
    let mut info = spa::param::audio::AudioInfoRaw::new();
    info.set_format(spa::param::audio::AudioFormat::F32LE);
    info.set_rate(SAMPLE_RATE as u32);
    info.set_channels(2);
    let obj = spa::pod::Object {
        type_: spa::utils::SpaTypes::ObjectParamFormat.as_raw(),
        id: spa::param::ParamType::EnumFormat.as_raw(),
        properties: info.into(),
    };
    let values: Vec<u8> = spa::pod::serialize::PodSerializer::serialize(
        std::io::Cursor::new(Vec::new()), &spa::pod::Value::Object(obj))
        .expect("serialize format pod").0.into_inner();
    let mut params = [Pod::from_bytes(&values).expect("format pod")];
    stream.connect(
        spa::utils::Direction::Input,
        None,
        pw::stream::StreamFlags::AUTOCONNECT | pw::stream::StreamFlags::MAP_BUFFERS,
        &mut params,
    )?;
    println!("PipeWire capture connected (sink monitor)");

    let mut next_probe = Instant::now();
    while shared.running.load(Ordering::Relaxed) && !failed.get() {
        mainloop.loop_().iterate(pw::loop_::Timeout::Finite(Duration::from_millis(50)));
        let now = Instant::now();
        if now < next_probe {
            continue;
        }
        next_probe = now + Duration::from_millis(250);
        let latency = match (read_dac_delay(), shared.latency()) {
            (None, _) => -1.0,
            (Some(d), prev) if prev < 0.0 => d,
            (Some(d), prev) => prev + 0.3 * (d - prev),
        };
        shared.out_latency.store(latency.to_bits(), Ordering::Relaxed);
    }
    Ok(())
}
