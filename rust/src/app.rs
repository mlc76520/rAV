//! The application: hardware setup, controls, sleep/wake and the 60 fps
//! render loop, for whichever panel the profile describes.

use crate::audio::Audio;
use crate::bluetooth::{Bluetooth, Cmd};
use crate::display::Display;
use crate::font::{Fonts, Size};
use crate::hw::{self, ButtonState, Buttons, OutputLine};
use crate::mpd::MpdClient;
use crate::profile::{self, Profile};
use crate::rotary::Rotary;
use crate::snapshot::FrameSnapshot;
use crate::transition::Transition;
use crate::viz::editor::{Outcome, Power};
use crate::viz::{Frame, Tier, Viz};
use anyhow::{Context, Result};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

/// Set by SIGINT/SIGTERM.
pub static SHUTDOWN: AtomicBool = AtomicBool::new(false);

const FRAME: Duration = Duration::from_millis(16);
const OVERLAY_DURATION: Duration = Duration::from_millis(1500);
// Transport buttons act once per press, with a lockout against bounce.
const TRANSPORT_LOCKOUT: Duration = Duration::from_millis(200);
// PLAY held >= PLAY_LONG opens the settings menu; a shorter press (acted on
// release) is play/pause.
const PLAY_LONG: Duration = Duration::from_millis(1000);
// VIZ held >= VIZ_LONG tours the hidden tier; shorter = next visualization.
const VIZ_LONG: Duration = Duration::from_millis(700);
const AUTO_CYCLE_IDLE: Duration = Duration::from_secs(30);
const AUTO_CYCLE_INTERVAL: Duration = Duration::from_secs(600);
const CRT_DURATION: Duration = Duration::from_millis(400);

/// Press/release edges and hold time of one button.
#[derive(Default)]
struct HoldButton {
    down: bool,
    armed_at: Option<Instant>,
}

impl HoldButton {
    /// Returns (released_after, long_hold_reached). The long hold fires once
    /// while still held; a release after it reports nothing.
    fn update(&mut self, pressed: bool, now: Instant, long: Duration) -> (Option<Duration>, bool) {
        let (edge, release) = (pressed && !self.down, !pressed && self.down);
        self.down = pressed;
        if edge {
            self.armed_at = Some(now);
        }
        let Some(t0) = self.armed_at else { return (None, false) };
        let held = now - t0;
        if pressed && held >= long {
            self.armed_at = None;
            return (None, true);
        }
        if release {
            self.armed_at = None;
            return (Some(held), false);
        }
        (None, false)
    }
}

pub struct App {
    profile: &'static Profile,
    left: Display,
    right: Display,
    fonts: Fonts,
    audio: Audio,
    mpd: Arc<MpdClient>,
    bt: Option<Arc<Bluetooth>>,
    buttons: Buttons,
    rotary: Rotary,
    led: OutputLine,
    _resets: (OutputLine, OutputLine),

    vizs: Vec<Box<dyn Viz>>,
    current: usize,
    editor: Option<usize>,
    hidden: Vec<usize>, // hold-VIZ tour, in order
    transition: Transition,

    snap: FrameSnapshot,
    last_snap: Instant,
    prev_buttons: ButtonState,
    play: HoldButton,
    viz: HoldButton,
    last_transport: Option<Instant>,
    overlay: Option<(i32, Instant)>, // volume %, shown since
    last_viz_change: Instant,
    last_cycle: Instant,
    sleeping: bool,
    powering_off: bool,
}

impl App {
    pub fn new(profile: &'static Profile) -> Result<Self> {
        let pins = &profile.pins;
        let (chip, label) = hw::find_gpio_chip()?;
        println!("Display {}, using {} ({label})", profile.name, chip.display());

        let rst_l = OutputLine::new(&chip, pins.left_rst, true)?;
        let rst_r = OutputLine::new(&chip, pins.right_rst, true)?;
        let mut left = Display::new(profile, profile::LEFT_SPI, OutputLine::new(&chip, pins.left_dc, false)?, &rst_l)?;
        let right = Display::new(profile, profile::RIGHT_SPI, OutputLine::new(&chip, pins.right_dc, false)?, &rst_r)?;
        println!("SPI initialized at {} Hz", profile.spi_hz);

        let fonts = profile.font_paths.iter().find_map(|p| Fonts::load(p, profile.font_px).ok())
            .context("no usable font")?;
        let (w, h) = (left.canvas.width(), left.canvas.height());
        fonts.draw_centered(&mut left.canvas, w / 2, h / 2 + 4, "Starting...", Size::Small, 15);
        left.present();

        let mpd = MpdClient::new("localhost", 6600);
        mpd.start();
        let bt = {
            let mpd = Arc::clone(&mpd);
            match Bluetooth::start(move |streaming| {
                if streaming {
                    println!("Bluetooth connected - pausing MPD");
                    mpd.pause();
                } else {
                    println!("Bluetooth disconnected");
                }
            }) {
                Ok(bt) => Some(bt),
                Err(e) => {
                    println!("Bluetooth unavailable: {e}");
                    None
                }
            }
        };

        let vizs = (profile.vizs)(Arc::clone(&mpd));
        let editor = vizs.iter().position(|v| v.tier() == Tier::Editor);
        let hidden = (0..vizs.len()).filter(|&i| vizs[i].tier() == Tier::Hidden).collect();

        let buttons = Buttons::new(&chip, pins);
        let led = OutputLine::new(&chip, pins.led, true)?;
        let rotary = Rotary::start(&chip, pins)?;
        left.canvas.clear();

        let now = Instant::now();
        Ok(Self {
            profile,
            left,
            right,
            fonts,
            audio: Audio::new(profile),
            mpd,
            bt,
            buttons,
            rotary,
            led,
            _resets: (rst_l, rst_r),
            vizs,
            current: 0,
            editor,
            hidden,
            transition: Transition::new(),
            snap: FrameSnapshot::new(profile.width, profile.bands.len()),
            last_snap: now,
            prev_buttons: ButtonState::default(),
            play: HoldButton::default(),
            viz: HoldButton::default(),
            last_transport: None,
            overlay: None,
            last_viz_change: now,
            last_cycle: now,
            sleeping: false,
            powering_off: false,
        })
    }

    /// Next visualization in the normal cycle (skips editor + hidden tier).
    fn next_viz(&self, cur: usize) -> usize {
        let n = self.vizs.len();
        let mut i = (cur + 1) % n;
        while self.vizs[i].tier() != Tier::Normal {
            i = (i + 1) % n;
        }
        i
    }

    fn switch_to(&mut self, i: usize, why: &str) {
        self.current = i;
        self.last_viz_change = Instant::now();
        self.last_cycle = self.last_viz_change;
        println!("{why}: {}", self.vizs[i].name());
        if self.profile.features.transition && !self.sleeping {
            self.transition.start(&self.left.canvas, &self.right.canvas);
        }
    }

    /// The phone gets the knob and transport buttons when it is the active
    /// source: streaming, or connected while MPD is idle (so a paused phone
    /// can be resumed). Otherwise MPD.
    fn bt_active(&self) -> Option<&Arc<Bluetooth>> {
        self.bt.as_ref().filter(|bt| {
            bt.has_player() && (bt.is_playing() || bt.is_streaming() || !self.mpd.current().is_playing)
        })
    }

    fn render_current(&mut self) {
        let now = Instant::now();
        self.snap.now = now;
        self.snap.dt = (now - self.last_snap).as_secs_f32().clamp(0.0, 0.1);
        self.last_snap = now;
        self.snap.bt_streaming = self.bt.as_ref().is_some_and(|bt| bt.is_streaming());
        self.snap.track = match (&self.bt, self.snap.bt_streaming) {
            (Some(bt), true) => bt.current(),
            _ => self.mpd.current(),
        };
        self.snap.elapsed_s = self.snap.track.elapsed_at(now);
        self.snap.total_s = self.snap.track.total_s;
        self.audio.fill_snapshot(&mut self.snap);
        let mut frame = Frame { left: &mut self.left.canvas, right: &mut self.right.canvas, fonts: &self.fonts };
        self.vizs[self.current].render(&mut frame, &self.snap);
    }

    /// Restart or power off, confirmed in the settings menu.
    fn power(&mut self, p: Power) {
        if self.powering_off {
            return;
        }
        self.powering_off = true;
        let verb = match p {
            Power::Restart => "reboot",
            Power::Off => "poweroff",
        };
        println!("Settings -> system {verb}");
        if !self.sleeping {
            self.animate_crt(false);
        }
        self.left.sleep();
        self.right.sleep();
        self.led.set(false);
        // systemd stops this service (SIGTERM -> clean exit) before halting.
        if std::process::Command::new("systemctl").args([verb, "--no-wall"]).spawn().is_err() {
            println!("{verb} request failed");
        }
    }

    fn poll_controls(&mut self) {
        let features = &self.profile.features;
        let rot = self.rotary.take_rotation();
        let rot_btn = self.rotary.take_press();
        let b = self.buttons.read();
        let now = Instant::now();
        let prev = std::mem::replace(&mut self.prev_buttons, b);
        let fw_pressed = b.fw && !prev.fw;
        let rw_pressed = b.rw && !prev.rw;

        // PLAY: with settings, a long hold opens them (editor included) and
        // play/pause acts on a short release; otherwise on press.
        let play_pressed = if features.settings {
            let (release, long) = self.play.update(b.play, now, PLAY_LONG);
            if long {
                if let Some(e) = self.editor {
                    if let Some(ed) = self.vizs[e].as_editor() {
                        ed.open_settings();
                    }
                    self.switch_to(e, "Opening settings (PLAY long)");
                }
                return;
            }
            release.is_some()
        } else {
            b.play && !prev.play
        };
        // VIZ: with a hidden tier, a long hold tours it and "next" acts on
        // release; otherwise on press.
        let (viz_pressed, viz_long) = if features.hidden_tier {
            let (release, long) = self.viz.update(b.viz, now, VIZ_LONG);
            (release.is_some(), long)
        } else {
            (b.viz && !prev.viz, false)
        };

        if b.viz || b.play || b.fw || b.rw || rot_btn || rot != 0 {
            self.audio.reset_sleep_timer();
        }

        if viz_long && !self.hidden.is_empty() {
            let next = match self.hidden.iter().position(|&h| h == self.current) {
                Some(k) if k + 1 < self.hidden.len() => self.hidden[k + 1],
                Some(_) => self.next_viz(self.current),
                None => self.hidden[0],
            };
            self.switch_to(next, "Viz (long)");
            return;
        }

        if Some(self.current) == self.editor {
            let editor = self.vizs[self.current].as_editor().expect("editor index");
            // Turning only moves the cursor
            for _ in 0..rot.abs() {
                editor.handle_input(rot < 0, rot > 0, false, false);
            }
            if !(rot_btn || viz_pressed) {
                return;
            }
            match editor.handle_input(false, false, rot_btn, viz_pressed) {
                Outcome::Stay => {}
                Outcome::Exit => self.switch_to(self.next_viz(self.current), "Leaving editor"),
                Outcome::Power(p) => self.power(p),
            }
            return;
        }

        if viz_pressed {
            self.switch_to(self.next_viz(self.current), "Viz");
        }
        if rot != 0 {
            // Phone active -> the phone's own volume (AVRCP absolute volume);
            // otherwise, or while its volume is still unknown, MPD.
            let delta = rot * self.profile.knob_up * 2;
            let v = self.bt_active().and_then(|bt| bt.adjust_volume(delta))
                .or_else(|| self.mpd.adjust_volume(delta));
            if let Some(v) = v {
                self.overlay = Some((v, now));
            }
        }
        if rot_btn {
            if let Some(e) = self.editor {
                if let Some(ed) = self.vizs[e].as_editor() {
                    ed.open_library();
                }
                self.switch_to(e, "Opening editor (ROT_SW)");
            }
        }

        let cmd = if play_pressed {
            Some(Cmd::Play) // resolved to play or pause below
        } else if fw_pressed {
            Some(Cmd::Next)
        } else if rw_pressed {
            Some(Cmd::Previous)
        } else {
            None
        };
        let Some(cmd) = cmd else { return };
        if self.last_transport.is_some_and(|t| now - t <= TRANSPORT_LOCKOUT) {
            return;
        }
        self.last_transport = Some(now);
        if let Some(bt) = self.bt_active() {
            bt.send(match cmd {
                Cmd::Play if bt.is_playing() => Cmd::Pause,
                c => c,
            });
        } else {
            match cmd {
                Cmd::Next => self.mpd.next_track(),
                Cmd::Previous => self.mpd.prev_track(),
                _ => self.mpd.play_pause(),
            }
        }
    }

    /// CRT power on/off over 400 ms: the picture pinches to a horizontal
    /// line, which shrinks to a dot (or the reverse).
    fn animate_crt(&mut self, power_on: bool) {
        let saved = [self.left.canvas.buf.clone(), self.right.canvas.buf.clone()];
        let (w, h) = (self.left.canvas.width(), self.left.canvas.height());
        let (mid_x, mid_y) = (w / 2, h / 2);
        let start = Instant::now();
        loop {
            let elapsed = start.elapsed();
            if elapsed >= CRT_DURATION {
                break;
            }
            let mut t = elapsed.as_secs_f32() / CRT_DURATION.as_secs_f32();
            if power_on {
                t = 1.0 - t;
            }
            for (d, saved) in [&mut self.left, &mut self.right].into_iter().zip(&saved) {
                let cv = &mut d.canvas;
                cv.buf.copy_from_slice(saved);
                if t < 0.5 {
                    let spread = ((mid_y - 1) as f32 * (1.0 - t * 2.0)) as i32;
                    let (top, bottom) = (mid_y - spread, mid_y + spread);
                    cv.blank_rows(0, top);
                    cv.blank_rows(bottom + 1, h);
                    cv.line(0, top, w - 1, top, 15);
                    cv.line(0, bottom, w - 1, bottom, 15);
                } else {
                    let half = (mid_x as f32 * (1.0 - (t - 0.5) * 2.0)) as i32;
                    cv.clear();
                    cv.line(mid_x - half, mid_y, mid_x + half, mid_y, 15);
                }
                d.present();
            }
            thread::sleep(FRAME);
        }
        for d in [&mut self.left, &mut self.right] {
            d.canvas.clear();
            d.present();
        }
    }

    fn pace(next_frame: &mut Instant) {
        let now = Instant::now();
        if now > *next_frame {
            *next_frame = now;
        }
        *next_frame += FRAME;
        thread::sleep(next_frame.saturating_duration_since(Instant::now()));
    }

    pub fn run(&mut self) {
        self.audio.start();
        let mut next_frame = Instant::now();
        while !SHUTDOWN.load(Ordering::Relaxed) {
            if self.powering_off {
                thread::sleep(Duration::from_millis(50)); // systemd will stop us
                continue;
            }
            self.poll_controls();

            let has_audio = self.audio.has_audio();
            if !has_audio && !self.sleeping {
                self.sleeping = true;
                self.animate_crt(false);
                self.left.sleep();
                self.right.sleep();
                self.led.set(false);
            } else if has_audio && self.sleeping {
                self.sleeping = false;
                self.last_cycle = Instant::now();
                self.left.wake();
                self.right.wake();
                self.led.set(true);
                self.render_current();
                self.animate_crt(true);
            }
            if self.sleeping {
                thread::sleep(Duration::from_millis(100));
                continue;
            }

            let in_editor = Some(self.current) == self.editor;
            if !in_editor && self.last_viz_change.elapsed() > AUTO_CYCLE_IDLE
                && self.last_cycle.elapsed() >= AUTO_CYCLE_INTERVAL {
                let next = self.next_viz(self.current);
                self.current = next;
                self.last_cycle = Instant::now();
                println!("Auto-cycle to: {}", self.vizs[next].name());
            }

            self.render_current();
            self.transition.update(&mut self.left.canvas, &mut self.right.canvas, &self.fonts);
            if let Some((vol, since)) = self.overlay {
                if since.elapsed() >= OVERLAY_DURATION {
                    self.overlay = None;
                } else if !in_editor {
                    (self.profile.volume_overlay)(&mut self.left.canvas, &self.fonts, vol);
                    (self.profile.volume_overlay)(&mut self.right.canvas, &self.fonts, vol);
                }
            }
            self.left.present();
            self.right.present();
            Self::pace(&mut next_frame);
        }
    }

    pub fn shutdown(&mut self) {
        println!("Shutting down...");
        self.audio.stop();
        self.mpd.stop();
        self.left.shutdown();
        self.right.shutdown();
        self.led.set(false);
        println!("Displays powered off");
    }
}
