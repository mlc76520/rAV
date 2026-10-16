//! Bluetooth (BlueZ over D-Bus): what the phone plays, and remote control.
//!
//! A monitor thread follows PropertiesChanged signals; signals and the
//! startup discovery (ObjectManager) go through the same property code, so
//! a phone already streaming when we start is picked up immediately.
//! A second one follows InterfacesRemoved: when the phone disconnects (or
//! sends its audio elsewhere) bluez deletes the transport without any State
//! change, which must still end "streaming" or the display would keep the
//! phone's track.
//! Commands (AVRCP play/pause/next/previous, absolute volume) are queued to
//! a worker thread so the render loop never waits on D-Bus.

use crate::snapshot::{NowPlaying, Published};
use anyhow::Result;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::mpsc::{self, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use zbus::blocking::{Connection, MessageIterator};
use zbus::zvariant::{OwnedObjectPath, OwnedValue, Value};

const PLAYER: &str = "org.bluez.MediaPlayer1";
const TRANSPORT: &str = "org.bluez.MediaTransport1";
const DEVICE: &str = "org.bluez.Device1";

#[derive(Clone, Copy, Debug)]
pub enum Cmd {
    Play,
    Pause,
    Next,
    Previous,
}

enum Job {
    Player(Cmd),
    Volume(u16),
}

#[derive(Default)]
struct State {
    player_path: String,
    transport_path: String,
    title: String,
    artist: String,
    duration_ms: u32,
    position_ms: u32,
    position_ts: Option<Instant>,
}

type Props = HashMap<String, OwnedValue>;

fn as_str<'a>(v: &'a Value<'_>) -> Option<&'a str> {
    if let Value::Str(s) = v { Some(s.as_str()) } else { None }
}

pub struct Bluetooth {
    conn: Connection,
    state: Mutex<State>,
    streaming: AtomicBool, // transport "active"
    playing: AtomicBool,
    has_player: AtomicBool,
    volume_raw: AtomicI32, // AVRCP 0-127, -1 = unknown
    // While the knob turns, the phone may echo values rounded to its own
    // volume steps; echoes are ignored for a second after the last knob step
    // so small steps accumulate. None = the knob was never used.
    last_knob: Mutex<Option<Instant>>,
    now_playing: Published,
    jobs: Mutex<Option<Sender<Job>>>,
    on_connection: Box<dyn Fn(bool) + Send + Sync>,
}

impl Bluetooth {
    pub fn start(on_connection: impl Fn(bool) + Send + Sync + 'static) -> Result<Arc<Self>> {
        let bt = Arc::new(Self {
            conn: Connection::system()?,
            state: Mutex::new(State::default()),
            streaming: AtomicBool::new(false),
            playing: AtomicBool::new(false),
            has_player: AtomicBool::new(false),
            volume_raw: AtomicI32::new(-1),
            last_knob: Mutex::new(None),
            now_playing: Published::default(),
            jobs: Mutex::new(None),
            on_connection: Box::new(on_connection),
        });

        let rule = zbus::MatchRule::builder()
            .msg_type(zbus::message::Type::Signal)
            .interface("org.freedesktop.DBus.Properties")?
            .member("PropertiesChanged")?
            .build();
        let signals = MessageIterator::for_match_rule(rule, &bt.conn, Some(64))?;
        let removed_rule = zbus::MatchRule::builder()
            .msg_type(zbus::message::Type::Signal)
            .sender("org.bluez")?
            .interface("org.freedesktop.DBus.ObjectManager")?
            .member("InterfacesRemoved")?
            .build();
        let removed = MessageIterator::for_match_rule(removed_rule, &bt.conn, Some(16))?;
        let me = Arc::clone(&bt);
        thread::Builder::new().name("bt-removed".into()).spawn(move || {
            for msg in removed.flatten() {
                if let Ok((path, ifaces)) = msg.body().deserialize::<(OwnedObjectPath, Vec<String>)>() {
                    me.interfaces_removed(path.as_str(), &ifaces);
                }
            }
        })?;
        let me = Arc::clone(&bt);
        thread::Builder::new().name("bt-monitor".into()).spawn(move || {
            me.discover();   // a phone may already be connected
            for msg in signals.flatten() {
                let Some(path) = msg.header().path().map(|p| p.to_string()) else { continue };
                if let Ok((iface, props, _)) = msg.body().deserialize::<(String, Props, Vec<String>)>() {
                    me.apply_interface(&iface, &path, props);
                }
            }
        })?;

        let (tx, rx) = mpsc::channel::<Job>();
        *bt.jobs.lock().unwrap() = Some(tx);
        let me = Arc::clone(&bt);
        thread::Builder::new().name("bt-commands".into()).spawn(move || {
            for job in rx {
                me.run_job(job);
            }
        })?;
        println!("Bluetooth monitor started");
        Ok(bt)
    }

    pub fn is_streaming(&self) -> bool {
        self.streaming.load(Ordering::Relaxed)
    }
    pub fn is_playing(&self) -> bool {
        self.playing.load(Ordering::Relaxed)
    }
    /// A phone with an AVRCP player is connected (streaming or not).
    pub fn has_player(&self) -> bool {
        self.has_player.load(Ordering::Relaxed)
    }
    pub fn current(&self) -> Arc<NowPlaying> {
        self.now_playing.get()
    }

    pub fn send(&self, cmd: Cmd) {
        if let Some(tx) = self.jobs.lock().unwrap().as_ref() {
            let _ = tx.send(Job::Player(cmd));
        }
    }

    /// Knob on the phone's own media volume (AVRCP absolute volume).
    /// Returns the new %, or None while the phone's volume is unknown.
    pub fn adjust_volume(&self, delta_pct: i32) -> Option<i32> {
        let raw = self.volume_raw.load(Ordering::Relaxed);
        if raw < 0 {
            return None;
        }
        let pct = ((raw as f32 * 100.0 / 127.0).round() as i32 + delta_pct).clamp(0, 100);
        let raw = (pct as f32 * 127.0 / 100.0).round() as i32;
        *self.last_knob.lock().unwrap() = Some(Instant::now());
        self.volume_raw.store(raw, Ordering::Relaxed);
        if let Some(tx) = self.jobs.lock().unwrap().as_ref() {
            let _ = tx.send(Job::Volume(raw as u16));
        }
        Some(pct)
    }

    // ---- property handling (monitor thread, and worker via discover) ----

    fn publish(&self, st: &State) {
        let mut np = NowPlaying {
            formatted_text: if st.title.is_empty() {
                "Bluetooth: Connected".into()
            } else if st.artist.is_empty() {
                st.title.clone()
            } else {
                format!("{} - {}", st.title, st.artist)
            },
            title: st.title.clone(),
            artist: st.artist.clone(),
            elapsed_ms: st.position_ms,
            total_s: st.duration_ms / 1000,
            is_playing: self.is_playing(),
            ..Default::default()
        };
        if let Some(ts) = st.position_ts {
            np.status_ts = ts;
        }
        self.now_playing.set(np);
    }

    fn apply_player_prop(&self, st: &mut State, key: &str, v: OwnedValue) {
        match key {
            "Status" => if let Some(s) = as_str(&v) {
                self.playing.store(s == "playing", Ordering::Relaxed);
            },
            "Position" => if let Value::U32(p) = *v {
                st.position_ms = p;
                st.position_ts = Some(Instant::now());
            },
            "Track" => if let Ok(track) = Props::try_from(v) {
                for (k, tv) in track {
                    match (k.as_str(), &*tv) {
                        ("Title", Value::Str(s)) => st.title = s.to_string(),
                        ("Artist", Value::Str(s)) => st.artist = s.to_string(),
                        ("Duration", Value::U32(d)) => st.duration_ms = *d,
                        _ => {}
                    }
                }
            },
            _ => {}
        }
    }

    /// Returns true if the streaming state flipped.
    fn apply_transport_prop(&self, key: &str, v: &Value<'_>) -> bool {
        match (key, v) {
            ("State", v) => {
                let active = as_str(v) == Some("active");
                self.streaming.swap(active, Ordering::Relaxed) != active
            }
            ("Volume", Value::U16(vol)) => {
                let echo = self.last_knob.lock().unwrap().is_some_and(|t| t.elapsed() < Duration::from_secs(1));
                if !echo {
                    self.volume_raw.store(*vol as i32, Ordering::Relaxed);
                }
                false
            }
            _ => false,
        }
    }

    fn apply_interface(&self, iface: &str, path: &str, props: Props) {
        let mut flipped = false;
        {
            let mut st = self.state.lock().unwrap();
            match iface {
                PLAYER => {
                    st.player_path = path.to_string();
                    self.has_player.store(true, Ordering::Relaxed);
                    for (k, v) in props {
                        self.apply_player_prop(&mut st, &k, v);
                    }
                    self.publish(&st);
                }
                TRANSPORT => {
                    st.transport_path = path.to_string();
                    for (k, v) in &props {
                        flipped |= self.apply_transport_prop(k, v);
                    }
                    if flipped {
                        self.publish(&st);
                    }
                }
                DEVICE => {
                    // Our phone disconnected: forget it, the display and the
                    // controls go back to MPD.
                    let ours = st.player_path.starts_with(path) || st.transport_path.starts_with(path);
                    let off = matches!(props.get("Connected").map(|v| &**v), Some(Value::Bool(false)));
                    if ours && off {
                        st.player_path.clear();
                        st.transport_path.clear();
                        self.has_player.store(false, Ordering::Relaxed);
                        self.playing.store(false, Ordering::Relaxed);
                        self.volume_raw.store(-1, Ordering::Relaxed);
                        flipped = self.streaming.swap(false, Ordering::Relaxed);
                    }
                }
                _ => {}
            }
        }
        if flipped {
            (self.on_connection)(self.is_streaming());
        }
    }

    /// Transport or player deleted by bluez (disconnect, audio routed back
    /// to the phone...).
    fn interfaces_removed(&self, path: &str, ifaces: &[String]) {
        let mut flipped = false;
        {
            let mut st = self.state.lock().unwrap();
            if ifaces.iter().any(|i| i == TRANSPORT) && st.transport_path == path {
                st.transport_path.clear();
                flipped = self.streaming.swap(false, Ordering::Relaxed);
            }
            if ifaces.iter().any(|i| i == PLAYER) && st.player_path == path {
                st.player_path.clear();
                self.has_player.store(false, Ordering::Relaxed);
                self.playing.store(false, Ordering::Relaxed);
            }
            if flipped {
                self.publish(&st);
            }
        }
        if flipped {
            (self.on_connection)(false);
        }
    }

    /// Reads the phone's current player/transport state: signals only
    /// report changes.
    fn discover(&self) {
        type Objects = HashMap<OwnedObjectPath, HashMap<String, Props>>;
        let Ok(reply) = self.conn.call_method(Some("org.bluez"), "/",
            Some("org.freedesktop.DBus.ObjectManager"), "GetManagedObjects", &()) else { return };
        let Ok(objects) = reply.body().deserialize::<Objects>() else { return };
        for (path, ifaces) in objects {
            for (iface, props) in ifaces {
                if iface == PLAYER || iface == TRANSPORT {
                    self.apply_interface(&iface, path.as_str(), props);
                }
            }
        }
    }

    fn run_job(&self, job: Job) {
        let (player, transport) = {
            let st = self.state.lock().unwrap();
            (st.player_path.clone(), st.transport_path.clone())
        };
        let (player, transport) = if player.is_empty() || transport.is_empty() {
            self.discover();
            let st = self.state.lock().unwrap();
            (st.player_path.clone(), st.transport_path.clone())
        } else {
            (player, transport)
        };
        match job {
            Job::Player(cmd) if !player.is_empty() => {
                let method = match cmd {
                    Cmd::Play => "Play",
                    Cmd::Pause => "Pause",
                    Cmd::Next => "Next",
                    Cmd::Previous => "Previous",
                };
                let _ = self.conn.call_method(Some("org.bluez"), player.as_str(), Some(PLAYER), method, &());
            }
            Job::Volume(raw) if !transport.is_empty() => {
                let _ = self.conn.call_method(Some("org.bluez"), transport.as_str(),
                    Some("org.freedesktop.DBus.Properties"), "Set",
                    &(TRANSPORT, "Volume", Value::U16(raw)));
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Needs a phone connected over A2DP; moves its volume by 10 % and back.
    /// Run with: cargo test --release -- --ignored phone_volume
    #[test]
    #[ignore]
    fn phone_volume() {
        let bt = Bluetooth::start(|_| {}).expect("D-Bus");
        // The phone's volume must be known right after startup (the startup
        // discovery used to be mistaken for a knob echo and dropped).
        let deadline = Instant::now() + Duration::from_secs(3);
        let before = loop {
            if let Some(v) = bt.adjust_volume(0) {
                break v;
            }
            assert!(Instant::now() < deadline, "phone volume still unknown after 3 s");
            std::thread::sleep(Duration::from_millis(50));
        };
        let down = bt.adjust_volume(-10).unwrap();
        assert_eq!(down, (before - 10).max(0));
        std::thread::sleep(Duration::from_millis(600));
        assert_eq!(bt.adjust_volume(before - down).unwrap(), before);
        std::thread::sleep(Duration::from_millis(600));
        println!("phone volume {before}% -> {down}% -> back to {before}%");
    }
}
