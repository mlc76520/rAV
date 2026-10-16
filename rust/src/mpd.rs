//! Minimal MPD client (the protocol is line-based text).
//!
//! A background thread keeps a connection in `idle player` to publish the
//! now-playing state; every second it leaves idle (`noidle`) to refresh
//! bitrate/elapsed. Control commands use short-lived connections so they
//! never interfere with the idle one.

use crate::snapshot::{NowPlaying, Published};
use anyhow::{Result, anyhow, bail};
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::os::fd::AsRawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

type Pairs = Vec<(String, String)>;

/// MPD tags are supposed to be UTF-8, but old files often carry Latin-1:
/// decode those byte-for-byte instead of losing the accented letters.
fn decode(bytes: &[u8]) -> String {
    match std::str::from_utf8(bytes) {
        Ok(s) => s.to_string(),
        Err(_) => bytes.iter().map(|&b| b as char).collect(),
    }
}

fn quote(s: &str) -> String {
    format!("\"{}\"", s.replace('\\', "\\\\").replace('"', "\\\""))
}

fn get<'a>(pairs: &'a Pairs, key: &str) -> Option<&'a str> {
    pairs.iter().find(|(k, _)| k.eq_ignore_ascii_case(key)).map(|(_, v)| v.as_str())
}

struct Conn {
    reader: BufReader<TcpStream>,
}

impl Conn {
    fn connect(host: &str, port: u16, timeout: Duration) -> Result<Self> {
        let addr = (host, port).to_socket_addrs()?.next().ok_or_else(|| anyhow!("bad MPD address"))?;
        let stream = TcpStream::connect_timeout(&addr, timeout)?;
        stream.set_read_timeout(Some(timeout.max(Duration::from_secs(2))))?;
        let mut c = Self { reader: BufReader::new(stream) };
        let greeting = c.read_line()?;
        if !greeting.starts_with("OK MPD") {
            bail!("unexpected MPD greeting: {greeting}");
        }
        Ok(c)
    }

    fn read_line(&mut self) -> Result<String> {
        let mut buf = Vec::new();
        if self.reader.read_until(b'\n', &mut buf)? == 0 {
            bail!("MPD closed the connection");
        }
        while buf.last().is_some_and(|&b| b == b'\n' || b == b'\r') {
            buf.pop();
        }
        Ok(decode(&buf))
    }

    fn send(&mut self, cmd: &str) -> Result<()> {
        let s = self.reader.get_mut();
        s.write_all(cmd.as_bytes())?;
        s.write_all(b"\n")?;
        Ok(())
    }

    fn read_response(&mut self) -> Result<Pairs> {
        let mut pairs = Vec::new();
        loop {
            let line = self.read_line()?;
            if line == "OK" {
                return Ok(pairs);
            }
            if line.starts_with("ACK") {
                bail!("MPD error: {line}");
            }
            if let Some((k, v)) = line.split_once(": ") {
                pairs.push((k.to_string(), v.to_string()));
            }
        }
    }

    fn run(&mut self, cmd: &str) -> Result<Pairs> {
        self.send(cmd)?;
        self.read_response()
    }

    /// Waits up to `timeout` for the server to send something.
    fn readable(&self, timeout: Duration) -> bool {
        if !self.reader.buffer().is_empty() {
            return true;
        }
        let mut pfd = libc::pollfd { fd: self.reader.get_ref().as_raw_fd(), events: libc::POLLIN, revents: 0 };
        unsafe { libc::poll(&mut pfd, 1, timeout.as_millis() as i32) > 0 }
    }
}

#[derive(Clone)]
pub struct TrackInfo {
    pub title: String,
    pub file: String,
    pub track_num: String,
}

pub struct MpdClient {
    host: String,
    port: u16,
    now_playing: Published,
    running: AtomicBool,
    thread: Mutex<Option<JoinHandle<()>>>,
}

impl MpdClient {
    pub fn new(host: &str, port: u16) -> Arc<Self> {
        Arc::new(Self {
            host: host.into(),
            port,
            now_playing: Published::default(),
            running: AtomicBool::new(false),
            thread: Mutex::new(None),
        })
    }

    pub fn start(self: &Arc<Self>) {
        if self.running.swap(true, Ordering::SeqCst) {
            return;
        }
        let me = Arc::clone(self);
        *self.thread.lock().unwrap() = Some(thread::Builder::new().name("mpd".into())
            .spawn(move || me.thread_loop()).expect("spawn mpd thread"));
    }

    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(t) = self.thread.lock().unwrap().take() {
            let _ = t.join();
        }
    }

    pub fn current(&self) -> Arc<NowPlaying> {
        self.now_playing.get()
    }

    // ---- background thread ----

    fn sleep_while_running(&self, d: Duration) {
        let end = Instant::now() + d;
        while self.running.load(Ordering::Relaxed) && Instant::now() < end {
            thread::sleep(Duration::from_millis(100));
        }
    }

    fn thread_loop(&self) {
        println!("MPD thread started");
        while self.running.load(Ordering::Relaxed) {
            let mut conn = match Conn::connect(&self.host, self.port, Duration::from_secs(5)) {
                Ok(c) => c,
                Err(_) => {
                    self.now_playing.set(NowPlaying::default());   // "Waiting for MPD..."
                    self.sleep_while_running(Duration::from_secs(5));
                    continue;
                }
            };
            println!("Connected to MPD at {}:{}", self.host, self.port);
            if let Err(e) = self.session(&mut conn) {
                println!("MPD connection lost ({e}), reconnecting...");
            }
        }
        println!("MPD thread stopped");
    }

    fn session(&self, conn: &mut Conn) -> Result<()> {
        self.full_update(conn)?;
        let mut last_refresh = Instant::now();
        while self.running.load(Ordering::Relaxed) {
            conn.send("idle player")?;
            loop {
                if !self.running.load(Ordering::Relaxed) {
                    return Ok(());
                }
                if conn.readable(Duration::from_millis(100)) {
                    let changed = conn.read_response()?;
                    if changed.iter().any(|(k, v)| k == "changed" && v == "player") {
                        self.full_update(conn)?;
                    }
                    break;
                }
                if last_refresh.elapsed() >= Duration::from_secs(1) {
                    last_refresh = Instant::now();
                    // noidle returns any event that raced in: a song change
                    // here must not be dropped.
                    let raced = conn.run("noidle")?;
                    if raced.iter().any(|(k, v)| k == "changed" && v == "player") {
                        self.full_update(conn)?;
                    } else {
                        self.refresh_status(conn)?;
                    }
                    break;
                }
            }
        }
        Ok(())
    }

    fn apply_status(np: &mut NowPlaying, st: &Pairs) {
        np.elapsed_ms = get(st, "elapsed").and_then(|v| v.parse::<f64>().ok())
            .map(|s| (s * 1000.0) as u32).unwrap_or(0);
        np.is_playing = get(st, "state") == Some("play");
        np.status_ts = Instant::now();
        np.bitrate = match get(st, "bitrate").and_then(|v| v.parse::<u32>().ok()) {
            Some(k) if k > 0 => format!("{k} kbps"),
            _ => "N/A".into(),
        };
    }

    fn full_update(&self, conn: &mut Conn) -> Result<()> {
        let song = conn.run("currentsong")?;
        if get(&song, "file").is_none() {
            self.now_playing.set(NowPlaying { formatted_text: "No song playing".into(), ..Default::default() });
            return Ok(());
        }
        let title = get(&song, "Title").unwrap_or("Unknown Title").to_string();
        let artist = get(&song, "Artist").unwrap_or("").to_string();
        let album = get(&song, "Album").unwrap_or("").to_string();
        let date = get(&song, "Date").filter(|d| d.len() >= 4).map(|d| d[..4].to_string()).unwrap_or_default();
        let track = get(&song, "Pos").and_then(|p| p.parse::<u32>().ok()).unwrap_or(0) + 1;

        let mut text = format!("{track:02}. {title}");
        if !artist.is_empty() {
            text += &format!(" - {artist}");
        }
        if !date.is_empty() {
            text += &format!(" ({date})");
        }

        let mut np = NowPlaying {
            formatted_text: text,
            track_number: track.to_string(),
            title,
            artist,
            album,
            date,
            format: "N/A".into(),
            ..Default::default()
        };
        let st = conn.run("status")?;
        Self::apply_status(&mut np, &st);
        np.total_s = get(&st, "duration").and_then(|v| v.parse::<f64>().ok()).map(|d| d as u32)
            .or_else(|| get(&st, "time").and_then(|t| t.split_once(':')).and_then(|(_, d)| d.parse().ok()))
            .unwrap_or(0);
        if let Some((rate, rest)) = get(&st, "audio").and_then(|a| a.split_once(':')) {
            if let (Ok(rate), Some((bits, ch))) = (rate.parse::<u32>(), rest.split_once(':')) {
                np.format = format!("{}kHz/{bits}bit/{ch}ch", rate / 1000);
            }
        }
        self.now_playing.set(np);
        Ok(())
    }

    fn refresh_status(&self, conn: &mut Conn) -> Result<()> {
        let st = conn.run("status")?;
        let mut np = (*self.current()).clone();   // copy-on-write
        Self::apply_status(&mut np, &st);
        self.now_playing.set(np);
        Ok(())
    }

    // ---- control (short-lived connections) ----

    fn with_conn<T>(&self, timeout: Duration, f: impl FnOnce(&mut Conn) -> Result<T>) -> Option<T> {
        let mut c = Conn::connect(&self.host, self.port, timeout).ok()?;
        f(&mut c).ok()
    }

    fn quick(&self, cmds: &[String]) {
        self.with_conn(Duration::from_secs(1), |c| {
            for cmd in cmds {
                c.run(cmd)?;
            }
            Ok(())
        });
    }

    pub fn play_pause(&self) {
        let cmd = if self.current().is_playing { "pause" } else { "play" };
        self.quick(&[cmd.into()]);
    }

    /// Idempotent: never starts playback (unlike play_pause's toggle).
    pub fn pause(&self) {
        self.quick(&["pause 1".into()]);
    }

    pub fn next_track(&self) {
        self.quick(&["next".into()]);
    }

    pub fn prev_track(&self) {
        self.quick(&["previous".into()]);
    }

    /// status + setvol on one connection; returns the new volume.
    pub fn adjust_volume(&self, delta: i32) -> Option<i32> {
        self.with_conn(Duration::from_secs(1), |c| {
            let st = c.run("status")?;
            let vol = get(&st, "volume").and_then(|v| v.parse::<i32>().ok()).ok_or_else(|| anyhow!("no volume"))?;
            let new = (vol + delta).clamp(0, 100);
            c.run(&format!("setvol {new}"))?;
            Ok(new)
        })
    }

    pub fn albums(&self) -> Vec<String> {
        let mut albums: Vec<String> = self
            .with_conn(Duration::from_secs(2), |c| c.run("list album"))
            .unwrap_or_default()
            .into_iter()
            .filter(|(k, v)| k == "Album" && !v.is_empty())
            .map(|(_, v)| v)
            .collect();
        albums.sort();
        albums
    }

    pub fn album_tracks(&self, album: &str) -> Vec<TrackInfo> {
        let pairs = self
            .with_conn(Duration::from_secs(2), |c| c.run(&format!("find album {}", quote(album))))
            .unwrap_or_default();
        let mut tracks: Vec<TrackInfo> = Vec::new();
        for (k, v) in pairs {
            match k.as_str() {
                "file" => tracks.push(TrackInfo { title: v.clone(), file: v, track_num: String::new() }),
                "Title" => if let Some(t) = tracks.last_mut() { t.title = v },
                "Track" => if let Some(t) = tracks.last_mut() {
                    t.track_num = v.split('/').next().unwrap_or("").to_string();
                },
                _ => {}
            }
        }
        tracks
    }

    pub fn add_album(&self, album: &str) {
        self.quick(&[format!("findadd album {}", quote(album))]);
    }

    pub fn clear_and_play_album(&self, album: &str) {
        self.quick(&["clear".into(), format!("findadd album {}", quote(album)), "play".into()]);
    }

    pub fn add_track(&self, file: &str) {
        self.quick(&[format!("add {}", quote(file))]);
    }
}
