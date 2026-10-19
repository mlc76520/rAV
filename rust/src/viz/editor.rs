//! Playlist editor and settings: menus on the left panel, what is playing
//! over a mirrored spectrum on the right.
//! The library menu opens the queue, the artists, the albums, the music
//! folders or the stored playlists, or rescans the library; an artist,
//! album, folder or playlist can be played, queued, or have single tracks
//! queued, and a queued song played. The settings menu shows the network,
//! switches Bluetooth and its visibility, and restarts or powers off the box
//! after a confirmation.
//! Queries run on a worker thread and hand their result back through a
//! channel, polled each frame. The logic is shared; each panel family
//! supplies its Layout.

use super::{Frame, Tier, Viz};
use crate::bluetooth;
use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::mpd::{self, Collection, Entry, MpdClient, TrackInfo};
use crate::snapshot::FrameSnapshot;
use crate::system;
use std::sync::Arc;
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::{Duration, Instant};

/// Geometry of the editor on one panel family. y values are baselines.
pub struct Layout {
    pub size: Size,
    pub visible_lines: usize,
    pub line_h: i32,
    pub list_y0: i32,  // baseline of the first row
    pub header_y: i32, // header baseline; its rule sits 2 px lower
    pub row_text_w: i32,
    /// Selection bar: from `dy` above the row baseline, `h` tall, `w` wide.
    pub selection: (i32, i32, i32),
    pub scrollbar_top_dy: i32, // above the first baseline
    pub scrollbar_w: i32,
    pub scrollbar_min: i32,
    pub loading_at: (i32, i32),
    pub empty_at: (i32, i32), // "Empty"
    pub feedback_box: (i32, i32, i32, i32),
    pub feedback_inset: i32, // of the inner frame
    pub feedback_baseline: i32,
    // Right panel.
    pub info_lines: (i32, i32), // title, artist baselines
    pub info_rule_y: i32,
    pub spectrum_axis_y: i32, // left channel above, right below
    pub spectrum_gap: i32,    // extra rows between the axis and the right bars
    pub spectrum_h: i32,
    pub help_y: i32,
}

/// A list the editor can show.
#[derive(Clone)]
enum Source {
    Queue,
    Artists,
    Albums,
    Playlists,
    Contents(Collection),
    Network,
    Bluetooth,
    Confirm(Power),
}

/// Restart or power off, confirmed in the settings menu.
#[derive(Clone, Copy)]
pub enum Power {
    Restart,
    Off,
}

/// What the app must do after an input.
pub enum Outcome {
    Stay,
    Exit,
    Power(Power),
}

impl Source {
    fn title(&self) -> String {
        match self {
            Source::Queue => "QUEUE".into(),
            Source::Artists => "ARTISTS".into(),
            Source::Albums => "ALBUMS".into(),
            Source::Playlists => "PLAYLISTS".into(),
            Source::Contents(Collection::Folder(p)) if p.is_empty() => "FOLDERS".into(),
            Source::Contents(Collection::Folder(p)) => mpd::basename(p).into(),
            Source::Contents(Collection::Album(n, _) | Collection::Artist(n)) => n.clone(),
            Source::Contents(Collection::Playlist(n)) => mpd::playlist_name(n).into(),
            Source::Network => "NETWORK".into(),
            Source::Bluetooth => "BLUETOOTH".into(),
            Source::Confirm(Power::Restart) => "RESTART?".into(),
            Source::Confirm(Power::Off) => "POWER OFF?".into(),
        }
    }
}

/// What selecting a row does.
#[derive(Clone)]
enum Action {
    Open(Source),
    Play(Collection),
    Add(Collection),
    PlayPos(usize),
    AddFile(String),
    UpdateDatabase,
    /// A line of information only.
    Info,
    Back,
    Power(Power),
    /// Sets a Bluetooth controller property (path, name, value).
    Bluetooth(String, &'static str, bool),
}

struct Row {
    label: String,
    action: Action,
}

impl Row {
    fn new(label: impl Into<String>, action: Action) -> Self {
        Self { label: label.into(), action }
    }
}

/// Rows of a list, with the row the cursor starts on (None: where it was,
/// for a list refreshed in place).
struct Listing {
    rows: Vec<Row>,
    cursor: Option<usize>,
}

fn track_label(t: &TrackInfo) -> String {
    if t.track_num.is_empty() { t.title.clone() } else { format!("{}. {}", t.track_num, t.title) }
}

fn playlist_row(name: String) -> Row {
    let label = mpd::playlist_name(&name).to_string();
    Row::new(label, Action::Open(Source::Contents(Collection::Playlist(name))))
}

/// Runs on the worker thread.
fn list(mpd: &MpdClient, source: &Source) -> Listing {
    let mut cursor = Some(0);
    let rows = match source {
        Source::Queue => {
            let (songs, current) = mpd.queue();
            cursor = Some(current.unwrap_or(0));
            let mark = |i| if Some(i) == current { "> " } else { "" };
            songs.iter().enumerate().map(|(i, t)| Row::new(format!("{}{}", mark(i), t.title), Action::PlayPos(i))).collect()
        }
        Source::Artists => mpd.artists().into_iter()
            .map(|a| Row::new(a.clone(), Action::Open(Source::Contents(Collection::Artist(a)))))
            .collect(),
        Source::Albums => mpd.albums().into_iter()
            .map(|a| Row::new(a.clone(), Action::Open(Source::Contents(Collection::Album(a, None)))))
            .collect(),
        Source::Playlists => mpd.playlists().into_iter()
            .map(playlist_row)
            .collect(),
        Source::Contents(c) => {
            // The whole library is not offered as one block
            let mut rows = match c {
                Collection::Folder(p) if p.is_empty() => Vec::new(),
                _ => {
                    let noun = match c {
                        Collection::Album(..) => "album",
                        Collection::Artist(_) => "artist",
                        Collection::Folder(_) => "folder",
                        Collection::Playlist(_) => "playlist",
                    };
                    vec![
                        Row::new(format!("> Play {noun}"), Action::Play(c.clone())),
                        Row::new(format!("+ Add {noun}"), Action::Add(c.clone())),
                    ]
                }
            };
            // An artist lists their albums, the others their songs
            if let Collection::Artist(artist) = c {
                rows.extend(mpd.artist_albums(artist).into_iter().map(|a| {
                    Row::new(a.clone(), Action::Open(Source::Contents(Collection::Album(a, Some(artist.clone())))))
                }));
                return Listing { rows, cursor };
            }
            rows.extend(mpd.contents(c).into_iter().map(|e| match e {
                Entry::Dir(d) => Row::new(format!("{}/", mpd::basename(&d)), Action::Open(Source::Contents(Collection::Folder(d)))),
                Entry::Song(t) => Row::new(track_label(&t), Action::AddFile(t.file)),
                Entry::Playlist(p) => playlist_row(p),
            }));
            rows
        }
        Source::Network => {
            cursor = None;
            let mut rows = vec![Row::new(format!("Host: {}", system::hostname()), Action::Info)];
            let ifaces = system::interfaces();
            if ifaces.is_empty() {
                rows.push(Row::new("No network", Action::Info));
            }
            for i in ifaces {
                if i.wifi {
                    rows.push(Row::new(format!("Wi-Fi: {}", i.name), Action::Info));
                    if let Some((ssid, signal)) = system::wifi_network() {
                        rows.push(Row::new(format!("{ssid} ({signal}%)"), Action::Info));
                    }
                } else {
                    rows.push(Row::new(format!("Wired: {}", i.name), Action::Info));
                }
                rows.push(Row::new(format!("IP: {}", i.addr), Action::Info));
            }
            rows
        }
        Source::Bluetooth => {
            cursor = None;
            match bluetooth::adapter() {
                None => vec![Row::new("No adapter", Action::Info)],
                Some(a) => {
                    let onoff = if a.powered { "On" } else { "Off" };
                    // bluez only makes a powered controller discoverable
                    let visible = match (a.powered, a.discoverable) {
                        (false, _) => Row::new("Visible: -", Action::Info),
                        (true, d) => Row::new(format!("Visible: {}", if d { "Yes" } else { "No" }),
                            Action::Bluetooth(a.path.clone(), "Discoverable", !d)),
                    };
                    vec![
                        Row::new(format!("Name: {}", a.alias), Action::Info),
                        Row::new(format!("Bluetooth: {onoff}"), Action::Bluetooth(a.path.clone(), "Powered", !a.powered)),
                        visible,
                    ]
                }
            }
        }
        Source::Confirm(p) => vec![Row::new("No", Action::Back), Row::new("Yes", Action::Power(*p))],
    };
    Listing { rows, cursor }
}

/// One level of the navigation.
struct Screen {
    title: String,
    rows: Vec<Row>,
    pos: ListPos,
}

impl Screen {
    fn new(title: impl Into<String>, rows: Vec<Row>) -> Self {
        Self { title: title.into(), rows, pos: ListPos::default() }
    }
}

const FEEDBACK_TIME: Duration = Duration::from_millis(1500);

/// Cursor + scroll offset of a list, keeping the cursor visible.
#[derive(Default)]
struct ListPos {
    cursor: usize,
    scroll: usize,
}

impl ListPos {
    fn step(&mut self, up: bool, down: bool, len: usize, visible: usize) {
        if len == 0 {
            return;
        }
        if down {
            self.cursor = (self.cursor + 1).min(len - 1);
            if self.cursor >= self.scroll + visible {
                self.scroll = self.cursor + 1 - visible;
            }
        }
        if up {
            self.cursor = self.cursor.saturating_sub(1);
            self.scroll = self.scroll.min(self.cursor);
        }
    }

    /// Cursor on `row`, scrolled so that it is in view.
    fn at(row: usize, len: usize, visible: usize) -> Self {
        let cursor = row.min(len.saturating_sub(1));
        Self { cursor, scroll: cursor.saturating_sub(visible / 2).min(len.saturating_sub(visible)) }
    }
}

pub struct PlaylistEditor {
    mpd: Arc<MpdClient>,
    layout: &'static Layout,
    /// The menu first; the last screen is the one shown.
    screens: Vec<Screen>,
    /// Query filling the last screen; dropped when the user backs out of it.
    loading: Option<Receiver<Listing>>,
    feedback: Option<(&'static str, Instant)>,
}

impl PlaylistEditor {
    pub fn new(mpd: Arc<MpdClient>, layout: &'static Layout) -> Self {
        let mut ed = Self { mpd, layout, screens: Vec::new(), loading: None, feedback: None };
        ed.open_library();
        ed
    }

    fn library() -> Screen {
        let rows = vec![
            Row::new("Queue", Action::Open(Source::Queue)),
            Row::new("Artists", Action::Open(Source::Artists)),
            Row::new("Albums", Action::Open(Source::Albums)),
            Row::new("Folders", Action::Open(Source::Contents(Collection::Folder(String::new())))),
            Row::new("Playlists", Action::Open(Source::Playlists)),
            Row::new("Update database", Action::UpdateDatabase),
        ];
        Screen::new("LIBRARY", rows)
    }

    fn settings() -> Screen {
        let rows = vec![
            Row::new("Network", Action::Open(Source::Network)),
            Row::new("Bluetooth", Action::Open(Source::Bluetooth)),
            Row::new("Restart", Action::Open(Source::Confirm(Power::Restart))),
            Row::new("Power off", Action::Open(Source::Confirm(Power::Off))),
        ];
        Screen::new("SETTINGS", rows)
    }

    /// Runs `job` on the worker; its listing fills the last screen.
    fn load(&mut self, job: impl FnOnce(&MpdClient) -> Listing + Send + 'static) {
        let (tx, rx) = mpsc::channel();
        let mpd = Arc::clone(&self.mpd);
        thread::spawn(move || {
            let _ = tx.send(job(&mpd));
        });
        self.loading = Some(rx);
    }

    /// Shows the source's list, filled when its query returns.
    fn open(&mut self, source: Source) {
        self.screens.push(Screen::new(source.title(), Vec::new()));
        self.load(move |mpd| list(mpd, &source));
    }

    /// Picks up a finished library query.
    fn poll(&mut self) {
        let Some(rx) = &self.loading else { return };
        match rx.try_recv() {
            Ok(listing) => {
                let screen = self.screens.last_mut().expect("menu screen");
                let cursor = listing.cursor.unwrap_or(screen.pos.cursor);
                screen.pos = ListPos::at(cursor, listing.rows.len(), self.layout.visible_lines);
                screen.rows = listing.rows;
                self.loading = None;
            }
            Err(mpsc::TryRecvError::Disconnected) => self.loading = None,
            Err(mpsc::TryRecvError::Empty) => {}
        }
    }

    fn reset(&mut self, root: Screen) {
        self.screens = vec![root];
        self.loading = None;
        self.feedback = None;
    }

    /// The editor always opens on one of its two menus.
    pub fn open_library(&mut self) {
        self.reset(Self::library());
    }

    pub fn open_settings(&mut self) {
        self.reset(Self::settings());
    }

    /// Shows msg for a moment, then goes back to the list.
    fn set_feedback(&mut self, msg: &'static str) {
        self.feedback = Some((msg, Instant::now()));
    }

    fn run(&mut self, action: Action) -> Outcome {
        match action {
            Action::Open(source) => self.open(source),
            Action::Play(c) => {
                self.mpd.clear_and_play(&c);
                self.set_feedback("Play !");
            }
            Action::Add(c) => {
                self.mpd.add(&c);
                self.set_feedback(match c {
                    Collection::Album(..) => "Album added !",
                    Collection::Artist(_) => "Artist added !",
                    Collection::Folder(_) => "Folder added !",
                    Collection::Playlist(_) => "Playlist added !",
                });
            }
            Action::PlayPos(pos) => {
                self.mpd.play_pos(pos);
                self.set_feedback("Play !");
            }
            Action::AddFile(file) => {
                self.mpd.add_track(&file);
                self.set_feedback("Track added !");
            }
            Action::UpdateDatabase => {
                self.mpd.update_database();
                self.set_feedback("Updating...");
            }
            Action::Info => {}
            Action::Back => return self.back(),
            Action::Power(p) => return Outcome::Power(p),
            Action::Bluetooth(path, prop, on) => {
                // Then re-read the controller: the rows show its new state
                self.load(move |mpd| {
                    bluetooth::set_adapter(&path, prop, on);
                    list(mpd, &Source::Bluetooth)
                });
            }
        }
        Outcome::Stay
    }

    /// One level up; out of the editor from a menu.
    fn back(&mut self) -> Outcome {
        if self.screens.len() == 1 {
            return Outcome::Exit;
        }
        self.screens.pop();
        self.loading = None;
        Outcome::Stay
    }

    pub fn handle_input(&mut self, up: bool, down: bool, select: bool, back: bool) -> Outcome {
        self.poll();
        if self.feedback.is_some() {
            if select || back {
                self.feedback = None;
            }
            return Outcome::Stay;
        }
        if back {
            return self.back();
        }
        if self.loading.is_some() {
            return Outcome::Stay;
        }
        let visible = self.layout.visible_lines;
        let screen = self.screens.last_mut().expect("menu screen");
        screen.pos.step(up, down, screen.rows.len(), visible);
        if select {
            if let Some(row) = screen.rows.get(screen.pos.cursor) {
                let action = row.action.clone();
                return self.run(action);
            }
        }
        Outcome::Stay
    }

    fn row(&self, cv: &mut Canvas, fonts: &Fonts, row: usize, label: &str, selected: bool) {
        let l = self.layout;
        let y = l.list_y0 + row as i32 * l.line_h;
        let text = fonts.truncate(label, l.row_text_w, l.size);
        if selected {
            let (dy, h, w) = l.selection;
            cv.rect(0, y - dy, w, h, true, 15);
            fonts.render(cv, &text, 2, y, l.size, true, 15, None); // dark on light
        } else {
            fonts.draw(cv, 2, y, &text, l.size, 15);
        }
    }

    fn scrollbar(&self, cv: &mut Canvas, total: usize, offset: usize) {
        let l = self.layout;
        if total <= l.visible_lines {
            return;
        }
        let (top, bottom) = (l.list_y0 - l.scrollbar_top_dy, l.list_y0 + l.visible_lines as i32 * l.line_h - 1);
        let h = bottom - top;
        let thumb = (h * l.visible_lines as i32 / total as i32).max(l.scrollbar_min);
        let ty = top + h * offset as i32 / total as i32;
        let x = cv.width() - 1;
        cv.line(x, top, x, bottom, 15);
        cv.rect(x + 1 - l.scrollbar_w, ty, l.scrollbar_w, thumb, true, 15);
    }

    fn header(&self, cv: &mut Canvas, fonts: &Fonts, text: &str) {
        let l = self.layout;
        fonts.draw(cv, 0, l.header_y, text, l.size, 15);
        cv.line(0, l.header_y + 2, cv.width() - 1, l.header_y + 2, 15);
    }

    fn render_browser(&mut self, cv: &mut Canvas, fonts: &Fonts) {
        let l = self.layout;
        cv.clear();
        if let Some((msg, ts)) = self.feedback {
            if ts.elapsed() > FEEDBACK_TIME {
                self.feedback = None;
            } else {
                let (x, y, w, h) = l.feedback_box;
                cv.rect(x, y, w, h, false, 15);
                let i = l.feedback_inset;
                cv.rect(x + i, y + i, w - 2 * i, h - 2 * i, false, 15);
                fonts.draw_centered(cv, cv.width() / 2, l.feedback_baseline, msg, l.size, 15);
                return;
            }
        }
        let screen = self.screens.last().expect("menu screen");
        self.header(cv, fonts, &fonts.truncate(&screen.title, l.row_text_w - 2, l.size));
        let message = |cv: &mut Canvas, (x, y): (i32, i32), text| fonts.draw(cv, x, y, text, l.size, 15);
        // A list refreshed in place stays up meanwhile
        if self.loading.is_some() && screen.rows.is_empty() {
            message(cv, l.loading_at, "Loading...");
            return;
        }
        if screen.rows.is_empty() {
            message(cv, l.empty_at, "Empty");
            return;
        }
        let (scroll, cursor) = (screen.pos.scroll, screen.pos.cursor);
        for (i, row) in screen.rows.iter().enumerate().skip(scroll).take(l.visible_lines) {
            self.row(cv, fonts, i - scroll, &row.label, i == cursor);
        }
        self.scrollbar(cv, screen.rows.len(), scroll);
    }

    fn render_info(&self, cv: &mut Canvas, fonts: &Fonts, snap: &FrameSnapshot) {
        let l = self.layout;
        cv.clear();
        let w = cv.width() - 2;
        fonts.draw(cv, 1, l.info_lines.0, &fonts.truncate(&snap.track.title, w, l.size), l.size, 15);
        fonts.draw(cv, 1, l.info_lines.1, &fonts.truncate(&snap.track.artist, w, l.size), l.size, 15);
        cv.line(0, l.info_rule_y, cv.width() - 1, l.info_rule_y, 15);
        let axis = l.spectrum_axis_y;
        for (i, (left, right)) in snap.spectrum_large_l.iter().zip(&snap.spectrum_large_r).enumerate() {
            let x = i as i32 * 2;
            let (hl, hr) = (left * l.spectrum_h / 255, right * l.spectrum_h / 255);
            cv.rect(x, axis - 1 - hl, 1, 1 + hl, true, 15);
            cv.rect(x, axis + 1 + l.spectrum_gap, 1, 1 + hr, true, 15);
        }
        cv.line(0, axis + l.spectrum_gap, cv.width() - 1, axis + l.spectrum_gap, 15);
        let screen = self.screens.last().expect("menu screen");
        let sel = match screen.rows.get(screen.pos.cursor).map(|r| &r.action) {
            Some(Action::Open(_)) => "SEL:open ",
            Some(Action::Play(_) | Action::PlayPos(_)) => "SEL:play ",
            Some(Action::UpdateDatabase | Action::Power(_)) => "SEL:run ",
            Some(Action::Bluetooth(..)) => "SEL:switch ",
            Some(Action::Back) => "SEL:back ",
            Some(Action::Info) => "",
            Some(Action::Add(_) | Action::AddFile(_)) => "SEL:add ",
            None => "",
        };
        let back = if self.screens.len() == 1 { "BCK:quit" } else { "BCK:back" };
        fonts.draw(cv, 0, l.help_y, &format!("{sel}{back}"), l.size, 15);
    }
}

impl Viz for PlaylistEditor {
    fn name(&self) -> &'static str {
        "Playlist Editor"
    }
    fn tier(&self) -> Tier {
        Tier::Editor
    }
    fn as_editor(&mut self) -> Option<&mut PlaylistEditor> {
        Some(self)
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.poll();
        self.render_browser(f.left, f.fonts);
        self.render_info(f.right, f.fonts, snap);
    }
}
