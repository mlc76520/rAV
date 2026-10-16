//! Playlist editor: browse albums (left panel), open one to play it, queue
//! it or queue single tracks; the right panel shows what is playing over a
//! mirrored spectrum. Library queries run on a worker thread and hand their
//! result back through a channel, polled each frame. The logic is shared;
//! each panel family supplies its Layout.

use super::{Frame, Tier, Viz};
use crate::display::Canvas;
use crate::font::{Fonts, Size};
use crate::mpd::{MpdClient, TrackInfo};
use crate::snapshot::FrameSnapshot;
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
    pub empty_at: (i32, i32), // "No album"
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

#[derive(PartialEq, Clone, Copy)]
enum State {
    Albums,
    Tracks,
    Feedback,
}

enum Loaded {
    Albums(Vec<String>),
    Tracks(Vec<TrackInfo>),
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
}

pub struct PlaylistEditor {
    mpd: Arc<MpdClient>,
    layout: &'static Layout,
    state: State,
    /// Where the feedback message returns to.
    feedback_return: State,
    albums: Vec<String>,
    albums_loaded: bool,
    tracks: Vec<TrackInfo>,
    loading: Option<Receiver<Loaded>>,
    album_pos: ListPos,
    track_pos: ListPos,
    selected_album: String,
    feedback: &'static str,
    feedback_ts: Instant,
}

impl PlaylistEditor {
    pub fn new(mpd: Arc<MpdClient>, layout: &'static Layout) -> Self {
        Self {
            mpd,
            layout,
            state: State::Albums,
            feedback_return: State::Albums,
            albums: Vec::new(),
            albums_loaded: false,
            tracks: Vec::new(),
            loading: None,
            album_pos: ListPos::default(),
            track_pos: ListPos::default(),
            selected_album: String::new(),
            feedback: "",
            feedback_ts: Instant::now(),
        }
    }

    fn load(&mut self, job: impl FnOnce(&MpdClient) -> Loaded + Send + 'static) {
        if self.loading.is_some() {
            return;
        }
        let (tx, rx) = mpsc::channel();
        let mpd = Arc::clone(&self.mpd);
        thread::spawn(move || {
            let _ = tx.send(job(&mpd));
        });
        self.loading = Some(rx);
    }

    /// Picks up a finished library query.
    fn poll(&mut self) {
        let Some(rx) = &self.loading else { return };
        match rx.try_recv() {
            Ok(Loaded::Albums(a)) => {
                self.albums = a;
                self.albums_loaded = true;
                self.loading = None;
            }
            Ok(Loaded::Tracks(t)) => {
                self.tracks = t;
                self.loading = None;
            }
            Err(mpsc::TryRecvError::Disconnected) => self.loading = None,
            Err(mpsc::TryRecvError::Empty) => {}
        }
    }

    pub fn on_activate(&mut self) {
        if !self.albums_loaded {
            self.load(|m| Loaded::Albums(m.albums()));
        }
    }

    /// Shows msg for a moment, then goes back to the list it came from.
    fn set_feedback(&mut self, msg: &'static str) {
        self.feedback = msg;
        self.feedback_ts = Instant::now();
        self.feedback_return = self.state;
        self.state = State::Feedback;
    }

    /// Returns true when the user backs out of the editor.
    pub fn handle_input(&mut self, up: bool, down: bool, select: bool, back: bool) -> bool {
        self.poll();
        let loading = self.loading.is_some();
        let visible = self.layout.visible_lines;
        match self.state {
            State::Feedback => {
                if select || back {
                    self.state = self.feedback_return;
                }
                false
            }
            State::Albums => {
                if !loading && !self.albums.is_empty() {
                    self.album_pos.step(up, down, self.albums.len(), visible);
                    if select {
                        self.selected_album = self.albums[self.album_pos.cursor].clone();
                        self.track_pos = ListPos::default();
                        self.state = State::Tracks;
                        let album = self.selected_album.clone();
                        self.load(move |m| Loaded::Tracks(m.album_tracks(&album)));
                    }
                }
                back
            }
            State::Tracks => {
                if back {
                    self.state = State::Albums;
                    return false;
                }
                if loading {
                    return false;
                }
                self.track_pos.step(up, down, 2 + self.tracks.len(), visible);
                if select {
                    match self.track_pos.cursor {
                        0 => {
                            self.mpd.clear_and_play_album(&self.selected_album);
                            self.set_feedback("Play !");
                        }
                        1 => {
                            self.mpd.add_album(&self.selected_album);
                            self.set_feedback("Album added !");
                        }
                        n => {
                            if let Some(t) = self.tracks.get(n - 2) {
                                self.mpd.add_track(&t.file);
                                self.set_feedback("Track added !");
                            }
                        }
                    }
                }
                false
            }
        }
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
        let loading = self.loading.is_some();
        let message = |cv: &mut Canvas, (x, y): (i32, i32), text| fonts.draw(cv, x, y, text, l.size, 15);
        if !self.albums_loaded && loading {
            message(cv, l.loading_at, "Loading...");
            return;
        }
        match self.state {
            State::Feedback => {
                if self.feedback_ts.elapsed() > FEEDBACK_TIME {
                    self.state = self.feedback_return;
                }
                let (x, y, w, h) = l.feedback_box;
                cv.rect(x, y, w, h, false, 15);
                let i = l.feedback_inset;
                cv.rect(x + i, y + i, w - 2 * i, h - 2 * i, false, 15);
                fonts.draw_centered(cv, cv.width() / 2, l.feedback_baseline, self.feedback, l.size, 15);
            }
            State::Albums => {
                self.header(cv, fonts, "LIBRARY");
                if self.albums.is_empty() {
                    message(cv, l.empty_at, "No album");
                    return;
                }
                let (scroll, cursor) = (self.album_pos.scroll, self.album_pos.cursor);
                for (i, album) in self.albums.iter().enumerate().skip(scroll).take(l.visible_lines) {
                    self.row(cv, fonts, i - scroll, album, i == cursor);
                }
                self.scrollbar(cv, self.albums.len(), scroll);
            }
            State::Tracks => {
                let header = fonts.truncate(&self.selected_album, l.row_text_w - 2, l.size);
                self.header(cv, fonts, &header);
                if loading {
                    message(cv, l.loading_at, "Loading...");
                    return;
                }
                let total = 2 + self.tracks.len();
                let (scroll, cursor) = (self.track_pos.scroll, self.track_pos.cursor);
                for idx in scroll..(scroll + l.visible_lines).min(total) {
                    let label = match idx {
                        0 => "> Play album".to_string(),
                        1 => "+ Add album".to_string(),
                        n => {
                            let t = &self.tracks[n - 2];
                            if t.track_num.is_empty() { t.title.clone() } else { format!("{}. {}", t.track_num, t.title) }
                        }
                    };
                    self.row(cv, fonts, idx - scroll, &label, idx == cursor);
                }
                self.scrollbar(cv, total, scroll);
            }
        }
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
        let help = if self.state == State::Albums { "SEL:enter BCK:quit" } else { "SEL:add BCK:back" };
        fonts.draw(cv, 0, l.help_y, help, l.size, 15);
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
