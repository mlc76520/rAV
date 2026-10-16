//! Offline render check: every visualization of every profile is driven
//! for a second with synthetic audio and written as a PNG (left | right),
//! as the panel would show it (SSD1309 frames are thresholded like the
//! driver does). Needs no hardware:
//!     cargo test --release render -- --nocapture
//! Images land in target/render/.

use crate::display::Canvas;
use crate::font::Fonts;
use crate::mpd::MpdClient;
use crate::profile::{PanelKind, Profile, SSD1309, SSD1322};
use crate::snapshot::{FrameSnapshot, NowPlaying};
use crate::viz::Frame;
use std::sync::Arc;

fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

/// 8-bit greyscale PNG with stored (uncompressed) deflate blocks.
fn png(width: usize, height: usize, pixels: &[u8]) -> Vec<u8> {
    let mut raw = Vec::with_capacity((width + 1) * height);
    for row in pixels.chunks(width) {
        raw.push(0); // filter: none
        raw.extend_from_slice(row);
    }
    let mut z = vec![0x78, 0x01];
    let chunks: Vec<&[u8]> = raw.chunks(65535).collect();
    for (i, c) in chunks.iter().enumerate() {
        z.push((i + 1 == chunks.len()) as u8);
        z.extend_from_slice(&(c.len() as u16).to_le_bytes());
        z.extend_from_slice(&(!(c.len() as u16)).to_le_bytes());
        z.extend_from_slice(c);
    }
    let (mut a, mut b) = (1u32, 0u32);
    for &x in &raw {
        a = (a + x as u32) % 65521;
        b = (b + a) % 65521;
    }
    z.extend_from_slice(&((b << 16) | a).to_be_bytes());

    let mut out = b"\x89PNG\r\n\x1a\n".to_vec();
    let mut chunk = |kind: &[u8], data: &[u8]| {
        out.extend_from_slice(&(data.len() as u32).to_be_bytes());
        let mut body = kind.to_vec();
        body.extend_from_slice(data);
        out.extend_from_slice(&body);
        out.extend_from_slice(&crc32(&body).to_be_bytes());
    };
    let mut ihdr = (width as u32).to_be_bytes().to_vec();
    ihdr.extend_from_slice(&(height as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 0, 0, 0, 0]); // 8-bit grey
    chunk(b"IHDR", &ihdr);
    chunk(b"IDAT", &z);
    chunk(b"IEND", &[]);
    out
}

/// Both panels side by side, 2x enlarged, with a grey separator.
fn save(profile: &Profile, name: &str, left: &Canvas, right: &Canvas) {
    const SCALE: usize = 2;
    const GAP: usize = 4;
    let (w, h) = (left.width() as usize, left.height() as usize);
    let (ow, oh) = ((2 * w * SCALE) + GAP, h * SCALE);
    let mut img = vec![60u8; ow * oh];
    for (panel, cv) in [left, right].into_iter().enumerate() {
        for y in 0..h {
            for x in 0..w {
                let lv = cv.buf[y * w + x];
                let v = match profile.panel {
                    PanelKind::Ssd1309 => if lv >= 8 { 255 } else { 0 },
                    PanelKind::Ssd1322 => lv * 17,
                };
                for dy in 0..SCALE {
                    for dx in 0..SCALE {
                        img[(y * SCALE + dy) * ow + panel * (w * SCALE + GAP) + x * SCALE + dx] = v;
                    }
                }
            }
        }
    }
    let file = format!("target/render/{}_{}.png", profile.name, name.replace(' ', "_"));
    std::fs::create_dir_all("target/render").unwrap();
    std::fs::write(&file, png(ow, oh, &img)).unwrap();
    println!("wrote {file}");
}

/// A loud, moving signal: bass-heavy spectrum sweeping with time.
fn fake_frame(snap: &mut FrameSnapshot, t: f32) {
    let n = snap.spectrum_large_l.len();
    for (i, v) in snap.spectrum_large_l.iter_mut().enumerate() {
        let f = i as f32 / n as f32;
        *v = ((200.0 * (1.0 - f) + 60.0 * (6.0 * f + 3.0 * t).sin()).max(0.0)) as i32;
    }
    for (i, v) in snap.spectrum_large_r.iter_mut().enumerate() {
        let f = i as f32 / n as f32;
        *v = ((170.0 * (1.0 - f) + 70.0 * (5.0 * f - 2.0 * t).cos()).max(0.0)) as i32;
    }
    for i in 0..7 {
        snap.spectrum_l[i] = (230 - i as i32 * 25 + (40.0 * (t * 3.0 + i as f32).sin()) as i32).clamp(0, 255);
        snap.spectrum_r[i] = (210 - i as i32 * 22 + (40.0 * (t * 2.0 - i as f32).cos()) as i32).clamp(0, 255);
    }
    snap.vu_l = snap.spectrum_l.iter().sum::<i32>() / 7;
    snap.vu_r = snap.spectrum_r.iter().sum::<i32>() / 7;
    let w = snap.waveform_l.len();
    for i in 0..w {
        let x = i as f32 / w as f32 * std::f32::consts::TAU;
        snap.waveform_l[i] = 0.6 * (3.0 * x + 5.0 * t).sin() + 0.2 * (17.0 * x).sin();
        snap.waveform_r[i] = 0.5 * (2.0 * x - 4.0 * t).sin() + 0.25 * (11.0 * x).cos();
    }
}

fn render_profile(profile: &'static Profile) {
    // The profile's own font, from the source tree (cargo runs tests in rust/).
    let own = match profile.panel {
        PanelKind::Ssd1309 => "../trixel-square.ttf",
        PanelKind::Ssd1322 => "../RPI5_SSD1322_Vizualiser_20260412_unified/FreePixel.ttf",
    };
    let fonts = std::iter::once(&own).chain(profile.font_paths.iter())
        .find_map(|p| Fonts::load(p, profile.font_px).ok())
        .expect("font");
    let mpd = MpdClient::new("localhost", 6600); // never started
    let mut vizs = (profile.vizs)(Arc::clone(&mpd));
    let mut snap = FrameSnapshot::new(profile.width, profile.bands.len());
    snap.track = Arc::new(NowPlaying {
        formatted_text: "03. Eikthyr Rides - Valheim OST (2021)".into(),
        track_number: "3".into(),
        title: "Eikthyr Rides".into(),
        artist: "Valheim OST".into(),
        album: "Valheim".into(),
        date: "2021".into(),
        bitrate: "160 kbps".into(),
        format: "48kHz/16bit/2ch".into(),
        elapsed_ms: 83_000,
        total_s: 214,
        is_playing: true,
        ..Default::default()
    });
    snap.elapsed_s = 83;
    snap.total_s = 214;
    let (mut left, mut right) = (Canvas::new(profile.width, profile.height), Canvas::new(profile.width, profile.height));
    for viz in vizs.iter_mut() {
        for frame in 0..60 {
            snap.dt = 1.0 / 60.0;
            fake_frame(&mut snap, frame as f32 / 60.0);
            viz.render(&mut Frame { left: &mut left, right: &mut right, fonts: &fonts }, &snap);
        }
        save(profile, viz.name(), &left, &right);
    }
    let mut cv = Canvas::new(profile.width, profile.height);
    (profile.volume_overlay)(&mut cv, &fonts, 65);
    save(profile, "volume overlay", &cv, &cv.clone());
}

#[test]
fn render() {
    render_profile(&SSD1309);
    render_profile(&SSD1322);
}
