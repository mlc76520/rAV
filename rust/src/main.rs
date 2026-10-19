//! Dual OLED audio visualizer for the Raspberry Pi: SSD1309 128x64 mono or
//! SSD1322 240x128 greyscale panels, chosen by the display profile.
//!
//! Audio from PipeWire (sink monitor), now-playing from MPD and Bluetooth
//! (BlueZ), front panel on GPIO, displays on spidev.

mod app;
mod audio;
mod bands;
mod bluetooth;
mod display;
mod font;
mod hw;
mod mpd;
mod profile;
mod rotary;
mod snapshot;
mod system;
mod transition;
mod ui;
mod viz;

use std::sync::atomic::Ordering;

extern "C" fn on_signal(_: libc::c_int) {
    app::SHUTDOWN.store(true, Ordering::Relaxed);
}

fn main() {
    unsafe {
        libc::signal(libc::SIGINT, on_signal as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, on_signal as *const () as libc::sighandler_t);
    }
    let mut app = match profile::load().and_then(app::App::new) {
        Ok(app) => app,
        Err(e) => {
            eprintln!("Error: {e:#}");
            std::process::exit(1);
        }
    };
    app.run();
    app.shutdown();
    println!("Clean exit.");
}

#[cfg(test)]
mod render_test;
