//! The hidden tier (hold VIZ): Spectrum XXL and La Poitrine.

use crate::viz::{Beat, Frame, Rng, Tier, Viz, norm7};
use crate::display::Canvas;
use crate::font::Fonts;
use crate::snapshot::FrameSnapshot;
use super::{disc, shaded_ellipse};
use crate::ui::Titles;

// ---- Spectrum XXL ----
// The 7-band spectrum, each band a shaded cartoon phallus at the classic bar
// positions. Length follows the band with asymmetric smoothing (fast rise,
// slow droop); balls squash on bass transients; sustained full extension
// fires a droplet spurt, followed by a refractory cooldown.

const MAX_DROPS: usize = 48; // per panel

#[derive(Default, Clone, Copy)]
struct Drop {
    x: f32,
    y: f32,
    vx: f32,
    vy: f32,
    life: f32,
    alive: bool,
}

struct XxlChannel {
    len: [f32; 7],
    beat: Beat, // level = balls' jiggle
    drops: [Drop; MAX_DROPS],
    next: usize,
    spurt: [f32; 7],
    cool: [f32; 7],
    acc: [f32; 7],
}

impl Default for XxlChannel {
    fn default() -> Self {
        Self {
            len: [0.0; 7],
            beat: Beat::default(),
            drops: [Drop::default(); MAX_DROPS],
            next: 0,
            spurt: [0.0; 7],
            cool: [0.0; 7],
            acc: [0.0; 7],
        }
    }
}

pub struct SpectrumXxl {
    ch: [XxlChannel; 2],
    rng: Rng,
    titles: Titles,
}

impl SpectrumXxl {
    // Shaft cylinder shading, one value per column: dark limbs, bright core,
    // sheen just left of centre.
    const SHAFT_LV: [u8; 15] = [2, 3, 5, 8, 11, 14, 15, 14, 12, 10, 8, 6, 4, 3, 2];
    const FOLD_RY: [f32; 3] = [8.6, 7.2, 5.8];
    const FOLD_LV: [u8; 3] = [12, 9, 6];

    pub fn new() -> Self {
        Self { ch: Default::default(), rng: Rng::new(0xFACA_DE01), titles: Titles::default() }
    }

    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, spec: &[i32; 7], left: bool, snap: &FrameSnapshot) {
        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);
        let dt = snap.dt;
        let st = &mut self.ch[!left as usize];
        let band = norm7(spec);
        st.beat.update(band[0].max(band[1]), dt, 3.0); // balls squash on the beat

        for i in 0..7 {
            let target = band[i];
            let rate = if target > st.len[i] { 12.0 } else { 3.5 }; // fast rise, slow droop
            st.len[i] += (target - st.len[i]) * (dt * rate).min(1.0);

            let cx = 15 + i as i32 * 34;
            let base_y = 110;
            let h = 12 + (st.len[i] * 66.0) as i32;
            let tip = (base_y - h).max(32); // glans top stays clear of the title

            for dx in -7..=7 {
                cv.line(cx + dx, tip + 6, cx + dx, base_y, Self::SHAFT_LV[(dx + 7) as usize]);
            }
            if h > 46 {
                cv.line(cx - 2, base_y - 8, cx - 1, tip + 12, 5); // vein
            }

            // Balls in front of the shaft root, lit from the upper left.
            let sq = st.beat.level * 1.5;
            shaded_ellipse(cv, cx - 7, 115, 8.0 + sq, 8.5 - sq, -0.3, -0.35);
            shaded_ellipse(cv, cx + 7, 115, 8.0 + sq, 8.5 - sq, -0.3, -0.35);

            // Glans: elongated dome, highlight up toward the meatus.
            shaded_ellipse(cv, cx, tip, 8.0, 9.5, 0.0, -0.45);
            // Corona: nested arcs fading inward, a gradient of skin.
            for fo in 0..3 {
                let frx = 7.2 - fo as f32 * 1.1;
                let mut t = 0.35f32;
                while t < 2.7915 {
                    let (ax, ay) = (cx + (frx * t.cos()) as i32, tip + (Self::FOLD_RY[fo] * t.sin()) as i32);
                    if ay > tip + 2 {
                        cv.pixel(ax, ay, Self::FOLD_LV[fo]);
                    }
                    t += 0.08;
                }
            }
            cv.line(cx - 8, tip + 6, cx - 7, tip + 9, 12); // sulcus
            cv.line(cx + 8, tip + 6, cx + 7, tip + 9, 12);
            // Meatus: black slit with lit edges.
            cv.line(cx - 1, tip - 7, cx - 1, tip - 2, 9);
            cv.line(cx + 1, tip - 7, cx + 1, tip - 2, 9);
            cv.line(cx, tip - 7, cx, tip - 2, 0);

            // Climax, then a refractory cooldown.
            st.cool[i] = (st.cool[i] - dt).max(0.0);
            if st.len[i] > 0.92 && st.cool[i] <= 0.0 {
                st.spurt[i] = 0.28;
                st.cool[i] = 1.8;
                st.beat.level = 1.0;
            }
            if st.spurt[i] > 0.0 {
                st.spurt[i] -= dt;
                st.acc[i] += dt * 50.0; // droplets per second
                while st.acc[i] >= 1.0 {
                    st.acc[i] -= 1.0;
                    let rng = &mut self.rng;
                    st.drops[st.next] = Drop {
                        x: cx as f32,
                        y: (tip - 11) as f32,
                        vx: (rng.next() - 0.5) * 44.0,
                        vy: -(80.0 + rng.next() * 70.0),
                        life: 1.0 + rng.next() * 0.4,
                        alive: true,
                    };
                    st.next = (st.next + 1) % MAX_DROPS;
                }
            }
        }

        // Droplets: ballistic, drawn last so they fly over everything.
        for q in st.drops.iter_mut().filter(|q| q.alive) {
            q.life -= dt;
            q.vy += 230.0 * dt;
            q.x += q.vx * dt;
            q.y += q.vy * dt;
            if q.life <= 0.0 || q.y > 127.0 {
                q.alive = false;
                continue;
            }
            let lvl = ((4.0 + 11.0 * q.life) as i32).clamp(3, 15) as u8;
            cv.pixel(q.x as i32, q.y as i32, lvl);
            cv.pixel(q.x as i32, q.y as i32 + 1, lvl >> 1); // tail
        }
    }
}

impl Viz for SpectrumXxl {
    fn name(&self) -> &'static str {
        "Spectrum XXL"
    }
    fn tier(&self) -> Tier {
        Tier::Hidden
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.draw(f.left, f.fonts, &snap.spectrum_l, true, snap);
        self.draw(f.right, f.fonts, &snap.spectrum_r, false, snap);
    }
}

// ---- La Poitrine ----
// Each breast is a 2D damped harmonic oscillator. Bass transients inject an
// impulse (drop + lateral kick alternating sides), then the springs
// overshoot and settle; the two sides have different stiffness so they
// never move in lockstep. Sustained bass adds a near-resonant shake, treble
// perks the nipples (fast rise, slow droop), energy drives a breathing lift.

#[derive(Clone, Copy)]
struct Breast {
    ox: f32,
    oy: f32,
    vx: f32,
    vy: f32,
    k: f32,
    c: f32,
}

struct PoitrineChannel {
    b: [Breast; 2],
    env: f32,
    perk: f32,
    breath: f32,
    shake_t: f32,
    kick_cool: f32, // refractory after a kick
    flip: bool,     // alternating kick side
}

impl Default for PoitrineChannel {
    fn default() -> Self {
        let spring = |k, c| Breast { ox: 0.0, oy: 0.0, vx: 0.0, vy: 0.0, k, c };
        Self {
            b: [spring(85.0, 4.0), spring(110.0, 4.6)], // softer left, stiffer right
            env: 0.05,
            perk: 0.0,
            breath: 0.0,
            shake_t: 0.0,
            kick_cool: 0.0,
            flip: false,
        }
    }
}

#[derive(Default)]
pub struct Poitrine {
    ch: [PoitrineChannel; 2],
    titles: Titles,
}

impl Poitrine {
    fn draw(&mut self, cv: &mut Canvas, fonts: &Fonts, spec: &[i32; 7], left: bool, snap: &FrameSnapshot) {
        cv.clear();
        self.titles.draw(cv, fonts, left, 5, snap);
        let dt = snap.dt;
        let st = &mut self.ch[!left as usize];

        let band = norm7(spec);
        let energy = band.iter().sum::<f32>() / 7.0;
        let bass = band[0].max(band[1]);
        let treble = (band[5] + band[6]) * 0.5;

        let rate = if treble > st.perk { 10.0 } else { 1.2 }; // perk: fast rise, slow droop
        st.perk += (treble - st.perk) * (dt * rate).min(1.0);

        st.breath += dt * (0.8 + 2.5 * energy);
        let lift = st.breath.sin() * (1.5 + 4.0 * energy);

        // One impulse per transient (refractory), not one per frame.
        st.env += (bass - st.env) * (dt * 1.5).min(1.0);
        st.kick_cool = (st.kick_cool - dt).max(0.0);
        let kick = bass > 0.22 && bass > st.env * 1.45 && st.kick_cool <= 0.0;
        if kick {
            st.kick_cool = 0.15;
        }
        st.shake_t += dt;

        let pdt = dt.min(0.033); // physics stability
        for (i, br) in st.b.iter_mut().enumerate() {
            if kick {
                br.vy += 75.0;
                let dir = if st.flip { 30.0 } else { -30.0 };
                br.vx += dir * if i == 0 { 1.0 } else { 0.8 };
            }
            // Spring-damper + a near-resonant shake while the bass rolls.
            let ax = -br.k * br.ox - br.c * br.vx;
            let ay = -br.k * br.oy - br.c * br.vy
                + bass * 120.0 * (st.shake_t * (9.5 + 3.0 * i as f32) + i as f32).sin();
            br.vx += ax * pdt;
            br.vy += ay * pdt;
            br.ox += br.vx * pdt;
            br.oy += br.vy * pdt;
            if br.ox.abs() > 12.0 {
                br.ox = br.ox.signum() * 12.0;
                br.vx = 0.0;
            }
            if br.oy.abs() > 12.0 {
                br.oy = br.oy.signum() * 12.0;
                br.vy = 0.0;
            }
        }
        if kick {
            st.flip = !st.flip;
        }

        let r = 36.0 * (1.0 + 0.02 * st.breath.sin());
        for (i, br) in st.b.iter().enumerate() {
            let side = if i == 1 { 1 } else { -1 };
            let bx = (120.0 + side as f32 * 34.0 + br.ox) as i32;
            let by = (78.0 + lift + br.oy) as i32;
            shaded_ellipse(cv, bx, by, r, r * 1.05, -0.25, -0.35);
            // Nipple on the lower-outer quadrant; perk lifts and sharpens it.
            let (nx, ny) = (bx + side * 9, by + 14 - (4.0 * st.perk) as i32);
            disc(cv, nx, ny, 7.0 + st.perk, 4, 2); // areola
            disc(cv, nx, ny, 2.2 + 2.0 * st.perk, 13, 9); // nipple
            if st.perk > 0.6 {
                cv.pixel(nx - 1, ny - 1, 15); // glint
            }
        }
    }
}

impl Viz for Poitrine {
    fn name(&self) -> &'static str {
        "La Poitrine"
    }
    fn tier(&self) -> Tier {
        Tier::Hidden
    }
    fn render(&mut self, f: &mut Frame, snap: &FrameSnapshot) {
        self.draw(f.left, f.fonts, &snap.spectrum_l, true, snap);
        self.draw(f.right, f.fonts, &snap.spectrum_r, false, snap);
    }
}
