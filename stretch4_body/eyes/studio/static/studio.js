'use strict';
/*
  Eyes Studio front end. No dependencies.

  1. EyeEngine: a port of the PIMU firmware's EyeAnimationManager, so the preview
     shows what the firmware renders from the last command.
  2. Firmware overrides from LightBarManager::step (low SOC half rings, runstop blink).
  3. Drawing: two S4-RING-LED V0.1 boards with 5050 WS2812B packages, one hotspot per
     pixel for hover and keyboard, and the left/right swap setting.
  4. Server link: ordered request queue, /api/eyes writes merged and spaced 50 ms.
  5. Controls.
*/
(() => {

// ---------------------------------------------------------------------------
// 1. Firmware port.
//    Ported from stretch_firmware_ii arduino/hello_pimu2/EyeAnimationManager.h and
//    EyeAnimationManager.cpp at origin/develop f97f1b9 (hello-pimu2 v0.1.13p13),
//    checked function by function on 2026-10-07. Names and constants follow the
//    firmware. snap() is the one addition: page load and the effect thumbnails
//    start on an animation without the crossfade the firmware would run.
// ---------------------------------------------------------------------------
const PORT_SOURCE = 'EyeAnimationManager.cpp, stretch_firmware_ii develop f97f1b9';

const NOP = 0, OFF = 1, IDLE_GLOW = 2, BLINK = 3, LOOK_LEFT = 4, LOOK_RIGHT = 5,
  RAINBOW_SPIN = 6, ALERT = 7, HAPPY = 8, LEFT_HALF = 9, RIGHT_HALF = 10,
  TOP_HALF = 11, BOTTOM_HALF = 12, CIRCLE_CW = 13, CIRCLE_CCW = 14, ANIM_COUNT = 15;
const EYE_FPS = 50;              // 250 Hz main loop / EYE_UPDATE_DIVISOR 5
const EYE_BLEND_FRAMES = 15;     // 300 ms crossfade
const FRAME_MS = 1000 / EYE_FPS;
const TWO_PI = 6.2831853;        // EYE_TWO_PI
const f32 = Math.fround;         // the SAMD51 renders in single precision
const u8 = (x) => Math.trunc(f32(x)) & 0xFF;   // C (uint8_t) cast: truncation, not rounding

function fmodf(a, b) { return a - b * Math.trunc(a / b); }

// Integer HSV to RGB, all channels 0..255, as EyeAnimationManager::hsv2rgb.
function hsv2rgbFw(h, s, v) {
  const region = Math.floor(h / 43);
  const remainder = ((h - region * 43) * 6) & 0xFF;
  const p = (v * (255 - s)) >> 8;
  const q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  const t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
  switch (region) {
    case 0: return [v, t, p];
    case 1: return [q, v, p];
    case 2: return [p, v, t];
    case 3: return [p, q, v];
    case 4: return [t, p, v];
    default: return [v, p, q];
  }
}

class EyeEngine {
  constructor(n = 10) {
    this.n = n;
    // Ring positions in pixel units, viewed from the front: EYE_POS_TOP 9.5 (between
    // index 9 and 0), EYE_POS_RIGHT 2.0 (index 2), EYE_POS_LEFT 7.0 (index 7) at n = 10.
    this.posTop = n - 0.5;
    this.posRight = n * 0.2;
    this.posLeft = n * 0.7;
    this.intensity = 255; this.r = 255; this.g = 255; this.b = 255;
    this.ovr = { active: false, left: 0, right: 0, intensity: 255, r: 255, g: 255, b: 255 };
    // Constructor state: both eyes OFF, no blend. setup() then commands IDLE_GLOW (40, 48, 60).
    this.eye = [0, 1].map(() => ({ current: OFF, target: OFF, frame: 0, buf: new Uint8Array(n * 3) }));
    this.blend = [0, 1].map(() => ({ snapshot: new Uint8Array(n * 3), alpha: 255, step: Math.floor(255 / EYE_BLEND_FRAMES), active: false }));
    this.out = [new Uint8Array(n * 3), new Uint8Array(n * 3)];
  }

  // setAnimation: colour and intensity always land, a target only when it is not NOP and in range.
  setAnimation(left, right, intensity, r, g, b) {
    this.intensity = intensity; this.r = r; this.g = g; this.b = b;
    if (left !== NOP && left < ANIM_COUNT) this.eye[0].target = left;
    if (right !== NOP && right < ANIM_COUNT) this.eye[1].target = right;
  }

  setOverride(active, left = 0, right = 0, intensity = 255, r = 255, g = 255, b = 255) {
    const o = this.ovr;
    o.active = active; o.left = left; o.right = right; o.intensity = intensity; o.r = r; o.g = g; o.b = b;
  }

  // Not in firmware: start an eye on an animation with no fade (page load, thumbnails).
  snap(e, anim) {
    const s = this.eye[e];
    s.current = s.target = anim; s.frame = 0;
    this.blend[e].active = false;
  }

  // One 50 Hz frame: EyeAnimationManager::step after the 250 Hz decimation.
  step() {
    for (let e = 0; e < 2; e++) {
      const s = this.eye[e], bl = this.blend[e], o = this.ovr;
      let target = s.target, I = this.intensity, R = this.r, G = this.g, B = this.b;
      if (o.active) {
        target = e === 0 ? o.left : o.right;
        I = o.intensity; R = o.r; G = o.g; B = o.b;
      }
      // A new effective target starts a crossfade, but only once the previous one has
      // finished: a change during a fade waits, it does not restart the fade.
      if (target !== s.current && !bl.active) {
        bl.snapshot.set(s.buf);
        bl.alpha = 0;
        bl.step = Math.max(1, Math.floor(255 / EYE_BLEND_FRAMES));   // 17
        bl.active = true;
        s.current = target;
        s.frame = 0;
      }
      this.render(e, I, R, G, B);
      s.frame = (s.frame + 1) & 0xFFFF;   // uint16_t
      if (bl.active) {
        const a = bl.alpha + bl.step;
        if (a >= 255) { bl.alpha = 255; bl.active = false; } else bl.alpha = a;
      }
    }
    this.commit();
  }

  // commitPixels: (snapshot * (255 - alpha) + buf * alpha) >> 8 while a fade runs.
  commit() {
    for (let e = 0; e < 2; e++) {
      const bl = this.blend[e], buf = this.eye[e].buf, out = this.out[e];
      if (bl.active) {
        const a = bl.alpha, ia = 255 - a;
        for (let k = 0; k < out.length; k++) out[k] = (bl.snapshot[k] * ia + buf[k] * a) >> 8;
      } else {
        out.set(buf);
      }
    }
  }

  // renderAnimation and the render* functions. Every renderer ends with
  //   float scale = ((float)_intensity / 255.0f) * v;  buf = (uint8_t)(_r * scale)
  // which put() reproduces in single precision with truncation.
  render(e, I, R, G, B) {
    const n = this.n, buf = this.eye[e].buf, f = this.eye[e].frame, half = n / 2;
    const put = (i, v) => {
      const scale = f32(f32(I / 255) * f32(v));
      buf[i * 3] = u8(R * scale); buf[i * 3 + 1] = u8(G * scale); buf[i * 3 + 2] = u8(B * scale);
    };
    const all = (v) => { for (let i = 0; i < n; i++) put(i, v); };
    const ringDist = (i, p) => { let d = Math.abs(i - p); if (d > half) d = n - d; return d; };
    // Look left/right pupil: v = max(0, 1 - d * 0.5) squared, two pixels wide.
    const pupil = (p) => {
      for (let i = 0; i < n; i++) { const v = Math.max(0, 1 - ringDist(i, p) * 0.5); put(i, v * v); }
    };
    // Circle bar: v = clamp(2 - d, 0, 1) squared, three pixels wide.
    const bar = (p) => {
      for (let i = 0; i < n; i++) { const v = Math.min(1, Math.max(0, 2 - ringDist(i, p))); put(i, v * v); }
    };
    // Look: smoothstep over 25 frames (0.5 s), then hold.
    const ease = () => { const t = f < 25 ? f / 25 : 1; return t * t * (3 - 2 * t); };

    switch (this.eye[e].current) {
      case IDLE_GLOW: {          // renderIdleGlow: 300 frame period, hold 200, cosine dip over 100, floor 5%
        const cf = f % 300;
        let br = 1;
        if (cf >= 200) br = (Math.cos((cf - 200) / 100 * TWO_PI) + 1) * 0.5;
        all(0.05 + br * 0.95);
        break;
      }
      case BLINK: {              // renderBlink: 150 frame period, close over 10, open over 10, hold
        const cf = f % 150;
        all(cf < 10 ? 1 - cf / 10 : cf < 20 ? (cf - 10) / 10 : 1);
        break;
      }
      case LOOK_LEFT:            // renderLookLeft: 9.5 down to 7.0, anticlockwise
        pupil(fmodf(this.posTop - ease() * (this.posTop - this.posLeft) + n, n));
        break;
      case LOOK_RIGHT:           // renderLookRight: 9.5 up to 2.0 through the top gap, clockwise
        pupil(fmodf(this.posTop + ease() * (this.posRight - this.posTop + n), n));
        break;
      case RAINBOW_SPIN: {       // renderRainbowSpin: hue advances 3 per frame, ignores RGB, intensity floor 40
        const base = (f * 3) & 0xFF, v = I > 40 ? I : 40;
        for (let i = 0; i < n; i++) {
          const rgb = hsv2rgbFw((base + Math.floor(i * 255 / n)) & 0xFF, 255, v);
          buf.set(rgb, i * 3);
        }
        break;
      }
      case ALERT:                // renderAlert: 15 frame (300 ms) sine pulse in the commanded colour
        all((Math.sin((f % 15) / 15 * TWO_PI) + 1) * 0.5);
        break;
      case HAPPY: {              // renderHappy: 100 frame period, burst from 9.5 over 30 frames, fade to 10% over 70
        const cf = f % 100, radius = (cf < 30 ? cf / 30 : 1) * half;
        for (let i = 0; i < n; i++) {
          const d = ringDist(i, this.posTop);
          let v = d <= radius ? 1 - (d / Math.max(radius, 0.1)) * 0.5 : 0;
          if (cf >= 30) v *= Math.max(1 - (cf - 30) / 70, 0.1);
          put(i, v);
        }
        break;
      }
      case LEFT_HALF: for (let i = 0; i < n; i++) put(i, i >= 5 && i <= 9 ? 1 : 0); break;
      case RIGHT_HALF: for (let i = 0; i < n; i++) put(i, i >= 0 && i <= 4 ? 1 : 0); break;
      case TOP_HALF: for (let i = 0; i < n; i++) put(i, i === 8 || i === 9 || i === 0 || i === 1 || i === 2 ? 1 : 0); break;
      case BOTTOM_HALF: for (let i = 0; i < n; i++) put(i, i >= 3 && i <= 7 ? 1 : 0); break;
      case CIRCLE_CW:            // renderCircleCW: 0.5 rev/s, 100 frames per turn, increasing index
        bar(fmodf(this.posTop + f / (EYE_FPS / 0.5) * n, n));
        break;
      case CIRCLE_CCW:           // renderCircleCCW: decreasing index, +10000 keeps fmodf positive
        bar(fmodf(this.posTop - f / (EYE_FPS / 0.5) * n + 1000 * n, n));
        break;
      default:                   // renderOff, also the default branch for anything unknown
        buf.fill(0);
    }
  }
}

// Logical index to place on the WS2812B chain (EyeAnimationManager::commitPixels):
// left eye chain 8 + (i + 7) % 10, right eye 18 + (i + 3) % 10.
const PHYS_OFFSET = [7, 3];
const CHAIN_OFFSET = [8, 18];
function chainIndex(e, i, n) { return CHAIN_OFFSET[e] + (i + PHYS_OFFSET[e]) % n; }
function designator(e, i, n) { return (i + PHYS_OFFSET[e]) % n + 1; }   // LEDn as printed in the board files

// ---------------------------------------------------------------------------
// 2. Overrides, from LightBarManager::step (same commit) and Pimu.cpp.
//    SOC <= 25: setOverride(true, LEFT_HALF, RIGHT_HALF, 255, PX_YELLOW), <= 12 PX_RED,
//    else setOverride(false); each change crossfades like a command.
//    Runstop: after every rendered eye frame all 20 eye pixels (chain 8..27) are
//    overwritten with one colour while runstop_led_on, black otherwise; the runstop
//    LED toggles every 500 ms (runstop_manager.toggle_led(500)). Colour PX_BAT_COLOR,
//    PX_YELLOW at SOC <= 25, PX_RED at SOC <= 12.
// ---------------------------------------------------------------------------
const PX_YELLOW = [110, 30, 0], PX_RED = [64, 0, 0], PX_BAT_COLOR = [40, 40, 40];
const RUNSTOP_TOGGLE_MS = 500;

const sim = {
  engine: new EyeEngine(10),
  ms: 0, acc: 0, lastToggle: 0, runstopLed: false,
  runstop: false, soc: 100, preview: 'live',
};

function overrideInputs() {
  let runstop = server.runstop_active === true;
  let soc = typeof server.battery_soc === 'number' ? server.battery_soc
    : server.low_soc_override === 'red' ? 12 : server.low_soc_override === 'yellow' ? 25 : 100;
  if (sim.preview === 'runstop') runstop = true;
  if (sim.preview === 'soc25') soc = 25;
  if (sim.preview === 'soc12') soc = 12;
  return { runstop, soc };
}

function simFrame() {
  // Five 4 ms main-loop ticks per eye frame; the runstop LED toggles every 500 ms.
  for (let k = 0; k < 5; k++) {
    sim.ms += 4;
    if (sim.ms - sim.lastToggle > RUNSTOP_TOGGLE_MS) { sim.lastToggle = sim.ms; sim.runstopLed = !sim.runstopLed; }
  }
  const { runstop, soc } = overrideInputs();
  sim.runstop = runstop; sim.soc = soc;
  if (soc <= 25) sim.engine.setOverride(true, LEFT_HALF, RIGHT_HALF, 255, ...(soc <= 12 ? PX_RED : PX_YELLOW));
  else sim.engine.setOverride(false);
  sim.engine.step();
  for (const t of thumbs) t.engine.step();
}

// What the 20 eye pixels show after the runstop overwrite.
function shownPixels(e) {
  if (!sim.runstop) return sim.engine.out[e];
  const n = sim.engine.n, px = new Uint8Array(n * 3);
  if (sim.runstopLed) {
    const c = sim.soc <= 12 ? PX_RED : sim.soc <= 25 ? PX_YELLOW : PX_BAT_COLOR;
    for (let i = 0; i < n; i++) px.set(c, i * 3);
  }
  return px;
}

// ---------------------------------------------------------------------------
// 3. Drawing. LED drive is linear in PWM duty, so a byte value is light output;
//    the screen shows it through the sRGB transfer curve.
// ---------------------------------------------------------------------------
const SRGB = new Uint8Array(256);
for (let i = 0; i < 256; i++) {
  const c = i / 255;
  SRGB[i] = Math.round(255 * (c <= 0.0031308 ? 12.92 * c : 1.055 * Math.pow(c, 1 / 2.4) - 0.055));
}
const LENS_BASE = [228, 220, 198];   // unlit lens, #E4DCC6

// Screen colour of pixel i in a GRB-free RGB buffer: its sRGB value, its brightness
// (0..1) and its chroma (the same hue and saturation at full brightness), so a dim
// pixel is drawn as a faintly lit lens of its colour, not as a dark one. null when off.
function litPixel(px, i) {
  const r = SRGB[px[i * 3]], g = SRGB[px[i * 3 + 1]], b = SRGB[px[i * 3 + 2]];
  const max = Math.max(r, g, b);
  if (max <= 0) return null;
  const s = 255 / max;
  const chroma = [Math.round(r * s), Math.round(g * s), Math.round(b * s)];
  return { peak: max / 255, rgb: chroma.join(','), chroma };
}

// Board geometry in millimetres from the released S4-RING-LED V0.1 files (pick and
// place, GM1 outline, V0 DXF): ten 5050 packages on an 11.45 mm radius at a 36 degree
// pitch, 8.0 mm centre hole, 15.0 mm outer radius, so about 1 mm beside each package
// and no room for silkscreen on the ring. The connector tab is 24 mm wide and reaches
// 21 mm; the two JST S3B-EH right-angle connectors (CN1 in, CN2 out) sit on the
// bottom side with their pins 16.335 mm from the centre and their housings
// overhanging the tab edge to 23 mm.
const MM = {
  ringIn: 8.0, ringOut: 15.0,
  tabHalf: 12.0, tabOut: 21.0, tabCorner: 0.5,
  cnRadial: 16.335, cnLateral: 6.15, cnPitch: 2.5, cnWidth: 10.0, cnOut: 23.0,
  ledR: 11.45, body: 5.0, window: 4.0, glow: 9,
};
const EYE_NAME = ['Left eye', 'Right eye'];
const SWAP_KEY = 'eyes-studio.swap';
// Orientation. The firmware mapping (commitPixels) implies the left board is fitted
// rotated 18 degrees clockwise and the right board 162 degrees, which puts both
// connector tabs toward the middle of the head. Verified by camera on robot 4000 on
// 2026-10-07: the firmware's "left" eye is the viewer's left, pixel 0 sits just
// clockwise of 12 o'clock and the index runs clockwise, circle_cw turns clockwise as
// the viewer sees it. view.swap flips left and right for an odd build, stored per browser.
const view = { swap: false };
try { view.swap = localStorage.getItem(SWAP_KEY) === '1'; } catch (err) { /* storage blocked */ }
const sideOf = (e) => (view.swap ? 1 - e : e);        // firmware eye -> drawn side, 0 = viewer's left
const eyeAtSide = (s) => (view.swap ? 1 - s : s);

function ledAngle(i, n) { return ((i + 0.5) / n) * Math.PI * 2; }   // clockwise from 12 o'clock
// Tab direction for the board drawn on a side: clock angle of the gap between LED10
// and LED1 after the installed rotation, 108 degrees on the left, 252 on the right.
function tabAngleForSide(s, n) { return (((n - PHYS_OFFSET[s]) % n) / n) * Math.PI * 2; }

function fitCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(canvas.clientWidth * dpr));
  const h = Math.max(1, Math.round(canvas.clientHeight * dpr));
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; return true; }
  return false;
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y); ctx.lineTo(x + w - r, y); ctx.quadraticCurveTo(x + w, y, x + w, y + r);
  ctx.lineTo(x + w, y + h - r); ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
  ctx.lineTo(x + r, y + h); ctx.quadraticCurveTo(x, y + h, x, y + h - r);
  ctx.lineTo(x, y + r); ctx.quadraticCurveTo(x, y, x + r, y);
  ctx.closePath();
}

const stage = { canvas: null, board: null, layout: null };

// Ring centres 0.52 W apart and the scale chosen so the two 23 mm tab reaches
// (pointing 18 degrees below horizontal, toward the middle) never meet. Hv is the
// height left in view: the sequence drawer covers the lower window while it is
// open, and the rings move up and shrink into what stays visible.
function ringLayout(W, H, Hv = H) {
  const k = Math.min(W * 0.16, Hv * 0.34) / MM.ringOut;   // px per mm
  const cy = Hv * 0.46;   // clear of the override banner at the top
  return { k, cy, sides: [{ cx: W * 0.24, cy }, { cx: W * 0.76, cy }], W, H, Hv };
}

// Canvas pixels of the window not covered by the drawer.
function visibleWindowHeight(canvas) {
  const seq = document.getElementById('seq');
  if (!seq || seq.hidden || getComputedStyle(seq).position !== 'absolute') return canvas.height;
  const c = canvas.getBoundingClientRect(), d = seq.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  return Math.max(60, Math.min(canvas.height, Math.round((d.top - c.top - 6) * dpr)));
}

// Static layer: boards, pads, unlit packages, labels. Redrawn on resize and swap.
function drawBoards() {
  const { W, H, k, sides } = stage.layout, n = sim.engine.n;
  const c = stage.board, ctx = c.getContext('2d');
  c.width = W; c.height = H;
  ctx.clearRect(0, 0, W, H);
  const mono = 'ui-monospace, "DejaVu Sans Mono", Menlo, monospace';
  sides.forEach((p, s) => {
    const e = eyeAtSide(s);
    ctx.save();
    ctx.translate(p.cx, p.cy);
    const ta = tabAngleForSide(s, n);
    const g = ctx.createRadialGradient(-MM.ringOut * k * 0.3, -MM.ringOut * k * 0.4, 0, 0, 0, MM.cnOut * k);
    g.addColorStop(0, '#222428'); g.addColorStop(1, '#101114');
    ctx.save();
    ctx.rotate(ta);
    // Connector housings on the bottom side: from the front only the part beyond the
    // tab edge shows, so draw them first and let the board cover the rest.
    ctx.fillStyle = '#2C2D31';
    for (const sx of [-1, 1]) {
      roundRect(ctx, (sx * MM.cnLateral - MM.cnWidth / 2) * k, -MM.cnOut * k, MM.cnWidth * k, (MM.cnOut - MM.tabOut + 1.5) * k, 0.4 * k);
      ctx.fill();
    }
    ctx.lineWidth = Math.max(1, 0.25 * k); ctx.strokeStyle = '#2E3136';
    // Board outline: the r = 15 disc, then the tab, open where it meets the disc.
    ctx.fillStyle = g;
    ctx.beginPath(); ctx.arc(0, 0, MM.ringOut * k, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    const th = MM.tabHalf * k, to = MM.tabOut * k, tr = MM.tabCorner * k;
    const yj = Math.sqrt(MM.ringOut * MM.ringOut - MM.tabHalf * MM.tabHalf) * k;   // where the tab sides meet the arc
    ctx.beginPath();
    ctx.moveTo(-th, -yj); ctx.lineTo(-th, -to + tr); ctx.quadraticCurveTo(-th, -to, -th + tr, -to);
    ctx.lineTo(th - tr, -to); ctx.quadraticCurveTo(th, -to, th, -to + tr); ctx.lineTo(th, -yj);
    ctx.fill(); ctx.stroke();
    // CN1 and CN2 pins, plated through, seen from the top.
    for (const sx of [-1, 1]) {
      for (let j = -1; j <= 1; j++) {
        const x = (sx * MM.cnLateral + j * MM.cnPitch) * k, y = -MM.cnRadial * k;
        ctx.beginPath(); ctx.arc(x, y, 0.8 * k, 0, Math.PI * 2); ctx.fillStyle = '#C9A961'; ctx.fill();
        ctx.beginPath(); ctx.arc(x, y, 0.45 * k, 0, Math.PI * 2); ctx.fillStyle = '#0A0B0C'; ctx.fill();
      }
    }
    ctx.restore();
    // Centre hole.
    ctx.beginPath(); ctx.arc(0, 0, MM.ringIn * k, 0, Math.PI * 2);
    ctx.fillStyle = '#0A0B0C'; ctx.fill();
    ctx.lineWidth = Math.max(1, 0.25 * k); ctx.strokeStyle = '#2E3136'; ctx.stroke();
    for (let i = 0; i < n; i++) {
      const a = ledAngle(i, n);
      ctx.save();
      ctx.rotate(a);
      ctx.translate(0, -MM.ledR * k);
      const sz = MM.body * k, h = sz / 2;
      // Four gull-wing pads, two each side (tangential edges).
      ctx.fillStyle = '#B9A06A';
      for (const sx of [-1, 1]) for (const sy of [-0.28, 0.28]) {
        ctx.fillRect(sx * h - (sx > 0 ? 0 : 0.7 * k), sy * sz - 0.45 * k, 0.7 * k, 0.9 * k);
      }
      // PLCC body, pin-1 chamfer top left.
      const body = ctx.createLinearGradient(0, -h, 0, h);
      body.addColorStop(0, '#D8D2C4'); body.addColorStop(1, '#B4AD9E');
      roundRect(ctx, -h, -h, sz, sz, 0.35 * k);
      ctx.fillStyle = body; ctx.fill();
      ctx.beginPath(); ctx.moveTo(-h, -h + 1.1 * k); ctx.lineTo(-h + 1.1 * k, -h); ctx.lineTo(-h, -h); ctx.closePath();
      ctx.fillStyle = '#0F1012'; ctx.fill();
      // Reflector window with the three emitter dies and the driver die.
      ctx.beginPath(); ctx.arc(0, 0, MM.window / 2 * k, 0, Math.PI * 2);
      ctx.fillStyle = '#E4DCC6'; ctx.fill();
      ctx.lineWidth = Math.max(0.5, 0.12 * k); ctx.strokeStyle = '#A8A090'; ctx.stroke();
      if (k > 3) {
        const d = 0.45 * k;
        [['#7A3A30', -0.75], ['#3E6A40', 0], ['#3A4A78', 0.75]].forEach(([col, dx]) => {
          ctx.fillStyle = col; ctx.fillRect(dx * k - d / 2, -0.55 * k - d / 2, d, d);
        });
        ctx.fillStyle = '#55524C'; ctx.fillRect(-0.55 * k, 0.35 * k, 1.1 * k, 0.7 * k);
      }
      ctx.restore();
    }
    ctx.restore();
    // Labels below the ring, in window ink: which firmware eye this is and its chain pixels.
    ctx.fillStyle = '#C3BDB0';
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    const fs = Math.max(11, Math.min(15, 1.9 * k));
    ctx.font = `600 ${fs}px ui-sans-serif, system-ui, sans-serif`;
    const y = p.cy + MM.ringOut * k + 0.55 * fs;
    ctx.fillText(EYE_NAME[e], p.cx, y);
    ctx.font = `${fs * 0.85}px ${mono}`;
    ctx.fillStyle = '#8E887A';
    const first = CHAIN_OFFSET[e];
    ctx.fillText(`chain px ${first} to ${first + n - 1}`, p.cx, y + fs * 1.25);
  });
  placeSpots();
}

// One hotspot per pixel over the window: hover or focus names the logical index,
// a click pins the readout, arrow keys walk the ring.
const spot = { pinned: null, shown: null };

function spotText(e, i) {
  const n = sim.engine.n;
  let t = `${EYE_NAME[e]} px ${i}, chain px ${chainIndex(e, i, n)}`;
  if (!view.swap) t += `, LED${designator(e, i, n)} on the board`;
  return t;
}

function renderSpotReadout() {
  const el = $('led-readout');
  const s = spot.shown || spot.pinned;
  el.textContent = s ? spotText(s.e, s.i) + (spot.pinned === s && spot.shown !== s ? ' (pinned)' : '')
    : 'Tap or hover a pixel for its index, chain position and board LED.';
}

function buildSpots(box, n) {
  box.textContent = '';
  for (let e = 0; e < 2; e++) {
    for (let i = 0; i < n; i++) {
      const b = document.createElement('button');
      b.type = 'button'; b.className = 'led-spot';
      b.dataset.eye = String(e); b.dataset.i = String(i);
      b.tabIndex = e === 0 && i === 0 ? 0 : -1;
      b.setAttribute('aria-pressed', 'false');
      const show = () => { spot.shown = { e, i, el: b }; renderSpotReadout(); };
      const hide = () => { if (spot.shown && spot.shown.el === b) spot.shown = null; renderSpotReadout(); };
      b.addEventListener('mouseenter', show); b.addEventListener('focus', show);
      b.addEventListener('mouseleave', hide); b.addEventListener('blur', hide);
      b.addEventListener('click', () => {
        const same = spot.pinned && spot.pinned.el === b;
        if (spot.pinned) spot.pinned.el.setAttribute('aria-pressed', 'false');
        spot.pinned = same ? null : { e, i, el: b };
        b.setAttribute('aria-pressed', String(!same));
        renderSpotReadout();
      });
      box.append(b);
    }
  }
  if (box.dataset.bound) return;
  box.dataset.bound = '1';
  box.addEventListener('keydown', (ev) => {
    const spots = Array.from(box.children), idx = spots.indexOf(document.activeElement);
    if (idx < 0) return;
    if (ev.key === 'Escape' && spot.pinned) { spot.pinned.el.setAttribute('aria-pressed', 'false'); spot.pinned = null; renderSpotReadout(); ev.preventDefault(); return; }
    const d = { ArrowRight: 1, ArrowDown: 1, ArrowLeft: -1, ArrowUp: -1 }[ev.key];
    const to = d ? (idx + d + spots.length) % spots.length : ev.key === 'Home' ? 0 : ev.key === 'End' ? spots.length - 1 : -1;
    if (to < 0) return;
    spots[idx].tabIndex = -1; spots[to].tabIndex = 0; spots[to].focus();
    ev.preventDefault();
  });
}

function placeSpots() {
  const { k, sides } = stage.layout, n = sim.engine.n, dpr = window.devicePixelRatio || 1;
  const box = $('spots');
  if (box.childElementCount !== 2 * n) buildSpots(box, n);
  // Hit circle the size of the pixel pitch, at least 24 px: on a phone the whole ring is
  // only about 100 px across, so 44 px targets cannot sit ten to a ring.
  const size = Math.max(24, Math.min(56, (2 * Math.PI * MM.ledR / n) * k / dpr));
  for (const el of box.children) {
    const e = +el.dataset.eye, i = +el.dataset.i, p = sides[sideOf(e)], a = ledAngle(i, n);
    const x = (p.cx + MM.ledR * k * Math.sin(a)) / dpr, y = (p.cy - MM.ledR * k * Math.cos(a)) / dpr;
    el.style.left = `${x - size / 2}px`; el.style.top = `${y - size / 2}px`;
    el.style.width = `${size}px`; el.style.height = `${size}px`;
    el.setAttribute('aria-label', spotText(e, i));
  }
  renderSpotReadout();
}

function drawStage() {
  const canvas = stage.canvas;
  const resized = fitCanvas(canvas), hv = visibleWindowHeight(canvas);
  if (resized || !stage.layout || stage.layout.Hv !== hv) {
    stage.layout = ringLayout(canvas.width, canvas.height, hv);
    drawBoards();
  }
  const { W, H, k, sides } = stage.layout, n = sim.engine.n;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, W, H);
  ctx.drawImage(stage.board, 0, 0);
  for (let e = 0; e < 2; e++) {
    const p = sides[sideOf(e)];
    const px = shownPixels(e);
    for (let i = 0; i < n; i++) {
      const lit = litPixel(px, i);
      if (!lit) continue;
      const { peak, rgb } = lit;
      const a = ledAngle(i, n);
      const x = p.cx + MM.ledR * k * Math.sin(a), y = p.cy - MM.ledR * k * Math.cos(a);
      // Lit window: the emitted colour dominates the pale lens in proportion to its
      // brightness, then a hot core whitens strong pixels.
      const w = 0.3 + 0.7 * peak;
      const lens = LENS_BASE.map((c, j) => Math.round(c * (1 - w) + lit.chroma[j] * w));
      ctx.globalCompositeOperation = 'source-over';
      ctx.beginPath(); ctx.arc(x, y, MM.window / 2 * k, 0, Math.PI * 2);
      ctx.fillStyle = `rgb(${lens.join(',')})`; ctx.fill();
      ctx.globalCompositeOperation = 'lighter';
      const core = ctx.createRadialGradient(x, y, 0, x, y, MM.window / 2 * k);
      core.addColorStop(0, `rgba(255,255,255,${0.75 * peak * peak})`);
      core.addColorStop(1, 'rgba(255,255,255,0)');
      ctx.fillStyle = core; ctx.fill();
      // The white PLCC body scatters light, then a soft spill onto the board.
      const s = MM.body * k;
      ctx.fillStyle = `rgba(${rgb},${0.28 * peak})`;
      ctx.save(); ctx.translate(x, y); ctx.rotate(a);
      roundRect(ctx, -s / 2, -s / 2, s, s, 0.35 * k); ctx.fill();
      ctx.restore();
      const halo = ctx.createRadialGradient(x, y, MM.window / 2 * k, x, y, MM.glow * k);
      halo.addColorStop(0, `rgba(${rgb},${0.55 * peak})`);
      halo.addColorStop(0.35, `rgba(${rgb},${0.18 * peak})`);
      halo.addColorStop(1, `rgba(${rgb},0)`);
      ctx.fillStyle = halo;
      ctx.beginPath(); ctx.arc(x, y, MM.glow * k, 0, Math.PI * 2); ctx.fill();
    }
  }
  ctx.globalCompositeOperation = 'source-over';
}


// Mini rings: effect thumbnails (one eye), step chips and library previews (both
// eyes). Same engine, drawn as dots without the board.
function drawMiniRing(ctx, px, n, cx, cy, rr, dot) {
  ctx.globalCompositeOperation = 'source-over';
  ctx.beginPath(); ctx.arc(cx, cy, rr + dot * 1.6, 0, Math.PI * 2);
  ctx.arc(cx, cy, rr - dot * 1.6, 0, Math.PI * 2, true);
  ctx.fillStyle = '#17181B'; ctx.fill('evenodd');
  for (let i = 0; i < n; i++) {
    const a = ledAngle(i, n), x = cx + rr * Math.sin(a), y = cy - rr * Math.cos(a);
    ctx.globalCompositeOperation = 'source-over';
    ctx.fillStyle = '#5A564E';
    ctx.fillRect(x - dot, y - dot, dot * 2, dot * 2);
    const lit = litPixel(px, i);
    if (lit) {
      const { peak, rgb } = lit;
      const dotCol = lit.chroma.map((c) => Math.round(c * (0.35 + 0.65 * peak)));
      ctx.fillStyle = `rgb(${dotCol.join(',')})`;
      ctx.beginPath(); ctx.arc(x, y, dot * 0.85, 0, Math.PI * 2); ctx.fill();
      ctx.globalCompositeOperation = 'lighter';
      const halo = ctx.createRadialGradient(x, y, 0, x, y, dot * 3);
      halo.addColorStop(0, `rgba(${rgb},${0.6 * peak})`);
      halo.addColorStop(1, `rgba(${rgb},0)`);
      ctx.fillStyle = halo; ctx.beginPath(); ctx.arc(x, y, dot * 3, 0, Math.PI * 2); ctx.fill();
    }
  }
  ctx.globalCompositeOperation = 'source-over';
}

// Effect thumbnails: one engine per preset, left eye only, no overrides.
const thumbs = [];

function drawThumb(t) {
  const c = t.canvas;
  fitCanvas(c);
  const ctx = c.getContext('2d'), W = c.width;
  ctx.fillStyle = '#0A0B0C'; ctx.fillRect(0, 0, W, W);
  drawMiniRing(ctx, t.engine.out[0], t.engine.n, W / 2, W / 2, W * 0.33, W * 0.075);
}

// Both eyes side by side, in the drawn orientation (swap respected).
function drawPair(canvas, engine) {
  fitCanvas(canvas);
  const ctx = canvas.getContext('2d'), W = canvas.width, H = canvas.height;
  ctx.fillStyle = '#0A0B0C'; ctx.fillRect(0, 0, W, H);
  const rr = Math.min(H * 0.3, W * 0.15), dot = rr * 0.24;
  for (let s = 0; s < 2; s++) drawMiniRing(ctx, engine.out[eyeAtSide(s)], engine.n, W * (s ? 0.73 : 0.27), H / 2, rr, dot);
}

// A sequence on its own engine, for chips and library previews. Steps are resolved
// (every field filled in); a single step just runs its animation. wrap is the set the
// passes after the first play (see wrapSteps).
class MiniPlayer {
  constructor(steps, wrap = steps) {
    this.steps = steps; this.wrap = wrap; this.idx = 0; this.t = 0;
    this.engine = new EyeEngine(sim.engine.n);
    this.apply(true);
  }
  apply(snap) {
    const s = this.steps[this.idx];
    if (!s) return;
    this.engine.setAnimation(animId(s.left), animId(s.right), s.intensity, ...s.color);
    if (snap) { this.engine.snap(0, animId(s.left)); this.engine.snap(1, animId(s.right)); }
  }
  frame() {
    if (this.steps.length > 1) {
      this.t += FRAME_MS;
      if (this.t >= Math.max(50, this.steps[this.idx].hold * 1000)) {
        this.t = 0; this.idx = (this.idx + 1) % this.steps.length;
        if (this.idx === 0) this.steps = this.wrap;
        this.apply(false);
      }
    }
    this.engine.step();
  }
}
// Canvas -> MiniPlayer for every chip and library preview on the page. Only the
// visible ones are stepped and drawn.
const minis = new Map();
function visibleMinis() {
  const out = [];
  for (const [canvas, player] of minis) {
    if (!canvas.isConnected) { minis.delete(canvas); continue; }
    if (canvas.offsetParent) out.push([canvas, player]);
  }
  return out;
}

// ---------------------------------------------------------------------------
// 4. Server link.
// ---------------------------------------------------------------------------
let server = {};           // last /api/state
let caps = {};
let anims = [];
const animByName = {};
const animById = {};

const queue = [];
let draining = false, lastEyesPost = 0, lastLocalEdit = 0, lastCaps = 0;
const EYES_SPACING_MS = 50;

function sleep(ms) { return new Promise((r) => setTimeout(r, ms)); }

async function api(path, body) {
  const opts = body === undefined ? {} : {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  };
  const res = await fetch(path, opts);
  const data = await res.json().catch(() => ({}));
  if (!res.ok) {
    const err = new Error((data && data.error) || `${res.status} ${res.statusText}`);
    err.status = res.status;
    throw err;
  }
  return data;
}

// Writes go out in order; consecutive /api/eyes writes merge, latest value wins.
// A direct write stops a playing sequence on the server (Eyes.set does), so the
// preview drops its copy of the playback at once.
function request(path, body) {
  if (path === '/api/eyes' || path === '/api/off' || path === '/api/idle') stopLocalPlayback();
  const tail = queue[queue.length - 1];
  if (path === '/api/eyes' && tail && tail.path === '/api/eyes') Object.assign(tail.body, body);
  else queue.push({ path, body: { ...body } });
  lastLocalEdit = performance.now();
  drain();
}

async function drain() {
  if (draining) return;
  draining = true;
  try {
    while (queue.length) {
      const job = queue[0];
      if (job.path === '/api/eyes') {
        const wait = lastEyesPost + EYES_SPACING_MS - performance.now();
        if (wait > 0) { await sleep(wait); continue; }
        lastEyesPost = performance.now();
      }
      queue.shift();
      try {
        adoptState(await api(job.path, job.body));
        showError('');
      } catch (err) {
        showError(`Could not apply: ${err.message}`);
        if (job.path === '/api/play') { play.pending = null; seqMessage(`Could not play: ${err.message}`, true); }
      }
    }
  } finally {
    draining = false;
  }
}

async function poll() {
  try {
    const st = await api('/api/state');
    setLink(true);
    if (!queue.length && !draining) adoptState(st);
    if (performance.now() - lastCaps > 5000) {
      lastCaps = performance.now();
      const c = await api('/api/capabilities');
      if (JSON.stringify(c) !== JSON.stringify(caps)) { caps = c || {}; applyCapabilities(); renderControls(); }
    }
  } catch (err) {
    setLink(false);
  }
}

// 1 Hz, 4 Hz while a sequence plays so the running step is highlighted promptly.
async function pollLoop() {
  await poll();
  setTimeout(pollLoop, server.playing ? 250 : 1000);
}

// ---------------------------------------------------------------------------
// 5. Controls.
// ---------------------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const IDLE_NAME = 'idle_glow';
const SENTRY = 'sentry_eye_animations';
// left and right are animation names, or null while that eye has not been commanded
// through the API (the firmware then still shows whatever it had, idle_glow after boot).
const ui = { left: null, right: null, color: [40, 48, 60], intensity: 255, link: 'both' };
const hsv = { h: 0, s: 0, v: 0 };
let dragging = false;

const REASONS = {
  per_eye_color: 'RPC 29 carries one r, g, b for both eyes. No PIMU protocol, P13 or the P14 work in progress, has a per-eye color byte.',
  per_pixel: 'Needs a pixel frame RPC, which no PIMU protocol has yet. P13 plays its 14 presets.',
};

const PRESETS = [
  { label: 'Boot idle', left: 'idle_glow', right: 'idle_glow', color: '#28303c', intensity: 255 },
  { label: 'Attention', left: 'alert', right: 'alert', color: '#ff2a00', intensity: 255 },
  { label: 'Happy', left: 'happy', right: 'happy', color: '#20ff50', intensity: 220 },
  { label: 'Glance left', left: 'look_left', right: 'look_left', color: '#ffffff', intensity: 160 },
  { label: 'Glance right', left: 'look_right', right: 'look_right', color: '#ffffff', intensity: 160 },
  { label: 'Scanning', left: 'circle_cw', right: 'circle_ccw', color: '#00a0ff', intensity: 255 },
  { label: 'Rainbow', left: 'rainbow_spin', right: 'rainbow_spin', color: null, intensity: 255 },
  { label: 'Night', left: 'idle_glow', right: 'idle_glow', color: '#ff6a10', intensity: 40 },
];

const FACTORY_SWATCHES = [
  ['#28303c', 'Boot idle, 40 48 60'], ['#ffffff', 'White'], ['#ff0000', 'Red'],
  ['#ff6a10', 'Amber'], ['#20ff50', 'Green'], ['#00a0ff', 'Sky'], ['#ff1493', 'Hot pink'],
];
const SWATCH_KEY = 'eyes-studio.swatches';
const MAX_SAVED = 12;

// stretch4_body.eyes.colors.NAMED_COLORS, for previewing looks that name a color.
const NAMED_COLORS = {
  black: [0, 0, 0], white: [255, 255, 255], red: [255, 0, 0], green: [0, 255, 0], blue: [0, 0, 255],
  cyan: [0, 255, 255], magenta: [255, 0, 255], yellow: [255, 255, 0], orange: [255, 80, 0],
  purple: [128, 0, 255], pink: [255, 192, 203], hot_pink: [255, 105, 180], stretch: [40, 48, 60],
};

const hex2 = (x) => x.toString(16).padStart(2, '0');
const toHex = (rgb) => '#' + rgb.map(hex2).join('');
function parseHex(text) {
  const m = /^#?([0-9a-f]{6})$/i.exec(String(text).trim());
  if (!m) return null;
  return [0, 2, 4].map((i) => parseInt(m[1].slice(i, i + 2), 16));
}
const sameRgb = (a, b) => a && b && a[0] === b[0] && a[1] === b[1] && a[2] === b[2];

function hsvToRgb(h, s, v) {
  const f = (n) => { const k = (n + h / 60) % 6; return v - v * s * Math.max(0, Math.min(k, 4 - k, 1)); };
  return [f(5), f(3), f(1)].map((x) => Math.round(x * 255));
}
function rgbToHsv([r, g, b]) {
  r /= 255; g /= 255; b /= 255;
  const max = Math.max(r, g, b), min = Math.min(r, g, b), d = max - min;
  let h = hsv.h;
  if (d > 0) {
    if (max === r) h = 60 * (((g - b) / d) % 6);
    else if (max === g) h = 60 * ((b - r) / d + 2);
    else h = 60 * ((r - g) / d + 4);
    if (h < 0) h += 360;
  }
  return { h, s: max ? d / max : 0, v: max };
}

function animId(name) { return animByName[name] ? animByName[name].id : NOP; }
function animLabel(name) { return animByName[name] ? (animByName[name].label || name) : String(name); }
const previewName = (name) => name || IDLE_NAME;   // an eye not commanded yet is drawn as the boot default
const protoLabel = (v) => String(v).replace(/^p(\d+)/i, 'P$1');
const protoNumber = (v) => { const m = /^p?(\d+)/i.exec(String(v == null ? '' : v).trim()); return m ? parseInt(m[1], 10) : null; };
// Colour and intensity bytes exist from requires_protocol on. A protocol the backend
// cannot read is assumed new enough; an older API shape may say color: false outright.
function colourAllowed() {
  if (caps.color === false) return false;
  const have = protoNumber(caps.protocol_version), need = protoNumber(caps.requires_protocol || 'p13');
  return have === null || need === null || have >= need;
}

// Which animation the server says an eye holds, or null when it was never commanded.
// Handles both API shapes: left null, or left set with left_commanded false.
function commandedEye(st, key) {
  if (!st || st[key] == null || st[key + '_commanded'] === false) return null;
  return st[key];
}

// Push the UI model into the firmware model, as RPC 29 would.
function applyToEngine() {
  sim.engine.setAnimation(animId(previewName(ui.left)), animId(previewName(ui.right)), ui.intensity, ...ui.color);
  for (const t of thumbs) t.engine.setAnimation(NOP, NOP, ui.intensity, ...ui.color);
}

function adoptState(st) {
  server = st && typeof st === 'object' ? st : {};
  renderStatus();
  followPlayback();
  // While a sequence plays the preview follows its own copy of the timeline.
  if (play.local) return;
  // Only adopt server values when no local edit is queued or under way, so a
  // reply to an older write never drags the controls back mid-gesture.
  if (queue.length || dragging || performance.now() - lastLocalEdit < 400) return;
  const left = commandedEye(server, 'left'), right = commandedEye(server, 'right');
  const color = Array.isArray(server.color) ? server.color.slice(0, 3) : parseHex(server.color_hex) || ui.color;
  const intensity = typeof server.intensity === 'number' ? server.intensity : ui.intensity;
  const changed = left !== ui.left || right !== ui.right || !sameRgb(color, ui.color) || intensity !== ui.intensity;
  if (!changed) return;
  ui.left = left; ui.right = right; ui.intensity = intensity;
  setColor(color, { send: false });
  applyToEngine();
  renderControls();
}

function setLink(ok) {
  $('lamp-link').className = 'lamp ' + (ok ? 'ok' : 'fault');
  const backend = server.studio ? server.studio.backend : caps.backend;
  $('link-text').textContent = ok ? `Connected, ${backend === 'fake' ? 'fake backend' : 'robot'}` : 'No connection';
  renderConnection(ok);
}

function showError(msg) {
  const el = $('error-notice');
  el.textContent = msg;
  el.hidden = !msg;
}

function renderStatus() {
  const studio = server.studio || {};
  const held = !!studio.control;
  $('lamp-control').className = 'lamp ' + (held ? 'ok' : '');
  $('control-text').textContent = held ? 'Host control on' : 'Host control off';
  const btn = $('btn-control');
  btn.textContent = held ? 'Release control' : 'Take control';
  btn.setAttribute('aria-pressed', String(held));
  $('sentry-notice').hidden = !(server.sentry_active && !server.in_control);

  // Command lamp: did the last command reach the robot.
  const src = server.source;
  const holder = server.lease_holder || 'another client';
  const cmd = src === 'commanded' ? ['ok', 'Last command accepted']
    : src === 'dropped' ? ['fault', 'Last command dropped']
    : ['', 'No command sent yet'];
  $('lamp-command').className = 'lamp ' + cmd[0];
  $('command-text').textContent = cmd[1];
  const dn = $('dropped-notice');
  dn.textContent = src === 'dropped'
    ? `stretch_body_server dropped the last eye command: ${holder} holds the command lease. Wait for it to finish, then send again.` : '';
  dn.hidden = src !== 'dropped';
  const failed = !!studio.sentry_resume_failed;
  $('resume-notice').hidden = !failed;
  $('resume-text').textContent = failed
    ? `Control was released but ${SENTRY} could not be resumed: ${holder} holds the command lease. It stays paused until a resume succeeds.` : '';

  const rs = server.runstop_active;
  $('lamp-runstop').className = 'lamp ' + (rs ? 'fault' : '');
  $('runstop-text').textContent = rs === true ? 'on' : rs === false ? 'off' : 'unknown';
  const low = server.low_soc_override;
  $('lamp-soc').className = 'lamp ' + (low === 'red' ? 'fault' : low === 'yellow' ? 'warn' : '');
  const soc = typeof server.battery_soc === 'number' ? `, SOC ${server.battery_soc}%` : '';
  $('soc-text').textContent = (low ? `${low} half rings` : 'off') + soc;

  $('ro-source').textContent = src === 'assumed' ? 'no command yet, boot default assumed'
    : src === 'commanded' ? 'last command'
    : src === 'dropped' ? `dropped, lease held by ${holder}` : (src || 'unknown');
  renderBanner();
  renderPlayback();
  renderConnection(true);
}

function renderConnection(linked) {
  const dl = $('conn');
  if (!dl) return;
  const st = server.studio || {};
  const rows = [
    ['Link', linked ? `connected to ${location.host}` : 'no connection'],
    ['Backend', st.backend || caps.backend || 'unknown'],
    ['Writes', st.write_hz ? `${st.writes || 0} sent, merged to at most ${st.write_hz} Hz` : 'unknown'],
    ['Host control', st.control ? 'held by this studio' : 'not held'],
    ['Sentry', server.sentry_active == null ? 'not installed' : server.sentry_active ? 'running' : 'paused'],
    ['Lease', server.lease_holder || 'free'],
    ['Library', caps.library === false ? `unavailable: ${caps.library_error || 'no looks API'}` : 'built-in, shared and this robot'],
  ];
  const key = JSON.stringify(rows);
  if (dl.dataset.key === key) return;
  dl.dataset.key = key;
  dl.textContent = '';
  for (const [k, v] of rows) {
    const dt = document.createElement('dt'); dt.textContent = k;
    const dd = document.createElement('dd'); dd.textContent = v;
    dl.append(dt, dd);
  }
}

function renderBanner() {
  const { runstop, soc } = overrideInputs();
  const banner = $('override-banner');
  const tag = sim.preview === 'live' ? '' : ' (preview)';
  if (runstop) {
    banner.textContent = `Runstop: PIMU blinks every eye pixel${tag}`;
    banner.className = 'override-banner';
  } else if (soc <= 25) {
    banner.textContent = `Battery ${soc <= 12 ? 'critical' : 'low'}: PIMU shows ${soc <= 12 ? 'red' : 'yellow'} half rings${tag}`;
    banner.className = 'override-banner warn';
  }
  banner.hidden = !(runstop || soc <= 25);
  // The Overrides softkey carries a lamp, so a preview left on is never forgotten.
  $('lamp-ovr').className = 'lamp ' + (runstop || soc <= 12 ? 'fault' : soc <= 25 ? 'warn' : '');
}

function renderControls() {
  // Effects
  for (const t of thumbs) {
    const onL = ui.left === t.name, onR = ui.right === t.name;
    const pressed = ui.link === 'both' ? onL && onR : ui.link === 'left' ? onL : onR;
    t.button.setAttribute('aria-pressed', String(pressed));
    t.lampL.className = 'lamp ' + (onL ? 'ok' : '');
    t.lampR.className = 'lamp ' + (onR ? 'ok' : '');
  }
  // Readout
  const eyeText = (name) => (name ? animLabel(name) : 'not commanded yet, preview shows idle glow');
  $('ro-left').textContent = eyeText(ui.left);
  $('ro-right').textContent = eyeText(ui.right);
  $('ro-color').textContent = `${toHex(ui.color)} (${ui.color.join(' ')})`;
  $('ro-intensity').textContent = String(ui.intensity);
  // Intensity
  $('intensity').value = String(ui.intensity);
  $('intensity-out').textContent = String(ui.intensity);
  document.documentElement.style.setProperty('--fader-top', toHex(ui.color));
  // Colour use
  const colourUsed = [previewName(ui.left), previewName(ui.right)].some((n) => !animByName[n] || animByName[n].uses_color);
  $('colour-h').parentElement.classList.toggle('dimmed', !colourUsed);
  renderPicker();
  renderSwatches();
}

// --- Effects list ---
function buildEffects() {
  const list = $('effects');
  list.textContent = '';
  thumbs.length = 0;
  for (const a of anims) {
    const li = document.createElement('li');
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'fx-btn';
    btn.dataset.anim = a.name;
    btn.setAttribute('aria-pressed', 'false');
    btn.title = a.description || '';
    const canvas = document.createElement('canvas');
    canvas.setAttribute('aria-hidden', 'true');
    const name = document.createElement('span'); name.className = 'fx-name'; name.textContent = a.label || a.name;
    const desc = document.createElement('span'); desc.className = 'fx-desc'; desc.textContent = a.description || '';
    if (a.description) {
      desc.id = `fx-desc-${a.name}`;
      btn.setAttribute('aria-describedby', desc.id);
    }
    const eyes = document.createElement('span'); eyes.className = 'fx-eyes'; eyes.setAttribute('aria-hidden', 'true');
    const mk = (label) => { const s = document.createElement('span'); const l = document.createElement('span'); l.className = 'lamp'; s.append(l, label); eyes.append(s); return l; };
    const lampL = mk('L'), lampR = mk('R');
    btn.append(canvas, name, desc, eyes);
    btn.addEventListener('click', () => chooseEffect(a.name));
    li.append(btn);
    list.append(li);
    const engine = new EyeEngine(sim.engine.n);
    engine.setAnimation(NOP, NOP, ui.intensity, ...ui.color);
    engine.snap(0, a.id);
    thumbs.push({ name: a.name, engine, canvas, button: btn, lampL, lampR });
  }
}

function chooseEffect(name) {
  const body = {};
  if (ui.link !== 'right') { ui.left = name; body.left = name; }
  if (ui.link !== 'left') { ui.right = name; body.right = name; }
  applyToEngine();
  renderControls();
  request('/api/eyes', body);
}

// --- Colour ---
function setColor(rgb, { send = true, fromHsv = false } = {}) {
  ui.color = rgb.map((x) => Math.max(0, Math.min(255, Math.round(x))));
  if (!fromHsv) Object.assign(hsv, rgbToHsv(ui.color));
  if (send) {
    applyToEngine();
    request('/api/eyes', { color: toHex(ui.color) });
  }
  const hex = $('hex');
  if (document.activeElement !== hex) { hex.value = toHex(ui.color); hex.removeAttribute('aria-invalid'); }
  ['r', 'g', 'b'].forEach((ch, i) => {
    const el = $('rgb-' + ch);
    if (document.activeElement !== el) el.value = String(ui.color[i]);
  });
  if (send) renderControls();
}

function setHsv(h, s, v) {
  hsv.h = ((h % 360) + 360) % 360; hsv.s = Math.max(0, Math.min(1, s)); hsv.v = Math.max(0, Math.min(1, v));
  setColor(hsvToRgb(hsv.h, hsv.s, hsv.v), { fromHsv: true });
}

function renderPicker() {
  const ring = $('hue-ring'), sq = $('sv-square');
  fitCanvas(ring); fitCanvas(sq);
  // Hue ring
  let ctx = ring.getContext('2d');
  const W = ring.width, c = W / 2, ro = c - 2, ri = ro * 0.79;
  ctx.clearRect(0, 0, W, W);
  for (let d = 0; d < 360; d++) {
    const a0 = (d - 90.6) * Math.PI / 180, a1 = (d - 89.2) * Math.PI / 180;
    ctx.beginPath(); ctx.arc(c, c, ro, a0, a1); ctx.arc(c, c, ri, a1, a0, true); ctx.closePath();
    ctx.fillStyle = `hsl(${d},100%,50%)`; ctx.fill();
  }
  const ha = (hsv.h - 90) * Math.PI / 180, hr = (ro + ri) / 2, hw = (ro - ri) / 2;
  ctx.lineWidth = Math.max(2, W / 120);
  ctx.beginPath(); ctx.arc(c + hr * Math.cos(ha), c + hr * Math.sin(ha), hw * 0.8, 0, Math.PI * 2);
  ctx.strokeStyle = '#fff'; ctx.stroke();
  ctx.beginPath(); ctx.arc(c + hr * Math.cos(ha), c + hr * Math.sin(ha), hw * 0.8 + ctx.lineWidth, 0, Math.PI * 2);
  ctx.strokeStyle = 'rgba(0,0,0,0.6)'; ctx.lineWidth /= 2; ctx.stroke();
  ring.setAttribute('aria-valuenow', String(Math.round(hsv.h)));
  ring.setAttribute('aria-valuetext', `${Math.round(hsv.h)} degrees`);
  // Saturation / value square
  ctx = sq.getContext('2d');
  const S = sq.width;
  ctx.fillStyle = `hsl(${hsv.h},100%,50%)`; ctx.fillRect(0, 0, S, S);
  let g = ctx.createLinearGradient(0, 0, S, 0);
  g.addColorStop(0, '#fff'); g.addColorStop(1, 'rgba(255,255,255,0)');
  ctx.fillStyle = g; ctx.fillRect(0, 0, S, S);
  g = ctx.createLinearGradient(0, 0, 0, S);
  g.addColorStop(0, 'rgba(0,0,0,0)'); g.addColorStop(1, '#000');
  ctx.fillStyle = g; ctx.fillRect(0, 0, S, S);
  const x = hsv.s * S, y = (1 - hsv.v) * S;
  ctx.lineWidth = Math.max(2, S / 70);
  ctx.beginPath(); ctx.arc(x, y, S / 22, 0, Math.PI * 2); ctx.strokeStyle = hsv.v > 0.55 && hsv.s < 0.5 ? '#000' : '#fff'; ctx.stroke();
  sq.setAttribute('aria-valuenow', String(Math.round(hsv.s * 100)));
  sq.setAttribute('aria-valuetext', `saturation ${Math.round(hsv.s * 100)}%, value ${Math.round(hsv.v * 100)}%`);
}

function bindPicker() {
  const ring = $('hue-ring'), sq = $('sv-square');
  const hueAt = (ev) => {
    const r = ring.getBoundingClientRect();
    const dx = ev.clientX - (r.left + r.width / 2), dy = ev.clientY - (r.top + r.height / 2);
    setHsv(Math.atan2(dy, dx) * 180 / Math.PI + 90, hsv.s, hsv.v);
  };
  const svAt = (ev) => {
    const r = sq.getBoundingClientRect();
    setHsv(hsv.h, (ev.clientX - r.left) / r.width, 1 - (ev.clientY - r.top) / r.height);
  };
  for (const [el, fn] of [[ring, hueAt], [sq, svAt]]) {
    el.addEventListener('pointerdown', (ev) => {
      if (!colourAllowed()) return;
      dragging = true; el.setPointerCapture(ev.pointerId); el.focus(); fn(ev); ev.preventDefault();
    });
    el.addEventListener('pointermove', (ev) => { if (dragging && el.hasPointerCapture(ev.pointerId)) fn(ev); });
    const end = () => { dragging = false; };
    el.addEventListener('pointerup', end);
    el.addEventListener('pointercancel', end);
  }
  ring.addEventListener('keydown', (ev) => {
    if (!colourAllowed()) return;
    const step = ev.shiftKey ? 10 : 1;
    const map = { ArrowRight: step, ArrowUp: step, ArrowLeft: -step, ArrowDown: -step, PageUp: 30, PageDown: -30 };
    if (ev.key in map) setHsv(hsv.h + map[ev.key], hsv.s, hsv.v);
    else if (ev.key === 'Home') setHsv(0, hsv.s, hsv.v);
    else if (ev.key === 'End') setHsv(359, hsv.s, hsv.v);
    else return;
    ev.preventDefault();
  });
  sq.addEventListener('keydown', (ev) => {
    if (!colourAllowed()) return;
    const step = ev.shiftKey ? 0.1 : 0.01;
    const ds = { ArrowRight: step, ArrowLeft: -step }[ev.key] || 0;
    const dv = { ArrowUp: step, ArrowDown: -step }[ev.key] || 0;
    if (!ds && !dv) return;
    setHsv(hsv.h, hsv.s + ds, hsv.v + dv);
    ev.preventDefault();
  });
  const hex = $('hex');
  const commitHex = () => {
    const rgb = parseHex(hex.value);
    if (!rgb) { hex.setAttribute('aria-invalid', 'true'); return; }
    hex.removeAttribute('aria-invalid');
    setColor(rgb);
    hex.value = toHex(rgb);
  };
  hex.addEventListener('change', commitHex);
  hex.addEventListener('keydown', (ev) => { if (ev.key === 'Enter') commitHex(); });
  ['r', 'g', 'b'].forEach((ch, i) => {
    $('rgb-' + ch).addEventListener('change', (ev) => {
      const v = Math.max(0, Math.min(255, parseInt(ev.target.value, 10) || 0));
      const rgb = ui.color.slice(); rgb[i] = v; ev.target.value = String(v);
      setColor(rgb);
    });
  });
}

// --- Swatches ---
function loadSaved() {
  try {
    const list = JSON.parse(localStorage.getItem(SWATCH_KEY) || '[]');
    return Array.isArray(list) ? list.filter((h) => parseHex(h)) : [];
  } catch (err) { return []; }
}
function storeSaved(list) {
  try { localStorage.setItem(SWATCH_KEY, JSON.stringify(list)); } catch (err) { /* private mode */ }
}

function renderSwatches() {
  const ul = $('swatches');
  const current = toHex(ui.color);
  const saved = loadSaved();
  const allowed = colourAllowed();
  const key = [current, saved.join(','), allowed].join('|');
  if (ul.dataset.key === key) return;
  ul.dataset.key = key;
  ul.textContent = '';
  const add = (hex, label, factory) => {
    const li = document.createElement('li');
    const b = document.createElement('button');
    b.type = 'button';
    b.className = 'swatch' + (factory ? ' factory' : ' saved');
    b.style.background = hex;
    b.dataset.hex = hex;
    b.title = label;
    b.disabled = !allowed;
    b.setAttribute('aria-label', `${label} ${hex}${factory ? '' : ', saved'}`);
    b.setAttribute('aria-pressed', String(hex === current));
    b.addEventListener('click', () => setColor(parseHex(hex)));
    if (!factory) {
      b.addEventListener('keydown', (ev) => {
        if (ev.key !== 'Delete' && ev.key !== 'Backspace') return;
        storeSaved(loadSaved().filter((h) => h !== hex));
        renderSwatches();
        const next = ul.querySelector('.saved') || $('swatch-save');
        next.focus();
        ev.preventDefault();
      });
    }
    li.append(b); ul.append(li);
  };
  FACTORY_SWATCHES.forEach(([hex, label]) => add(hex, label, true));
  saved.forEach((hex) => add(hex, 'Saved', false));
  $('swatch-save').disabled = !allowed;
  $('swatch-clear').disabled = saved.length === 0 || !allowed;
}

function bindSwatches() {
  $('swatch-save').addEventListener('click', () => {
    const hex = toHex(ui.color);
    const list = loadSaved().filter((h) => h !== hex);
    list.push(hex);
    storeSaved(list.slice(-MAX_SAVED));
    renderSwatches();
  });
  $('swatch-clear').addEventListener('click', () => {
    storeSaved([]); renderSwatches();
    $('swatch-save').focus();   // the clear key just disabled itself; keep focus in the plate
  });
}

// --- Presets, transport, link, overrides, control, orientation ---
function buildPresets() {
  const ul = $('presets');
  ul.textContent = '';
  for (const p of PRESETS) {
    if (!animByName[p.left] || !animByName[p.right]) continue;
    const li = document.createElement('li');
    const b = document.createElement('button');
    b.type = 'button'; b.className = 'key'; b.dataset.preset = p.label;
    const dot = document.createElement('span'); dot.className = 'preset-dot';
    dot.style.background = p.color || 'conic-gradient(red, yellow, lime, cyan, blue, magenta, red)';
    b.append(dot, p.label);
    b.addEventListener('click', () => {
      ui.left = p.left; ui.right = p.right;
      const body = { left: p.left, right: p.right };
      if (colourAllowed()) {
        ui.intensity = p.intensity; body.intensity = p.intensity;
        if (p.color) { setColor(parseHex(p.color), { send: false }); body.color = p.color; }
      }
      applyToEngine(); renderControls();
      request('/api/eyes', body);
    });
    li.append(b); ul.append(li);
  }
}

function setSwap(on) {
  view.swap = on;
  try { localStorage.setItem(SWAP_KEY, on ? '1' : '0'); } catch (err) { /* storage blocked */ }
  $('btn-swap').setAttribute('aria-pressed', String(on));
  $('orientation-caption').textContent = on
    ? 'Swapped on this browser: the firmware left eye is drawn on the viewer\'s right. Board LED numbers are not shown in this view because the firmware mapping no longer fixes them.'
    : 'Orientation verified by camera on robot 4000 (2026-10-07): the firmware left eye is the viewer\'s left, pixel 0 just clockwise of 12 o\'clock, index and circle_cw run clockwise. Swap only for a build that differs.';
  if (spot.pinned) { spot.pinned.el.setAttribute('aria-pressed', 'false'); spot.pinned = null; }
  stage.layout = null;   // boards, labels and hotspots move on the next frame
}

function bindKeys() {
  $('btn-off').addEventListener('click', () => {
    ui.left = ui.right = 'off';
    applyToEngine(); renderControls();
    request('/api/off', {});
  });
  $('btn-idle').addEventListener('click', () => {
    ui.left = ui.right = IDLE_NAME; ui.intensity = 255;
    setColor([40, 48, 60], { send: false });
    applyToEngine(); renderControls();
    request('/api/idle', {});
  });
  $('btn-control').addEventListener('click', () => {
    const held = !!(server.studio && server.studio.control);
    request('/api/control', { take: !held });
  });
  $('btn-resume').addEventListener('click', () => request('/api/sentry', { active: true }));
  $('btn-swap').addEventListener('click', () => setSwap(!view.swap));
  document.querySelectorAll('input[name="link"]').forEach((r) => r.addEventListener('change', () => {
    ui.link = r.value; renderControls();
  }));
  document.querySelectorAll('input[name="ovr"]').forEach((r) => r.addEventListener('change', () => {
    sim.preview = r.value; renderBanner();
  }));
  const fader = $('intensity');
  fader.addEventListener('input', () => {
    ui.intensity = parseInt(fader.value, 10);
    applyToEngine(); renderControls();
    request('/api/eyes', { intensity: ui.intensity });
  });
  $('btn-play-stop').addEventListener('click', () => { seqMessage(STOP_NOTE); request('/api/stop', {}); });
}

function applyCapabilities() {
  const n = caps.pixels_per_eye || 10;
  if (n !== sim.engine.n) { sim.engine = new EyeEngine(n); stage.layout = null; }
  // Protocol: the API says which protocol it was written for and, when the backend can
  // read it, which one the PIMU runs. Through the server it cannot be read.
  const need = protoLabel(caps.requires_protocol || 'p13');
  const have = caps.protocol_version ? protoLabel(caps.protocol_version) : '';
  $('protocol').textContent = have ? `PIMU protocol ${have}` : `PIMU protocol unknown, ${need} assumed`;
  $('protocol-detail').textContent = have
    ? `The PIMU reports protocol ${have}. The eye RPC with its color and intensity bytes needs ${need} (hello-pimu2 v0.1.9p13 or newer).`
    : `stretch_body_server does not publish the PIMU protocol, so ${need} is assumed. stretch_system_check reports the board's version.`;
  document.querySelectorAll('.cap').forEach((el) => {
    const cap = el.dataset.cap;
    const control = el.querySelector('input, button');
    const reason = el.querySelector('.reason');
    // Neither control has a handler yet, so both stay off whatever the backend offers.
    el.classList.add('unsupported');
    control.disabled = true;
    reason.textContent = caps[cap] ? 'Not built in Eyes Studio yet.' : REASONS[cap] || 'Not supported by this backend.';
    if (reason.id) control.setAttribute('aria-describedby', reason.id);
  });
  // The eye RPC with its colour and intensity bytes exists from requires_protocol on;
  // an older board ignores every push. Say so once at the top, and on the two panels
  // whose bytes do not exist there.
  const allowed = colourAllowed();
  const why = allowed ? '' : `PIMU protocol ${have || 'unknown'} is below ${need}, which added the eye RPC with its color and intensity bytes. Nothing reaches the rings until the firmware is updated.`;
  $('colour-reason').textContent = why;
  $('intensity-reason').textContent = why;
  const pn = $('protocol-notice');
  pn.textContent = allowed ? ''
    : `This PIMU runs protocol ${have || 'unknown'} and the eye RPC needs ${need} (hello-pimu2 v0.1.9p13 or newer): no eye command reaches the rings. The preview still simulates the ${need} firmware.`;
  pn.hidden = allowed;
  $('colour-h').parentElement.classList.toggle('unsupported', !allowed);
  $('level').classList.toggle('unsupported', !allowed);
  document.querySelector('.swatches').classList.toggle('unsupported', !allowed);
  for (const id of ['hex', 'rgb-r', 'rgb-g', 'rgb-b', 'intensity']) $(id).disabled = !allowed;
  for (const id of ['hue-ring', 'sv-square']) {
    $(id).tabIndex = allowed ? 0 : -1;
    $(id).setAttribute('aria-disabled', String(!allowed));
  }
  $('px-hint').textContent = `${n} WS2812B per eye. Chain px ${CHAIN_OFFSET[0]} to ${CHAIN_OFFSET[0] + n - 1} left, ${CHAIN_OFFSET[1]} to ${CHAIN_OFFSET[1] + n - 1} right; px 0 to 7 are the light bar, firmware only.`;
  $('stage-caption').textContent = caps.readback
    ? 'Simulated from the PIMU renderer at 50 Hz.'
    : `Simulated from the PIMU renderer at 50 Hz using the last command. ${need} firmware has no eye readback.`;
  // Library and saving need the looks API on the server.
  const lib = caps.library !== false;
  for (const id of ['seq-save', 'seq-import']) $(id).disabled = !lib;
  if (!lib) $('library-where').textContent = `The looks library is not available on this server: ${caps.library_error || 'the eyes API has no Library'}.`;
  renderSwatches();
}

// ---------------------------------------------------------------------------
// 6. Pop-over plates and the sequence drawer.
//    Plates are modal while open: focus is trapped, Escape or a click outside
//    closes them and focus goes back to the softkey. The drawer is not modal on
//    purpose: building a sequence means picking effects and colors from the
//    faceplate while it is open. Escape closes it while focus is inside it.
// ---------------------------------------------------------------------------
const pop = { open: null, opener: null };
const POP_HOOKS = { 'pop-library': () => refreshLibrary() };
const desktop = () => matchMedia('(min-width: 1024px) and (min-height: 600px)').matches;

function focusables(root) {
  return Array.from(root.querySelectorAll('button, input, a[href], [tabindex="0"]')).filter((el) => {
    if (el.disabled || el.hidden || el.type === 'file' || el.getClientRects().length === 0) return false;
    if (el.type === 'radio' && !el.checked) return false;
    return el.tabIndex >= 0;
  });
}

function placePop(el, opener) {
  if (!desktop()) { el.style.left = el.style.top = el.style.bottom = el.style.maxHeight = ''; return; }
  const r = opener.getBoundingClientRect(), w = el.offsetWidth;
  const left = Math.max(12, Math.min(window.innerWidth - w - 12, r.left + r.width / 2 - w / 2));
  el.style.left = `${left}px`;
  // Above the key when there is more room above (softkeys sit low), else below.
  if (r.top > window.innerHeight - r.bottom) {
    el.style.top = ''; el.style.bottom = `${window.innerHeight - r.top + 8}px`;
    el.style.maxHeight = `${r.top - 20}px`;
  } else {
    el.style.bottom = ''; el.style.top = `${r.bottom + 8}px`;
    el.style.maxHeight = `${window.innerHeight - r.bottom - 20}px`;
  }
}

function openPop(id, opener) {
  if (pop.open) closePop({ restore: false });
  const el = $(id);
  el.hidden = false;
  placePop(el, opener);
  opener.setAttribute('aria-expanded', 'true');
  pop.open = el; pop.opener = opener;
  if (POP_HOOKS[id]) POP_HOOKS[id]();
  const first = focusables(el).find((f) => !f.classList.contains('pop-close')) || focusables(el)[0];
  if (first) first.focus();
}

function closePop({ restore = true } = {}) {
  if (!pop.open) return;
  if (pop.open.id === 'pop-library') library.confirmDelete = null;   // a closed plate drops the question
  pop.open.hidden = true;
  pop.opener.setAttribute('aria-expanded', 'false');
  const opener = pop.opener;
  pop.open = null; pop.opener = null;
  if (restore) opener.focus();
}

function openSeq() {
  $('seq').hidden = false;
  document.querySelector('.stage').classList.add('seq-open');
  $('key-seq').setAttribute('aria-expanded', 'true');
  placeDrawer();
  renderTimeline();
  $('seq-add').focus();
}
function closeSeq({ restore = true } = {}) {
  $('seq').hidden = true;
  document.querySelector('.stage').classList.remove('seq-open');
  $('seq-confirm').hidden = true;
  $('key-seq').setAttribute('aria-expanded', 'false');
  if (restore) $('key-seq').focus();
}
// The drawer sits just above the softkeys on a desktop.
function placeDrawer() {
  const st = document.querySelector('.stage'), sk = document.querySelector('.softkeys');
  const h = st.getBoundingClientRect().bottom - sk.getBoundingClientRect().top + 8;
  st.style.setProperty('--softkeys-h', `${Math.round(h)}px`);
}

function bindPops() {
  document.querySelectorAll('.softkey[data-pop]').forEach((key) => key.addEventListener('click', () => {
    if (pop.open && pop.opener === key) closePop();
    else openPop(key.dataset.pop, key);
  }));
  document.querySelectorAll('.pop-close').forEach((b) => b.addEventListener('click', () => closePop()));
  $('key-seq').addEventListener('click', () => { if ($('seq').hidden) openSeq(); else closeSeq(); });
  $('seq-close').addEventListener('click', () => closeSeq());
  document.addEventListener('keydown', (ev) => {
    if (ev.key === 'Escape' && !ev.defaultPrevented) {
      if (pop.open) { closePop(); ev.preventDefault(); return; }
      const seq = $('seq');
      if (!seq.hidden && seq.contains(document.activeElement)) {
        if (!$('seq-confirm').hidden) { $('seq-confirm').hidden = true; confirmState.no(); } else closeSeq();
        ev.preventDefault();
      }
      return;
    }
    if (ev.key === 'Tab' && pop.open) {
      const list = focusables(pop.open);
      if (!list.length) { ev.preventDefault(); return; }
      const i = list.indexOf(document.activeElement);
      let to = null;
      if (i < 0) to = ev.shiftKey ? list[list.length - 1] : list[0];
      else if (!ev.shiftKey && i === list.length - 1) to = list[0];
      else if (ev.shiftKey && i === 0) to = list[list.length - 1];
      if (to) { to.focus(); ev.preventDefault(); }
    }
  });
  // A press anywhere outside the open plate (and not on its own key) closes it.
  document.addEventListener('pointerdown', (ev) => {
    if (pop.open && !pop.open.contains(ev.target) && !pop.opener.contains(ev.target)) closePop({ restore: false });
  }, true);
  window.addEventListener('resize', () => { if (pop.open) placePop(pop.open, pop.opener); if (!$('seq').hidden) placeDrawer(); });
}

// ---------------------------------------------------------------------------
// 7. Looks: the sequence editor, the library and playback.
//    File format stretch-eyes/1 (stretch4_body.eyes.looks). A step's missing
//    left, right, color or intensity keeps the previous step's value, as
//    Eyes.set(None) does; the editor always captures all four.
// ---------------------------------------------------------------------------
const LOOK_NAME = /^[a-z0-9][a-z0-9_-]{0,47}$/;
const HOLD_MIN = 0.05, HOLD_MAX = 600, HOLD_STEP = 0.25, MAX_STEPS = 200;
const BOOT_LOOK = { left: IDLE_NAME, right: IDLE_NAME, color: [40, 48, 60], intensity: 255 };
const round2 = (x) => Math.round(x * 100) / 100;
const fmtS = (x) => `${round2(x)} s`;

function colorToRgb(c) {
  if (Array.isArray(c) && c.length === 3) return c.map((x) => Math.max(0, Math.min(255, x | 0)));
  if (typeof c !== 'string') return null;
  const key = c.trim().toLowerCase().replace(/[- ]/g, '_');
  if (NAMED_COLORS[key]) return NAMED_COLORS[key].slice();
  let t = c.trim().replace(/^#/, '');
  if (/^[0-9a-f]{3}$/i.test(t)) t = t.split('').map((x) => x + x).join('');
  return parseHex(t);
}
// Eyes.set: an int is the raw byte, a float a fraction. JSON cannot tell 1 from 1.0,
// so a preview reads a whole number as raw; saved files always hold the raw int.
function intensityByte(x) {
  if (typeof x !== 'number' || !isFinite(x)) return null;
  return Number.isInteger(x) ? Math.max(0, Math.min(255, x)) : Math.round(Math.max(0, Math.min(1, x)) * 255);
}
function animName(x) {
  if (typeof x === 'number') return animById[x] ? animById[x].name : null;
  if (typeof x === 'string') return animByName[x] ? x : (/^\d+$/.test(x) && animById[+x] ? animById[+x].name : null);
  return null;
}
function resolveSteps(steps, start) {
  let cur = { ...start, color: start.color.slice() };
  return (steps || []).map((s) => {
    cur = {
      left: s.left != null ? animName(s.left) || cur.left : cur.left,
      right: s.right != null ? animName(s.right) || cur.right : cur.right,
      color: s.color != null ? colorToRgb(s.color) || cur.color : cur.color,
      intensity: s.intensity != null ? intensityByte(s.intensity) ?? cur.intensity : cur.intensity,
      hold: Number(s.hold) > 0 ? Number(s.hold) : 1,
    };
    return cur;
  });
}
// The steps of every pass after the first: on the robot a looped step 0 that leaves a
// field out keeps the last step's value, not the look from before the play.
function wrapSteps(raw, first) {
  return first.length ? resolveSteps(raw, first[first.length - 1]) : first;
}
const totalHold = (steps) => (steps || []).reduce((a, s) => a + (Number(s.hold) || 0), 0);
function currentLook() {
  return { left: previewName(ui.left), right: previewName(ui.right), color: ui.color.slice(), intensity: ui.intensity };
}

// --- Editor model ---
const editor = { name: '', title: '', loop: false, extra: {}, steps: [] };
let stepSeq = 0;
const newStep = (data) => ({ key: ++stepSeq, data });

function editorDict(nameFallback) {
  const name = editor.name || nameFallback;
  const d = { format: 'stretch-eyes/1', name, title: editor.title || name };
  Object.assign(d, editor.extra);
  d.loop = editor.loop;
  d.steps = editor.steps.map((s) => ({ ...s.data }));
  return d;
}

function loadIntoEditor(dict) {
  const { format, name, title, loop, steps, ...extra } = dict || {};
  editor.name = typeof name === 'string' ? name : '';
  editor.title = typeof title === 'string' && title !== name ? title : '';
  editor.loop = !!loop;
  editor.extra = extra;
  editor.steps = (Array.isArray(steps) ? steps : []).map((s) => newStep({ ...s }));
  $('seq-name').value = editor.name;
  $('seq-title').value = editor.title;
  $('seq-name').removeAttribute('aria-invalid');
  $('seq-loop').setAttribute('aria-pressed', String(editor.loop));
  renderTimeline();
}

function seqMessage(text, bad = false) {
  const el = $('seq-msg');
  el.textContent = text;
  el.classList.toggle('bad', bad);
}

function checkName() {
  const el = $('seq-name');
  const ok = LOOK_NAME.test(editor.name);
  if (ok) el.removeAttribute('aria-invalid'); else el.setAttribute('aria-invalid', 'true');
  if (!ok) {
    seqMessage(editor.name ? 'Name: lower case letters, digits, - and _, starting with a letter or digit, up to 48.' : 'Give the look a name first (lower case, digits, - and _).', true);
    el.focus();
  }
  return ok;
}

const confirmState = { yes: () => {}, no: () => {} };
function askConfirm(text, yesLabel) {
  return new Promise((resolve) => {
    $('seq-confirm-text').textContent = text;
    $('seq-confirm-yes').textContent = yesLabel;
    $('seq-confirm').hidden = false;
    confirmState.yes = () => { $('seq-confirm').hidden = true; resolve(true); };
    confirmState.no = () => { $('seq-confirm').hidden = true; resolve(false); };
    $('seq-confirm-no').focus();
  });
}

// Save a sequence dict to this robot's library, asking before an overwrite. With text
// (an imported file) the server parses the file itself: JSON.parse turns intensity 1.0
// (full) into 1, which the API reads as raw 1.
async function saveLook(dict, { focusAfter, text } = {}) {
  const body = (overwrite) => (text != null ? { text, overwrite } : { sequence: dict, overwrite });
  try {
    return await api('/api/library', body(false));
  } catch (err) {
    if (err.status !== 409) throw err;
    const ok = await askConfirm(`A look named ${dict.name} is already saved on this robot. Overwrite it?`, 'Overwrite');
    if (focusAfter) focusAfter.focus();
    if (!ok) return null;
    return api('/api/library', body(true));
  }
}

function renderTimeline() {
  const ol = $('timeline');
  const resolved = resolveSteps(editor.steps.map((s) => s.data), currentLookForEditor());
  const keep = document.activeElement && ol.contains(document.activeElement)
    ? { key: document.activeElement.closest('.step')?.dataset.key, role: document.activeElement.dataset.role } : null;
  ol.textContent = '';
  let t = 0;
  editor.steps.forEach((s, i) => {
    const r = resolved[i], n = i + 1;
    const li = document.createElement('li');
    li.className = 'step'; li.draggable = true; li.dataset.key = String(s.key); li.dataset.index = String(i);
    li.setAttribute('aria-label', `Step ${n}`);
    const chip = document.createElement('canvas'); chip.className = 'chip'; chip.setAttribute('aria-hidden', 'true');
    minis.set(chip, new MiniPlayer([r]));
    const label = document.createElement('div'); label.className = 'step-label';
    const no = document.createElement('span'); no.className = 'step-no';
    const lamp = document.createElement('span'); lamp.className = 'lamp'; lamp.setAttribute('aria-hidden', 'true');
    no.append(lamp, `#${n} at ${fmtS(t)}`);
    const what = document.createElement('span'); what.className = 'step-what';
    what.textContent = r.left === r.right ? animLabel(r.left) : `${animLabel(r.left)}, ${animLabel(r.right)}`;
    what.title = `Left ${animLabel(r.left)}, right ${animLabel(r.right)}`;
    const col = document.createElement('span'); col.className = 'step-col';
    col.textContent = `${toHex(r.color)} ${r.intensity}`;
    label.append(no, what, col);
    // Hold: number field with - and + keys.
    const hold = document.createElement('div'); hold.className = 'hold';
    const dec = mkKey('-', `Shorter hold for step ${n}`, 'hold-dec');
    const inc = mkKey('+', `Longer hold for step ${n}`, 'hold-inc');
    const lab = document.createElement('label');
    const sr = document.createElement('span'); sr.className = 'visually-hidden'; sr.textContent = `Hold for step ${n}, seconds`;
    const input = document.createElement('input');
    input.type = 'number'; input.min = String(HOLD_MIN); input.max = String(HOLD_MAX); input.step = '0.05';
    input.inputMode = 'decimal'; input.className = 'hold-input'; input.dataset.role = 'hold-input';
    input.value = String(round2(Number(s.data.hold) || 1));
    const unit = document.createElement('span'); unit.className = 'unit'; unit.textContent = 's'; unit.setAttribute('aria-hidden', 'true');
    lab.append(sr, input, unit);
    hold.append(dec, lab, inc);
    dec.addEventListener('click', () => setHold(s, (Number(s.data.hold) || 1) - HOLD_STEP));
    inc.addEventListener('click', () => setHold(s, (Number(s.data.hold) || 1) + HOLD_STEP));
    input.addEventListener('change', () => {
      const v = Number(input.value);
      if (!isFinite(v) || v < HOLD_MIN || v > HOLD_MAX) {
        input.setAttribute('aria-invalid', 'true');
        seqMessage(`Hold must be ${HOLD_MIN} to ${HOLD_MAX} s.`, true);
        return;
      }
      setHold(s, v);
    });
    input.addEventListener('keydown', (ev) => { if (ev.key === 'Enter') input.dispatchEvent(new Event('change')); });
    // Order, duplicate, delete.
    const keys = document.createElement('div'); keys.className = 'step-keys';
    const up = mkKey('‹', `Move step ${n} earlier`, 'step-up');
    const down = mkKey('›', `Move step ${n} later`, 'step-down');
    const dup = mkKey('Copy', `Duplicate step ${n}`, 'step-dup');
    const del = mkKey('✕', `Delete step ${n}`, 'step-del');
    up.disabled = i === 0; down.disabled = i === editor.steps.length - 1;
    dup.disabled = editor.steps.length >= MAX_STEPS;
    up.addEventListener('click', () => moveStep(i, i - 1, 'step-up'));
    down.addEventListener('click', () => moveStep(i, i + 1, 'step-down'));
    dup.addEventListener('click', () => {
      editor.steps.splice(i + 1, 0, newStep({ ...s.data }));
      seqMessage(`Step ${n} duplicated.`);
      renderTimeline(); focusStep(i + 1, 'step-dup');
    });
    del.addEventListener('click', () => {
      editor.steps.splice(i, 1);
      seqMessage(`Step ${n} deleted.`);
      renderTimeline();
      if (editor.steps.length) focusStep(Math.min(i, editor.steps.length - 1), 'step-del'); else $('seq-add').focus();
    });
    keys.append(up, down, dup, del);
    li.append(chip, label, hold, keys);
    bindDrag(li, i);
    ol.append(li);
    t += Number(s.data.hold) || 0;
  });
  const n = editor.steps.length;
  $('seq-total').textContent = `${n} step${n === 1 ? '' : 's'}, ${fmtS(totalHold(editor.steps.map((s) => s.data)))}${editor.loop ? ', loops' : ''}`;
  $('seq-add').disabled = n >= MAX_STEPS;
  for (const id of ['seq-play', 'seq-export']) $(id).disabled = n === 0;
  $('seq-save').disabled = n === 0 || caps.library === false;
  if (keep && keep.key) {
    const li = ol.querySelector(`.step[data-key="${keep.key}"]`);
    const el = li && li.querySelector(`[data-role="${keep.role}"]`);
    if (el && !el.disabled) el.focus();
  }
  renderRunning();
}
// The editor previews steps from the look the eyes would start from.
function currentLookForEditor() { return play.local ? BOOT_LOOK : currentLook(); }

function mkKey(text, label, role) {
  const b = document.createElement('button');
  b.type = 'button'; b.className = 'key'; b.textContent = text;
  b.setAttribute('aria-label', label); b.title = label; b.dataset.role = role;
  return b;
}
function focusStep(i, role) {
  const li = $('timeline').children[i];
  const el = li && li.querySelector(`[data-role="${role}"]`);
  if (el && !el.disabled) el.focus();
  else if (li) (li.querySelector('[data-role="hold-input"]') || li).focus();
}
function setHold(s, v) {
  s.data.hold = round2(Math.max(HOLD_MIN, Math.min(HOLD_MAX, v)));
  renderTimeline();
}
function moveStep(from, to, role) {
  if (to < 0 || to >= editor.steps.length || from === to) return;
  const [s] = editor.steps.splice(from, 1);
  editor.steps.splice(to, 0, s);
  seqMessage(`Step moved to position ${to + 1}.`);
  renderTimeline();
  focusStep(to, role);
}

// Drag to reorder: a copper bar marks where the step will land.
let dragFrom = null;
function bindDrag(li, i) {
  li.addEventListener('dragstart', (ev) => {
    dragFrom = i; li.classList.add('dragging');
    ev.dataTransfer.effectAllowed = 'move';
    ev.dataTransfer.setData('text/plain', String(i));
  });
  li.addEventListener('dragend', () => {
    dragFrom = null;
    document.querySelectorAll('.step').forEach((el) => el.classList.remove('dragging', 'drop-before', 'drop-after'));
  });
  li.addEventListener('dragover', (ev) => {
    if (dragFrom === null) return;
    ev.preventDefault();
    const r = li.getBoundingClientRect(), after = ev.clientX > r.left + r.width / 2;
    document.querySelectorAll('.step').forEach((el) => el.classList.remove('drop-before', 'drop-after'));
    li.classList.add(after ? 'drop-after' : 'drop-before');
  });
  li.addEventListener('drop', (ev) => {
    if (dragFrom === null) return;
    ev.preventDefault();
    const r = li.getBoundingClientRect(), after = ev.clientX > r.left + r.width / 2;
    let to = i + (after ? 1 : 0);
    if (dragFrom < to) to -= 1;
    const from = dragFrom;
    dragFrom = null;
    if (to !== from) moveStep(from, to, 'hold-input'); else renderTimeline();
  });
}

function bindEditor() {
  $('seq-name').addEventListener('input', (ev) => { editor.name = ev.target.value.trim(); ev.target.removeAttribute('aria-invalid'); });
  $('seq-title').addEventListener('input', (ev) => { editor.title = ev.target.value; });
  $('seq-loop').addEventListener('click', () => {
    editor.loop = !editor.loop;
    $('seq-loop').setAttribute('aria-pressed', String(editor.loop));
    renderTimeline();
  });
  $('seq-add').addEventListener('click', () => {
    if (editor.steps.length >= MAX_STEPS) return;
    const look = currentLook();
    editor.steps.push(newStep({ left: look.left, right: look.right, color: toHex(look.color), intensity: look.intensity, hold: 1 }));
    seqMessage(`Step ${editor.steps.length} added: ${animLabel(look.left)}${look.left === look.right ? '' : `, ${animLabel(look.right)}`}, ${toHex(look.color)} at ${look.intensity}.`);
    renderTimeline();
    const ol = $('timeline');
    ol.scrollLeft = ol.scrollWidth;
  });
  $('seq-new').addEventListener('click', () => { loadIntoEditor({}); seqMessage('New empty sequence.'); $('seq-name').focus(); });
  $('seq-play').addEventListener('click', () => {
    if (!editor.steps.length) return;
    const dict = editorDict(LOOK_NAME.test(editor.name) ? editor.name : 'draft');
    if (!LOOK_NAME.test(dict.name)) dict.name = 'draft';
    play.pending = { name: dict.name, dict };
    seqMessage(`Playing ${dict.name}${editor.loop ? ' on a loop' : ''}.`);
    request('/api/play', { sequence: dict, loop: editor.loop });
  });
  $('seq-stop').addEventListener('click', () => {
    seqMessage(STOP_NOTE);
    request('/api/stop', {});
  });
  $('seq-save').addEventListener('click', async () => {
    if (!checkName()) return;
    const dict = editorDict();
    try {
      const saved = await saveLook(dict, { focusAfter: $('seq-save') });
      if (saved) seqMessage(`Saved ${dict.name} to this robot's library.`);
      else seqMessage('Not saved.');
      if (saved && pop.open === $('pop-library')) refreshLibrary();
    } catch (err) { seqMessage(`Not saved: ${err.message}`, true); }
  });
  $('seq-export').addEventListener('click', () => {
    if (!editor.steps.length) return;
    const dict = editorDict(LOOK_NAME.test(editor.name) ? editor.name : 'look');
    download(`${dict.name}.json`, JSON.stringify(dict, null, 2) + '\n');
    seqMessage(`Exported ${dict.name}.json.`);
  });
  $('seq-import').addEventListener('click', () => $('seq-file').click());
  $('seq-file').addEventListener('change', async (ev) => {
    const file = ev.target.files && ev.target.files[0];
    ev.target.value = '';
    if (!file) return;
    const text = await file.text();
    let dict;
    try { dict = JSON.parse(text); }   // only for the name; the server parses the file
    catch (err) { seqMessage(`${file.name} is not JSON: ${err.message}`, true); return; }
    try {
      const saved = await saveLook(dict, { focusAfter: $('seq-import'), text });
      if (!saved) { seqMessage('Import cancelled.'); return; }
      loadIntoEditor(await api(`/api/library/${encodeURIComponent(dict.name)}`));
      seqMessage(`Imported ${file.name} as ${dict.name} into this robot's library.`);
      if (pop.open === $('pop-library')) refreshLibrary();
    } catch (err) { seqMessage(`Not imported: ${err.message}`, true); }
  });
  $('seq-confirm-yes').addEventListener('click', () => confirmState.yes());
  $('seq-confirm-no').addEventListener('click', () => confirmState.no());
}

function download(filename, text) {
  const url = URL.createObjectURL(new Blob([text], { type: 'application/json' }));
  const a = document.createElement('a');
  a.href = url; a.download = filename; a.hidden = true;
  document.body.append(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 5000);
}

// --- Library ---
const library = { looks: [], seqs: {}, confirmDelete: null };
const SOURCE_LABEL = { builtin: 'built-in', shared: 'shared', user: 'this robot' };

function libMessage(text, bad = false) {
  const el = $('library-msg');
  el.textContent = text; el.classList.toggle('bad', bad);
}

async function refreshLibrary() {
  if (caps.library === false) { renderLibrary(); return; }
  try {
    const r = await api('/api/library');
    library.looks = Array.isArray(r.looks) ? r.looks : [];
    // Files the library skipped (bad JSON, a name that does not match its file name).
    const skipped = Array.isArray(r.errors) ? r.errors : [];
    libMessage(skipped.length ? `Skipped ${skipped.map((e) => `${e.file} (${e.source}): ${e.error}`).join('; ')}` : '', skipped.length > 0);
    library.seqs = r.sequences && typeof r.sequences === 'object' ? r.sequences : {};
    renderLibrary();
  } catch (err) {
    libMessage(`Could not read the library: ${err.message}`, true);
  }
}

function renderLibrary() {
  const ul = $('library');
  const active = document.activeElement && ul.contains(document.activeElement)
    ? { name: document.activeElement.closest('.lib-row')?.dataset.name, role: document.activeElement.dataset.role } : null;
  ul.textContent = '';
  for (const l of library.looks) {
    const li = document.createElement('li');
    li.className = 'lib-row'; li.dataset.name = l.name; li.dataset.source = l.source;
    const canvas = document.createElement('canvas'); canvas.setAttribute('aria-hidden', 'true');
    const dict = library.seqs[l.name];
    if (dict) {
      const steps = resolveSteps(dict.steps, BOOT_LOOK);
      minis.set(canvas, new MiniPlayer(steps, wrapSteps(dict.steps, steps)));
    }
    const info = document.createElement('div');
    const title = document.createElement('div'); title.className = 'lib-title'; title.textContent = l.title || l.name;
    const meta = document.createElement('div'); meta.className = 'lib-meta';
    const nm = document.createElement('span'); nm.className = 'mono'; nm.textContent = l.name;
    const badge = document.createElement('span'); badge.className = `badge badge-${l.source}`; badge.textContent = SOURCE_LABEL[l.source] || l.source;
    const count = document.createElement('span');
    const steps = typeof l.steps === 'number' ? l.steps : (dict ? dict.steps.length : 0);
    const dur = typeof l.duration === 'number' ? l.duration : totalHold(dict && dict.steps);
    count.textContent = `${steps} step${steps === 1 ? '' : 's'}, ${fmtS(dur)}${l.loop ? ', loops' : ''}`;
    meta.append(nm, badge, count);
    if (l.shadowed) { const sh = document.createElement('span'); sh.textContent = 'overrides a look of the same name'; meta.append(sh); }
    info.append(title, meta);
    const keys = document.createElement('div'); keys.className = 'lib-keys';
    const playK = mkKey('Play', `Play ${l.name}`, 'lib-play');
    playK.textContent = 'Play'; playK.classList.add('key-go');
    const load = mkKey('Load', `Load ${l.name} into the sequence editor`, 'lib-load');
    const dup = mkKey('Duplicate', `Duplicate ${l.name} into this robot's library`, 'lib-dup');
    const exp = document.createElement('a');
    exp.className = 'key'; exp.textContent = 'Export'; exp.dataset.role = 'lib-export';
    exp.href = `/api/library/${encodeURIComponent(l.name)}`; exp.download = `${l.name}.json`;
    exp.setAttribute('aria-label', `Export ${l.name} as JSON`); exp.title = `Export ${l.name} as JSON`;
    keys.append(playK, load, dup, exp);
    playK.addEventListener('click', () => {
      play.pending = dict ? { name: l.name, dict } : null;
      libMessage(`Playing ${l.name}.`);
      request('/api/play', { name: l.name });
    });
    load.addEventListener('click', () => {
      if (!dict) return;
      loadIntoEditor(dict);
      closePop({ restore: false });
      openSeq();
      seqMessage(`Loaded ${l.name} (${SOURCE_LABEL[l.source] || l.source}). Save writes a copy to this robot's library.`);
    });
    dup.addEventListener('click', () => duplicateLook(l, dict));
    if (l.source === 'user') {
      const del = mkKey('Delete', `Delete ${l.name} from this robot's library`, 'lib-del');
      del.classList.add('key-danger');
      del.addEventListener('click', () => { library.confirmDelete = l.name; renderLibrary(); focusLib(l.name, 'lib-del-no'); });
      keys.append(del);
    }
    li.append(canvas, info, keys);
    if (library.confirmDelete === l.name) {
      const c = document.createElement('div'); c.className = 'lib-confirm'; c.setAttribute('role', 'alert');
      const t = document.createElement('span'); t.textContent = `Delete ${l.name} from this robot? This cannot be undone.`;
      const yes = mkKey('Delete', `Confirm delete ${l.name}`, 'lib-del-yes'); yes.classList.add('key-danger');
      const no = mkKey('Cancel', 'Keep it', 'lib-del-no');
      yes.addEventListener('click', async () => {
        library.confirmDelete = null;
        try { await api('/api/library/delete', { name: l.name }); libMessage(`Deleted ${l.name}.`); }
        catch (err) { libMessage(`Not deleted: ${err.message}`, true); }
        await refreshLibrary();
        const first = $('library').querySelector('[data-role="lib-play"]');
        if (first) first.focus();
      });
      no.addEventListener('click', () => { library.confirmDelete = null; renderLibrary(); focusLib(l.name, 'lib-del'); });
      c.append(t, yes, no);
      li.append(c);
    }
    ul.append(li);
  }
  if (!library.looks.length) {
    const li = document.createElement('li'); li.className = 'hint';
    li.textContent = caps.library === false ? 'No library on this server.' : 'No looks found.';
    ul.append(li);
  }
  if (active && active.name) focusLib(active.name, active.role);
  renderRunning();
}
function focusLib(name, role) {
  const row = Array.from($('library').children).find((li) => li.dataset.name === name);
  const el = row && row.querySelector(`[data-role="${role}"]`);
  if (el) el.focus();
}

async function duplicateLook(l, dict) {
  if (!dict) return;
  const taken = new Set(library.looks.map((x) => x.name));
  const base = l.name.slice(0, 40);
  let name = `${base}-copy`;
  for (let k = 2; taken.has(name); k++) name = `${base}-copy${k}`;
  const copy = { ...dict, name, title: `${dict.title || l.name} (copy)` };
  try {
    await api('/api/library', { sequence: copy, overwrite: false });
    libMessage(`Saved ${name} to this robot's library.`);
  } catch (err) { libMessage(`Not duplicated: ${err.message}`, true); }
  await refreshLibrary();
  focusLib(name, 'lib-load');
}

// --- Playback: the server plays (Eyes.play); the page follows it. ---
// play.local is the page's copy of the running timeline: it drives the preview at
// 50 Hz with the exact holds and is pulled back to the server's step when they
// disagree for longer than a step boundary race.
const play = { local: null, pending: null, fetching: null };
const STOP_NOTE = 'Stopped. The eyes are back on the look from before Play.';

function stopLocalPlayback() {
  if (!play.local) return;
  play.local = null;
  renderPlayback();
}

function startLocal(name, dict, loop, started) {
  const steps = resolveSteps(dict.steps, currentLook());
  if (!steps.length) return;
  play.local = { name, steps, wrap: loop ? wrapSteps(dict.steps, steps) : steps, loop, started, idx: 0, t: 0, done: false };
  applyPlayStep();
}

// A page that did not start the playback asks once what is playing.
async function fetchPlaying(p) {
  if (play.fetching === p.started) return;
  play.fetching = p.started;
  let r;
  try { r = await api('/api/playing'); } catch (err) { play.fetching = null; return; }   // the next poll asks again
  const now = server.playing;
  if (!r.sequence || !now || !r.playing || r.playing.started !== now.started || r.playing.name !== now.name) return;
  if (play.local && play.local.started === now.started) return;
  startLocal(now.name, r.sequence, now.loop == null ? false : !!now.loop, now.started);
}

function applyPlayStep() {
  const L = play.local, s = L.steps[L.idx];
  ui.left = s.left; ui.right = s.right; ui.intensity = s.intensity;
  setColor(s.color, { send: false });
  applyToEngine();
  renderControls();
}

// One 50 Hz frame of the local timeline.
function playTick() {
  const L = play.local;
  if (!L || L.done) return;
  L.t += FRAME_MS;
  if (L.t < L.steps[L.idx].hold * 1000) return;
  if (L.idx + 1 < L.steps.length) L.idx++;
  else if (L.loop) { L.idx = 0; L.steps = L.wrap; }
  else { L.done = true; return; }   // the last step stays on, as on the robot
  L.t = 0;
  applyPlayStep();
}

function followPlayback() {
  const p = server.playing;
  if (!p) {
    if (play.local && !queue.some((j) => j.path === '/api/play')) { play.local = null; play.pending = null; renderPlayback(); }
    return;
  }
  const step = typeof p.step === 'number' ? p.step : 0;
  const loop = p.loop == null ? false : !!p.loop;
  // A new playback (another name, or the same one started again) restarts the copy.
  if (!play.local || play.local.name !== p.name || play.local.started !== p.started) {
    // A page that started it already knows the sequence; any other asks the server.
    const dict = play.pending && play.pending.name === p.name ? play.pending.dict : null;
    play.pending = null;
    if (!dict) { fetchPlaying(p); return; }
    startLocal(p.name, dict, loop, p.started);
  }
  const L = play.local;
  if (L && step !== L.idx && step < L.steps.length && L.t > 300) {
    if (L.loop && step < L.idx) L.steps = L.wrap;   // the server has wrapped
    L.idx = step; L.t = 0; L.done = false; applyPlayStep();
  }
}

function renderPlayback() {
  const p = server.playing;
  const badge = $('play-badge');
  badge.hidden = !p;
  if (p) {
    const n = p.steps || (play.local ? play.local.steps.length : 0);
    const step = (typeof p.step === 'number' ? p.step : 0) + 1;
    $('play-text').textContent = `${p.name}, step ${step} of ${n}${p.loop ? ', loop' : ''}`;
  }
  $('seq-stop').disabled = !p;
  renderRunning();
}

// Highlight the running step in the editor and the playing look in the library.
function renderRunning() {
  const p = server.playing;
  const mine = p && (p.name === editor.name || (!LOOK_NAME.test(editor.name) && p.name === 'draft'))
    && (p.steps == null || p.steps === editor.steps.length);
  document.querySelectorAll('#timeline .step').forEach((li) => {
    const on = !!mine && Number(li.dataset.index) === p.step;
    li.classList.toggle('running', on);
    if (on) li.setAttribute('aria-current', 'step'); else li.removeAttribute('aria-current');
    li.querySelector('.lamp').className = 'lamp' + (on ? ' ok' : '');
  });
  document.querySelectorAll('#library .lib-row').forEach((li) => li.classList.toggle('running', !!p && li.dataset.name === p.name));
}

// --- Main loop ---
let lastT = 0;
function frame(t) {
  const dt = lastT ? Math.min(250, t - lastT) : 0;
  lastT = t;
  sim.acc += dt;
  const shown = visibleMinis();
  while (sim.acc >= FRAME_MS) {
    sim.acc -= FRAME_MS;
    playTick();
    simFrame();
    for (const [, player] of shown) player.frame();
  }
  drawStage();
  for (const th of thumbs) drawThumb(th);
  for (const [canvas, player] of shown) drawPair(canvas, player.engine);
  requestAnimationFrame(frame);
}

async function start() {
  stage.canvas = $('rings');
  stage.board = document.createElement('canvas');
  $('port-source').textContent = PORT_SOURCE;
  bindPicker(); bindSwatches(); bindKeys(); bindPops(); bindEditor();
  setSwap(view.swap);
  try {
    [caps, anims, server] = await Promise.all([api('/api/capabilities'), api('/api/animations'), api('/api/state')]);
  } catch (err) {
    setLink(false);
    showError(`Cannot reach the Eyes Studio server: ${err.message}`);
    return;
  }
  caps = caps && typeof caps === 'object' ? caps : {};
  anims = Array.isArray(anims) ? anims.filter((a) => a && a.name) : [];
  server = server && typeof server === 'object' ? server : {};
  anims.forEach((a) => { animByName[a.name] = a; animById[a.id] = a; });
  lastCaps = performance.now();
  applyCapabilities();
  setLink(true);
  ui.left = commandedEye(server, 'left'); ui.right = commandedEye(server, 'right');
  if (typeof server.intensity === 'number') ui.intensity = server.intensity;
  const color = Array.isArray(server.color) ? server.color.slice(0, 3) : parseHex(server.color_hex);
  setColor(color || ui.color, { send: false });
  buildEffects(); buildPresets();
  applyToEngine();
  sim.engine.snap(0, animId(previewName(ui.left))); sim.engine.snap(1, animId(previewName(ui.right)));
  renderStatus(); renderControls(); renderTimeline();
  followPlayback();
  window.addEventListener('resize', () => { stage.layout = null; renderPicker(); });
  setTimeout(pollLoop, 1000);
  requestAnimationFrame(frame);
  // Hook for the browser test suite and for poking at the model from devtools.
  window.eyesStudio = {
    sim, ui, view, EyeEngine, MM, PORT_SOURCE, sideOf, chainIndex, designator, shownPixels,
    caps: () => caps, layout: () => stage.layout, editor, play, library, server: () => server,
  };
}

start();
})();
