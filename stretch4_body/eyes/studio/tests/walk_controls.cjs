// Headless walk of every Eyes Studio control against the fake backend.
//
//   NODE_PATH=<dir with playwright in node_modules> PYTHONPATH=<repo> \
//     node walk_controls.cjs [screenshot_dir]
//
// Starts `python3 -m stretch4_body.eyes.studio --fake` on a free port with its
// looks library pointed at a fresh temporary folder (HELLO_FLEET_PATH,
// HELLO_FLEET_ID and STRETCH_EYES_LIBRARY), so saving, importing, duplicating
// and deleting never touch a robot's real ~/stretch_user or fleet folder. The
// folder is removed at exit. EYES_STUDIO_URL points the walk at a running
// server instead; the checks that write to the library are then skipped unless
// EYES_STUDIO_SCRATCH_LIBRARY=1 says that server's library is a scratch one.
//
// Drives the page in Chromium and checks both the POSTs the page makes and the
// state the server ends up in: every control, every pop-over (open, focus trap,
// Escape, outside click, no layout shift), the one-screen layout at 1280x720,
// 1440x900 and 1920x1080, the sequence editor (build, hold, reorder, play,
// stop, save, export, import) and the looks library. With a screenshot
// directory it also writes desktop shots in light and dark, the editor while a
// sequence plays, the library, details, override previews and phone shots.
'use strict';
const { chromium } = require('playwright');
const { spawn } = require('child_process');
const fs = require('fs');
const net = require('net');
const os = require('os');
const path = require('path');

const REPO = path.resolve(__dirname, '../../../..');
const SHOTS = process.argv[2] || null;
const results = [];
const EXTERNAL = !!process.env.EYES_STUDIO_URL;
const LIBRARY_WRITES = !EXTERNAL || process.env.EYES_STUDIO_SCRATCH_LIBRARY === '1';

async function check(name, fn) {
  try { await fn(); results.push([true, name]); console.log(`ok    ${name}`); }
  catch (err) { results.push([false, name]); console.log(`FAIL  ${name}\n      ${err.message.split('\n')[0]}`); }
}
function expect(cond, msg) { if (!cond) throw new Error(msg); }
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function freePort() {
  return new Promise((resolve) => {
    const s = net.createServer().listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => resolve(p)); });
  });
}

// A scratch looks library: user dir under a fake fleet folder, one shared look.
const TMP = fs.mkdtempSync(path.join(os.tmpdir(), 'eyes-studio-walk-'));
const FLEET = path.join(TMP, 'fleet'), FLEET_ID = 'stretch-se4-walk', SHARED = path.join(TMP, 'shared');
const USER_DIR = path.join(FLEET, FLEET_ID, 'eyes');
fs.mkdirSync(SHARED, { recursive: true });
fs.writeFileSync(path.join(SHARED, 'walk-shared.json'), JSON.stringify({
  format: 'stretch-eyes/1', name: 'walk-shared', title: 'Shared from the team folder', loop: false,
  steps: [{ left: 'blink', right: 'blink', color: '#ff6a10', intensity: 200, hold: 0.6 }, { left: 'happy', right: 'happy', hold: 0.6 }],
}, null, 2));
process.on('exit', () => { try { fs.rmSync(TMP, { recursive: true, force: true }); } catch (err) { /* already gone */ } });

async function startServer() {
  if (EXTERNAL) return { url: process.env.EYES_STUDIO_URL, proc: null };
  const port = await freePort();
  const env = {
    ...process.env, PYTHONPATH: process.env.PYTHONPATH || REPO,
    HELLO_FLEET_PATH: FLEET, HELLO_FLEET_ID: FLEET_ID, STRETCH_EYES_LIBRARY: SHARED,
  };
  const proc = spawn('python3', ['-m', 'stretch4_body.eyes.studio', '--fake', '--host', '127.0.0.1', '--port', String(port)],
    { cwd: REPO, env, stdio: ['ignore', 'pipe', 'inherit'] });
  // Whatever ends this process (a browser that fails to launch, an uncaught error, the
  // final exit) must not leave the server running on its port.
  process.on('exit', () => { if (proc.exitCode === null) proc.kill(); });
  await new Promise((resolve, reject) => {
    proc.stdout.on('data', (d) => { if (String(d).includes('Eyes Studio')) resolve(); });
    proc.on('exit', (code) => reject(new Error(`server exited with ${code}`)));
  });
  return { url: `http://127.0.0.1:${port}`, proc };
}

(async () => {
  const { url, proc } = await startServer();
  const api = async (p, body) => {
    const res = await fetch(url + p, body ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {});
    return res.json();
  };
  const apiRes = async (p, body, headers = {}) => {
    const res = await fetch(url + p, body !== undefined ? { method: 'POST', headers: { 'Content-Type': 'application/json', ...headers }, body: typeof body === 'string' ? body : JSON.stringify(body) } : { headers });
    return { status: res.status, body: await res.json().catch(() => ({})) };
  };
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1440, height: 900 }, acceptDownloads: true });
  // Record every string drawn on a canvas, to prove the board layer carries no designators.
  await page.addInitScript(() => {
    const orig = CanvasRenderingContext2D.prototype.fillText;
    window.__drawnText = [];
    CanvasRenderingContext2D.prototype.fillText = function (t, ...rest) { window.__drawnText.push(String(t)); return orig.call(this, t, ...rest); };
  });
  const errors = [];
  // The walk provokes a 409 (save over an existing look, which the page answers with
  // its overwrite question) and a 400 (importing an invalid look) on purpose; Chromium
  // logs every 4xx fetch as a console error. Anything else counts.
  const DELIBERATE = /^Failed to load resource: the server responded with a status of (400|409) /;
  page.on('console', (m) => { if (m.type() === 'error' && !DELIBERATE.test(m.text())) errors.push(m.text()); });
  page.on('pageerror', (e) => errors.push(String(e)));
  const posts = [];
  page.on('request', (r) => { if (r.method() === 'POST') posts.push({ path: new URL(r.url()).pathname, body: JSON.parse(r.postData() || '{}') }); });
  const gets = [];
  page.on('request', (r) => { if (r.method() === 'GET') gets.push(new URL(r.url()).pathname); });

  // Run an action and return the bodies of the POSTs it caused to `apiPath`.
  async function postsFrom(apiPath, action, settle = 250) {
    const from = posts.length;
    const done = page.waitForResponse((r) => new URL(r.url()).pathname === apiPath && r.request().method() === 'POST');
    await action();
    await done;
    await sleep(settle);
    return posts.slice(from).filter((p) => p.path === apiPath).map((p) => p.body);
  }
  const merged = (bodies) => Object.assign({}, ...bodies);
  const ui = () => page.evaluate(() => ({ ...window.eyesStudio.ui, color: window.eyesStudio.ui.color.slice() }));
  const ready = async () => {
    await page.waitForFunction(() => window.eyesStudio && document.querySelectorAll('.fx-btn').length > 0 && document.querySelectorAll('.led-spot').length > 0);
    await sleep(300);
  };
  // Pop-over plates: key-X opens pop-X.
  const POPS = ['swatches', 'looks', 'library', 'overrides', 'details'];
  async function openPop(name) {
    if (await page.isHidden(`#pop-${name}`)) await page.click(`#key-${name}`);
    await page.waitForSelector(`#pop-${name}:not([hidden])`);
  }
  async function closePops() {
    for (const name of POPS) {
      if (await page.isVisible(`#pop-${name}`)) {
        await page.keyboard.press('Escape');
        await page.waitForSelector(`#pop-${name}`, { state: 'hidden' });
      }
    }
  }
  async function openSeq() {
    if (await page.isHidden('#seq')) await page.click('#key-seq');
    await page.waitForSelector('#seq:not([hidden])');
  }
  async function closeSeq() {
    if (await page.isVisible('#seq')) await page.click('#seq-close');
    await page.waitForSelector('#seq', { state: 'hidden' });
  }
  const editorSteps = () => page.evaluate(() => window.eyesStudio.editor.steps.map((s) => ({ ...s.data })));
  // One sample of the preview against the robot: /api/state fetched from the page, and
  // the preview's look when its step is the server's step (else null).
  const previewVsServer = () => page.evaluate(async () => {
    const st = await (await fetch('/api/state')).json();
    const e = window.eyesStudio, L = e.play.local;
    if (!st.playing || !L || st.playing.step !== L.idx) return null;
    return { step: L.idx, robot: [st.left, st.color.join(), st.intensity], preview: [e.ui.left, e.ui.color.join(), e.ui.intensity] };
  });
  const lookOf = (st) => ({ left: st.left, right: st.right, color: st.color.join(), intensity: st.intensity });
  async function download(action) {
    const [dl] = await Promise.all([page.waitForEvent('download'), action()]);
    return { name: dl.suggestedFilename(), text: fs.readFileSync(await dl.path(), 'utf8') };
  }
  // No page scroll and every faceplate control inside the viewport.
  const noScroll = () => page.evaluate(() => {
    const de = document.documentElement, out = [];
    if (de.scrollHeight !== innerHeight) out.push(`scrollHeight ${de.scrollHeight} vs ${innerHeight}`);
    if (de.scrollWidth > innerWidth) out.push(`scrollWidth ${de.scrollWidth} vs ${innerWidth}`);
    for (const el of document.querySelectorAll('.fx-btn, #rings, #hue-ring, #hex, #intensity, .softkeys .key, #btn-control, .readout')) {
      const r = el.getBoundingClientRect();
      if (r.bottom > innerHeight + 0.5 || r.right > innerWidth + 0.5 || r.top < -0.5) out.push(`${el.id || el.className} at ${Math.round(r.left)},${Math.round(r.top)} to ${Math.round(r.right)},${Math.round(r.bottom)}`);
    }
    const fx = document.getElementById('effects');
    if (fx.scrollHeight > fx.clientHeight + 1) out.push(`effect bank scrolls inside: ${fx.scrollHeight} > ${fx.clientHeight}`);
    return out;
  });
  const rectsOf = (sels) => page.evaluate((sels) => sels.map((s) => { const r = document.querySelector(s).getBoundingClientRect(); return [s, Math.round(r.left), Math.round(r.top), Math.round(r.width), Math.round(r.height)].join(' '); }), sels);
  const FACEPLATE = ['.fx', '.window', '.colour', '.softkeys', '.head'];

  // Sample the runstop blink for 1.2 s: the runstop LED is seen on and off, every eye
  // pixel is (40, 40, 40) while it is on and dark while it is off.
  async function expectRunstopBlink(what) {
    await page.waitForFunction(() => window.eyesStudio.sim.runstop, null, { timeout: 2000 });
    const seen = { on: 0, off: 0 };
    for (let k = 0; k < 24; k++) {
      const s = await page.evaluate(() => {
        const e = window.eyesStudio;
        return { runstop: e.sim.runstop, led: e.sim.runstopLed, px: Array.from(e.shownPixels(0)).concat(Array.from(e.shownPixels(1))) };
      });
      expect(s.runstop, `${what}: sim.runstop went off`);
      expect(s.px.length === 60 && s.px.every((x) => x === (s.led ? 40 : 0)), `${what}: LED ${s.led}, pixels ${s.px.slice(0, 6)}`);
      seen[s.led ? 'on' : 'off']++;
      await sleep(50);
    }
    expect(seen.on > 0 && seen.off > 0, `${what}: LED on in ${seen.on}, off in ${seen.off} of 24 samples`);
  }

  try {
    await page.goto(url);
    await ready();
    const animations = await api('/api/animations');

    await check('page loads with no console errors', async () => {
      expect(errors.length === 0, errors.join(' | '));
      expect(await page.title() === 'Eyes Studio', 'title');
    });

    await check('fresh start: eyes not commanded, preview falls back to idle glow with a label', async () => {
      const st = await api('/api/state');
      expect(st.left == null || st.left_commanded === false, `server says left is commanded: ${JSON.stringify(st)}`);
      expect((await page.textContent('#ro-left')).includes('not commanded'), 'left readout');
      expect((await page.textContent('#ro-right')).includes('idle glow'), 'right readout');
      expect((await page.textContent('#ro-source')).includes('no command'), 'source readout');
      const cur = await page.evaluate(() => window.eyesStudio.sim.engine.eye.map((s) => s.current));
      expect(cur[0] === 2 && cur[1] === 2, `engine current ${cur}, expected IDLE_GLOW 2`);
      expect(await page.locator('.fx-btn[aria-pressed="true"]').count() === 0, 'an effect row is pressed');
      expect(await page.locator('#lamp-command.ok').count() === 0, 'command lamp lit before any command');
      expect((await page.textContent('#command-text')) === 'No command sent yet', 'command text');
    });

    await check('header names the protocol the API needs', async () => {
      const t = await page.textContent('#protocol');
      expect(/P13/.test(t), t);
    });

    await check(`all ${animations.length} effects listed`, async () => {
      expect(await page.locator('.fx-btn').count() === animations.length, 'effect count');
    });

    for (const a of animations) {
      await check(`effect ${a.name} posts both eyes and lands`, async () => {
        const bodies = await postsFrom('/api/eyes', () => page.click(`.fx-btn[data-anim="${a.name}"]`));
        const body = merged(bodies);
        expect(body.left === a.name && body.right === a.name, JSON.stringify(body));
        const st = await api('/api/state');
        expect(st.left === a.name && st.right === a.name, `state ${st.left}/${st.right}`);
        expect(await page.getAttribute(`.fx-btn[data-anim="${a.name}"]`, 'aria-pressed') === 'true', 'aria-pressed');
      });
    }

    await check('command lamp lights once a command is accepted', async () => {
      await page.waitForFunction(() => document.getElementById('lamp-command').classList.contains('ok'), null, { timeout: 3000 });
      expect((await page.textContent('#ro-source')) === 'last command', await page.textContent('#ro-source'));
    });

    await check('link Left sends only the left eye', async () => {
      await page.click('.link label:has(input[value="left"])');
      const body = merged(await postsFrom('/api/eyes', () => page.click('.fx-btn[data-anim="blink"]')));
      expect(body.left === 'blink' && !('right' in body), JSON.stringify(body));
      const st = await api('/api/state');
      expect(st.left === 'blink' && st.right === 'circle_ccw', `state ${st.left}/${st.right}`);
    });
    await check('link Right sends only the right eye', async () => {
      await page.click('.link label:has(input[value="right"])');
      const body = merged(await postsFrom('/api/eyes', () => page.click('.fx-btn[data-anim="happy"]')));
      expect(body.right === 'happy' && !('left' in body), JSON.stringify(body));
      const st = await api('/api/state');
      expect(st.left === 'blink' && st.right === 'happy', `state ${st.left}/${st.right}`);
    });
    await check('link Both sends both eyes', async () => {
      await page.click('.link label:has(input[value="both"])');
      const body = merged(await postsFrom('/api/eyes', () => page.click('.fx-btn[data-anim="look_right"]')));
      expect(body.left === 'look_right' && body.right === 'look_right', JSON.stringify(body));
    });

    await check('hex input sets colour on the server', async () => {
      const body = merged(await postsFrom('/api/eyes', async () => { await page.fill('#hex', '#12abef'); await page.press('#hex', 'Enter'); }));
      expect(body.color === '#12abef', JSON.stringify(body));
      expect((await api('/api/state')).color_hex === '#12abef', 'state colour');
    });
    await check('bad hex is flagged and not sent', async () => {
      const from = posts.length;
      await page.fill('#hex', 'zz12'); await page.press('#hex', 'Enter');
      await sleep(200);
      expect(await page.getAttribute('#hex', 'aria-invalid') === 'true', 'aria-invalid');
      expect(posts.length === from, 'posted');
    });
    await check('RGB field sets one channel', async () => {
      const body = merged(await postsFrom('/api/eyes', async () => { await page.fill('#rgb-r', '200'); await page.press('#rgb-r', 'Enter'); }));
      expect(body.color === '#c8abef', JSON.stringify(body));
    });
    await check('RGB fields set the green and blue channels', async () => {
      await page.fill('#hex', '#102030'); await page.press('#hex', 'Enter');
      await sleep(200);
      let body = merged(await postsFrom('/api/eyes', async () => { await page.fill('#rgb-g', '150'); await page.press('#rgb-g', 'Enter'); }));
      expect(body.color === '#109630', JSON.stringify(body));
      body = merged(await postsFrom('/api/eyes', async () => { await page.fill('#rgb-b', '7'); await page.press('#rgb-b', 'Enter'); }));
      expect(body.color === '#109607', JSON.stringify(body));
      expect((await api('/api/state')).color_hex === '#109607', 'state colour');
    });
    await check('hex commits on blur without Enter', async () => {
      const body = merged(await postsFrom('/api/eyes', async () => { await page.fill('#hex', '#223344'); await page.keyboard.press('Tab'); }));
      expect(body.color === '#223344', JSON.stringify(body));
      expect((await api('/api/state')).color_hex === '#223344', 'state colour');
    });

    await check('hue ring drag changes colour, writes are rate limited', async () => {
      const before = (await api('/api/state')).color_hex;
      const writes0 = (await api('/api/state')).studio.writes;
      const box = await page.locator('#hue-ring').boundingBox();
      const cx = box.x + box.width / 2, cy = box.y + box.height / 2, r = box.width * 0.44;
      const from = posts.length;
      const t0 = Date.now();
      await page.mouse.move(cx + r, cy);
      await page.mouse.down();
      for (let k = 0; k <= 60; k++) {
        const a = k / 60 * Math.PI;
        await page.mouse.move(cx + r * Math.cos(a), cy + r * Math.sin(a));
      }
      await page.mouse.up();
      const secs = (Date.now() - t0) / 1000;
      await sleep(400);
      const st = await api('/api/state');
      const local = await ui();
      const sent = posts.slice(from).filter((p) => p.path === '/api/eyes').length;
      const writes = st.studio.writes - writes0;
      expect(st.color_hex !== before, 'colour unchanged');
      expect(st.color_hex === '#' + local.color.map((x) => x.toString(16).padStart(2, '0')).join(''), `server ${st.color_hex} page ${local.color}`);
      const limit = Math.ceil(secs * 20) + 2;
      expect(sent <= limit && writes <= limit, `${sent} posts, ${writes} writes in ${secs.toFixed(2)} s, limit ${limit}`);
      console.log(`      61 drag events in ${secs.toFixed(2)} s: ${sent} posts, ${writes} backend writes`);
    });
    await check('saturation/value square drag reaches full value', async () => {
      const box = await page.locator('#sv-square').boundingBox();
      await postsFrom('/api/eyes', async () => {
        await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
        await page.mouse.down();
        await page.mouse.move(box.x + box.width - 1, box.y + 1, { steps: 8 });
        await page.mouse.up();
      }, 400);
      const st = await api('/api/state');
      expect(Math.max(...st.color) >= 250, `color ${st.color}`);
    });
    await check('hue ring answers the keyboard', async () => {
      await page.focus('#hue-ring');
      const h0 = Number(await page.getAttribute('#hue-ring', 'aria-valuenow'));
      const body = merged(await postsFrom('/api/eyes', () => page.keyboard.press('Shift+ArrowRight')));
      const h1 = Number(await page.getAttribute('#hue-ring', 'aria-valuenow'));
      expect((h1 - h0 + 360) % 360 === 10, `hue ${h0} -> ${h1}`);
      expect(typeof body.color === 'string', JSON.stringify(body));
    });
    await check('hue ring PageUp, Home and End', async () => {
      await page.focus('#hue-ring');
      const hue = async () => Number(await page.getAttribute('#hue-ring', 'aria-valuenow'));
      const h0 = await hue();
      await postsFrom('/api/eyes', () => page.keyboard.press('PageUp'));
      expect(await hue() === (h0 + 30) % 360, `PageUp: ${h0} -> ${await hue()}`);
      await postsFrom('/api/eyes', () => page.keyboard.press('Home'));
      expect(await hue() === 0, `Home: ${await hue()}`);
      await postsFrom('/api/eyes', () => page.keyboard.press('End'));
      expect(await hue() === 359, `End: ${await hue()}`);
      const st = await api('/api/state'), local = await ui();
      expect(st.color_hex === '#' + local.color.map((x) => x.toString(16).padStart(2, '0')).join(''), `server ${st.color_hex} page ${local.color}`);
    });
    await check('saturation/value square answers the keyboard', async () => {
      await page.fill('#hex', '#4080c0'); await page.press('#hex', 'Enter');
      await sleep(200);
      await page.focus('#sv-square');
      let before = (await api('/api/state')).color;
      let bodies = await postsFrom('/api/eyes', () => page.keyboard.press('ArrowUp'));
      let st = await api('/api/state');
      expect(bodies.length === 1 && Math.max(...st.color) > Math.max(...before), `ArrowUp: ${bodies.length} posts, ${before} -> ${st.color}`);
      before = st.color;
      bodies = await postsFrom('/api/eyes', () => page.keyboard.press('Shift+ArrowRight'));
      st = await api('/api/state');
      expect(bodies.length === 1 && Math.min(...st.color) < Math.min(...before), `Shift+ArrowRight: ${bodies.length} posts, ${before} -> ${st.color}`);
    });

    await check('swatch save, apply, persist, delete', async () => {
      await page.fill('#hex', '#336699'); await page.press('#hex', 'Enter');
      await sleep(200);
      await openPop('swatches');
      await page.click('#swatch-save');
      expect(await page.locator('.swatch.saved[data-hex="#336699"]').count() === 1, 'not saved');
      await page.click('.swatch.factory[data-hex="#ffffff"]');
      await sleep(250);
      expect((await api('/api/state')).color_hex === '#ffffff', 'factory swatch');
      await page.reload();
      await ready();
      await openPop('swatches');
      const body = merged(await postsFrom('/api/eyes', () => page.click('.swatch.saved[data-hex="#336699"]')));
      expect(body.color === '#336699', JSON.stringify(body));
      await page.focus('.swatch.saved[data-hex="#336699"]');
      await page.keyboard.press('Delete');
      expect(await page.locator('.swatch.saved').count() === 0, 'not deleted');
      await closePops();
    });
    await check('swatch clear removes every saved swatch and disables itself', async () => {
      for (const hex of ['#aa5500', '#0055aa']) {
        await page.fill('#hex', hex); await page.press('#hex', 'Enter');
        await sleep(200);
        await openPop('swatches');
        await page.click('#swatch-save');
      }
      expect(await page.locator('.swatch.saved').count() === 2, 'two swatches saved');
      expect(!(await page.isDisabled('#swatch-clear')), 'clear key disabled with swatches saved');
      await page.click('#swatch-clear');
      expect(await page.locator('.swatch.saved').count() === 0, 'swatches remain');
      expect(await page.evaluate(() => localStorage.getItem('eyes-studio.swatches')) === '[]', 'localStorage not emptied');
      expect(await page.isDisabled('#swatch-clear'), 'clear key still enabled with nothing saved');
      expect(await page.evaluate(() => document.getElementById('pop-swatches').contains(document.activeElement)), 'focus left the plate after clear');
      await closePops();
    });

    await check('intensity fader posts and lands', async () => {
      const body = merged(await postsFrom('/api/eyes', () => page.fill('#intensity', '100')));
      expect(body.intensity === 100, JSON.stringify(body));
      await page.focus('#intensity');
      await postsFrom('/api/eyes', () => page.keyboard.press('ArrowLeft'));
      expect((await api('/api/state')).intensity === 99, 'state intensity');
      expect(await page.textContent('#intensity-out') === '99', 'output');
    });

    await check('quick looks post a full look', async () => {
      await openPop('looks');
      const n = await page.locator('#presets .key').count();
      expect(n >= 6, `${n} presets`);
      const body = merged(await postsFrom('/api/eyes', () => page.click('#presets .key[data-preset="Scanning"]')));
      expect(body.left === 'circle_cw' && body.right === 'circle_ccw' && body.color === '#00a0ff' && body.intensity === 255, JSON.stringify(body));
      await closePops();
    });

    await check('preview matches firmware math (look right, white, 255)', async () => {
      await page.fill('#hex', '#ffffff'); await page.press('#hex', 'Enter');
      await page.click('.fx-btn[data-anim="look_right"]');
      await sleep(900);   // 0.5 s ease plus the 0.3 s crossfade
      const px = await page.evaluate(() => Array.from(window.eyesStudio.sim.engine.out[0]));
      // Pupil at index 2: v = 1 at idx 2, (1 - 0.5)^2 = 0.25 at idx 1 and 3, truncated to 63.
      expect(px[6] === 255 && px[3] === 63 && px[9] === 63 && px[0] === 0, `pixels ${px.slice(0, 15)}`);
    });

    await check('preview matches firmware math (rainbow floor, intensity 10 renders at 40)', async () => {
      await page.fill('#intensity', '10');
      await postsFrom('/api/eyes', () => page.click('.fx-btn[data-anim="rainbow_spin"]'));
      await sleep(500);
      const px = await page.evaluate(() => Array.from(window.eyesStudio.sim.engine.out[0]));
      const peak = Math.max(...px);
      expect(peak === 40, `brightest channel ${peak}, expected the floor of 40`);
      await page.fill('#intensity', '255');
      await sleep(200);
    });

    await check('crossfade takes 15 frames and a change during a fade waits', async () => {
      const r = await page.evaluate(() => {
        const E = window.eyesStudio.EyeEngine;
        const e = new E(10);
        e.setAnimation(1, 1, 255, 255, 255, 255);   // OFF
        e.snap(0, 1); e.snap(1, 1);
        e.setAnimation(9, 9, 255, 255, 255, 255);   // LEFT_HALF, full white
        const ramp = [];
        for (let f = 0; f < 20; f++) {
          if (f === 3) e.setAnimation(10, 10, 255, 255, 255, 255);   // RIGHT_HALF while fading
          e.step();
          ramp.push([e.out[0][15], e.out[0][0], e.blend[0].active, e.eye[0].current]);
        }
        return ramp;
      });
      // Frames 0..13 blend toward LEFT_HALF (pixel 5 lit), frame 14 is the first unblended one.
      expect(r[0][0] === Math.floor(255 * 17 / 256) && r[13][2] === true && r[14][2] === false && r[14][0] === 255, JSON.stringify(r.slice(0, 15)));
      // The RIGHT_HALF request waited for the fade to finish, then started its own at frame 15.
      expect(r[14][3] === 9 && r[15][3] === 10 && r[15][2] === true, JSON.stringify(r.slice(13, 17)));
    });

    await check('off key', async () => {
      await postsFrom('/api/off', () => page.click('#btn-off'));
      const st = await api('/api/state');
      expect(st.left === 'off' && st.right === 'off', `${st.left}/${st.right}`);
      await sleep(450);
      const px = await page.evaluate(() => Array.from(window.eyesStudio.sim.engine.out[0]).concat(Array.from(window.eyesStudio.sim.engine.out[1])));
      expect(px.every((v) => v === 0), 'preview not dark after the crossfade');
    });
    await check('idle key restores the boot default', async () => {
      await postsFrom('/api/idle', () => page.click('#btn-idle'));
      const st = await api('/api/state');
      expect(st.left === 'idle_glow' && st.color_hex === '#28303c' && st.intensity === 255, JSON.stringify(st));
    });

    await check('take and release control', async () => {
      expect(await page.isVisible('#sentry-notice'), 'sentry notice should show while sentry runs');
      await postsFrom('/api/control', () => page.click('#btn-control'));
      let st = await api('/api/state');
      expect(st.in_control && !st.sentry_active, JSON.stringify(st));
      expect(await page.textContent('#btn-control') === 'Release control', 'button text');
      expect(!(await page.isVisible('#sentry-notice')), 'notice still visible');
      const body = merged(await postsFrom('/api/control', () => page.click('#btn-control')));
      expect(body.take === false, JSON.stringify(body));
      st = await api('/api/state');
      expect(!st.in_control && st.sentry_active, JSON.stringify(st));
    });

    await check('cross-site POSTs are refused: text/plain 415, foreign Origin 403, no preflight answer', async () => {
      const post = (headers) => fetch(url + '/api/control', { method: 'POST', headers: { Origin: 'http://evil.example', ...headers }, body: '{"take":true}' });
      let res = await post({ 'Content-Type': 'text/plain' });
      expect(res.status === 415, `text/plain: ${res.status}`);
      res = await post({ 'Content-Type': 'application/json' });
      expect(res.status === 403, `foreign Origin with JSON: ${res.status}`);
      res = await fetch(url + '/api/control', { method: 'OPTIONS', headers: { Origin: 'http://evil.example', 'Access-Control-Request-Method': 'POST' } });
      expect(res.status === 501 && !res.headers.get('access-control-allow-origin'), `preflight: ${res.status}`);
      const st = await api('/api/state');
      expect(st.in_control === false, `in_control ${st.in_control}`);
    });

    await check('release that cannot resume the sentry shows a notice with a resume key', async () => {
      await postsFrom('/api/control', () => page.click('#btn-control'));
      await api('/api/fake', { sentry_refuse: true });
      await postsFrom('/api/control', () => page.click('#btn-control'));
      await page.waitForFunction(() => !document.getElementById('resume-notice').hidden, null, { timeout: 4000 });
      let st = await api('/api/state');
      expect(st.sentry_active === false && st.studio.sentry_resume_failed === true, JSON.stringify(st.studio));
      expect((await page.textContent('#resume-text')).includes('could not be resumed'), 'notice text');
      await api('/api/fake', { sentry_refuse: false });
      await postsFrom('/api/sentry', () => page.click('#btn-resume'));
      st = await api('/api/state');
      expect(st.sentry_active === true && st.studio.sentry_resume_failed === false, JSON.stringify(st.studio));
      await page.waitForFunction(() => document.getElementById('resume-notice').hidden, null, { timeout: 3000 });
    });

    await check('dropped command lights the fault lamp and names the lease holder', async () => {
      await api('/api/fake', { drop: true, lease_holder: 'stretch_gamepad_teleop' });
      await postsFrom('/api/eyes', () => page.click('.fx-btn[data-anim="alert"]'));
      await page.waitForFunction(() => document.getElementById('lamp-command').classList.contains('fault'), null, { timeout: 3000 });
      const st = await api('/api/state');
      expect(st.source === 'dropped', `source ${st.source}`);
      const text = await page.textContent('#dropped-notice');
      expect(await page.isVisible('#dropped-notice') && text.includes('stretch_gamepad_teleop') && text.includes('lease'), text);
      expect((await page.textContent('#ro-source')).includes('dropped'), 'source readout');
      await api('/api/fake', { drop: false });
      await postsFrom('/api/eyes', () => page.click('.fx-btn[data-anim="blink"]'));
      await page.waitForFunction(() => document.getElementById('lamp-command').classList.contains('ok'), null, { timeout: 3000 });
      expect(!(await page.isVisible('#dropped-notice')), 'notice still shown after an accepted command');
    });

    await check('unsupported controls are disabled with a reason', async () => {
      expect(await page.isDisabled('#per-eye-color'), 'per-eye colour enabled');
      const text = await page.textContent('#per-eye-color-reason');
      expect(/one r, g, b|no PIMU protocol/i.test(text) && !/needs P14/i.test(text), `per-eye reason: ${text}`);
      expect(await page.isDisabled('#btn-paint'), 'paint enabled');
      expect((await page.textContent('#paint-reason')).length > 10, 'paint reason');
      expect(await page.getAttribute('#btn-paint', 'aria-describedby') === 'paint-reason', 'describedby');
    });

    await check('protocol unknown is said so; below P13 disables colour and intensity', async () => {
      await api('/api/fake', { protocol: null });
      await page.reload(); await ready();
      const head = await page.textContent('#protocol');
      expect(/unknown|assumed/i.test(head) && /P13/.test(head), head);
      await api('/api/fake', { protocol: 'p12' });
      await page.reload(); await ready();
      expect(await page.isDisabled('#hex') && await page.isDisabled('#intensity') && await page.isDisabled('#swatch-save'), 'colour controls enabled on p12');
      expect(await page.getAttribute('#hue-ring', 'aria-disabled') === 'true', 'hue ring not marked disabled');
      expect((await page.textContent('#colour-reason')).includes('P13') && (await page.textContent('#intensity-reason')).includes('P12'), 'reason');
      const notice = await page.textContent('#protocol-notice');
      expect(await page.isVisible('#protocol-notice') && notice.includes('P12') && notice.includes('P13'), `protocol notice: ${notice}`);
      await openPop('looks');
      const body = merged(await postsFrom('/api/eyes', () => page.click('#presets .key[data-preset="Attention"]')));
      await closePops();
      expect(body.left === 'alert' && !('color' in body) && !('intensity' in body), JSON.stringify(body));
      await api('/api/fake', { protocol: 'p13' });
      await page.reload(); await ready();
      expect(!(await page.isDisabled('#hex')) && (await page.textContent('#colour-reason')) === '', 'colour still disabled on p13');
      expect(!(await page.isVisible('#protocol-notice')), 'protocol notice still shown on p13');
    });

    await check('live runstop and low SOC from server state', async () => {
      await api('/api/fake', { runstop: true });
      await page.waitForFunction(() => document.getElementById('lamp-runstop').classList.contains('fault'), null, { timeout: 3000 });
      expect((await page.textContent('#override-banner')).startsWith('Runstop'), 'banner');
      await expectRunstopBlink('live runstop');
      await api('/api/fake', { runstop: false, battery_soc: 10 });
      await page.waitForFunction(() => document.getElementById('lamp-soc').classList.contains('fault'), null, { timeout: 3000 });
      await sleep(450);
      const px = await page.evaluate(() => Array.from(window.eyesStudio.sim.engine.out[0]));
      // Left eye shows LEFT_HALF in (64, 0, 0): logical 5..9 lit.
      expect(px[15] === 64 && px[16] === 0 && px[0] === 0, `pixels ${px}`);
      await api('/api/fake', { battery_soc: 100 });
      await page.waitForFunction(() => !document.getElementById('lamp-soc').classList.contains('fault'), null, { timeout: 3000 });
      // The engine drops the override on its next frame; leave it back on the live command.
      await page.waitForFunction(() => !window.eyesStudio.sim.engine.ovr.active, null, { timeout: 2000 });
    });
    await check('override preview selector drives the engine override and the runstop blink', async () => {
      const live = await page.evaluate(() => window.eyesStudio.sim.engine.eye[0].current);
      await openPop('overrides');
      for (const [v, rgb] of [['runstop', null], ['soc25', [110, 30, 0]], ['soc12', [64, 0, 0]]]) {
        await page.click(`.link label:has(input[name="ovr"][value="${v}"])`);
        expect(await page.isVisible('#override-banner'), `banner hidden for ${v}`);
        expect((await page.textContent('#override-banner')).includes('preview'), 'preview tag');
        if (!rgb) { await expectRunstopBlink('preview runstop'); continue; }
        await sleep(450);   // past the 300 ms crossfade into the override
        const px = await page.evaluate(() => Array.from(window.eyesStudio.sim.engine.out[0]));
        // Left eye shows LEFT_HALF in the override colour: logical 5..9 lit, 0..4 dark.
        expect(px.slice(15, 18).join() === rgb.join() && px.slice(0, 3).every((x) => x === 0), `${v} pixels ${px}`);
      }
      expect(await page.evaluate(() => document.getElementById('lamp-ovr').className.includes('fault')), 'Overrides key lamp not lit on the SOC 12 preview');
      await page.click('.link label:has(input[name="ovr"][value="live"])');
      await closePops();
      await sleep(1200);
      expect(!(await page.isVisible('#override-banner')), 'banner still visible on live');
      expect(await page.getAttribute('#lamp-ovr', 'class') === 'lamp ', 'Overrides key lamp still lit on live');
      const back = await page.evaluate(() => [window.eyesStudio.sim.engine.ovr.active, window.eyesStudio.sim.engine.eye[0].current, window.eyesStudio.sim.runstop]);
      expect(back[0] === false && back[1] === live && back[2] === false, `after live: override ${back[0]}, animation ${back[1]} not ${live}, runstop ${back[2]}`);
    });

    await check('board geometry follows the V0.1 files and the tabs never meet', async () => {
      const g = await page.evaluate(() => {
        const s = window.eyesStudio, L = s.layout();
        return { MM: s.MM, k: L.k, sep: L.sides[1].cx - L.sides[0].cx, W: L.W, H: L.H, cy: L.cy };
      });
      const m = g.MM;
      expect(m.ringIn === 8 && m.ringOut === 15 && m.tabHalf === 12 && m.tabOut === 21 && m.cnOut === 23 && m.ledR === 11.45, JSON.stringify(m));
      const reach = 2 * m.cnOut * g.k * Math.sin(108 * Math.PI / 180);   // both tabs point 18 degrees below horizontal, inward
      expect(g.sep > reach + 4, `tabs meet: centres ${g.sep.toFixed(0)} px apart, reach ${reach.toFixed(0)} px`);
      expect(g.cy + m.ringOut * g.k + 30 < g.H && g.cy - m.ringOut * g.k > 20, 'rings do not fit the window vertically');
    });

    await check('no designator or title silkscreen on the boards; hotspots name the logical index', async () => {
      const drawn = await page.evaluate(() => window.__drawnText.filter((t) => /^LED\d+$/.test(t) || /RING/.test(t)));
      expect(drawn.length === 0, `drawn on the canvas: ${drawn.slice(0, 5)}`);
      expect(await page.locator('.led-spot').count() === 20, 'spot count');
      await page.hover('.led-spot[data-eye="0"][data-i="2"]');
      let t = await page.textContent('#led-readout');
      expect(t.includes('Left eye px 2') && t.includes('chain px 17') && t.includes('LED10'), t);
      await page.mouse.move(2, 2);
      await sleep(50);
      t = await page.textContent('#led-readout');
      expect(!t.includes(' px ') && t.startsWith('Tap'), `readout kept after leaving: ${t}`);
    });

    await check('hotspots answer the keyboard: arrows walk the ring, Enter pins, Escape unpins', async () => {
      await page.focus('.led-spot[data-eye="0"][data-i="0"]');
      await page.keyboard.press('ArrowRight');
      expect(await page.evaluate(() => document.activeElement.dataset.i) === '1', 'focus did not move');
      let t = await page.textContent('#led-readout');
      expect(t.includes('Left eye px 1') && t.includes('chain px 16') && t.includes('LED9'), t);
      await page.keyboard.press('Enter');
      expect(await page.getAttribute('.led-spot[data-eye="0"][data-i="1"]', 'aria-pressed') === 'true', 'not pinned');
      await page.focus('#btn-idle');
      t = await page.textContent('#led-readout');
      expect(t.includes('px 1') && t.includes('pinned'), `pin lost: ${t}`);
      await page.focus('.led-spot[data-eye="0"][data-i="1"]');
      await page.keyboard.press('Escape');
      expect(await page.getAttribute('.led-spot[data-eye="0"][data-i="1"]', 'aria-pressed') === 'false', 'not unpinned');
    });

    await check('left/right swap is one click, moves the boards and persists', async () => {
      const spotX = async (e, i) => (await page.locator(`.led-spot[data-eye="${e}"][data-i="${i}"]`).boundingBox()).x;
      const win = await page.locator('.window').boundingBox();
      const mid = win.x + win.width / 2;
      expect(await spotX(0, 0) < mid, 'firmware left eye not drawn on the viewer\'s left by default');
      expect((await page.textContent('#orientation-caption')).includes('verified by camera'), 'caption does not say verified');
      await openPop('details');
      await page.click('#btn-swap');
      await sleep(200);
      expect(await page.getAttribute('#btn-swap', 'aria-pressed') === 'true', 'aria-pressed');
      expect(await page.evaluate(() => window.eyesStudio.sideOf(0)) === 1, 'sideOf');
      expect(await spotX(0, 0) > mid, 'left eye hotspots did not move to the right');
      expect((await page.textContent('#orientation-caption')).includes('Swapped'), 'caption');
      expect(!(await page.getAttribute('.led-spot[data-eye="0"][data-i="2"]', 'aria-label')).includes('LED'), 'board LED number still claimed while swapped');
      await closePops();
      await page.reload(); await ready();
      expect(await page.getAttribute('#btn-swap', 'aria-pressed') === 'true', 'swap not persisted');
      expect(await page.evaluate(() => localStorage.getItem('eyes-studio.swap')) === '1', 'localStorage');
      expect(await spotX(0, 0) > mid, 'swap not applied after reload');
      await openPop('details');
      await page.click('#btn-swap');
      await closePops();
      await sleep(200);
      expect(await page.evaluate(() => window.eyesStudio.sideOf(0)) === 0 && await spotX(0, 0) < mid, 'unswap');
    });

    await check('every control is at least 44 px tall', async () => {
      const small = await page.evaluate(() => {
        const sel = '.key, .fx-btn, .swatch, .link label, .field input, .hold input, #intensity, .check';
        const out = [];
        for (const el of document.querySelectorAll(sel)) {
          if (!el.offsetParent) continue;   // hidden notices
          const r = el.getBoundingClientRect();
          if (r.height < 43.5 || r.width < 43.5) out.push(`${el.id || el.className} ${Math.round(r.width)}x${Math.round(r.height)}`);
        }
        return out;
      });
      expect(small.length === 0, small.join(', '));
    });

    await check('keyboard focus shows a 2 px ring on keys, rows, fields and hotspots', async () => {
      // Chromium starts sequential focus from the last clicked element, so walk a few
      // Tabs and check the ring on each landing spot rather than on one fixed element.
      const seen = [];
      for (let k = 0; k < 8; k++) {
        await page.keyboard.press('Tab');
        seen.push(await page.evaluate(() => {
          const el = document.activeElement, s = getComputedStyle(el);
          const ring = el.matches('input[name="link"], input[name="ovr"]') ? getComputedStyle(el.nextElementSibling) : s;
          return `${el.id || el.className}: ${ring.outlineStyle} ${ring.outlineWidth}`;
        }));
      }
      const bad = seen.filter((t) => !t.endsWith(': solid 2px'));
      expect(bad.length === 0, bad.join(' | '));
    });

    await check('every control reachable by Tab', async () => {
      const ids = ['btn-control', 'btn-idle', 'btn-off', 'led-spot', 'hue-ring', 'sv-square', 'hex',
        'rgb-r', 'rgb-g', 'rgb-b', 'intensity', 'key-swatches', 'key-looks', 'key-library', 'key-seq', 'key-overrides', 'key-details'];
      await page.focus('body');
      const seen = new Set();
      for (let k = 0; k < 140; k++) {
        await page.keyboard.press('Tab');
        seen.add(await page.evaluate(() => document.activeElement.id || document.activeElement.className));
      }
      const missing = ids.filter((id) => !seen.has(id));
      expect(missing.length === 0, `not reached: ${missing}`);
      expect(!seen.has('led-spot') || (await page.locator('.led-spot[tabindex="0"]').count()) === 1, 'more than one hotspot in the tab order');
    });

    await check('no em or en dashes in any visible string', async () => {
      const bad = await page.evaluate(() => {
        const texts = [document.title, document.body.textContent];
        for (const el of document.querySelectorAll('[aria-label], [title]')) texts.push(el.getAttribute('aria-label') || '', el.getAttribute('title') || '');
        return texts.filter((t) => /[\u2013\u2014]/.test(t)).map((t) => t.slice(0, 80));
      });
      expect(bad.length === 0, bad.join(' | '));
    });


    // -----------------------------------------------------------------------
    // One screen on a desktop, pop-overs and the drawer.
    // -----------------------------------------------------------------------
    for (const [w, h] of [[1280, 720], [1440, 900], [1920, 1080]]) {
      await check(`one screen at ${w}x${h}: no page scroll, every faceplate control in view, rings large`, async () => {
        await page.setViewportSize({ width: w, height: h });
        await sleep(350);
        expect(await page.isVisible('#sentry-notice'), 'walk expects the sentry notice to be showing (worst case height)');
        const bad = await noScroll();
        expect(bad.length === 0, bad.join(', '));
        const win = await page.locator('.window').boundingBox();
        expect(win.width >= 0.5 * w && win.height >= 0.45 * h, `window ${Math.round(win.width)}x${Math.round(win.height)} is small for ${w}x${h}`);
        const fxCount = await page.locator('.fx-btn').count();
        expect(fxCount === animations.length, `${fxCount} effect tiles`);
      });
    }
    await page.setViewportSize({ width: 1440, height: 900 });
    await sleep(300);

    for (const name of POPS) {
      await check(`pop-over ${name}: opens from its key, takes focus, traps Tab, Escape and outside click close it, layout does not move`, async () => {
        const before = await rectsOf(FACEPLATE);
        const key = `#key-${name}`, pop = `#pop-${name}`;
        await page.click(key);
        await page.waitForSelector(`${pop}:not([hidden])`);
        expect(await page.getAttribute(key, 'aria-expanded') === 'true', 'aria-expanded');
        expect(await page.evaluate((p) => document.querySelector(p).contains(document.activeElement), pop), 'focus not moved into the plate');
        const inView = await page.evaluate((p) => { const r = document.querySelector(p).getBoundingClientRect(); return r.left >= 0 && r.top >= 0 && r.right <= innerWidth && r.bottom <= innerHeight; }, pop);
        expect(inView, 'plate not fully in the viewport');
        const n = await page.evaluate((p) => document.querySelector(p).querySelectorAll('button:not([disabled]), input:not([disabled]), a[href]').length, pop);
        for (let k = 0; k < n + 3; k++) {
          await page.keyboard.press(k % 3 === 2 ? 'Shift+Tab' : 'Tab');
          expect(await page.evaluate((p) => document.querySelector(p).contains(document.activeElement), pop), `focus escaped after ${k + 1} Tabs`);
        }
        const after = await rectsOf(FACEPLATE);
        expect(JSON.stringify(after) === JSON.stringify(before), `layout moved: ${after.filter((r, i) => r !== before[i])}`);
        expect((await noScroll()).length === 0, 'page scrolls with the plate open');
        await page.keyboard.press('Escape');
        await page.waitForSelector(pop, { state: 'hidden' });
        expect(await page.getAttribute(key, 'aria-expanded') === 'false', 'aria-expanded after Escape');
        expect(await page.evaluate((k) => document.activeElement === document.querySelector(k), key), 'focus not back on the key');
        // Outside click.
        await page.click(key);
        await page.waitForSelector(`${pop}:not([hidden])`);
        await page.mouse.click(8, 8);
        await page.waitForSelector(pop, { state: 'hidden' });
        // The key toggles, and opening another plate closes this one.
        await page.click(key);
        await page.click(key);
        await page.waitForSelector(pop, { state: 'hidden' });
        await page.click(key);
        const other = POPS[(POPS.indexOf(name) + 1) % POPS.length];
        await page.click(`#key-${other}`);
        await page.waitForSelector(pop, { state: 'hidden' });
        await page.waitForSelector(`#pop-${other}:not([hidden])`);
        await closePops();
      });
    }

    await check('sequence drawer: opens over the lower window without moving the faceplate, rings move into view, Escape closes', async () => {
      const before = await rectsOf(['.fx', '.colour', '.softkeys', '.head']);
      const H0 = await page.evaluate(() => window.eyesStudio.layout().H);
      await page.click('#key-seq');
      await page.waitForSelector('#seq:not([hidden])');
      await sleep(150);
      expect(await page.getAttribute('#key-seq', 'aria-expanded') === 'true', 'aria-expanded');
      expect(await page.evaluate(() => document.activeElement.id) === 'seq-add', 'focus not on Add step');
      expect(JSON.stringify(await rectsOf(['.fx', '.colour', '.softkeys', '.head'])) === JSON.stringify(before), 'faceplate moved');
      expect((await noScroll()).length === 0, (await noScroll()).join(', '));
      const g = await page.evaluate(() => {
        const L = window.eyesStudio.layout(), dpr = devicePixelRatio || 1;
        const c = document.getElementById('rings').getBoundingClientRect(), d = document.getElementById('seq').getBoundingClientRect();
        return { H: L.H, Hv: L.Hv, ringBottom: c.top + (L.cy + 15 * L.k) / dpr, drawerTop: d.top };
      });
      expect(g.Hv < g.H && g.H === H0, `layout did not use the visible height: ${JSON.stringify(g)}`);
      expect(g.ringBottom < g.drawerTop, `rings run under the drawer: ring bottom ${g.ringBottom.toFixed(0)}, drawer top ${g.drawerTop.toFixed(0)}`);
      // Non-modal: the effect bank and the color fields stay usable while it is open.
      await postsFrom('/api/eyes', () => page.click('.fx-btn[data-anim="blink"]'));
      expect(await page.isVisible('#seq'), 'an effect click closed the drawer');
      await page.focus('#seq-name');
      await page.keyboard.press('Escape');
      await page.waitForSelector('#seq', { state: 'hidden' });
      expect(await page.evaluate(() => document.activeElement.id) === 'key-seq', 'focus not back on the Sequence key');
      // The layout follows on the next animation frame.
      await page.waitForFunction(() => window.eyesStudio.layout().Hv === window.eyesStudio.layout().H, null, { timeout: 1000 })
        .catch(() => { throw new Error('rings did not go back'); });
    });

    // -----------------------------------------------------------------------
    // Sequence editor.
    // -----------------------------------------------------------------------
    const BUILD = [['look_left', '#00a0ff', 128], ['look_right', '#20ff50', 200], ['happy', '#ff6a10', 255]];
    await check('sequence: Add step captures left, right, color and intensity from the current look', async () => {
      await openSeq();
      await page.click('#seq-new');
      for (const [anim, hex, level] of BUILD) {
        await page.click(`.fx-btn[data-anim="${anim}"]`);
        await page.fill('#hex', hex); await page.press('#hex', 'Enter');
        await page.fill('#intensity', String(level));
        await sleep(120);
        await page.click('#seq-add');
      }
      const steps = await editorSteps();
      expect(steps.length === 3, `${steps.length} steps`);
      BUILD.forEach(([anim, hex, level], i) => {
        const s = steps[i];
        expect(s.left === anim && s.right === anim && s.color === hex && s.intensity === level && s.hold === 1, `step ${i + 1}: ${JSON.stringify(s)}`);
      });
      expect(await page.locator('#timeline .step').count() === 3, 'cards');
      expect((await page.textContent('#seq-total')).startsWith('3 steps, 3 s'), await page.textContent('#seq-total'));
      // Each chip is a live preview: lit pixels of the step's color on the dark window.
      await sleep(400);
      const lit = await page.evaluate(() => Array.from(document.querySelectorAll('#timeline .chip')).map((c) => {
        const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
        let max = 0; for (let k = 0; k < d.length; k += 4) max = Math.max(max, d[k], d[k + 1], d[k + 2]);
        return max;
      }));
      expect(lit.length === 3 && lit.every((m) => m > 120), `chips not lit: ${lit}`);
    });

    await check('sequence: hold time edits with the - and + keys and by typing; bad values are flagged', async () => {
      const hold = (i) => page.evaluate((i) => window.eyesStudio.editor.steps[i].data.hold, i);
      const card = (i) => `#timeline .step:nth-child(${i + 1})`;
      await page.click(`${card(0)} [data-role="hold-inc"]`);
      expect(await hold(0) === 1.25, `+: ${await hold(0)}`);
      await page.click(`${card(0)} [data-role="hold-dec"]`);
      await page.click(`${card(0)} [data-role="hold-dec"]`);
      expect(await hold(0) === 0.75, `-: ${await hold(0)}`);
      expect(await page.inputValue(`${card(0)} .hold-input`) === '0.75', 'field');
      await page.fill(`${card(1)} .hold-input`, '0.5'); await page.press(`${card(1)} .hold-input`, 'Enter');
      expect(await hold(1) === 0.5, `typed: ${await hold(1)}`);
      await page.fill(`${card(2)} .hold-input`, '0'); await page.press(`${card(2)} .hold-input`, 'Enter');
      expect(await page.getAttribute(`${card(2)} .hold-input`, 'aria-invalid') === 'true' && await hold(2) === 1, 'zero hold accepted');
      expect(await page.evaluate(() => document.getElementById('seq-msg').classList.contains('bad')), 'no error message');
      await page.fill(`${card(2)} .hold-input`, '1'); await page.press(`${card(2)} .hold-input`, 'Enter');
      for (let k = 0; k < 8; k++) await page.click(`${card(1)} [data-role="hold-dec"]`);
      expect(await hold(1) === 0.05, `floor: ${await hold(1)}`);
      await page.fill(`${card(1)} .hold-input`, '0.5'); await page.press(`${card(1)} .hold-input`, 'Enter');
      expect((await page.textContent('#seq-total')).includes('2.25 s'), await page.textContent('#seq-total'));
    });

    await check('sequence: reorder by keyboard with the earlier and later keys, focus follows the step', async () => {
      const order = async () => (await editorSteps()).map((s) => s.left).join(',');
      expect(await order() === 'look_left,look_right,happy', await order());
      expect(await page.isDisabled('#timeline .step:nth-child(1) [data-role="step-up"]') && await page.isDisabled('#timeline .step:nth-child(3) [data-role="step-down"]'), 'end keys enabled');
      await page.focus('#timeline .step:nth-child(3) [data-role="step-up"]');
      await page.keyboard.press('Enter');
      expect(await order() === 'look_left,happy,look_right', await order());
      expect(await page.evaluate(() => document.activeElement.dataset.role === 'step-up' && document.activeElement.closest('.step').dataset.index === '1'), 'focus did not follow');
      await page.keyboard.press('Space');
      expect(await order() === 'happy,look_left,look_right', await order());
      await page.focus('#timeline .step:nth-child(1) [data-role="step-down"]');
      await page.keyboard.press('Enter');
      await page.focus('#timeline .step:nth-child(2) [data-role="step-down"]');
      await page.keyboard.press('Enter');
      expect(await order() === 'look_left,look_right,happy', `back: ${await order()}`);
    });

    await check('sequence: reorder by drag and drop', async () => {
      const order = async () => (await editorSteps()).map((s) => s.left).join(',');
      const box = await page.locator('#timeline .step:nth-child(3)').boundingBox();
      await page.dragAndDrop('#timeline .step:nth-child(1) .step-label', '#timeline .step:nth-child(3)', { targetPosition: { x: box.width - 8, y: 20 } });
      expect(await order() === 'look_right,happy,look_left', `after drag: ${await order()}`);
      await page.dragAndDrop('#timeline .step:nth-child(3) .step-label', '#timeline .step:nth-child(1)', { targetPosition: { x: 8, y: 20 } });
      expect(await order() === 'look_left,look_right,happy', `drag back: ${await order()}`);
      expect(await page.locator('.step.drop-before, .step.drop-after, .step.dragging').count() === 0, 'drag marks left behind');
    });

    await check('sequence: duplicate and delete a step', async () => {
      await page.click('#timeline .step:nth-child(1) [data-role="step-dup"]');
      let steps = await editorSteps();
      expect(steps.length === 4 && JSON.stringify(steps[0]) === JSON.stringify(steps[1]), JSON.stringify(steps.slice(0, 2)));
      await page.click('#timeline .step:nth-child(2) [data-role="step-del"]');
      steps = await editorSteps();
      expect(steps.length === 3 && steps.map((s) => s.left).join() === 'look_left,look_right,happy', JSON.stringify(steps));
      expect(await page.evaluate(() => document.activeElement.dataset.role) === 'step-del', 'focus lost after delete');
    });

    await check('sequence: Play runs on the server, the running step is lit from /api/state and the preview follows; Stop ends it', async () => {
      await page.fill('#seq-name', 'walk-seq');
      await page.fill('#seq-title', 'Walker sequence');
      await page.click('#seq-loop');
      expect(await page.getAttribute('#seq-loop', 'aria-pressed') === 'true', 'loop key');
      const before = lookOf(await api('/api/state'));
      await postsFrom('/api/play', () => page.click('#seq-play'));
      const st = await api('/api/state');
      expect(st.playing && st.playing.name === 'walk-seq' && st.playing.loop === true && st.playing.steps === 3, `server playing ${JSON.stringify(st.playing)}`);
      expect(!('sequence' in st.studio), 'every poll carries the playing sequence');
      const pl = await api('/api/playing');
      expect(pl.sequence && pl.sequence.steps.length === 3 && pl.playing.started === st.playing.started, 'GET /api/playing does not carry the sequence');
      // Sample for one loop and a bit (0.75 + 0.5 + 1 s).
      const samples = [], looks = [];
      for (let k = 0; k < 32; k++) {
        samples.push(await page.evaluate(() => {
          const e = window.eyesStudio, run = document.querySelector('#timeline .step.running');
          const L = e.play.local, p = e.server().playing;
          return { run: run ? Number(run.dataset.index) : null, server: p ? p.step : null, local: L ? L.idx : null };
        }));
        looks.push(await previewVsServer());
        await sleep(60);
      }
      const runs = new Set(samples.map((s) => s.run));
      expect([0, 1, 2].every((i) => runs.has(i)), `running steps seen: ${[...runs]}`);
      expect(samples.every((s) => s.run === s.server), `highlight disagrees with /api/state: ${JSON.stringify(samples.find((s) => s.run !== s.server))}`);
      const agree = samples.filter((s) => s.local === s.server).length;
      expect(agree >= samples.length * 0.75, `preview step matched the server in ${agree} of ${samples.length} samples`);
      const matched = looks.filter(Boolean), off = matched.filter((x) => x.robot.join() !== x.preview.join());
      expect(matched.length >= 16 && off.length <= 1, `preview look differs from /api/state in ${off.length} of ${matched.length}: ${JSON.stringify(off[0])}`);
      await postsFrom('/api/stop', () => page.click('#seq-stop'));
      const after = await api('/api/state');
      expect(after.playing === null, `still playing ${JSON.stringify(after.playing)}`);
      expect(JSON.stringify(lookOf(after)) === JSON.stringify(before), `Stop left ${JSON.stringify(lookOf(after))}, not the look from before Play ${JSON.stringify(before)}`);
      await page.waitForFunction(() => !document.querySelector('#timeline .step.running') && !window.eyesStudio.play.local, null, { timeout: 2000 });
      await page.waitForFunction((b) => { const u = window.eyesStudio.ui; return u.left === b.left && u.color.join() === b.color && u.intensity === b.intensity; }, before, { timeout: 2500 });
      expect(await page.isDisabled('#seq-stop'), 'Stop still enabled');
      expect((await page.textContent('#seq-msg')).includes('back on the look from before Play'), await page.textContent('#seq-msg'));
      for (const id of ['#seq-stop', '#btn-play-stop']) expect(/put back the look from before Play/.test(await page.getAttribute(id, 'title')), `${id} title`);
    });

    await check('sequence: a direct effect click during playback stops it (one writer, no race)', async () => {
      await postsFrom('/api/play', () => page.click('#seq-play'));
      expect((await api('/api/state')).playing, 'not playing');
      await postsFrom('/api/eyes', () => page.click('.fx-btn[data-anim="alert"]'));
      const st = await api('/api/state');
      expect(st.playing === null && st.left === 'alert', `playing ${JSON.stringify(st.playing)}, left ${st.left}`);
      expect(await page.evaluate(() => window.eyesStudio.play.local === null), 'preview still follows the sequence');
      // And the reverse order: a write, then a play, leaves the play running.
      await api('/api/eyes', { intensity: 50 });
      await api('/api/play', { name: 'thinking' });
      const st2 = await api('/api/state');
      expect(st2.playing && st2.playing.name === 'thinking', JSON.stringify(st2.playing));
      await api('/api/stop', {});
      expect((await api('/api/state')).playing === null, 'stop over the API');
    });

    await check('sequence: a loop started elsewhere is fetched once, and its preview matches the robot on later passes', async () => {
      // Step 0 leaves color and intensity out, so from the second pass on it keeps step 1's.
      // Its 1.2 s hold is longer than the 1 s poll, so the page joins during step 0 of the
      // first pass, which still shows the white look from before.
      await api('/api/eyes', { left: 'alert', right: 'alert', color: '#ffffff', intensity: 50 });
      await page.waitForFunction(() => { const u = window.eyesStudio.ui; return u.left === 'alert' && u.color.join() === '255,255,255' && u.intensity === 50; }, null, { timeout: 3000 });
      const from = gets.length;
      await api('/api/play', { sequence: { format: 'stretch-eyes/1', name: 'walk-loopy', loop: true,
        steps: [{ left: 'blink', hold: 1.2 }, { left: 'happy', right: 'happy', color: '#0000ff', intensity: 200, hold: 0.6 }] } });
      await page.waitForFunction(() => window.eyesStudio.play.local && window.eyesStudio.play.local.name === 'walk-loopy', null, { timeout: 3000 });
      await sleep(1300);
      const looks = [];
      for (let k = 0; k < 30; k++) { looks.push(await previewVsServer()); await sleep(60); }
      const matched = looks.filter(Boolean), off = matched.filter((x) => x.robot.join() !== x.preview.join());
      expect(matched.length >= 10 && off.length <= 1, `preview differs from /api/state in ${off.length} of ${matched.length}: ${JSON.stringify(off[0])}`);
      expect(matched.some((x) => x.step === 0), 'never sampled step 0');
      const fetched = gets.slice(from).filter((p) => p === '/api/playing').length;
      expect(fetched === 1, `GET /api/playing ${fetched} times`);
      await api('/api/stop', {});
    });

    await check('sequence: Export downloads the editor JSON', async () => {
      const dl = await download(() => page.click('#seq-export'));
      const d = JSON.parse(dl.text);
      expect(dl.name === 'walk-seq.json', dl.name);
      expect(d.format === 'stretch-eyes/1' && d.name === 'walk-seq' && d.title === 'Walker sequence' && d.loop === true && d.steps.length === 3, dl.text.slice(0, 200));
      expect(d.steps[1].hold === 0.5 && d.steps[0].color === '#00a0ff', JSON.stringify(d.steps));
    });

    if (LIBRARY_WRITES) {
      await check('sequence: Save writes to this robot\'s library (temp folder), asks before overwriting', async () => {
        // Bad name first: flagged, nothing posted.
        await page.fill('#seq-name', 'Bad Name');
        const from = posts.length;
        await page.click('#seq-save');
        await sleep(200);
        expect(await page.getAttribute('#seq-name', 'aria-invalid') === 'true' && posts.slice(from).every((p) => p.path !== '/api/library'), 'bad name posted');
        await page.fill('#seq-name', 'walk-seq');
        await postsFrom('/api/library', () => page.click('#seq-save'));
        expect(fs.existsSync(path.join(USER_DIR, 'walk-seq.json')), `not in ${USER_DIR}`);
        // The saved steps are the editor's, in its order with its holds.
        const keys = ['left', 'right', 'color', 'intensity', 'hold'];
        const pick = (steps) => JSON.stringify(steps.map((x) => keys.map((k) => (k === 'hold' ? Number(x[k]) : x[k]))));
        const savedSteps = (await api('/api/library/walk-seq')).steps;
        expect(pick(savedSteps) === pick(await editorSteps()), `saved ${pick(savedSteps)} vs editor ${pick(await editorSteps())}`);
        expect(savedSteps.map((x) => `${x.left} ${x.hold}`).join() === 'look_left 0.75,look_right 0.5,happy 1', JSON.stringify(savedSteps));
        const lib = await api('/api/library');
        const e = lib.looks.find((l) => l.name === 'walk-seq');
        expect(e && e.source === 'user' && e.steps === 3 && e.loop === true && !('path' in e), JSON.stringify(e));
        // Save again: confirm bar, Cancel keeps the file.
        await page.fill('#seq-title', 'Walker sequence v2');
        await page.click('#seq-save');
        await page.waitForSelector('#seq-confirm:not([hidden])');
        expect(await page.evaluate(() => document.activeElement.id) === 'seq-confirm-no', 'focus not on Cancel');
        expect((await page.textContent('#seq-confirm-text')).includes('walk-seq'), 'confirm text');
        await page.click('#seq-confirm-no');
        await sleep(200);
        expect((await api('/api/library/walk-seq')).title === 'Walker sequence', 'cancel overwrote');
        await page.click('#seq-save');
        await page.waitForSelector('#seq-confirm:not([hidden])');
        await postsFrom('/api/library', () => page.click('#seq-confirm-yes'));
        expect((await api('/api/library/walk-seq')).title === 'Walker sequence v2', 'overwrite did not land');
        expect((await page.textContent('#seq-msg')).includes('Saved walk-seq'), await page.textContent('#seq-msg'));
      });

      await check('sequence: Import reads a file, posts it to the library and loads it; overwrite asks; bad JSON is reported', async () => {
        const look = { format: 'stretch-eyes/1', name: 'walk-imported', title: 'Imported by the walker', loop: false,
          steps: [{ left: 'top_half', right: 'bottom_half', color: '#ff1493', intensity: 0.5, hold: 0.4 }, { left: 'blink', hold: 0.6 }] };
        await page.setInputFiles('#seq-file', { name: 'walk-imported.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify(look)) });
        await page.waitForFunction(() => window.eyesStudio.editor.name === 'walk-imported', null, { timeout: 3000 });
        expect(fs.existsSync(path.join(USER_DIR, 'walk-imported.json')), 'not saved to the user folder');
        const saved = await api('/api/library/walk-imported');
        expect(saved.steps[0].intensity === 128 && saved.steps[1].color === undefined, `canonical form: ${JSON.stringify(saved.steps)}`);
        expect((await editorSteps()).length === 2, 'editor not loaded');
        await page.setInputFiles('#seq-file', { name: 'walk-imported.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify({ ...look, title: 'Second import' })) });
        await page.waitForSelector('#seq-confirm:not([hidden])');
        await postsFrom('/api/library', () => page.click('#seq-confirm-yes'));
        await page.waitForFunction(() => window.eyesStudio.editor.title === 'Second import', null, { timeout: 3000 });
        // Intensity 1.0 is full. The server parses the file text, so it is not read as raw 1.
        const fullText = '{"format": "stretch-eyes/1", "name": "walk-full", "steps": [{"left": "blink", "intensity": 1.0, "hold": 0.5}]}';
        const sent = await postsFrom('/api/library', () => page.setInputFiles('#seq-file', { name: 'walk-full.json', mimeType: 'application/json', buffer: Buffer.from(fullText) }));
        expect(sent.length === 1 && sent[0].text === fullText && !('sequence' in sent[0]), JSON.stringify(sent));
        await page.waitForFunction(() => window.eyesStudio.editor.name === 'walk-full', null, { timeout: 3000 });
        expect((await api('/api/library/walk-full')).steps[0].intensity === 255, 'intensity 1.0 not saved as 255');
        expect((await editorSteps())[0].intensity === 255, 'editor shows the raw byte, not 255');
        await page.setInputFiles('#seq-file', { name: 'broken.json', mimeType: 'application/json', buffer: Buffer.from('{"format": ') });
        await page.waitForFunction(() => document.getElementById('seq-msg').classList.contains('bad'), null, { timeout: 2000 });
        expect((await page.textContent('#seq-msg')).includes('not JSON'), await page.textContent('#seq-msg'));
        await page.setInputFiles('#seq-file', { name: 'bad-hold.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify({ ...look, name: 'walk-bad', steps: [{ left: 'blink', hold: 0 }] })) });
        await page.waitForFunction(() => /steps\[0\]\.hold/.test(document.getElementById('seq-msg').textContent), null, { timeout: 2000 });
        expect(!fs.existsSync(path.join(USER_DIR, 'walk-bad.json')), 'invalid look saved');
      });
    }
    await closeSeq();

    // -----------------------------------------------------------------------
    // Looks library.
    // -----------------------------------------------------------------------
    const BUILTINS = ['attention', 'glance', 'happy', 'idle', 'off', 'sleepy', 'thinking'];
    const row = (name) => `#library .lib-row[data-name="${name}"]`;
    await check('library: built-ins, the shared look and this robot\'s looks listed with source, steps, duration and an animated preview', async () => {
      const from = gets.length;
      await openPop('library');
      await page.waitForSelector(row('glance'));
      await sleep(300);
      const libGets = gets.slice(from).filter((p) => p.startsWith('/api/library'));
      expect(libGets.join() === '/api/library', `opening the plate fetched ${libGets.join(', ')}`);
      expect(await page.evaluate(() => !!window.eyesStudio.library.seqs.glance), 'no sequences in GET /api/library');
      for (const name of BUILTINS) {
        expect(await page.locator(`${row(name)} .badge-builtin`).count() === 1, `${name} missing or not built-in`);
        expect(await page.locator(`${row(name)} [data-role="lib-del"]`).count() === 0, `${name} offers Delete`);
      }
      expect((await page.textContent(`${row('glance')} .lib-meta`)).includes('3 steps, 2.5 s'), await page.textContent(`${row('glance')} .lib-meta`));
      expect((await page.textContent(`${row('thinking')} .lib-meta`)).includes('loops'), 'loop not shown');
      expect(await page.locator(`${row('walk-shared')} .badge-shared`).count() === 1, 'shared look missing');
      if (LIBRARY_WRITES) {
        expect(await page.locator(`${row('walk-seq')} .badge-user`).count() === 1, 'saved look missing');
        expect(await page.locator(`${row('walk-seq')} [data-role="lib-del"]`).count() === 1, 'no Delete on a user look');
      }
      const frame = () => page.evaluate((sel) => {
        const c = document.querySelector(sel); const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
        let h = 0; for (let k = 0; k < d.length; k += 16) h = (h * 31 + d[k] + 2 * d[k + 1] + 3 * d[k + 2]) | 0; return h;
      }, `${row('glance')} canvas`);
      const a = await frame(); await sleep(1100); const b = await frame();
      expect(a !== b, 'glance preview did not move');
    });

    await check('library: Play plays the look on the server and lights its row; the window badge stops it', async () => {
      await postsFrom('/api/play', () => page.click(`${row('glance')} [data-role="lib-play"]`));
      const st = await api('/api/state');
      expect(st.playing && st.playing.name === 'glance', JSON.stringify(st.playing));
      await page.waitForFunction((sel) => document.querySelector(sel).classList.contains('running'), row('glance'), { timeout: 2000 });
      await closePops();
      await page.waitForSelector('#play-badge:not([hidden])');
      expect((await page.textContent('#play-text')).startsWith('glance, step'), await page.textContent('#play-text'));
      expect(await page.evaluate(() => window.eyesStudio.play.local && window.eyesStudio.play.local.name === 'glance'), 'preview not following');
      await postsFrom('/api/stop', () => page.click('#btn-play-stop'));
      expect((await api('/api/state')).playing === null, 'not stopped');
      await page.waitForSelector('#play-badge', { state: 'hidden' });
    });

    await check('library: Export downloads GET /api/library/<name>', async () => {
      await openPop('library');
      const dl = await download(() => page.click(`${row('glance')} [data-role="lib-export"]`));
      expect(dl.name === 'glance.json', dl.name);
      expect(JSON.stringify(JSON.parse(dl.text)) === JSON.stringify(await api('/api/library/glance')), 'download differs from the API');
    });

    await check('library: Load puts a look into the editor, partial steps kept as they are', async () => {
      await page.click(`${row('sleepy')} [data-role="lib-load"]`);
      await page.waitForSelector('#seq:not([hidden])');
      expect(await page.isHidden('#pop-library'), 'library still open');
      const ed = await page.evaluate(() => ({ name: window.eyesStudio.editor.name, steps: window.eyesStudio.editor.steps.map((s) => s.data) }));
      expect(ed.name === 'sleepy' && ed.steps.length === 3 && ed.steps[1].left === undefined && ed.steps[1].intensity === 25, JSON.stringify(ed));
      expect(await page.inputValue('#seq-name') === 'sleepy', 'name field');
      await closeSeq();
    });

    if (LIBRARY_WRITES) {
      await check('library: Duplicate copies a built-in into this robot\'s library', async () => {
        await openPop('library');
        await postsFrom('/api/library', () => page.click(`${row('glance')} [data-role="lib-dup"]`));
        await page.waitForSelector(`${row('glance-copy')} .badge-user`);
        expect(fs.existsSync(path.join(USER_DIR, 'glance-copy.json')), 'copy not on disk');
        const d = await api('/api/library/glance-copy');
        expect(d.title === 'Glance left and right (copy)' && d.steps.length === 3, JSON.stringify(d).slice(0, 120));
        await page.waitForFunction(() => document.activeElement && document.activeElement.dataset.role === 'lib-load' && document.activeElement.closest('.lib-row').dataset.name === 'glance-copy', null, { timeout: 2000 });
      });

      await check('library: a delete question is dropped when the plate closes and does not take focus on reopen', async () => {
        await page.click(`${row('glance-copy')} [data-role="lib-del"]`);
        await page.waitForSelector(`${row('glance-copy')} .lib-confirm`);
        expect(await page.evaluate(() => document.activeElement.dataset.role) === 'lib-del-no', 'focus not on Cancel');
        await page.keyboard.press('Escape');
        await page.waitForSelector('#pop-library', { state: 'hidden' });
        await openPop('library');
        await sleep(400);
        expect(await page.locator('#library .lib-confirm').count() === 0, 'the question came back');
        const role = await page.evaluate(() => document.activeElement.dataset.role || document.activeElement.id);
        expect(role !== 'lib-del-no' && await page.evaluate(() => document.getElementById('pop-library').contains(document.activeElement)), `focus on ${role}`);
        expect(fs.existsSync(path.join(USER_DIR, 'glance-copy.json')), 'Escape deleted');
      });

      await check('library: text in lamp colours (this robot badge, Delete, error lines) is 4.5:1 or better in both themes', async () => {
        const ratios = () => page.evaluate(() => {
          const rgb = (c) => c.match(/[\d.]+/g).slice(0, 3).map(Number);
          const lum = (c) => { const f = (x) => { x /= 255; return x <= 0.03928 ? x / 12.92 : ((x + 0.055) / 1.055) ** 2.4; }; return 0.2126 * f(c[0]) + 0.7152 * f(c[1]) + 0.0722 * f(c[2]); };
          const ratio = (a, b) => { const [x, y] = [lum(a), lum(b)].sort((p, q) => q - p); return (x + 0.05) / (y + 0.05); };
          const ink = (el) => rgb(getComputedStyle(el).color);
          const probe = document.createElement('span'); document.body.append(probe);
          const tok = (n) => { probe.style.color = getComputedStyle(document.documentElement).getPropertyValue(n).trim(); return ink(probe); };
          const msg = document.getElementById('library-msg'), had = msg.classList.contains('bad');
          msg.classList.add('bad'); const bad = ink(msg); msg.classList.toggle('bad', had);
          const out = {
            badge: ratio(ink(document.querySelector('.badge-user')), tok('--plate')),
            del: ratio(ink(document.querySelector('.key-danger')), tok('--key')),
            badOnPlate: ratio(bad, tok('--plate')), badOnRecess: ratio(bad, tok('--recess')),
          };
          probe.remove();
          return out;
        });
        const low = [];
        for (const scheme of ['light', 'dark']) {
          await page.emulateMedia({ colorScheme: scheme }); await sleep(150);
          for (const [k, v] of Object.entries(await ratios())) if (v < 4.5) low.push(`${scheme} ${k} ${v.toFixed(2)}`);
        }
        await page.emulateMedia({ colorScheme: null });
        expect(low.length === 0, low.join(', '));
      });

      await check('library: Delete asks first, Cancel keeps the look, Delete removes it from the list and the folder', async () => {
        await page.click(`${row('glance-copy')} [data-role="lib-del"]`);
        await page.waitForSelector(`${row('glance-copy')} .lib-confirm`);
        await page.click(`${row('glance-copy')} [data-role="lib-del-no"]`);
        expect(await page.locator(row('glance-copy')).count() === 1 && fs.existsSync(path.join(USER_DIR, 'glance-copy.json')), 'cancel deleted');
        await page.click(`${row('glance-copy')} [data-role="lib-del"]`);
        await postsFrom('/api/library/delete', () => page.click(`${row('glance-copy')} [data-role="lib-del-yes"]`));
        await page.waitForSelector(row('glance-copy'), { state: 'detached' });
        expect(!fs.existsSync(path.join(USER_DIR, 'glance-copy.json')), 'file still there');
        expect(await page.evaluate(() => document.getElementById('pop-library').contains(document.activeElement)), 'focus left the plate');
        await closePops();
      });
    }
    await closePops();

    await check('server: library errors are 4xx with the library\'s message; POSTs keep the JSON and same-origin guard', async () => {
      let r = await apiRes('/api/library/nope');
      expect(r.status === 404 && /No look named/.test(r.body.error), `GET unknown: ${r.status} ${r.body.error}`);
      r = await apiRes('/api/library/delete', { name: 'glance' });
      expect(r.status === 403 && /read-only|builtin|built-in/i.test(r.body.error), `delete built-in: ${r.status} ${r.body.error}`);
      r = await apiRes('/api/library', { sequence: { format: 'stretch-eyes/1', name: 'walk-bad', steps: [{ left: 'blink', hold: 0 }] } });
      expect(r.status === 400 && /steps\[0\]\.hold/.test(r.body.error), `invalid: ${r.status} ${r.body.error}`);
      r = await apiRes('/api/library', { sequence: { format: 'stretch-eyes/1', name: 'walk-bad', steps: [{ left: 'blink', hold: 1 }] }, extra: 1 });
      expect(r.status === 400, `unknown body key: ${r.status}`);
      if (LIBRARY_WRITES) {
        r = await apiRes('/api/library', { sequence: await api('/api/library/walk-seq') });
        expect(r.status === 409 && /exists/.test(r.body.error), `duplicate: ${r.status} ${r.body.error}`);
      }
      r = await apiRes('/api/play', { name: 'nope' });
      expect(r.status === 404, `play unknown: ${r.status}`);
      r = await apiRes('/api/play', { name: 'glance', sequence: {} });
      expect(r.status === 400, `play both: ${r.status}`);
      r = await apiRes('/api/play', { name: 'glance', loop: 'yes' });
      expect(r.status === 400, `play loop type: ${r.status}`);
      r = await apiRes('/api/play', '{"name":"glance"}', { 'Content-Type': 'text/plain' });
      expect(r.status === 415, `text/plain: ${r.status}`);
      r = await apiRes('/api/library/delete', { name: 'walk-seq' }, { Origin: 'http://evil.example' });
      expect(r.status === 403 && /cross-origin/.test(r.body.error), `cross-origin: ${r.status}`);
      if (LIBRARY_WRITES) expect(fs.existsSync(path.join(USER_DIR, 'walk-seq.json')), 'cross-origin delete went through');
      expect((await api('/api/state')).playing === null, 'something is playing');
    });


    if (SHOTS) {
      const shot = (name, opts = {}) => page.screenshot({ path: path.join(SHOTS, `${name}.png`), ...opts });
      const scheme = async (s) => { await page.emulateMedia({ colorScheme: s }); await sleep(250); };
      await page.click('.fx-btn[data-anim="circle_cw"]');
      await page.fill('#hex', '#00a0ff'); await page.press('#hex', 'Enter');
      await page.fill('#intensity', '255');
      await sleep(700);
      for (const [w, h, s] of [[1280, 720, 'light'], [1280, 720, 'dark'], [1440, 900, 'light'], [1920, 1080, 'dark'], [1920, 1080, 'light']]) {
        await page.setViewportSize({ width: w, height: h }); await scheme(s);
        await shot(`studio-${w}x${h}-${s}`);
      }
      // The sequence editor while a look plays, at the smallest desktop size.
      await page.setViewportSize({ width: 1280, height: 720 });
      for (const s of ['light', 'dark']) {
        await scheme(s);
        await openPop('library');
        await page.click(`${row('glance')} [data-role="lib-load"]`);
        await page.click('#seq-loop');
        await postsFrom('/api/play', () => page.click('#seq-play'));
        await sleep(1300);
        await shot(`studio-1280x720-${s}-sequence`);
        await postsFrom('/api/stop', () => page.click('#seq-stop'));
        await closeSeq();
      }
      await scheme('light');
      await openPop('library'); await sleep(900);
      await shot('studio-1280x720-light-library');
      await closePops();
      await openPop('details'); await shot('studio-1280x720-light-details'); await closePops();
      await openPop('swatches'); await shot('studio-1280x720-light-swatches'); await closePops();
      await scheme('dark');
      await openPop('looks'); await shot('studio-1280x720-dark-looks'); await closePops();
      await page.setViewportSize({ width: 1440, height: 900 });
      await scheme('light');
      await page.click('.led-spot[data-eye="1"][data-i="7"]');
      await page.hover('.led-spot[data-eye="1"][data-i="7"]');
      await sleep(150);
      await page.locator('.window').screenshot({ path: path.join(SHOTS, 'studio-hover.png') });
      await page.keyboard.press('Escape');
      await page.mouse.move(2, 2);
      await openPop('overrides');
      await page.click('.link label:has(input[name="ovr"][value="soc12"])');
      await closePops();
      await sleep(600);
      await page.locator('.window').screenshot({ path: path.join(SHOTS, 'studio-override-soc12.png') });
      await openPop('overrides');
      await page.click('.link label:has(input[name="ovr"][value="runstop"])');
      await sleep(400);
      // Catch the blink in its lit phase (the runstop LED toggles every 500 ms).
      await page.waitForFunction(() => window.eyesStudio.sim.runstop && window.eyesStudio.sim.runstopLed, null, { timeout: 2000 });
      await sleep(60);
      await shot('studio-1440x900-light-overrides-runstop');
      await page.click('.link label:has(input[name="ovr"][value="live"])');
      await closePops();
      await sleep(400);
    }

    for (const w of [360, 430]) {
      await check(`phone width ${w}: no horizontal scroll, the page may scroll, plates are bottom sheets`, async () => {
        await page.setViewportSize({ width: w, height: 800 });
        await sleep(400);
        const m = await page.evaluate(() => ({ sw: document.documentElement.scrollWidth, sh: document.documentElement.scrollHeight }));
        expect(m.sw <= w, `scrollWidth ${m.sw}`);
        await openPop('library');
        const r = await page.evaluate(() => { const b = document.getElementById('pop-library').getBoundingClientRect(); return [b.left, b.right, b.bottom, innerWidth, innerHeight].map(Math.round); });
        expect(r[0] === 0 && r[1] === r[3] && r[2] === r[4], `sheet at ${r}`);
        expect((await page.evaluate(() => document.documentElement.scrollWidth)) <= w, 'library sheet scrolls sideways');
        await closePops();
        if (SHOTS && w === 360) {
          await page.emulateMedia({ colorScheme: 'light' }); await sleep(200);
          await page.screenshot({ path: path.join(SHOTS, 'studio-phone-light.png'), fullPage: true });
          await page.emulateMedia({ colorScheme: 'dark' }); await sleep(200);
          await page.screenshot({ path: path.join(SHOTS, 'studio-phone-dark.png'), fullPage: true });
          await page.emulateMedia({ colorScheme: 'light' });
        }
      });
    }

    await check('phone: the sequence editor sits in the flow and builds and plays a step', async () => {
      await page.setViewportSize({ width: 390, height: 844 });
      await sleep(300);
      await openSeq();
      await page.click('#seq-new');
      expect(await page.evaluate(() => getComputedStyle(document.getElementById('seq')).position) === 'static', 'drawer not in the flow on a phone');
      await page.click('.fx-btn[data-anim="happy"]');
      await page.click('#seq-add');
      await page.click('.fx-btn[data-anim="blink"]');
      await page.click('#seq-add');
      expect(await page.locator('#timeline .step').count() === 2, 'steps');
      expect((await page.evaluate(() => document.documentElement.scrollWidth)) <= 390, 'drawer scrolls sideways');
      await postsFrom('/api/play', () => page.click('#seq-play'));
      expect((await api('/api/state')).playing, 'not playing');
      if (SHOTS) {
        await page.emulateMedia({ colorScheme: 'dark' }); await sleep(600);
        await page.screenshot({ path: path.join(SHOTS, 'studio-phone-dark-sequence.png'), fullPage: true });
        await page.emulateMedia({ colorScheme: 'light' });
      }
      await postsFrom('/api/stop', () => page.click('#seq-stop'));
      await closeSeq();
    });

    await check('phone width keeps the controls at 44 px', async () => {
      await page.setViewportSize({ width: 360, height: 780 });
      await sleep(300);
      await openSeq();
      await page.click('#seq-add');
      const small = await page.evaluate(() => {
        const out = [];
        for (const el of document.querySelectorAll('.key, .fx-btn, .swatch, .link label, .field input, .hold input, #intensity')) {
          if (!el.offsetParent) continue;
          const r = el.getBoundingClientRect();
          if (r.height < 43.5) out.push(`${el.id || el.className} ${Math.round(r.height)}`);
        }
        return out;
      });
      await closeSeq();
      expect(small.length === 0, small.join(', '));
    });

    await check('44 px targets inside every plate and the drawer (desktop)', async () => {
      await page.setViewportSize({ width: 1280, height: 720 });
      await sleep(300);
      const small = [];
      const measure = () => page.evaluate(() => {
        const out = [];
        for (const el of document.querySelectorAll('.pop:not([hidden]) .key, .pop:not([hidden]) .swatch, .pop:not([hidden]) .link label, #seq:not([hidden]) .key, #seq:not([hidden]) input:not([type="file"])')) {
          if (!el.offsetParent) continue;
          const r = el.getBoundingClientRect();
          if (r.height < 43.5 || r.width < 43.5) out.push(`${el.id || el.dataset.role || el.className} ${Math.round(r.width)}x${Math.round(r.height)}`);
        }
        return out;
      });
      for (const name of POPS) { await openPop(name); await sleep(name === 'library' ? 400 : 50); small.push(...await measure()); await closePops(); }
      await openSeq(); small.push(...await measure()); await closeSeq();
      expect(small.length === 0, small.join(', '));
    });

    await check('no em or en dashes after the walk (library, editor, details)', async () => {
      await openPop('library'); await sleep(300); await closePops();
      const bad = await page.evaluate(() => {
        const texts = [document.title, document.body.textContent];
        for (const el of document.querySelectorAll('[aria-label], [title]')) texts.push(el.getAttribute('aria-label') || '', el.getAttribute('title') || '');
        return texts.filter((t) => /[\u2013\u2014]/.test(t)).map((t) => t.slice(0, 80));
      });
      expect(bad.length === 0, bad.join(' | '));
    });

    await check('no console errors after the walk', async () => { expect(errors.length === 0, errors.join(' | ')); });
  } finally {
    await browser.close();
    if (proc) proc.kill();
  }
  const failed = results.filter(([ok]) => !ok).length;
  console.log(`\n${results.length - failed} passed, ${failed} failed`);
  process.exit(failed ? 1 : 0);
})().catch((err) => { console.error(err); process.exit(2); });
