// Headless walk of every Eyes Studio control against the fake backend.
//
//   NODE_PATH=<dir with playwright in node_modules> PYTHONPATH=<repo> \
//     node walk_controls.cjs [screenshot_dir]
//
// Starts `python3 -m stretch4_body.eyes.studio --fake` on a free port (or uses
// EYES_STUDIO_URL if set), drives the page in Chromium and checks both the
// POSTs the page makes and the state the server ends up in. With a screenshot
// directory it also writes desktop and 360 px phone shots in light and dark,
// the two firmware override previews and a pixel hover.
'use strict';
const { chromium } = require('playwright');
const { spawn } = require('child_process');
const net = require('net');
const path = require('path');

const REPO = path.resolve(__dirname, '../../../..');
const SHOTS = process.argv[2] || null;
const results = [];

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

async function startServer() {
  if (process.env.EYES_STUDIO_URL) return { url: process.env.EYES_STUDIO_URL, proc: null };
  const port = await freePort();
  const env = { ...process.env, PYTHONPATH: process.env.PYTHONPATH || REPO };
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
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  // Record every string drawn on a canvas, to prove the board layer carries no designators.
  await page.addInitScript(() => {
    const orig = CanvasRenderingContext2D.prototype.fillText;
    window.__drawnText = [];
    CanvasRenderingContext2D.prototype.fillText = function (t, ...rest) { window.__drawnText.push(String(t)); return orig.call(this, t, ...rest); };
  });
  const errors = [];
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  page.on('pageerror', (e) => errors.push(String(e)));
  const posts = [];
  page.on('request', (r) => { if (r.method() === 'POST') posts.push({ path: new URL(r.url()).pathname, body: JSON.parse(r.postData() || '{}') }); });

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
      await page.click('#swatch-save');
      expect(await page.locator('.swatch.saved[data-hex="#336699"]').count() === 1, 'not saved');
      await page.click('.swatch.factory[data-hex="#ffffff"]');
      await sleep(250);
      expect((await api('/api/state')).color_hex === '#ffffff', 'factory swatch');
      await page.reload();
      await ready();
      const body = merged(await postsFrom('/api/eyes', () => page.click('.swatch.saved[data-hex="#336699"]')));
      expect(body.color === '#336699', JSON.stringify(body));
      await page.focus('.swatch.saved[data-hex="#336699"]');
      await page.keyboard.press('Delete');
      expect(await page.locator('.swatch.saved').count() === 0, 'not deleted');
    });
    await check('swatch clear removes every saved swatch and disables itself', async () => {
      for (const hex of ['#aa5500', '#0055aa']) {
        await page.fill('#hex', hex); await page.press('#hex', 'Enter');
        await sleep(200);
        await page.click('#swatch-save');
      }
      expect(await page.locator('.swatch.saved').count() === 2, 'two swatches saved');
      expect(!(await page.isDisabled('#swatch-clear')), 'clear key disabled with swatches saved');
      await page.click('#swatch-clear');
      expect(await page.locator('.swatch.saved').count() === 0, 'swatches remain');
      expect(await page.evaluate(() => localStorage.getItem('eyes-studio.swatches')) === '[]', 'localStorage not emptied');
      expect(await page.isDisabled('#swatch-clear'), 'clear key still enabled with nothing saved');
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
      const n = await page.locator('#presets .key').count();
      expect(n >= 6, `${n} presets`);
      const body = merged(await postsFrom('/api/eyes', () => page.click('#presets .key[data-preset="Scanning"]')));
      expect(body.left === 'circle_cw' && body.right === 'circle_ccw' && body.color === '#00a0ff' && body.intensity === 255, JSON.stringify(body));
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
      const body = merged(await postsFrom('/api/eyes', () => page.click('#presets .key[data-preset="Attention"]')));
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
      await page.click('.link label:has(input[name="ovr"][value="live"])');
      await sleep(1200);
      expect(!(await page.isVisible('#override-banner')), 'banner still visible on live');
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
      expect((await page.textContent('#orientation-caption')).includes('assumed'), 'caption does not say assumed');
      await page.click('#btn-swap');
      await sleep(200);
      expect(await page.getAttribute('#btn-swap', 'aria-pressed') === 'true', 'aria-pressed');
      expect(await page.evaluate(() => window.eyesStudio.sideOf(0)) === 1, 'sideOf');
      expect(await spotX(0, 0) > mid, 'left eye hotspots did not move to the right');
      expect((await page.textContent('#orientation-caption')).includes('Swapped'), 'caption');
      expect(!(await page.getAttribute('.led-spot[data-eye="0"][data-i="2"]', 'aria-label')).includes('LED'), 'board LED number still claimed while swapped');
      await page.reload(); await ready();
      expect(await page.getAttribute('#btn-swap', 'aria-pressed') === 'true', 'swap not persisted');
      expect(await page.evaluate(() => localStorage.getItem('eyes-studio.swap')) === '1', 'localStorage');
      expect(await spotX(0, 0) > mid, 'swap not applied after reload');
      await page.click('#btn-swap');
      await sleep(200);
      expect(await page.evaluate(() => window.eyesStudio.sideOf(0)) === 0 && await spotX(0, 0) < mid, 'unswap');
    });

    await check('every control is at least 44 px tall', async () => {
      const small = await page.evaluate(() => {
        const sel = '.key, .fx-btn, .swatch, .link label, .field input, #intensity, .check';
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
      const ids = ['btn-control', 'btn-idle', 'btn-off', 'btn-swap', 'led-spot', 'hue-ring', 'sv-square', 'hex',
        'rgb-r', 'rgb-g', 'rgb-b', 'intensity', 'swatch-save', 'swatch-clear'];
      await page.click('#swatch-save');   // the clear key is only enabled with a saved swatch
      await page.focus('body');
      const seen = new Set();
      for (let k = 0; k < 140; k++) {
        await page.keyboard.press('Tab');
        seen.add(await page.evaluate(() => document.activeElement.id || document.activeElement.className));
      }
      await page.click('#swatch-clear');
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

    if (SHOTS) {
      await page.click('.fx-btn[data-anim="circle_cw"]');
      await page.fill('#hex', '#00a0ff'); await page.press('#hex', 'Enter');
      await sleep(700);
      await page.screenshot({ path: path.join(SHOTS, 'studio-desktop.png'), fullPage: true });
      await page.emulateMedia({ colorScheme: 'dark' });
      await sleep(200);
      await page.screenshot({ path: path.join(SHOTS, 'studio-desktop-dark.png'), fullPage: true });
      await page.emulateMedia({ colorScheme: 'light' });
      await page.click('.led-spot[data-eye="1"][data-i="7"]');
      await page.hover('.led-spot[data-eye="1"][data-i="7"]');
      await sleep(150);
      await page.locator('.stage').screenshot({ path: path.join(SHOTS, 'studio-hover.png') });
      await page.keyboard.press('Escape');
      await page.mouse.move(2, 2);
      await page.click('.link label:has(input[name="ovr"][value="soc12"])');
      await sleep(600);
      await page.locator('.stage').screenshot({ path: path.join(SHOTS, 'studio-override-soc12.png') });
      await page.click('.link label:has(input[name="ovr"][value="runstop"])');
      await sleep(400);
      // Catch the blink in its lit phase (the runstop LED toggles every 500 ms).
      await page.waitForFunction(() => window.eyesStudio.sim.runstop && window.eyesStudio.sim.runstopLed, null, { timeout: 2000 });
      await sleep(60);
      await page.locator('.stage').screenshot({ path: path.join(SHOTS, 'studio-override-runstop.png') });
      await page.click('.link label:has(input[name="ovr"][value="live"])');
      await sleep(400);
    }

    await check('phone width 360 has no horizontal scroll', async () => {
      await page.setViewportSize({ width: 360, height: 780 });
      await sleep(400);
      const w = await page.evaluate(() => document.documentElement.scrollWidth);
      expect(w <= 360, `scrollWidth ${w}`);
      if (SHOTS) {
        await page.screenshot({ path: path.join(SHOTS, 'studio-phone.png'), fullPage: true });
        await page.emulateMedia({ colorScheme: 'dark' });
        await sleep(200);
        await page.screenshot({ path: path.join(SHOTS, 'studio-phone-dark.png'), fullPage: true });
        await page.emulateMedia({ colorScheme: 'light' });
      }
    });

    await check('phone width keeps the controls at 44 px', async () => {
      const small = await page.evaluate(() => {
        const out = [];
        for (const el of document.querySelectorAll('.key, .fx-btn, .swatch, .link label, .field input, #intensity')) {
          if (!el.offsetParent) continue;
          const r = el.getBoundingClientRect();
          if (r.height < 43.5) out.push(`${el.id || el.className} ${Math.round(r.height)}`);
        }
        return out;
      });
      expect(small.length === 0, small.join(', '));
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
