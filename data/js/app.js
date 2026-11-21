"use strict";

// ======================================================================
//  Panel robota: mapa w prawdziwej skali (cm), sterowanie "jedź dopóki trzymam"
//  Układ mapy: 1 komórka = `cell` cm, (0,0) = punkt startowy, Y rośnie "do góry".
//  Kąt robota h: 0 = oś X, rośnie przeciwnie do zegara; start = 90 (do góry).
// ======================================================================
const POLL_MS_IDLE = 1000;
const POLL_MS_ACTIVE = 450;
const POLL_TIMEOUT_MS = 8000;
const KEEPALIVE_MS = 200;
const MAP_RANGE_CM = 120;         // zasięg czujników - pierścień podczas skanu

const $ = (id) => document.getElementById(id);
const cssVar = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

// ---------------------------------------------------------------- stan
let S = null;                      // ostatni /api/state
let mapVer = -1, scanVer = -1;
let grid = { n: 0, cell: 5, cls: null };
let scanPts = [];
let lastOk = 0;
let polling = false;

const view = { cx: 0, cy: 0, s: 1 };   // środek widoku [cm], skala [px/cm]
let follow = false;
let userMoved = false;
let dirty = true;

const mapCanvas = $('map');
const ctx = mapCanvas.getContext('2d');
const off = document.createElement('canvas');   // 1 piksel = 1 komórka mapy
const offCtx = off.getContext('2d');
let W = 0, H = 0, DPR = 1;

// ---------------------------------------------------------------- narzędzia
function toast(msg, isErr) {
  const t = $('toast');
  t.textContent = msg;
  t.classList.toggle('err', !!isErr);
  t.classList.add('show');
  clearTimeout(toast._t);
  toast._t = setTimeout(() => t.classList.remove('show'), 3200);
}

async function getJson(url, ms) {
  const c = new AbortController();
  const timer = setTimeout(() => c.abort(), ms || POLL_TIMEOUT_MS);
  try {
    const r = await fetch(url + (url.includes('?') ? '&' : '?') + 't=' + Date.now(), { cache: 'no-store', signal: c.signal });
    if (!r.ok) throw new Error('HTTP ' + r.status);
    return await r.json();
  } finally { clearTimeout(timer); }
}

async function api(cmd, extra) {
  try {
    const r = await getJson('api/cmd?c=' + cmd + (extra || ''), 6000);
    if (!r.ok) toast(r.err || 'Nie udało się', true);
    pollNow();
    return r.ok === 1;
  } catch (e) {
    toast('Brak połączenia z robotem', true);
    return false;
  }
}

// ---------------------------------------------------------------- mapa: pobieranie
function decodeMap(buf, n) {
  const a = new Uint8Array(buf);
  const out = new Uint8Array(n * n);
  let p = 0;
  for (let i = 0; i + 1 < a.length && p < out.length; i += 2) {
    const run = a[i], v = a[i + 1];
    if (v) out.fill(v, p, Math.min(p + run, out.length));
    p += run;
  }
  return out;
}

function renderGridImage() {
  const n = grid.n;
  if (!n || !grid.cls) return;
  off.width = n; off.height = n;
  const img = offCtx.createImageData(n, n);
  const hex = (c) => { const v = parseInt(c.replace('#', ''), 16); return [v >> 16 & 255, v >> 8 & 255, v & 255]; };
  const free = hex(cssVar('--map-free')), wall = hex(cssVar('--map-wall'));
  for (let cy = 0; cy < n; cy++) {
    const row = n - 1 - cy;                  // wiersz 0 obrazu = największe Y
    for (let cx = 0; cx < n; cx++) {
      const c = grid.cls[cy * n + cx];
      if (!c) continue;
      const k = (row * n + cx) * 4, col = c === 2 ? wall : free;
      img.data[k] = col[0]; img.data[k + 1] = col[1]; img.data[k + 2] = col[2]; img.data[k + 3] = 255;
    }
  }
  offCtx.putImageData(img, 0, 0);
}

async function fetchMap() {
  const r = await fetch('api/map?t=' + Date.now(), { cache: 'no-store' });
  if (!r.ok) throw new Error('map ' + r.status);
  const buf = await r.arrayBuffer();
  grid.n = S.map.cells; grid.cell = S.map.cell;
  grid.cls = decodeMap(buf, grid.n);
  renderGridImage();
  updateDims();
  dirty = true;
}

async function fetchScan() {
  const r = await getJson('api/scan');
  scanPts = [];
  for (let i = 0; i + 1 < r.pts.length; i += 2) scanPts.push([r.pts[i], r.pts[i + 1]]);
  dirty = true;
}

// ---------------------------------------------------------------- odpytywanie stanu
async function poll() {
  if (polling) return;
  polling = true;
  try {
    S = await getJson('api/state');
    lastOk = Date.now();
    applyState();
    if (S.map.ver !== mapVer) { await fetchMap(); mapVer = S.map.ver; }
    if (S.scan.ov !== scanVer) { await fetchScan(); scanVer = S.scan.ov; }
  } catch (e) {
    // brak odpowiedzi - wskaźnik połączenia pokaże problem
  } finally {
    polling = false;
    updateConn();
  }
}
function pollNow() { poll(); }
(function loop() {
  const active = S && (S.scan.on || S.dir !== 'Stop' || S.auto);
  setTimeout(() => { poll().finally(loop); }, active ? POLL_MS_ACTIVE : POLL_MS_IDLE);
})();

function updateBt() {
  // bt: 1 = klawiatura podłączona, 0 = niepodłączona, -1 = nieznany (biblioteka nie podaje stanu, nie było jeszcze klawisza)
  const ok = Date.now() - lastOk < 4000;
  const v = (ok && S && S.bt !== undefined) ? S.bt : -2;
  const el = $('bt');
  el.className = 'conn' + (v === 1 ? ' ok' : (v === 0 ? ' bad' : ''));
  $('btText').textContent = 'Klawiatura Bluetooth' + (v === 1 ? ': podłączona' : v === 0 ? ': niepodłączona' : '');
  el.title = v === 1 ? 'Klawiatura Bluetooth jest podłączona (strzałki sterują robotem, spacja = stop)'
    : v === 0 ? 'Klawiatura Bluetooth nie jest podłączona'
    : 'Stan klawiatury Bluetooth nieznany (naciśnij dowolny klawisz na klawiaturze)';
}

function updateConn() {
  updateBt();
  const ok = Date.now() - lastOk < 4000;
  $('conn').className = 'conn ' + (ok ? 'ok' : 'bad');
  $('connText').textContent = ok ? 'połączono' : 'brak połączenia';
}
setInterval(updateConn, 1000);

// ---------------------------------------------------------------- stan -> interfejs
function exploreText(s) {
  const e = s.ex;
  if (s.auto && s.wf && s.wf.on) return 'Jazda przy lewej ścianie' + (s.wf.d >= 0 ? ' (odległość ' + s.wf.d + ' cm)' : '') + (s.wf.k === 1 ? ' – skręt w prawo (róg).' : s.wf.k === 2 ? ' – szukam ściany za rogiem.' : s.wf.k === 3 ? ' – nie widzę ściany, jadę prosto.' : '.');
  if (s.auto && s.am === 4) return 'Zwiększanie dokładności: jadę do kolejnych punktów i robię dodatkowe skany (cel: ' + Math.round(e.tx) + ', ' + Math.round(e.ty) + ' cm).';
  if (!s.auto && s.am === 4 && e.st === 1) return 'Zwiększanie dokładności zakończone.';
  if (!s.auto && s.am === 4 && e.st >= 2 && e.st !== 4) return 'Zwiększanie dokładności przerwane (zobacz Dziennik).';
  if (s.auto && e.home) return 'Robot wraca na start (0, 0).';
  if (s.auto && e.st === 0) return 'Automat: jedzie do niezbadanego miejsca (' + Math.round(e.tx) + ', ' + Math.round(e.ty) + ' cm). Granic do sprawdzenia: ' + e.fr + '.';
  if (s.auto) return 'Automat startuje…';
  switch (e.st) {
    case 1: return e.home ? 'Robot wrócił na start.' : 'Automat skończył: nie ma już niezbadanych miejsc, do których robot może dojechać.';
    case 2: return e.home ? 'Powrót przerwany: brak drogi na start albo robot wielokrotnie napotykał przeszkody (zobacz Dziennik).' : 'Automat zatrzymał się: robot wielokrotnie napotykał przeszkody. Sprawdź, czy nie utknął.';
    case 3: return 'Automat nie mógł zaplanować trasy (za mało pamięci RAM albo robot stoi w przeszkodzie na mapie – zobacz Dziennik).';
    case 5: return 'Automat osiągnął limit kroków i się zatrzymał.';
    default: return '';
  }
}
function normDeg(a) { a = ((a + 180) % 360 + 360) % 360 - 180; return a; }

function applyState() {
  const s = S;
  $('stPose').textContent = Math.round(s.x) + ', ' + Math.round(s.y);
  const rel = normDeg(s.h - (s.sh === undefined ? 90 : s.sh));
  $('stHead').textContent = (rel > 0 ? '+' : '') + Math.round(rel) + '°';
  $('stPath').textContent = (s.path / 100).toFixed(2);
  const area = s.map.free * s.map.cell * s.map.cell / 10000;
  $('stArea').textContent = area.toFixed(1) + ' m²';
  $('dirText').textContent = s.dir;
  $('vMotors').textContent = s.vm.toFixed(2);
  $('vEsp').textContent = s.ve.toFixed(2);
  $('wifi').textContent = s.wifi;
  $('temp').textContent = s.temp.toFixed(1);
  $('alt').textContent = s.alt;

  if (document.activeElement !== $('speed')) { $('speed').value = s.speed; }
  $('speedVal').textContent = s.speed;

  const moving = s.auto || s.dir !== 'Stop';
  $('bScan').disabled = !!s.scan.on || moving || !!s.map.err;
  $('bLocate').disabled = !!s.scan.on || moving || s.map.occ < 40;
  $('bPick').disabled = !!s.scan.on || moving || s.map.occ < 40;
  $('bAbort').hidden = !s.scan.on;
  $('bHome').disabled = !!s.scan.on || moving || !!s.map.err || s.map.occ < 40;
  const am = s.auto ? (s.am === undefined ? 0 : s.am) : -1;
  const wallOn = !!(s.auto && s.wf && s.wf.on);
  const refOn = am === 4, special = wallOn || refOn;
  $('bWall').classList.toggle('active', wallOn);
  $('bWall').textContent = wallOn ? 'Zatrzymaj jazdę przy ścianie' : 'Jedź przy lewej ścianie';
  $('bWall').disabled = !!s.scan.on || (s.auto && !wallOn) || (s.dir !== 'Stop' && !s.auto);
  $('bRefine').classList.toggle('active', refOn);
  $('bRefine').textContent = refOn ? 'Zatrzymaj zwiększanie dokładności' : 'Zwiększ dokładność mapy';
  $('bRefine').disabled = !!s.scan.on || (s.auto && !refOn) || (s.dir !== 'Stop' && !s.auto) || s.map.occ < 40;
  $('bAuto').disabled = special;
  $('bAuto').classList.toggle('active', !!s.auto && !special);
  $('bAuto').textContent = (s.auto && !special) ? 'Zatrzymaj automat' : 'Zmapuj pokój automatycznie';
  $('bClear').disabled = !!s.scan.on || moving;
  $('heap').textContent = Math.round(s.heap / 1024);
  $('minheap').textContent = 'min ' + Math.round(s.minheap / 1024);
  $('gsign').textContent = s.gsign ? 'nauczona' : 'uczy się (po kilku obrotach)';
  $('exNote').textContent = exploreText(s);

  const sb = $('scanbar');
  sb.hidden = !s.scan.on;
  $('scanfill').style.width = s.scan.pct + '%';
  $('scantext').textContent = 'Skanowanie 360° – ' + s.scan.pct + '%';

  let note = [];
  if (s.map.err) note.push('Za mało pamięci na mapę – zmniejsz MAP_CELLS w Config.hpp.');
  else {
    note.push(s.map.loaded && s.scan.n === 0 ? 'Wczytano zapisaną mapę. Jeśli robot stoi w miejscu, w którym go zostawiono, użyj „Odnajdź się na mapie”.' : '');
    note.push(s.map.dirty ? 'Niezapisane zmiany (zapis automatyczny po skanie).' : (s.map.occ ? 'Mapa zapisana w pamięci robota.' : 'Mapa pusta – naciśnij „Skanuj tutaj”.'));
    if (s.scan.match === 1) note.push('Ostatni skan skorygował pozycję o ' + Math.hypot(s.scan.dx, s.scan.dy).toFixed(1) + ' cm i ' + s.scan.dh.toFixed(1) + '°.');
    if (s.scan.match === 2) note.push('Ostatni skan nie pasował pewnie do mapy – pozycja bez korekty.');
  }
  $('mapNote').textContent = note.filter(Boolean).join(' ');

  renderDiag(s);

  if (follow && !userMoved) { view.cx = s.x; view.cy = s.y; }
  dirty = true;
}

// ---------------------------------------------------------------- widok mapy
function resize() {
  const r = mapCanvas.getBoundingClientRect();
  DPR = Math.min(window.devicePixelRatio || 1, 2.5);
  W = Math.max(1, Math.round(r.width)); H = Math.max(1, Math.round(r.height));
  mapCanvas.width = Math.round(W * DPR); mapCanvas.height = Math.round(H * DPR);
  dirty = true;
}
window.addEventListener('resize', () => { resize(); });

function knownBounds() {
  let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  if (grid.cls) {
    const n = grid.n, c = grid.cell, half = n / 2;
    for (let cy = 0; cy < n; cy++) for (let cx = 0; cx < n; cx++) {
      if (grid.cls[cy * n + cx]) {
        const wx = (cx - half) * c, wy = (cy - half) * c;
        if (wx < x0) x0 = wx; if (wy < y0) y0 = wy;
        if (wx + c > x1) x1 = wx + c; if (wy + c > y1) y1 = wy + c;
      }
    }
  }
  if (S) { x0 = Math.min(x0, S.x - 20); x1 = Math.max(x1, S.x + 20); y0 = Math.min(y0, S.y - 20); y1 = Math.max(y1, S.y + 20); }
  if (x0 > x1) { x0 = -200; x1 = 200; y0 = -200; y1 = 200; }
  return [x0, y0, x1, y1];
}

function fitView() {
  const [x0, y0, x1, y1] = knownBounds();
  const bw = Math.max(x1 - x0, 150), bh = Math.max(y1 - y0, 150);
  view.s = Math.min(W / (bw * 1.18), H / (bh * 1.18));
  view.cx = (x0 + x1) / 2; view.cy = (y0 + y1) / 2;
  userMoved = false; dirty = true;
}

const wx2px = (x) => W / 2 + (x - view.cx) * view.s;
const wy2py = (y) => H / 2 - (y - view.cy) * view.s;

function zoomAt(factor, px, py) {
  const before = [view.cx + (px - W / 2) / view.s, view.cy - (py - H / 2) / view.s];
  view.s = Math.min(40, Math.max(0.15, view.s * factor));
  view.cx = before[0] - (px - W / 2) / view.s;
  view.cy = before[1] + (py - H / 2) / view.s;
  userMoved = true; dirty = true;
}

// ---------------------------------------------------------------- rysowanie
function niceLen(maxCm) {
  const steps = [5, 10, 20, 25, 50, 100, 200, 250, 500, 1000];
  let best = steps[0];
  for (const s of steps) if (s <= maxCm) best = s;
  return best;
}

function drawRobot(p, r) {
  ctx.save();
  ctx.translate(wx2px(p.x), wy2py(p.y));
  ctx.rotate(-p.h * Math.PI / 180);
  ctx.scale(view.s, -view.s);                 // od teraz jednostki = cm; +x = przód robota, +y = lewa strona
  const L = r.len, Wd = r.wid;
  // koła
  ctx.fillStyle = '#0b121c';
  const ww = L * 0.28, wh = Wd * 0.16;
  [[-L * 0.32, 1], [L * 0.32, 1], [-L * 0.32, -1], [L * 0.32, -1]].forEach(([x, side]) => {
    ctx.fillRect(x - ww / 2, side > 0 ? Wd / 2 - wh : -Wd / 2, ww, wh);   // koła wewnątrz obrysu 25 x 15
  });
  // korpus
  ctx.beginPath();
  const rad = Math.min(L, Wd) * 0.2;
  ctx.roundRect ? ctx.roundRect(-L / 2, -Wd / 2, L, Wd, rad) : ctx.rect(-L / 2, -Wd / 2, L, Wd);
  ctx.fillStyle = '#ffb547'; ctx.fill();
  ctx.lineWidth = Math.max(0.6, 1.4 / view.s); ctx.strokeStyle = '#7a4b00'; ctx.stroke();
  // przód
  ctx.beginPath();
  ctx.moveTo(L / 2 - 1, 0); ctx.lineTo(L / 2 - Math.min(7, L * 0.3), Wd * 0.28); ctx.lineTo(L / 2 - Math.min(7, L * 0.3), -Wd * 0.28);
  ctx.closePath(); ctx.fillStyle = '#7a4b00'; ctx.fill();
  // środek czujników skanera
  ctx.beginPath(); ctx.arc(r.sx, r.sy, Math.max(1.3, 3 / view.s), 0, Math.PI * 2);
  ctx.fillStyle = '#fff'; ctx.fill();
  ctx.restore();
}

function draw() {
  dirty = false;
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  ctx.fillStyle = cssVar('--map-unknown');
  ctx.fillRect(0, 0, W, H);

  // mapa
  if (grid.n && off.width) {
    const sz = grid.n * grid.cell, x0 = -sz / 2, y1 = sz / 2;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(off, wx2px(x0), wy2py(y1), sz * view.s, sz * view.s);

    // siatka co 50 cm (grubsza co 100) tylko na obszarze mapy
    ctx.save();
    ctx.beginPath(); ctx.rect(wx2px(x0), wy2py(y1), sz * view.s, sz * view.s); ctx.clip();
    for (let v = -sz / 2; v <= sz / 2; v += 50) {
      const major = Math.abs(v % 100) < 1e-6;
      if (!major && view.s < 0.5) continue;
      ctx.strokeStyle = major ? 'rgba(20,50,90,.22)' : 'rgba(20,50,90,.10)';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(Math.round(wx2px(v)) + .5, 0); ctx.lineTo(Math.round(wx2px(v)) + .5, H); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, Math.round(wy2py(v)) + .5); ctx.lineTo(W, Math.round(wy2py(v)) + .5); ctx.stroke();
    }
    ctx.restore();
  }

  // punkt startowy (0,0)
  ctx.strokeStyle = 'rgba(255,255,255,.5)'; ctx.lineWidth = 1.5;
  const ox = wx2px(0), oy = wy2py(0);
  ctx.beginPath(); ctx.moveTo(ox - 6, oy); ctx.lineTo(ox + 6, oy); ctx.moveTo(ox, oy - 6); ctx.lineTo(ox, oy + 6); ctx.stroke();

  // ostatni skan
  if (scanPts.length) {
    ctx.fillStyle = '#ff9d3c';
    const rr = Math.max(1.8, 1.6 * view.s);
    for (const [x, y] of scanPts) { ctx.beginPath(); ctx.arc(wx2px(x), wy2py(y), rr, 0, Math.PI * 2); ctx.fill(); }
  }

  if (S && S.auto && S.ex && S.ex.st === 0) {
    // trasa eksploracji i cel
    const wp = S.ex.wp;
    ctx.strokeStyle = 'rgba(61,220,151,.9)'; ctx.lineWidth = 2; ctx.setLineDash([7, 5]);
    ctx.beginPath(); ctx.moveTo(wx2px(S.x), wy2py(S.y));
    for (let i = 0; i + 1 < wp.length; i += 2) ctx.lineTo(wx2px(wp[i]), wy2py(wp[i + 1]));
    ctx.stroke(); ctx.setLineDash([]);
    const tx = wx2px(S.ex.tx), ty = wy2py(S.ex.ty);
    ctx.strokeStyle = '#3ddc97'; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.arc(tx, ty, 9, 0, Math.PI * 2); ctx.moveTo(tx - 13, ty); ctx.lineTo(tx + 13, ty); ctx.moveTo(tx, ty - 13); ctx.lineTo(tx, ty + 13); ctx.stroke();
  }

  if (S) {
    // pierścień zasięgu podczas skanu
    if (S.scan.on) {
      ctx.strokeStyle = 'rgba(76,194,255,.7)'; ctx.setLineDash([6, 6]); ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(wx2px(S.x), wy2py(S.y), MAP_RANGE_CM * view.s, 0, Math.PI * 2); ctx.stroke();
      ctx.setLineDash([]);
    }
    drawRobot(S, S.robot);
  }

  // podziałka
  const target = 110 / view.s;
  const len = niceLen(target);
  const px = len * view.s, bx = 14, by = H - 40;
  ctx.fillStyle = 'rgba(15,23,34,.78)';
  ctx.fillRect(bx - 8, by - 20, px + 16, 34);
  ctx.strokeStyle = '#e8eef7'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(bx, by - 5); ctx.lineTo(bx, by + 5); ctx.moveTo(bx, by); ctx.lineTo(bx + px, by); ctx.moveTo(bx + px, by - 5); ctx.lineTo(bx + px, by + 5); ctx.stroke();
  ctx.fillStyle = '#e8eef7'; ctx.font = '600 12px system-ui, sans-serif'; ctx.textBaseline = 'bottom';
  ctx.fillText(len >= 100 ? (len / 100) + ' m' : len + ' cm', bx, by - 7);
}
(function frame() { if (dirty) draw(); requestAnimationFrame(frame); })();

// ---------------------------------------------------------------- mapa: gesty
const pointers = new Map();
let pickMode = false, pickStart = null;
function setPick(v) {
  pickMode = v;
  $('bPick').classList.toggle('active', v);
  mapCanvas.style.cursor = v ? 'crosshair' : '';
  if (v) toast('Kliknij na mapie miejsce, w którym teraz stoi robot (może być przybliżone, ok. 20 cm)');
}
let lastPinch = 0;
mapCanvas.addEventListener('pointerdown', (e) => {
  mapCanvas.setPointerCapture(e.pointerId);
  pointers.set(e.pointerId, [e.clientX, e.clientY]);
  pickStart = [e.clientX, e.clientY];
  lastPinch = 0;
});
mapCanvas.addEventListener('pointermove', (e) => {
  if (!pointers.has(e.pointerId)) return;
  const prev = pointers.get(e.pointerId);
  pointers.set(e.pointerId, [e.clientX, e.clientY]);
  const r = mapCanvas.getBoundingClientRect();
  if (pointers.size === 1) {
    view.cx -= (e.clientX - prev[0]) / view.s;
    view.cy += (e.clientY - prev[1]) / view.s;
    userMoved = true; dirty = true; setFollow(false);
  } else if (pointers.size === 2) {
    const [a, b] = [...pointers.values()];
    const d = Math.hypot(a[0] - b[0], a[1] - b[1]);
    if (lastPinch) zoomAt(d / lastPinch, (a[0] + b[0]) / 2 - r.left, (a[1] + b[1]) / 2 - r.top);
    lastPinch = d; setFollow(false);
  }
});
['pointerup', 'pointercancel'].forEach((t) => mapCanvas.addEventListener(t, (e) => {
  if (t === 'pointerup' && pickMode && pointers.size === 1 && pickStart && Math.hypot(e.clientX - pickStart[0], e.clientY - pickStart[1]) < 6) {
    const r = mapCanvas.getBoundingClientRect();
    const wx = view.cx + (e.clientX - r.left - W / 2) / view.s, wy = view.cy - (e.clientY - r.top - H / 2) / view.s;
    setPick(false);
    api('pose', '&x=' + wx.toFixed(1) + '&y=' + wy.toFixed(1)).then((ok) => { if (ok) toast('Szukam dokładnej pozycji i kierunku wokół wskazanego miejsca…'); });
  }
  pointers.delete(e.pointerId); lastPinch = 0;
}));
mapCanvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const r = mapCanvas.getBoundingClientRect();
  zoomAt(Math.exp(-e.deltaY * 0.0015), e.clientX - r.left, e.clientY - r.top);
  setFollow(false);
}, { passive: false });
mapCanvas.addEventListener('dblclick', () => { fitView(); setFollow(false); });

function setFollow(v) {
  follow = v;
  $('follow').setAttribute('aria-pressed', v ? 'true' : 'false');
  if (v) { userMoved = false; if (S) { view.cx = S.x; view.cy = S.y; dirty = true; } }
}
$('zoomIn').addEventListener('click', () => zoomAt(1.4, W / 2, H / 2));
$('zoomOut').addEventListener('click', () => zoomAt(1 / 1.4, W / 2, H / 2));
$('fit').addEventListener('click', () => { setFollow(false); fitView(); });
$('follow').addEventListener('click', () => setFollow(!follow));

// ---------------------------------------------------------------- sterowanie: jedź dopóki trzymam
const held = [false, false, false, false];

function sendCommand(params) {
  const q = Object.entries(params).map(([k, v]) => `${k}=${v}`).join('&');
  return fetch(`ajax_inputs?cmd=1&${q}&t=${Date.now()}`, { cache: 'no-store' }).catch(() => {});
}
function setHeld(i, v) {
  if (held[i] === v) return;
  held[i] = v;
  $('MOTOR' + (i + 1)).classList.toggle('pressed', v);
  sendCommand({ ['MOTOR' + (i + 1)]: v ? 1 : 0 });
}
function releaseAll() { for (let i = 0; i < 4; i++) setHeld(i, false); }

for (let i = 0; i < 4; i++) {
  const el = $('MOTOR' + (i + 1));
  el.addEventListener('pointerdown', (e) => { e.preventDefault(); try { el.setPointerCapture(e.pointerId); } catch (_) {} setHeld(i, true); });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((t) => el.addEventListener(t, () => setHeld(i, false)));
  el.addEventListener('contextmenu', (e) => e.preventDefault());
}
setInterval(() => {
  const p = {};
  held.forEach((h, i) => { if (h) p['MOTOR' + (i + 1)] = 1; });
  if (Object.keys(p).length) sendCommand(p);
}, KEEPALIVE_MS);

window.addEventListener('blur', releaseAll);
window.addEventListener('pagehide', releaseAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });

const keyMap = { ArrowUp: 0, Numpad8: 0, ArrowDown: 1, Numpad5: 1, ArrowLeft: 2, Numpad4: 2, ArrowRight: 3, Numpad6: 3 };
document.addEventListener('keydown', (e) => {
  const i = keyMap[e.code];
  if (i === undefined || (e.target && e.target.tagName === 'INPUT')) return;
  e.preventDefault(); setHeld(i, true);
});
document.addEventListener('keyup', (e) => {
  const i = keyMap[e.code];
  if (i === undefined) return;
  e.preventDefault(); setHeld(i, false);
});

// prędkość
let speedTimer = 0;
$('speed').addEventListener('input', (e) => {
  $('speedVal').textContent = e.target.value;
  clearTimeout(speedTimer);
  speedTimer = setTimeout(() => sendCommand({ Slider: e.target.value }), 120);
});

// ---------------------------------------------------------------- przyciski mapowania
$('bScan').addEventListener('click', () => api('scan'));
$('bLocate').addEventListener('click', () => api('locate'));
$('bPick').addEventListener('click', () => setPick(!pickMode));
$('bAbort').addEventListener('click', () => api('abort'));
$('bSave').addEventListener('click', async () => { if (await api('save')) toast('Mapa zapisana'); });
$('bClear').addEventListener('click', () => { if (confirm('Zacząć nową mapę? Obecna zostanie wyczyszczona, a to miejsce i kierunek robota staną się punktem (0,0).')) api('clear'); });
$('bWall').addEventListener('click', () => {
  releaseAll();
  if (S && S.auto && S.wf && S.wf.on) sendCommand({ AUTO_DRIVE: 0 }).then(pollNow); else api('wall').then(pollNow);
});
$('bRefine').addEventListener('click', () => {
  releaseAll();
  if (S && S.auto && S.am === 4) sendCommand({ AUTO_DRIVE: 0 }).then(pollNow); else api('refine').then(pollNow);
});
$('bHome').addEventListener('click', () => { releaseAll(); api('home').then(pollNow); });
$('bAuto').addEventListener('click', () => {
  releaseAll();
  sendCommand({ AUTO_DRIVE: S && S.auto ? 0 : 1 }).then(pollNow);
});


// ---------------------------------------------------------------- wymiary i eksport
// Wymiary z siatki: obwiednia znanych wolnych komórek (to wnętrze pomieszczenia) i liczba komórek.
function mapDims() {
  if (!grid.cls) return null;
  const n = grid.n, c = grid.cell, half = n / 2;
  let fx0 = 1e9, fy0 = 1e9, fx1 = -1e9, fy1 = -1e9, nf = 0, no = 0;
  let ox0 = 1e9, oy0 = 1e9, ox1 = -1e9, oy1 = -1e9;
  for (let cy = 0; cy < n; cy++) for (let cx = 0; cx < n; cx++) {
    const v = grid.cls[cy * n + cx];
    if (!v) continue;
    const wx = (cx - half) * c, wy = (cy - half) * c;
    if (v === 2) {
      no++;
      if (wx < ox0) ox0 = wx; if (wy < oy0) oy0 = wy; if (wx + c > ox1) ox1 = wx + c; if (wy + c > oy1) oy1 = wy + c;
    } else {
      nf++;
      if (wx < fx0) fx0 = wx; if (wy < fy0) fy0 = wy; if (wx + c > fx1) fx1 = wx + c; if (wy + c > fy1) fy1 = wy + c;
    }
  }
  if (!nf) return null;
  return {
    free: { x0: fx0, y0: fy0, x1: fx1, y1: fy1 },
    occ: no ? { x0: ox0, y0: oy0, x1: ox1, y1: oy1 } : null,
    widthCm: fx1 - fx0, lengthCm: fy1 - fy0,
    freeAreaM2: nf * c * c / 10000, occCells: no, cellCm: c,
  };
}

function fmtCm(v) { return v >= 100 ? (v / 100).toFixed(2).replace('.', ',') + ' m' : Math.round(v) + ' cm'; }

function updateDims() {
  const d = mapDims();
  if (!d) { ['dimW', 'dimL', 'dimA', 'dimO'].forEach((id) => { $(id).textContent = '–'; }); return; }
  $('dimW').textContent = Math.round(d.widthCm) + ' cm (' + fmtCm(d.widthCm) + ')';
  $('dimL').textContent = Math.round(d.lengthCm) + ' cm (' + fmtCm(d.lengthCm) + ')';
  $('dimA').textContent = d.freeAreaM2.toFixed(2).replace('.', ',') + ' m²';
  $('dimO').textContent = d.occCells + ' kom. (' + (d.occCells * d.cellCm * d.cellCm / 10000).toFixed(2).replace('.', ',') + ' m²)';
}

function stamp() {
  const t = new Date(), p = (v) => String(v).padStart(2, '0');
  return t.getFullYear() + '-' + p(t.getMonth() + 1) + '-' + p(t.getDate()) + '_' + p(t.getHours()) + '-' + p(t.getMinutes());
}

function download(blob, name) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  document.body.appendChild(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1500);
}

// Obraz do druku: jasne tło, ściany ciemne, siatka co 50 cm z opisem, wymiary całkowite, pozycja startu i robota.
function exportPng() {
  const d = mapDims();
  if (!d) { toast('Mapa jest jeszcze pusta', true); return; }
  const pad = 40, PX = 2;                              // 2 piksele na cm
  const b = d.occ ? { x0: Math.min(d.free.x0, d.occ.x0), y0: Math.min(d.free.y0, d.occ.y0), x1: Math.max(d.free.x1, d.occ.x1), y1: Math.max(d.free.y1, d.occ.y1) } : d.free;
  const w = (b.x1 - b.x0) * PX, h = (b.y1 - b.y0) * PX;
  const top = 74, bottom = 56, W2 = Math.round(w + pad * 2 + 20), H2 = Math.round(h + top + bottom + pad);
  const cv = document.createElement('canvas');
  cv.width = Math.max(W2, 520); cv.height = H2;
  const g = cv.getContext('2d');
  const ox = pad + 20, oy = top;                       // lewy górny róg mapy
  const X = (x) => ox + (x - b.x0) * PX, Y = (y) => oy + (b.y1 - y) * PX;
  g.fillStyle = '#ffffff'; g.fillRect(0, 0, cv.width, cv.height);
  g.fillStyle = '#10202f'; g.font = 'bold 20px system-ui, sans-serif';
  g.fillText('Mapa pomieszczenia', pad, 34);
  g.font = '13px system-ui, sans-serif'; g.fillStyle = '#4a5b70';
  g.fillText(new Date().toLocaleString('pl-PL') + '   ·   ' + d.widthCm.toFixed(0) + ' × ' + d.lengthCm.toFixed(0) + ' cm   ·   ' + d.freeAreaM2.toFixed(2).replace('.', ',') + ' m²', pad, 56);
  // komórki
  const n = grid.n, c = grid.cell, half = n / 2;
  for (let cy = 0; cy < n; cy++) for (let cx = 0; cx < n; cx++) {
    const v = grid.cls[cy * n + cx];
    if (!v) continue;
    const wx = (cx - half) * c, wy = (cy - half) * c;
    g.fillStyle = v === 2 ? '#1c3a5e' : '#e8eef5';
    g.fillRect(Math.floor(X(wx)), Math.floor(Y(wy + c)), Math.ceil(c * PX) + 0.5, Math.ceil(c * PX) + 0.5);
  }
  // siatka co 50 cm
  g.strokeStyle = 'rgba(30,60,100,.18)'; g.lineWidth = 1; g.fillStyle = '#6a7a8e'; g.font = '10px system-ui, sans-serif';
  for (let x = Math.ceil(b.x0 / 50) * 50; x <= b.x1; x += 50) { g.beginPath(); g.moveTo(X(x) + .5, oy); g.lineTo(X(x) + .5, oy + h); g.stroke(); g.fillText(String(x), X(x) - 8, oy + h + 14); }
  for (let y = Math.ceil(b.y0 / 50) * 50; y <= b.y1; y += 50) { g.beginPath(); g.moveTo(ox, Y(y) + .5); g.lineTo(ox + w, Y(y) + .5); g.stroke(); g.fillText(String(y), ox - 30, Y(y) + 3); }
  // wymiary całkowite (obwiednia wolnej przestrzeni)
  g.strokeStyle = '#d6451f'; g.fillStyle = '#d6451f'; g.lineWidth = 1.5; g.font = 'bold 12px system-ui, sans-serif';
  const fx0 = X(d.free.x0), fx1 = X(d.free.x1), fy0 = Y(d.free.y1), fy1 = Y(d.free.y0), ay = oy + h + 32;
  g.beginPath(); g.moveTo(fx0, ay); g.lineTo(fx1, ay); g.moveTo(fx0, ay - 5); g.lineTo(fx0, ay + 5); g.moveTo(fx1, ay - 5); g.lineTo(fx1, ay + 5); g.stroke();
  const tw = Math.round(d.widthCm) + ' cm'; g.fillText(tw, (fx0 + fx1) / 2 - g.measureText(tw).width / 2, ay - 6);
  const ax = ox - 38;
  g.beginPath(); g.moveTo(ax, fy0); g.lineTo(ax, fy1); g.moveTo(ax - 5, fy0); g.lineTo(ax + 5, fy0); g.moveTo(ax - 5, fy1); g.lineTo(ax + 5, fy1); g.stroke();
  g.save(); g.translate(ax - 6, (fy0 + fy1) / 2); g.rotate(-Math.PI / 2);
  const tl = Math.round(d.lengthCm) + ' cm'; g.fillText(tl, -g.measureText(tl).width / 2, 0); g.restore();
  // start (0,0) i robot
  g.strokeStyle = '#0a8f5a'; g.lineWidth = 2; g.beginPath(); g.arc(X(0), Y(0), 7, 0, 7); g.moveTo(X(0) - 11, Y(0)); g.lineTo(X(0) + 11, Y(0)); g.moveTo(X(0), Y(0) - 11); g.lineTo(X(0), Y(0) + 11); g.stroke();
  g.fillStyle = '#0a8f5a'; g.font = '11px system-ui, sans-serif'; g.fillText('start', X(0) + 10, Y(0) - 8);
  if (S) {
    g.save(); g.translate(X(S.x), Y(S.y)); g.rotate(-S.h * Math.PI / 180);
    g.fillStyle = 'rgba(255,170,40,.95)'; g.strokeStyle = '#7a4a00'; g.lineWidth = 1.5;
    g.fillRect(-12.5 * PX, -7.5 * PX, 25 * PX, 15 * PX); g.strokeRect(-12.5 * PX, -7.5 * PX, 25 * PX, 15 * PX);
    g.beginPath(); g.moveTo(12.5 * PX, 0); g.lineTo(5 * PX, -4 * PX); g.lineTo(5 * PX, 4 * PX); g.closePath(); g.fillStyle = '#7a4a00'; g.fill();
    g.restore();
  }
  // legenda
  g.font = '11px system-ui, sans-serif'; g.fillStyle = '#e8eef5'; g.strokeStyle = '#9fb0c4'; g.fillRect(pad, cv.height - 22, 12, 12); g.strokeRect(pad, cv.height - 22, 12, 12);
  g.fillStyle = '#4a5b70'; g.fillText('wolne', pad + 17, cv.height - 12);
  g.fillStyle = '#1c3a5e'; g.fillRect(pad + 70, cv.height - 22, 12, 12); g.fillStyle = '#4a5b70'; g.fillText('przeszkoda', pad + 87, cv.height - 12);
  g.fillText('siatka co 50 cm · rozdzielczość ' + c + ' cm · robot 25 × 15 cm', pad + 170, cv.height - 12);
  cv.toBlob((blob) => { if (blob) { download(blob, 'mapa_' + stamp() + '.png'); toast('Zapisano obraz mapy'); } }, 'image/png');
}

$('bPng').addEventListener('click', exportPng);

// ---------------------------------------------------------------- diagnostyka
const SENS_NAMES = ['Bok (przód)', 'Przód lewy', 'Przód', 'Przód prawy', 'Tył', 'Bok (tył)'];
let lastLogSeq = -1, logBusy = false;
function renderDiag(s) {
  const sn = s.sn;
  if (sn) {
    let rows = '<tr><th>Czujnik</th><th>cm</th><th>Ostatni odczyt</th></tr>';
    for (let i = 0; i < 6; i++) {
      const stale = sn.age[i] > 600;
      rows += '<tr' + (stale ? ' class="bad"' : '') + '><td>' + SENS_NAMES[i] + '</td><td>' + (sn.v[i] >= 300 ? 'far' : sn.v[i]) + '</td><td>' + (sn.age[i] >= 9999 ? 'brak' : sn.age[i] + ' ms temu') + '</td></tr>';
    }
    $('sensTable').innerHTML = rows;
    if (robotViewOn) drawRobotView(s);
    const why = (blocked, fresh) => blocked ? (fresh ? 'przeszkoda blisko' : 'brak świeżych danych') : 'wolny';
    $('sensNote').textContent = 'Przód: ' + why(sn.fb, sn.ff) + '. Tył: ' + why(sn.rb, sn.bf) + '.';
  }
  const rel = normDeg(s.h - (s.sh === undefined ? 90 : s.sh));
  $('pHead').textContent = (rel > 0 ? '+' : '') + rel.toFixed(1) + '°';
  if (s.gy) {
    $('pGy').textContent = s.gy.ready ? (s.gy.sign ? (s.gy.sign > 0 ? 'zgodnie' : 'odwrotnie') + ', ' + s.gy.dps.toFixed(1) + '°/s' : 'uczy się') : 'brak MPU';
    $('pBias').textContent = s.gy.bias.toFixed(2) + '°/s, skala ' + s.gy.scale.toFixed(3);
    $('pCy').textContent = s.gy.cy + ' / ' + s.gy.cymax + ' ms';
  }
  $('pMatch').textContent = (s.scan.wd ? 'kierunek wg ścian ' + (s.scan.wd > 0 ? '+' : '') + s.scan.wd.toFixed(1) + '°, ' : '') + (s.scan.match === 1 ? 'korekta ' + Math.hypot(s.scan.dx, s.scan.dy).toFixed(1) + ' cm, ' + s.scan.dh.toFixed(1) + '°'
    : s.scan.match === 2 ? 'niepewne – bez korekty' : 'bez korekty');
  $('pEx').textContent = exploreText(s) || '–';
  if (s.log !== lastLogSeq && !logBusy) {
    logBusy = true;
    fetch('/api/log').then(r => r.json()).then(a => {
      lastLogSeq = s.log;
      const box = $('logBox');
      const atEnd = box.scrollTop + box.clientHeight >= box.scrollHeight - 8;
      box.textContent = a.length ? a.join('\n') : '–';
      if (atEnd) box.scrollTop = box.scrollHeight;
    }).catch(() => {}).finally(() => { logBusy = false; });
  }
}

// ---------------------------------------------------------------- podgląd robota z czujnikami (jak na ekranie robota)
let robotViewOn = false;
const RV = { px: 6, x0: 77, y0: 106, w: 86, l: 156, axle: 13.0, wheel: 7.1, servo: 3.4 };
function rvColor(cm, fresh) { return !fresh ? '#4a4f57' : cm < 15 ? '#ff4d4d' : cm < 40 ? '#ffd23f' : '#2ee66b'; }
function drawRobotView(s) {
  const sn = s.sn; if (!sn) return;
  // kolejność w sn.v: bok(przód), przód lewy, przód, przód prawy, tył, bok(tył)
  const q = (i) => ({ cm: sn.v[i], fresh: sn.age[i] <= 600 && sn.age[i] < 9999 });
  const lab = (c) => !c.fresh ? '--' : c.cm >= 150 ? 'far' : c.cm;
  const len = (c) => c.fresh ? Math.min(c.cm, 100) * 0.36 : 0;
  const bx = RV.x0, by = RV.y0, bw = RV.w, bl = RV.l;
  const sf = q(0), fl = q(1), fc = q(2), fr = q(3), rr = q(4), sr = q(5);
  const wh = RV.wheel * RV.px, ay = by + RV.axle * RV.px, sy = by + RV.servo * RV.px;
  let g = '<svg viewBox="0 0 240 320" xmlns="http://www.w3.org/2000/svg" font-family="system-ui,sans-serif" font-size="14" text-anchor="middle">';
  g += `<rect x="${bx - 5}" y="${ay - wh / 2}" width="5" height="${wh}" rx="2" fill="#3a4250"/><rect x="${bx + bw}" y="${ay - wh / 2}" width="5" height="${wh}" rx="2" fill="#3a4250"/>`;
  g += `<rect x="${bx}" y="${by}" width="${bw}" height="${bl}" rx="10" fill="#2b3647" stroke="#4a5a74" stroke-width="1.5"/>`;
  g += `<polygon points="120,${by + 60} 108,${by + 84} 132,${by + 84}" fill="#3a4250"/><text x="120" y="${by + 100}" fill="#8fa1ba" font-size="9">PRZÓD</text>`;
  g += `<circle cx="120" cy="${sy}" r="11" fill="#173a63" stroke="#8fa1ba"/>`;
  const beam = (x, y, dx, dy, c) => {
    if (!c.fresh) return '';
    const L = len(c), col = rvColor(c.cm, true);
    return dx && dy ? `<line x1="${x}" y1="${y}" x2="${x + dx * L * 0.7}" y2="${y + dy * L * 0.7}" stroke="${col}" stroke-width="6" stroke-linecap="round"/>`
      : `<line x1="${x}" y1="${y}" x2="${x + dx * L}" y2="${y + dy * L}" stroke="${col}" stroke-width="10"/>`;
  };
  g += beam(96, by + 3, -1, -1, fl) + beam(120, by + 4, 0, -1, fc) + beam(144, by + 3, 1, -1, fr) + beam(bx, by + 44, -1, 0, sf) + beam(bx, by + 112, -1, 0, sr) + beam(120, by + bl, 0, 1, rr);
  const dot = (x, y) => `<rect x="${x - 4}" y="${y - 2}" width="8" height="4" fill="#fff"/>`;
  g += dot(96, by + 3) + dot(120, by + 4) + dot(144, by + 3) + dot(bx, by + 44) + dot(bx, by + 112) + dot(120, by + bl - 9);
  const t = (x, y, c) => `<text x="${x}" y="${y}" fill="${rvColor(c.cm, c.fresh)}" font-weight="700">${lab(c)}</text>`;
  g += t(58, 64, fl) + t(120, 54, fc) + t(182, 64, fr) + t(36, by + 49, sf) + t(36, by + 117, sr) + t(172, by + bl + 24, rr);
  g += '</svg>';
  $('robotView').innerHTML = g;
}
$('bRobotView').addEventListener('click', () => {
  robotViewOn = !robotViewOn;
  $('robotView').hidden = !robotViewOn;
  $('bRobotView').setAttribute('aria-expanded', robotViewOn);
  $('bRobotView').textContent = robotViewOn ? 'Ukryj podgląd robota' : 'Podgląd robota';
  if (robotViewOn && S) drawRobotView(S);
});

// ---------------------------------------------------------------- zapisane mapy (karta SD)
function prettyMapName(f) {
  const m = /^mapa_(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})\d{2}\.bin$/.exec(f);
  return m ? m[3] + '.' + m[2] + '.' + m[1] + ' ' + m[4] + ':' + m[5] : f.replace(/\.bin$/, '');
}
async function actMap(a, f, extra) {
  try {
    const r = await getJson('api/maps/act?a=' + a + (f ? '&f=' + encodeURIComponent(f) : '') + (extra || ''), 15000);
    if (!r.ok) toast(r.err || 'Nie udało się', true);
    pollNow();
    return r.ok === 1;
  } catch (e) { toast('Brak połączenia z robotem', true); return false; }
}
async function loadMapList() {
  const box = $('mapList');
  try {
    const r = await getJson('api/maps', 8000);
    if (!r.sd) { box.innerHTML = '<p class="hint">Brak karty SD – zapisane mapy są na karcie.</p>'; return; }
    if (!r.maps.length) { box.innerHTML = '<p class="hint">Brak zapisanych map. Zapisz bieżącą przyciskiem poniżej.</p>'; return; }
    box.innerHTML = '';
    r.maps.forEach((f) => {
      const d = document.createElement('div'); d.className = 'mapitem';
      const nm = document.createElement('b'); nm.textContent = prettyMapName(f); nm.title = f;
      const bt = document.createElement('div'); bt.className = 'btns';
      const mk = (txt, fn, cls) => { const b = document.createElement('button'); b.className = 'btn ' + (cls || 'ghost'); b.textContent = txt; b.addEventListener('click', fn); bt.appendChild(b); };
      mk('Wczytaj', async () => { if (confirm('Wczytać tę mapę? Zastąpi obecną mapę robota.') && await actMap('load', f)) toast('Mapa wczytana'); });
      mk('Nazwa', async () => {
        const cur = /^mapa_\d{8}_\d{6}\.bin$/.test(f) ? '' : f.replace(/\.bin$/, '');
        const n = prompt('Nowa nazwa mapy (litery, cyfry, _ i -):', cur);
        if (n && n.trim() && await actMap('rename', f, '&n=' + encodeURIComponent(n.trim()))) { toast('Zmieniono nazwę'); loadMapList(); }
      });
      mk('Usuń', async () => { if (confirm('Usunąć mapę z karty? Tego nie można cofnąć.') && await actMap('delete', f)) { toast('Mapa usunięta'); loadMapList(); } }, 'ghost danger');
      d.appendChild(nm); d.appendChild(bt); box.appendChild(d);
    });
  } catch (e) { box.innerHTML = '<p class="hint">Nie udało się pobrać listy.</p>'; }
}
// rysuje zapisana mape na kanwie (przyciete do zbadanej czesci); zwraca zbadana powierzchnie w m2
function paintSavedMap(cv, cls, n) {
  let x0 = n, y0 = n, x1 = -1, y1 = -1, free = 0;
  for (let y = 0; y < n; y++) for (let x = 0; x < n; x++) {
    const v = cls[y * n + x]; if (!v) continue;
    if (v === 1) free++;
    if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y;
  }
  const c = cv.getContext('2d');
  c.fillStyle = cssVar('--map-unknown'); c.fillRect(0, 0, cv.width, cv.height);
  if (x1 >= 0) {
    x0 = Math.max(0, x0 - 3); y0 = Math.max(0, y0 - 3); x1 = Math.min(n - 1, x1 + 3); y1 = Math.min(n - 1, y1 + 3);
    const w = x1 - x0 + 1, h = y1 - y0 + 1, k = Math.min(cv.width / w, cv.height / h);
    const ox = (cv.width - w * k) / 2, oy = (cv.height - h * k) / 2;
    const fc = cssVar('--map-free'), wc = cssVar('--map-wall');
    for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
      const v = cls[y * n + x]; if (!v) continue;
      c.fillStyle = v === 2 ? wc : fc;
      c.fillRect(ox + (x - x0) * k, oy + (y1 - y) * k, Math.ceil(k), Math.ceil(k));
    }
  }
  return free * (S ? S.map.cell : 5) * (S ? S.map.cell : 5) / 10000;
}
async function fetchSavedMap(f) {
  const r = await fetch('api/maps/view?f=' + encodeURIComponent(f) + '&t=' + Date.now(), { cache: 'no-store' });
  if (!r.ok) throw new Error('HTTP ' + r.status + ' ' + (await r.text()).slice(0, 60));
  const n = S ? S.map.cells : 200;
  return { cls: decodeMap(await r.arrayBuffer(), n), n };
}
async function previewMap(f) {
  try {
    const { cls, n } = await fetchSavedMap(f);
    const m2 = paintSavedMap($('prevCanvas'), cls, n);
    $('mapModalTitle').textContent = prettyMapName(f);
    $('prevInfo').textContent = 'Powierzchnia zbadana: ' + m2.toFixed(1) + ' m²';
    $('mapModal').hidden = false;
  } catch (e) { toast('Podgląd: ' + (e.message || 'błąd'), true); }
}
$('mapModalClose').addEventListener('click', () => { $('mapModal').hidden = true; });
$('mapModal').addEventListener('click', (e) => { if (e.target === $('mapModal')) $('mapModal').hidden = true; });
$('bReloadSd').addEventListener('click', loadMapList);
$('bSaveSd').addEventListener('click', async () => { if (await actMap('save')) { toast('Zapisano na karcie'); loadMapList(); } });
loadMapList();

// ---------------------------------------------------------------- start
resize();
fitView();
poll().then(() => { if (!userMoved) fitView(); });

// kopiowanie dziennika (strona po http:// nie ma navigator.clipboard, wiec zaznaczenie + execCommand)
$('bCopyLog') && $('bCopyLog').addEventListener('click', () => {
  const box = $('logBox'), r = document.createRange();
  r.selectNodeContents(box); const sel = getSelection(); sel.removeAllRanges(); sel.addRange(r);
  let ok = false; try { ok = document.execCommand('copy'); } catch (e) {}
  toast(ok ? 'Dziennik skopiowany' : 'Zaznaczono - skopiuj ręcznie');
});
