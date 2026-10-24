import { Dog3D } from './dog3d.js';

const $ = id => document.getElementById(id);
const LEG_NAMES = { FL: 'перед. левая', FR: 'перед. правая', RL: 'задн. левая', RR: 'задн. правая' };
const JOINT_NAMES = { hip: 'бедро', knee: 'колено' };
const POSE_NAMES = { stand: 'Стойка', sit: 'Сидеть', lie: 'Лечь', high: 'Высоко', mount: 'Монтаж', load: 'Под нагрузкой' };
const MOVE_NAMES = { jump: 'Прыжок', joy: 'Радость', nod: 'Кивок', wag: 'Виляние', bow: 'Поклон' };
const DPAD_LABELS = { up: 'крестовина ↑', down: 'крестовина ↓', left: 'крестовина ←', right: 'крестовина →' };
const ACTION_NAMES = {
  stand: 'стойка', sit: 'сидеть', lie: 'лечь', high: 'высоко', mount: 'монтаж', load: 'под нагрузкой', jump: 'прыжок', joy: 'радость',
  nod: 'кивок', wag: 'виляние', bow: 'поклон', patrol: 'патруль вкл/выкл',
  torque_toggle: 'серво вкл/выкл', estop: 'аварийный стоп', next_gait: 'следующий профиль шага',
  roll_left: 'крен −1° (до выключения)', roll_right: 'крен +1° (до выключения)',
};
const AXIS_NAMES = { drive: 'вперёд/назад', turn: 'поворот', pitch: 'тангаж', roll: 'крен', height_down: 'опустить, пока нажат', height_up: 'поднять, пока нажат' };
const BUTTON_LABELS = {
  BTN_A: 'A', BTN_B: 'B', BTN_X: 'X', BTN_Y: 'Y',
  BTN_SOUTH: 'A / ✕', BTN_EAST: 'B / ○', BTN_WEST: 'Y / □', BTN_NORTH: 'X / △',
  BTN_START: 'Start / Options', BTN_SELECT: 'Back / Share', BTN_TL: 'LB / L1', BTN_TR: 'RB / R1',
};
const AXIS_LABELS = {
  ABS_X: 'левый стик ↔', ABS_Y: 'левый стик ↕', ABS_RX: 'правый стик ↔', ABS_RY: 'правый стик ↕',
  ABS_Z: 'правый стик ↔', ABS_RZ: 'правый стик ↕', ABS_HAT0Y: 'крестовина ↕', ABS_HAT0X: 'крестовина ↔',
  ABS_BRAKE: 'левый курок', ABS_GAS: 'правый курок',
};

// Адрес собаки (API на Raspberry Pi). Портал хостится отдельно, на ПК (ui/serve.py).
// Порядок: ?dog=… в ссылке → сохранённый в браузере → edog.local:8080.
const DOG_KEY = 'edog.dog';
const DOG_HOST = (() => {
  const fromUrl = new URLSearchParams(location.search).get('dog');
  try {
    if (fromUrl) localStorage.setItem(DOG_KEY, fromUrl);
    return fromUrl || localStorage.getItem(DOG_KEY) || 'edog.local:8080';
  } catch {
    return fromUrl || 'edog.local:8080';
  }
})();
const API = `http://${DOG_HOST}`;

let config, state, ws, dog;
let selected = null;
let calibDirty = false;
let lastStateAt = 0;  // когда пришло последнее состояние (для статуса связи)
let showAllBt = false;
const rows = {};

// ---------- API ----------

async function api(method, path, body) {
  const r = await fetch(API + path, {
    method,
    headers: body ? { 'Content-Type': 'application/json' } : {},
    body: body ? JSON.stringify(body) : undefined,
  });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) {
    toast(typeof data.detail === 'string' ? data.detail : `Ошибка ${r.status}`, 'error');
    throw new Error(r.status);
  }
  return data;
}

function send(msg) {
  if (ws?.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

let toastTimer;
// Всплывающее сообщение сверху. Тип виден и без цвета: ⚠ — ошибка, ✓ — сообщение.
function toast(text, kind = 'info') {
  const el = $('toast');
  el.textContent = `${kind === 'error' ? '⚠' : '✓'} ${text}`;
  el.className = `toast ${kind}`;
  el.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.hidden = true; }, kind === 'error' ? 6000 : 3000);
}

function throttle(fn, ms) {
  let last = 0, timer;
  return (...args) => {
    clearTimeout(timer);
    const wait = last + ms - Date.now();
    if (wait <= 0) { last = Date.now(); fn(...args); }
    else timer = setTimeout(() => { last = Date.now(); fn(...args); }, wait);
  };
}

// ---------- WebSocket ----------

function connect() {
  ws = new WebSocket(`ws://${DOG_HOST}/ws`);
  ws.onopen = () => renderLink();
  ws.onclose = () => { renderLink(); setTimeout(connect, 1000); };
  ws.onmessage = e => {
    const msg = JSON.parse(e.data);
    if (msg.type === 'state') render(msg);
    else if (msg.type === 'error') toast(msg.message, 'error');
  };
}

// Статус связи — текстом (не только цветом): «есть», «НЕТ», сколько секунд нет данных.
function renderLink() {
  const age = lastStateAt ? (Date.now() - lastStateAt) / 1000 : Infinity;
  const ok = ws?.readyState === WebSocket.OPEN && age < 2;
  setPill('st-link', ok ? 'Связь: есть' : '⚠ Связь: НЕТ', ok ? 'ok' : 'bad');
  $('offline').hidden = ok;
  $('offline-info').textContent = (Number.isFinite(age)
    ? `данных нет ${Math.round(age)} с · переподключение…` : 'подключение…') + ` · адрес собаки: ${DOG_HOST}`;
}
setInterval(renderLink, 1000);

function setPill(id, text, cls) {
  const el = $(id);
  el.textContent = text;
  el.className = `pill ${cls ?? ''}`;
}

// ---------- отрисовка ----------

function buildStatic() {
  for (const name of config.poses) {
    const b = document.createElement('button');
    b.textContent = POSE_NAMES[name] ?? name;
    b.onclick = () => api('POST', `/api/pose/${name}`);
    $('poses').append(b);
  }
  for (const name of config.moves ?? []) {
    const b = document.createElement('button');
    b.textContent = MOVE_NAMES[name] ?? name;
    b.dataset.move = name;
    b.onclick = () => api('POST', `/api/move/${name}`);
    $('moves').append(b);
  }
  const [hmin, hmax] = config.limits.height;
  Object.assign($('height'), { min: hmin, max: hmax });
  Object.assign($('pitch'), { min: -config.limits.pitch, max: config.limits.pitch });
  Object.assign($('roll'), { min: -config.limits.roll, max: config.limits.roll });

  for (const s of config.servos) {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td><span class="dot"></span>${s.id}</td><td><input type="checkbox" data-k="installed"></td><td>${s.channel}</td>
      <td data-k="joint"></td><td data-k="current"></td><td data-k="target"></td><td data-k="pulse"></td><td data-k="limits"></td>`;
    tr.querySelector('.dot').style.background =
      config.legs.find(l => l.name === s.leg)?.side === 'right' ? '#f08a3c' : '#5b92f0';
    tr.onclick = e => { if (!e.target.matches('input')) select(s.id); };
    tr.querySelector('[data-k="installed"]').onchange = e => setInstalled(s.id, e.target.checked);
    $('servo-rows').append(tr);
    rows[s.id] = tr;
  }

  const map = $('gp-map');
  for (const [role, code] of Object.entries(config.gamepad.axes)) {
    map.insertAdjacentHTML('beforeend', `<tr><td>${AXIS_LABELS[code] ?? code}</td><td>${AXIS_NAMES[role] ?? role}</td></tr>`);
  }
  for (const [code, action] of Object.entries(config.gamepad.buttons)) {
    map.insertAdjacentHTML('beforeend', `<tr><td>${BUTTON_LABELS[code] ?? code}</td><td>${ACTION_NAMES[action] ?? action}</td></tr>`);
  }
  for (const [side, action] of Object.entries(config.gamepad.dpad ?? {})) {
    map.insertAdjacentHTML('beforeend', `<tr><td>${DPAD_LABELS[side] ?? side}</td><td>${ACTION_NAMES[action] ?? action}</td></tr>`);
  }
}

function render(s) {
  state = s;
  lastStateAt = Date.now();
  dog?.update(s);

  $('torque').checked = s.enabled;
  for (const b of $('modes').children) b.classList.toggle('active', b.dataset.mode === s.mode);
  for (const b of $('poses').children) b.classList.toggle('active', config.poses[[...$('poses').children].indexOf(b)] === s.pose);
  for (const b of $('moves').children) b.classList.toggle('active', b.dataset.move === s.move);

  // Сервер: собака (настоящий Linux с evdev/Bluetooth) или ПК с заглушкой — чтобы не путать вкладки.
  const onDog = !s.gamepad.unavailable;
  setPill('st-server', onDog ? `Собака: ${DOG_HOST}` : `⚠ Собака: ${DOG_HOST} — ЭТО ЗАГЛУШКА НА ПК`, onDog ? 'ok' : 'warn');
  if (s.driver.error) setPill('st-driver', '⚠ Плата: НЕТ PCA9685 (заглушка)', 'bad');
  else if (s.driver.kind === 'mock') setPill('st-driver', '⚠ Плата: заглушка (без железа)', 'warn');
  else setPill('st-driver', 'Плата: PCA9685 есть', 'ok');
  $('st-driver').title = s.driver.error ?? '';
  setPill('st-pad', s.gamepad.connected ? `Геймпад: ${s.gamepad.name}` : s.gamepad.unavailable ? '⚠ Геймпад: недоступен' : 'Геймпад: не подключён',
    s.gamepad.connected ? 'ok' : s.gamepad.unavailable ? 'warn' : '');
  renderGamepad(s.gamepad);

  for (const k of ['height', 'pitch', 'roll']) {
    const v = s.body[k], input = $(k);
    if (document.activeElement !== input) input.value = v;
    $(`o-${k}`).textContent = k === 'height' ? `${v.toFixed(0)} мм` : `${v.toFixed(0)}°`;
  }
  if (s.roll_offset) $('o-roll').textContent += ` · бамперы ${s.roll_offset > 0 ? '+' : ''}${s.roll_offset.toFixed(0)}°`;
  const d = s.drive;
  $('drive-info').textContent = `vx ${d.vx.toFixed(2)} · поворот ${d.turn.toFixed(2)} · ${{ web: 'портал', patrol: 'патруль' }[d.source] ?? 'геймпад'}`;
  $('overlay').textContent = `${s.enabled ? 'серво включены' : 'серво отключены'} · ${{ stand: 'стойка', manual: 'ручной', demo: 'демо' }[s.mode]}`;
  renderDemo(s);
  renderGait(s.gait);
  renderCamera(s);
  renderPatrol(s.patrol);
  renderVision(s.vision);
  drawRadar(s.patrol);

  renderPower(s.power);
  renderTrim(s.trim);
  $('cal-dirty').textContent = calibDirty ? '⚠ есть несохранённые изменения' : 'всё сохранено';
  const full = powerFull(s.power);
  for (const sv of s.servos) {
    const tr = rows[sv.id];
    if (!tr) continue;
    const cell = k => tr.querySelector(`[data-k="${k}"]`);
    cell('installed').checked = sv.installed;
    cell('installed').disabled = full && !sv.installed;
    cell('installed').title = full && !sv.installed ? 'Лимит тока: сначала отключите другое серво' : '';
    tr.classList.toggle('off', !sv.installed);
    cell('joint').textContent = sv.joint_current.toFixed(1);
    cell('current').textContent = sv.current.toFixed(1);
    const marks = [sv.clamped && '⚠ предел', sv.frame_blocked && '⚠ упор в раму'].filter(Boolean).join(' ');
    cell('target').textContent = marks ? `${sv.target.toFixed(1)} ${marks}` : sv.target.toFixed(1);
    cell('target').classList.toggle('clamped', sv.clamped || sv.frame_blocked);
    cell('pulse').textContent = sv.signal ? sv.pulse_us : '—';
    cell('limits').textContent = `${sv.min_deg}…${sv.max_deg}`;
  }
  renderDetail();
}

// Видео с камеры собаки (MJPEG, свой порт). Блок есть, только если плата сообщает о камере (XIAO Sense).
// Поток один на всех: собака отдаёт его зрителю с id из /stream?id= и сообщает в состоянии camera.viewer.
// Если смотрит другое устройство (приложение, другая вкладка), портал спрашивает, переключить ли видео сюда;
// после POST /api/camera/take прежний зритель отключается. Поток обрывается и молча (собака перезагрузилась,
// Wi-Fi пропал): если зрителей нет, а картинка должна идти, — переподключаемся.

const CAM_KEY = 'edog.camera';
const OVERLAY_KEY = 'edog.overlay';
const camId = 1 + Math.floor(Math.random() * 0x7fffffff);
// wait — решить по следующему состоянию, watch — смотрим, ask — занято другим, off — выключено
let camUrl = null, camStarted = 0, camMode = 'off';
let camAbort = null, camFrame = null, camHeaders = {}, camPicking = false;
const PATROL_NAMES = {
  off: 'патруль выключен', countdown: 'старт через', start: 'кивок — начинаю', paused: 'пауза: управляет человек',
  walk: 'иду', scan: 'осматриваюсь', turn: 'поворачиваю', approach: 'человек — подхожу', greet: 'приветствую',
  leave: 'ухожу дальше', backup: 'отхожу назад',
};

// Поток MJPEG читается через fetch, а не <img>: у каждого кадра свои заголовки X-Detect, X-Free, X-Vision,
// X-Patrol — что увидело зрение. Кадр рисуется на холсте, разметка — поверх.
const CRLF2 = [13, 10, 13, 10];
function findSeq(buf, seq, from = 0) {
  outer: for (let i = from; i <= buf.length - seq.length; i++) {
    for (let j = 0; j < seq.length; j++) if (buf[i + j] !== seq[j]) continue outer;
    return i;
  }
  return -1;
}

async function camRead(url) {
  const ctrl = new AbortController();
  camAbort = ctrl;
  try {
    const r = await fetch(url, { signal: ctrl.signal, cache: 'no-store' });
    if (!r.ok) throw new Error(r.status);
    const reader = r.body.getReader();
    let buf = new Uint8Array(0);
    for (;;) {
      const { value, done } = await reader.read();
      if (done) break;
      const next = new Uint8Array(buf.length + value.length);
      next.set(buf);
      next.set(value, buf.length);
      buf = next;
      for (;;) {
        const end = findSeq(buf, CRLF2);
        if (end < 0) break;
        const head = new TextDecoder().decode(buf.subarray(0, end));
        const headers = {};
        for (const line of head.split('\r\n')) {
          const k = line.indexOf(':');
          if (k > 0) headers[line.slice(0, k).trim().toLowerCase()] = line.slice(k + 1).trim();
        }
        const len = Number(headers['content-length']);
        if (!Number.isFinite(len)) { buf = buf.subarray(end + 4); continue; }
        if (buf.length < end + 4 + len) break;
        const jpeg = buf.slice(end + 4, end + 4 + len);
        buf = buf.slice(end + 4 + len);
        createImageBitmap(new Blob([jpeg], { type: 'image/jpeg' })).then(bmp => {
          camFrame?.close();
          camFrame = bmp;
          camHeaders = headers;
          drawCam();
          if ($('cam-msg').textContent.startsWith('подключение')) $('cam-msg').textContent = '';
        }).catch(() => { /* битый кадр */ });
      }
    }
  } catch { /* закрыли сами, забрали или оборвалось */ }
  if (camAbort !== ctrl || camMode !== 'watch') return;
  camMode = 'wait';
  camStarted = Date.now();
  $('cam-msg').textContent = '⚠ Нет видео · переподключение…';
}

function camStart() {
  camMode = 'watch';
  camStarted = Date.now();
  $('cam-ask').hidden = true;
  camAbort?.abort();
  $('cam-msg').textContent = 'подключение к видео…';
  camRead(`${camUrl}?id=${camId}&t=${camStarted}`);
}

function camStop(msg) {
  camAbort?.abort();
  camAbort = null;
  camFrame?.close();
  camFrame = null;
  const c = $('cam-canvas');
  c.getContext('2d').clearRect(0, 0, c.width, c.height);
  $('cam-msg').textContent = msg;
}

function camAsk(text) {
  camMode = 'ask';
  camStop('видео на другом устройстве');
  $('cam-ask-text').textContent = text;
  $('cam-ask').hidden = false;
}

async function camTake() {
  $('cam-take').disabled = true;
  try {
    await api('POST', '/api/camera/take', { id: camId });
    camStart();
  } catch { /* ошибку показал api() */ }
  $('cam-take').disabled = false;
}

// Строка кадра 640×480, на которой лежит точка пола на расстоянии mm (геометрия камеры из состояния собаки).
function floorRow(mm) {
  const fl = state?.vision?.floor;
  if (!fl) return null;
  const f = 320 / Math.tan((state.patrol?.map?.fov_deg ?? 56) / 2 * Math.PI / 180);
  const a = Math.atan(fl.height_mm / mm) - fl.tilt_deg * Math.PI / 180;
  return 240 + f * Math.tan(a);
}

const metres = cm => cm < 0 ? '?' : cm >= 300 ? 'далеко' : `${(cm / 100).toFixed(1).replace('.', ',')} м`;

// Подпись с подложкой: читается на любом фоне, не полагаясь на цвет.
function label(ctx, text, x, y) {
  ctx.font = '600 15px system-ui, sans-serif';
  const w = ctx.measureText(text).width + 10;
  ctx.fillStyle = 'rgba(0,0,0,0.65)';
  ctx.fillRect(x, y - 18, w, 22);
  ctx.fillStyle = '#fff';
  ctx.fillText(text, x + 5, y - 2);
}

function drawCam() {
  const c = $('cam-canvas'), ctx = c.getContext('2d');
  if (!camFrame) return;
  ctx.drawImage(camFrame, 0, 0, c.width, c.height);
  if (!$('cam-overlay').checked) return;
  const h = camHeaders;
  // граница свободного пола по 16 полосам — пунктир
  const free = (h['x-free'] ?? '').split(',').filter(Boolean).map(Number);
  if (free.length === 16) {
    ctx.lineWidth = 3;
    ctx.strokeStyle = '#ffd400';
    ctx.setLineDash([8, 6]);
    ctx.beginPath();
    let started = false;
    free.forEach((cm, i) => {
      if (cm < 0) { started = false; return; }
      const y = Math.min(479, floorRow(cm * 10) ?? 479);
      const x0 = i * 40, x1 = x0 + 40;
      if (!started) ctx.moveTo(x0, y); else ctx.lineTo(x0, y);
      ctx.lineTo(x1, y);
      started = true;
    });
    ctx.stroke();
    ctx.setLineDash([]);
    for (const i of [1, 5, 10, 14]) {
      if (free[i] < 0) continue;
      const y = Math.min(470, floorRow(free[i] * 10) ?? 470);
      label(ctx, metres(free[i]), i * 40 - 6, y - 6);
    }
  }
  // люди — сплошная рамка с подписью
  for (const item of (h['x-detect'] ?? '').split(';').filter(Boolean)) {
    const [x1, y1, x2, y2, score, cm] = item.split(',').map(Number);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 4;
    ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
    label(ctx, `человек ${score}% · ${cm < 0 ? 'расстояние ?' : metres(cm)}`, x1, Math.max(22, y1));
  }
  // состояние патруля и закрытость кадра
  const kv = text => Object.fromEntries((text ?? '').split(';').filter(Boolean).map(p => p.split('=')));
  const patrol = kv(h['x-patrol']), vis = kv(h['x-vision']);
  const lines = [];
  if (patrol.state) lines.push(PATROL_NAMES[patrol.state] + (patrol.left ? ` ${patrol.left} с` : ''));
  if (vis.close) lines.push(`близко: ${Number(vis.close).toFixed(2)}${Number(vis.close) > 0.7 ? ' — почти упёрлась' : ''}`);
  if (vis.age && Number(vis.age) > 1500) lines.push(`анализ ${(Number(vis.age) / 1000).toFixed(1)} с назад`);
  lines.forEach((t, i) => label(ctx, t, 8, 26 + i * 26));
  if (camPicking) label(ctx, `кликните по низу предмета на ${$('vis-floor-mm').value} мм`, 8, 470);
}

// Клик по видео в режиме калибровки пола: строка кадра 640×480 с учётом полей (object-fit: contain).
function camClick(e) {
  if (!camPicking) return;
  const c = $('cam-canvas'), r = c.getBoundingClientRect();
  const k = Math.min(r.width / c.width, r.height / c.height);
  const y = (e.clientY - r.top - (r.height - c.height * k) / 2) / k;
  camPicking = false;
  c.classList.remove('picking');
  $('vis-floor-pick').classList.remove('active');
  if (y < 0 || y >= 480) return;
  api('POST', '/api/vision/floor_point', { y: Math.round(y), mm: Number($('vis-floor-mm').value) })
    .then(s => toast(`Точка записана: камера ${s.vision.floor.height_mm} мм над полом, наклон ${s.vision.floor.tilt_deg}°`));
}

function setupCamera() {
  const cam = config.camera;
  if (!cam) return;
  $('camera-card').hidden = false;
  if (!cam.ok) {
    $('cam-msg').textContent = `⚠ Камера не работает: ${cam.error ?? 'сенсор не найден'}`;
    $('cam-on').disabled = true;
    return;
  }
  camUrl = `http://${DOG_HOST.replace(/:\d+$/, '')}:${cam.port}${cam.path}`;
  const box = $('cam-on');
  try { box.checked = localStorage.getItem(CAM_KEY) !== '0'; } catch { box.checked = true; }
  const apply = () => {
    $('cam-ask').hidden = true;
    if (box.checked) {
      camMode = 'wait';
      camStarted = 0;
      camStop('подключение к видео…');
    } else {
      camMode = 'off';
      camStop('видео выключено');
    }
  };
  box.onchange = () => {
    try { localStorage.setItem(CAM_KEY, box.checked ? '1' : '0'); } catch { /* без хранилища */ }
    apply();
  };
  const overlay = $('cam-overlay');
  try { overlay.checked = localStorage.getItem(OVERLAY_KEY) !== '0'; } catch { overlay.checked = true; }
  overlay.onchange = () => {
    try { localStorage.setItem(OVERLAY_KEY, overlay.checked ? '1' : '0'); } catch { /* без хранилища */ }
    drawCam();
  };
  $('cam-take').onclick = camTake;
  $('cam-canvas').onclick = camClick;
  apply();
}

function renderCamera(s) {
  const cam = config.camera;
  if (!cam?.ok || !s.camera) return;
  const viewer = s.camera.viewer ?? 0, other = viewer !== 0 && viewer !== camId;
  if (camMode === 'watch' && other) {
    camAsk('Видео переключили на другое устройство. Вернуть сюда?');
  } else if (camMode === 'wait' && Date.now() - camStarted > 2000) {
    other ? camAsk('Видео смотрит другое устройство. Переключить сюда?') : camStart();
  } else if (camMode === 'ask' && !viewer) {
    camStart();   // другое устройство отключилось — поток свободен
  } else if (camMode === 'watch') {
    if (!s.camera.clients && Date.now() - camStarted > 4000) camStart();
  }
  const status = { watch: `${s.camera.fps} кадр/с из ${cam.fps}`, ask: 'на другом устройстве', off: 'выключено', wait: 'подключение' }[camMode];
  $('cam-info').textContent = `${cam.sensor} · ${status} · ${camUrl}`;
}

// ---------- патруль, зрение, радар ----------

function setupPatrol() {
  if (!config.camera?.ok) return;
  $('patrol-card').hidden = $('vision-card').hidden = $('radar-card').hidden = false;
  $('patrol-toggle').onclick = () => api('POST', '/api/patrol', { action: state?.patrol?.state === 'off' ? 'start' : 'stop' });
  $('patrol-auto').onchange = e => api('PATCH', '/api/patrol', { autostart: e.target.checked })
    .then(() => toast(e.target.checked ? 'Патруль начнётся сам после включения питания' : 'Авто-старт патруля выключен'));
  $('vis-phase-scan').onclick = () => api('POST', '/api/vision/phase_scan', { seconds: 20 })
    .then(() => toast('Подбор фазы: 20 с — пусть собака идёт вперёд'));
  $('vis-floor-pick').onclick = () => {
    camPicking = !camPicking;
    $('cam-canvas').classList.toggle('picking', camPicking);
    $('vis-floor-pick').classList.toggle('active', camPicking);
    if (camPicking) toast('Кликните на видео по низу предмета, где он касается пола');
    drawCam();
  };
  $('vis-apply').onclick = () => api('PATCH', '/api/vision', { height_mm: Number($('vis-h').value), tilt_deg: Number($('vis-tilt').value) })
    .then(() => toast('Высота и наклон камеры применены'));
  $('vis-floor-reset').onclick = () => api('POST', '/api/vision/floor_reset').then(() => toast('Калибровка пола сброшена'));
}

function renderPatrol(p) {
  if (!p) return;
  const on = p.state !== 'off';
  $('patrol-toggle').textContent = on ? 'Остановить патруль' : 'Запустить патруль';
  $('patrol-toggle').classList.toggle('primary', !on);
  if (document.activeElement !== $('patrol-auto')) $('patrol-auto').checked = p.autostart;
  $('patrol-state').textContent = (on ? 'Патруль: ' : '') + (PATROL_NAMES[p.state] ?? p.state)
    + (p.left_s !== undefined ? ` ${p.left_s} с` : '') + (p.wall ? ' · вдоль стены' : '');
  $('patrol-info').textContent = [
    !on && p.reason && `причина: ${p.reason}`,
    `приветствий ${p.greetings} · осмотров ${p.scans} · упиралась ${p.stucks}`,
    p.ignore_s && `людей не замечает ещё ${p.ignore_s} с`,
    `курс ${p.heading_deg}° · разворот ~${p.turn_dps}°/с`,
  ].filter(Boolean).join(' · ');
}

function renderVision(v) {
  if (!v) return;
  $('vis-info').textContent = v.active
    ? `распаковка ${v.ms.decode} мс · люди ${v.ms.detect} мс · пол ${v.ms.analyze} мс · ${v.rate} анализов/с · `
      + `плата ${v.temp_c}°C · свободно ${v.heap_kb} КБ${v.model ? '' : ' · ⚠ модель людей не загружена'}`
    : `зрение спит: видео никто не смотрит и патруль выключен · плата ${v.temp_c}°C`;
  const ph = v.phase;
  $('vis-phase').textContent = `фаза ${ph.target.toFixed(3)}${ph.both ? ' и +0,5' : ''} · разброс ${ph.jitter_ms} мс`
    + ` · кадров на ходу ${ph.synced_rate}/с`;
  const sc = ph.scan;
  const bins = sc.bins.map((b, i) => b === null ? null : `${(i / 16 + 1 / 32).toFixed(2)}: ${b}`).filter(Boolean);
  $('vis-phase-scan-info').textContent = sc.active ? `идёт подбор: осталось ${sc.left_s} с — собака должна идти`
    : sc.best !== undefined ? `выбрана ${sc.best.toFixed(3)} · разница по фазам ${bins.join(', ')}` : '';
  $('vis-phase-scan').disabled = sc.active;
  const fl = v.floor;
  $('vis-floor').textContent = `камера ${fl.height_mm} мм над полом, наклон ${fl.tilt_deg}° · ближе ${(fl.blind_mm / 10).toFixed(0)} см пол не виден`
    + ` · точек калибровки ${fl.points?.length ?? 0}${fl.model ? '' : ' · пол ещё не распознан'}`;
  for (const [id, val] of [['vis-h', fl.height_mm], ['vis-tilt', fl.tilt_deg]])
    if (document.activeElement !== $(id)) $(id).value = val;
}

// Радар: вид сверху, собака в центре, нос вверх, по часовой — вправо. Радиус — 3 м.
function drawRadar(p) {
  const c = $('radar');
  if (!p || c.closest('[hidden]')) return;
  const css = getComputedStyle(document.documentElement);
  const fg = css.getPropertyValue('--text').trim(), muted = css.getPropertyValue('--muted').trim();
  const accent = css.getPropertyValue('--accent').trim();
  const dpr = window.devicePixelRatio || 1, size = c.clientWidth;
  if (!size) return;
  if (c.width !== Math.round(size * dpr)) c.width = c.height = Math.round(size * dpr);
  const ctx = c.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, size, size);
  const cx = size / 2, cy = size / 2, R = size / 2 - 18, M = 300;   // см на краю
  const pt = (deg, cm) => {
    const a = deg * Math.PI / 180, r = Math.min(cm, M) / M * R;
    return [cx + r * Math.sin(a), cy - r * Math.cos(a)];
  };
  // кольца
  ctx.strokeStyle = muted;
  ctx.fillStyle = muted;
  ctx.font = '11px system-ui, sans-serif';
  ctx.lineWidth = 1;
  for (const cm of [50, 100, 200, 300]) {
    ctx.beginPath();
    ctx.arc(cx, cy, cm / M * R, 0, 2 * Math.PI);
    ctx.globalAlpha = 0.4;
    ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.fillText(`${cm / 100} м`.replace('.', ','), cx + 3, cy - cm / M * R + 12);
  }
  // сектора: длина — свободно, прозрачность — давность, неизвестные — штриховка и «?»
  const map = p.map, step = 360 / map.cm.length;
  map.cm.forEach((cm, i) => {
    const a0 = (i * step - step / 2 - 90) * Math.PI / 180, a1 = (i * step + step / 2 - 90) * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    if (cm < 0) {
      ctx.arc(cx, cy, R * 0.35, a0, a1);
      ctx.closePath();
      ctx.save();
      ctx.clip();
      ctx.strokeStyle = muted;
      ctx.globalAlpha = 0.35;
      for (let k = -size; k < size; k += 7) { ctx.beginPath(); ctx.moveTo(k, 0); ctx.lineTo(k + size, size); ctx.stroke(); }
      ctx.restore();
      const [tx, ty] = pt(i * step, M * 0.25);
      ctx.fillStyle = muted;
      ctx.fillText('?', tx - 3, ty + 4);
      return;
    }
    ctx.arc(cx, cy, Math.min(cm, M) / M * R, a0, a1);
    ctx.closePath();
    ctx.fillStyle = accent;
    ctx.globalAlpha = Math.max(0.2, 0.75 * (1 - (map.age_s[i] ?? 0) / 90));
    ctx.fill();
    ctx.globalAlpha = 1;
  });
  // что видит камера
  const half = (map.fov_deg ?? 56) / 2;
  ctx.strokeStyle = fg;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(...pt(-half, M));
  ctx.moveTo(cx, cy);
  ctx.lineTo(...pt(half, M));
  ctx.stroke();
  ctx.setLineDash([]);
  // куда идёт
  if (map.target_deg !== undefined) {
    const [tx, ty] = pt(map.target_deg, M * 0.55);
    ctx.strokeStyle = fg;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(tx, ty);
    ctx.stroke();
    const a = Math.atan2(ty - cy, tx - cx);
    ctx.beginPath();
    ctx.moveTo(tx, ty);
    ctx.lineTo(tx - 10 * Math.cos(a - 0.5), ty - 10 * Math.sin(a - 0.5));
    ctx.lineTo(tx - 10 * Math.cos(a + 0.5), ty - 10 * Math.sin(a + 0.5));
    ctx.closePath();
    ctx.fillStyle = fg;
    ctx.fill();
    ctx.lineWidth = 1;
  }
  // люди
  for (const q of map.people) {
    const [x, y] = pt(q.deg, q.cm < 0 ? 150 : q.cm);
    ctx.fillStyle = fg;
    ctx.beginPath();
    ctx.moveTo(x, y - 8); ctx.lineTo(x + 7, y); ctx.lineTo(x, y + 8); ctx.lineTo(x - 7, y);
    ctx.closePath();
    ctx.fill();
    ctx.font = '600 12px system-ui, sans-serif';
    ctx.fillText(`человек ${q.cm < 0 ? '?' : metres(q.cm)}`, x + 10, y + 4);
  }
  // собака
  ctx.fillStyle = fg;
  ctx.fillRect(cx - 5, cy - 10, 10, 20);
  ctx.beginPath();
  ctx.moveTo(cx - 5, cy - 10); ctx.lineTo(cx, cy - 16); ctx.lineTo(cx + 5, cy - 10);
  ctx.fill();
  const near = map.cm.map((cm, i) => [cm, i]).filter(([cm]) => cm >= 0).sort((a, b) => a[0] - b[0])[0];
  const deg = i => Math.round(i * step > 180 ? i * step - 360 : i * step);
  $('radar-text').textContent = `${PATROL_NAMES[p.state]}${p.left_s !== undefined ? ` ${p.left_s} с` : ''}`
    + (near ? ` · ближе всего ${metres(near[0])} (${deg(near[1])}°)` : ' · вокруг пока ничего не видно')
    + (map.people.length ? ` · людей: ${map.people.length}` : '');
}

// Прошивка по воздуху (XIAO): файл .bin уходит на собаку, она пишет его во второй слот и перезагружается.
// Новая прошивка видна по отпечатку в GET / — когда он сменился, портал перезагружается сам.

function setupFirmware() {
  if (!config.firmware) return;
  $('fw-card').hidden = false;
  $('fw-id').textContent = `Сейчас: ${config.firmware}`;
  $('fw-load').onclick = () => $('fw-file').click();
  $('fw-file').onchange = e => {
    const file = e.target.files[0];
    e.target.value = '';
    if (!file) return;
    const info = $('fw-info'), btn = $('fw-load');
    btn.disabled = true;
    const xhr = new XMLHttpRequest();
    xhr.open('POST', API + '/api/firmware');
    xhr.setRequestHeader('Content-Type', 'application/octet-stream');
    xhr.upload.onprogress = ev => { info.textContent = `Загрузка ${file.name}: ${Math.round(ev.loaded / ev.total * 100)}%`; };
    xhr.onerror = () => { btn.disabled = false; info.textContent = '⚠ Передача оборвалась, прошивка не менялась'; };
    xhr.onload = () => {
      let detail;
      try { detail = JSON.parse(xhr.responseText).detail; } catch { /* не JSON */ }
      if (xhr.status !== 200) {
        btn.disabled = false;
        info.textContent = `⚠ Не обновлено: ${detail ?? `ошибка ${xhr.status}`}`;
        return;
      }
      info.textContent = 'Записано, собака перезагружается…';
      waitFirmware(config.firmware, Date.now());
    };
    xhr.send(file);
  };
}

async function waitFirmware(old, started, wentDown = false) {
  try {
    const r = await fetch(API + '/');
    const fw = (await r.json()).firmware;
    if (fw && fw !== old) { toast(`Прошивка обновлена: ${fw}`); setTimeout(() => location.reload(), 1000); return; }
    if (fw && wentDown) {
      $('fw-info').textContent = `Собака перезагрузилась, прошивка та же (${fw}): залит тот же файл?`;
      $('fw-load').disabled = false;
      return;
    }
  } catch { wentDown = true; /* ещё перезагружается */ }
  if (Date.now() - started > 60000) {
    $('fw-info').textContent = '⚠ Собака не вернулась за минуту с новой прошивкой — проверьте её (Serial, питание)';
    $('fw-load').disabled = false;
    return;
  }
  setTimeout(() => waitFirmware(old, started, wentDown), 2000);
}

// Демо: ноги по кругу шагают в воздухе; сколько ног сразу — подбирает собака по лимиту тока.

function renderDemo(s) {
  const d = s.demo;
  $('demo').textContent = d ? 'Остановить демо' : 'Демо';
  $('demo').classList.toggle('active', !!d);
  const n = s.power.max_servos;
  $('demo-info').textContent = d
    ? `Идёт: ${d.title} · шагает ${d.legs.join(', ')} (${d.number} из ${d.numbers}) · ${Math.round(d.progress * 100)}%`
    : n !== null && n < 2 ? '⚠ Лимит тока меньше двух серво — демо недоступно'
    : `Ноги по кругу с правой передней, по 4 цикла шага · по лимиту тока: ${n === null || n >= 8 ? 'все ноги' : n >= 4 ? 'диагональными парами' : 'по одной ноге'}`;
  $('demo').disabled = !d && n !== null && n < 2;
}

// Профили ходьбы: размах суставов и пик скорости посчитаны собакой для шага полной длины в стойке.

function renderGait(name) {
  const sel = $('gait');
  if (document.activeElement !== sel) sel.value = name;
  const g = config.gaits.find(x => x.name === name);
  if (!g) return;
  $('gait-info').textContent = `${g.gait_hz} шаг/с · длина ${g.stride_mm} мм · подъём ${g.lift_mm} мм · `
    + (g.height_mm ? `высота ${g.height_mm} мм, лапа ${g.foot_x_mm > 0 ? '+' : ''}${g.foot_x_mm ?? 0} мм от бедра · ` : '')
    + `бедро ±${(g.hip_swing / 2).toFixed(0)}°, колено ±${(g.knee_swing / 2).toFixed(0)}° · `
    + `пик ${Math.max(g.hip_peak_dps, g.knee_peak_dps)} °/с из ${g.speed_dps}`
    + (g.limited ? ' · ⚠ упор в скорость серво: реальный шаг меньше' : '');
}

// Лимит тока: подключить можно не больше max_servos приводов (null — без ограничения).
const powerFull = p => p.max_servos !== null && p.installed >= p.max_servos;

// Поля не перезаписываются состоянием, пока введённое значение не применено.
let powerDirty = false;

function renderPower(p) {
  if (!powerDirty) {
    $('pw-limit').value = p.limit_a;
    $('pw-servo').value = p.servo_a;
  }
  $('pw-info').textContent = (powerDirty ? '⚠ не применено · ' : '') + (p.max_servos === null
    ? `Без ограничения (лимит 0). Подключено ${p.installed} серво, в пике до ${(p.installed * p.servo_a).toFixed(1)} А.`
    : `Подключено ${p.installed} из ${p.max_servos} возможных · в пике до ${(p.installed * p.servo_a).toFixed(1)} А из ${p.limit_a} А`
      + (powerFull(p) ? ' · ⚠ лимит исчерпан' : ''));
}

let trimDirty = false;

function renderTrim(t) {
  if (!trimDirty) {
    $('trim-pitch').value = t.pitch_deg;
    $('trim-roll').value = t.roll_deg;
  }
  $('trim-info').textContent = trimDirty ? '⚠ не применено' : 'Действует всегда, поверх наклона из стоек и стика';
}

function renderGamepad(gp) {
  const label = gp.connected ? 'подключён' : gp.unavailable ? '⚠ НЕДОСТУПЕН' : 'не подключён';
  setPill('gp-state', label, gp.connected ? 'ok' : gp.unavailable ? 'warn' : '');
  $('gp-kv').innerHTML = gp.connected
    ? `<span>Устройство</span><span>${gp.name}</span><span>Кнопка</span><span>${gp.last_button ?? '—'}</span>`
    : `<span>Статус</span><span>${gp.unavailable ?? 'ожидание подключения'}</span>`;
  const axes = gp.axes ?? {};
  $('gp-axes').innerHTML = Object.entries(AXIS_NAMES).filter(([k]) => k !== 'height').map(([k, name]) => {
    const v = axes[k] ?? 0;
    const left = 50 + Math.min(v, 0) * 50, width = Math.abs(v) * 50;
    return `<span>${name.split(' ')[0]}</span><span class="bar"><i style="left:${left}%;width:${width}%"></i></span><span>${v.toFixed(2)}</span>`;
  }).join('');
}

function select(id) {
  selected = id;
  dog?.select(id);
  for (const [sid, tr] of Object.entries(rows)) tr.classList.toggle('selected', sid === id);
  const sv = state?.servos.find(s => s.id === id);
  if (sv) {
    $('cal-joint').value = sv.ref_deg;
    $('cal-load-joint').value = sv.load_deg;
    $('cal-zero').value = sv.zero;
    $('cal-scale').value = sv.scale;
    $('cal-dir').value = sv.dir;
    $('cal-min').value = sv.min_deg;
    $('cal-max').value = sv.max_deg;
  }
  renderDetail();
}

const loadWhat = sv => Math.abs(sv.load_deg - sv.ref_deg) >= 10 ? 'scale' : 'zero';

function renderDetail() {
  const sv = state?.servos.find(s => s.id === selected);
  $('sd-empty').hidden = !!sv;
  $('sd-body').hidden = !sv;
  if (!sv) return;
  $('sd-name').textContent = `${sv.id} · ${LEG_NAMES[sv.leg]}, ${JOINT_NAMES[sv.joint]}`;
  if (document.activeElement !== $('sd-installed')) $('sd-installed').checked = sv.installed;
  $('sd-installed').disabled = powerFull(state.power) && !sv.installed;
  const signal = !sv.installed ? 'не подключено' : !state.enabled ? 'серво выключены' : `${sv.pulse_us} мкс`;
  $('sd-kv').innerHTML = [
    ['Канал PCA9685', sv.channel],
    ['Угол сустава', `${sv.joint_current.toFixed(1)}° (цель ${sv.joint_target.toFixed(1)}°)`],
    ['Угол серво', `${sv.current.toFixed(1)}° (цель ${sv.target.toFixed(1)}°)`],
    ['Сигнал', signal],
    ['zero / scale / dir', `${sv.zero}° / ${sv.scale} / ${sv.dir > 0 ? '+1' : '−1'}`],
    ['Пределы', `${sv.min_deg}° … ${sv.max_deg}°`],
    ['Рама', !state.frame_guard ? '⚠ защита от упора выключена (нет leg_hull.json)'
      : sv.frame_blocked ? '⚠ УПОР: колено дошло до плоскости рамы, дальше не пойдёт' : 'колено ниже плоскости рамы'],
  ].map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join('');
  $('wz-zero-hint').textContent = sv.joint === 'hip'
    ? `бедро смотрело вертикально вниз (${sv.ref_deg}°)` : `голень стояла под ${sv.ref_deg}° к бедру`;
  $('wz-load-hint').textContent = sv.joint === 'hip'
    ? `бедро отклонилось нижним концом назад на ${-sv.load_deg}° от вертикали (угольник ${-sv.load_deg}° к раме)`
    : `голень стоит под ${sv.load_deg}° к бедру`;
  // далеко от монтажного положения — вторая точка (масштаб), рядом — тот же zero, но под весом
  $('cal-load-set').textContent = loadWhat(sv) === 'scale' ? 'Записать масштаб' : 'Записать zero под нагрузкой';
  $('wz-dir-hint').textContent = sv.joint === 'hip'
    ? 'бедро должно качнуться вперёд, к голове' : 'голень должна повернуться вперёд, к голове';
  $('sd-sweep').textContent = sv.sweeping ? 'Остановить проход' : 'Проход по ходу';
  $('sd-sweep').dataset.test = sv.sweeping ? 'stop' : 'sweep';
  const slider = $('sd-angle');
  Object.assign(slider, { min: sv.min_deg, max: sv.max_deg });
  if (document.activeElement !== slider) slider.value = sv.target;
  $('o-sd-angle').textContent = `${sv.target.toFixed(1)}°`;
}

// ---------- управление ----------

function bindControls() {
  $('torque').onchange = e => api('POST', '/api/enable', { enabled: e.target.checked });
  $('estop').onclick = () => api('POST', '/api/estop');
  $('gait').innerHTML = config.gaits.map(g => `<option value="${g.name}">${g.title} · ~${(g.speed_mm_s / 10).toFixed(1)} см/с</option>`).join('');
  $('gait').onchange = e => api('POST', '/api/gait', { name: e.target.value })
    .then(() => { calibDirty = true; e.target.blur(); toast('Профиль шага выбран (не забудьте сохранить)'); });
  $('demo').onclick = () => api('POST', '/api/demo', { action: state?.demo ? 'stop' : 'start' });
  for (const b of $('modes').children) b.onclick = () => api('POST', '/api/mode', { mode: b.dataset.mode });

  const sendBody = throttle(k => api('POST', '/api/body', { [k]: Number($(k).value) }), 80);
  for (const k of ['height', 'pitch', 'roll']) $(k).oninput = () => sendBody(k);

  const sendServo = throttle(() => send({ type: 'servo', id: selected, angle: Number($('sd-angle').value) }), 40);
  $('sd-angle').oninput = sendServo;

  const current = () => state.servos.find(x => x.id === selected);
  const patch = body => api('PATCH', `/api/servos/${selected}/calibration`, body)
    .then(() => { calibDirty = true; select(selected); });
  $('cal-apply').onclick = () => patch({
    zero: Number($('cal-zero').value), scale: Number($('cal-scale').value), dir: Number($('cal-dir').value),
    min_deg: Number($('cal-min').value), max_deg: Number($('cal-max').value),
  });
  $('cal-set').onclick = () => api('POST', `/api/servos/${selected}/calibrate`, { joint: Number($('cal-joint').value) })
    .then(s => { calibDirty = true; select(selected); toast(`zero = ${s.servos.find(x => x.id === selected).zero}°`); });
  $('cal-load-set').onclick = () => api('POST', `/api/servos/${selected}/calibrate`,
    { joint: Number($('cal-load-joint').value), what: loadWhat(current()) })
    .then(s => {
      const sv = s.servos.find(x => x.id === selected);
      calibDirty = true; select(selected); toast(`zero = ${sv.zero}°, scale = ${sv.scale}`);
    });
  $('cal-load-pose').onclick = () => api('POST', '/api/pose/load');
  $('cal-flip').onclick = () => patch({ dir: -current().dir });
  $('cal-min-cur').onclick = () => patch({ min_deg: Math.round(current().current * 2) / 2 });
  $('cal-max-cur').onclick = () => patch({ max_deg: Math.round(current().current * 2) / 2 });
  $('cal-save').onclick = () => api('POST', '/api/calibration/save')
    .then(r => { calibDirty = false; toast(`Сохранено: ${r.saved}`); });
  // Калибровка из файла (calibration.json с Pi): применяется сразу, в память собаки — кнопкой «Сохранить».
  $('cal-load').onclick = () => $('cal-file').click();
  $('cal-file').onchange = async e => {
    const file = e.target.files[0];
    e.target.value = '';
    if (!file) return;
    let data;
    try { data = JSON.parse(await file.text()); } catch { toast(`${file.name}: это не JSON`, 'error'); return; }
    api('PUT', '/api/calibration', data).then(s => {
      calibDirty = true;
      render(s);
      if (selected) select(selected);
      toast(`Калибровка из ${file.name} загружена (не забудьте сохранить)`);
    });
  };
  $('sd-installed').onchange = e => setInstalled(selected, e.target.checked);
  for (const b of document.querySelectorAll('[data-delta]')) {
    // Шаг по углу СУСТАВА (не серво): «+» — всегда к голове на любой ноге, а после
    // «Инвертировать dir» серво закрутится в другую сторону — так проверка направления честная.
    b.onclick = () => {
      const sv = current();
      api('PUT', `/api/servos/${selected}`, { joint: sv.joint_target + Number(b.dataset.delta) });
    };
  }
  for (const b of document.querySelectorAll('[data-test]')) {
    b.onclick = () => api('POST', `/api/servos/${selected}/test`, { action: b.dataset.test });
  }

  // Движение: кнопки и клавиатура, команда повторяется, пока удерживается.
  const held = new Map();
  const tick = () => {
    let vx = 0, turn = 0;
    for (const [dvx, dturn] of held.values()) { vx += dvx; turn += dturn; }
    if (held.size) send({ type: 'drive', vx: Math.max(-1, Math.min(1, vx)), turn: Math.max(-1, Math.min(1, turn)) });
  };
  setInterval(tick, 100);
  const press = (key, v) => { held.set(key, v); tick(); };
  const release = key => { if (held.delete(key) && !held.size) send({ type: 'drive', vx: 0, turn: 0 }); };

  for (const b of $('dpad').children) {
    const v = [Number(b.dataset.vx), Number(b.dataset.turn)];
    b.onpointerdown = e => { b.setPointerCapture(e.pointerId); b.classList.add('held'); press(b, v); };
    b.onpointerup = b.onpointercancel = () => { b.classList.remove('held'); release(b); };
  }
  const KEYS = {
    KeyW: [1, 0], ArrowUp: [1, 0], KeyS: [-1, 0], ArrowDown: [-1, 0],
    KeyA: [0, -1], ArrowLeft: [0, -1], KeyD: [0, 1], ArrowRight: [0, 1],
  };
  addEventListener('keydown', e => {
    if (e.target.closest('input, select')) return;
    if (e.code === 'Space') { e.preventDefault(); api('POST', '/api/estop'); return; }
    if (KEYS[e.code]) { e.preventDefault(); press(e.code, KEYS[e.code]); }
  });
  addEventListener('keyup', e => release(e.code));
  addEventListener('blur', () => { for (const k of [...held.keys()]) release(k); });

  const applyPower = () => api('PATCH', '/api/power', {
    limit_a: Number($('pw-limit').value), servo_a: Number($('pw-servo').value),
  }).then(s => { powerDirty = false; calibDirty = true; toast(`Лимит тока: ${s.power.limit_a} А (не забудьте сохранить)`); });
  $('pw-apply').onclick = applyPower;
  for (const id of ['pw-limit', 'pw-servo']) {
    $(id).oninput = () => { powerDirty = true; };
    $(id).onkeydown = e => { if (e.key === 'Enter') applyPower(); };
  }

  const applyTrim = () => api('PATCH', '/api/trim', {
    pitch_deg: Number($('trim-pitch').value), roll_deg: Number($('trim-roll').value),
  }).then(s => { trimDirty = false; calibDirty = true; toast(`Поправка: тангаж ${s.trim.pitch_deg}°, крен ${s.trim.roll_deg}° (не забудьте сохранить)`); });
  $('trim-apply').onclick = applyTrim;
  for (const id of ['trim-pitch', 'trim-roll']) {
    $(id).oninput = () => { trimDirty = true; };
    $(id).onkeydown = e => { if (e.key === 'Enter') applyTrim(); };
  }

  $('bt-scan').onclick = scanBluetooth;
}

function setInstalled(id, installed) {
  api('PATCH', `/api/servos/${id}/calibration`, { installed })
    .then(() => { calibDirty = true; toast(`${id}: ${installed ? 'подключено' : 'отключено'} (не забудьте сохранить)`); });
}

async function scanBluetooth() {
  const btn = $('bt-scan');
  btn.disabled = true;
  $('bt-status').textContent = 'поиск…';
  try {
    const { devices: all } = await api('GET', '/api/bluetooth/scan');
    // Безымянные BLE-устройства (имя = MAC) — соседские датчики и т.п.; геймпады имя передают.
    const nameless = d => /^([0-9A-F]{2}[-:]){5}[0-9A-F]{2}$/i.test(d.name);
    const devices = showAllBt ? all : all.filter(d => d.paired || !nameless(d));
    const hidden = all.length - devices.length;
    $('bt-status').textContent = !all.length ? 'ничего не найдено' : hidden ? `скрыто безымянных: ${hidden}` : '';
    $('bt-status').onclick = hidden ? () => { showAllBt = true; scanBluetooth(); } : null;
    $('bt-status').style.cursor = hidden ? 'pointer' : '';
    $('bt-status').title = hidden ? 'Показать все' : '';
    $('bt-list').replaceChildren(...devices.map(d => {
      const li = document.createElement('li');
      li.innerHTML = `<span title="${d.mac}">${d.name}</span>`;
      const b = document.createElement('button');
      b.textContent = d.connected ? 'подключён' : d.paired ? 'подключить' : 'сопрячь';
      b.disabled = d.connected;
      b.onclick = async () => {
        b.disabled = true;
        b.textContent = '…';
        try { await api('POST', '/api/bluetooth/pair', { mac: d.mac }); b.textContent = 'подключён'; toast(`${d.name}: подключён`); }
        catch { b.disabled = false; b.textContent = 'повторить'; }
      };
      li.append(b);
      if (d.paired) {
        const rm = document.createElement('button');
        rm.textContent = '✕';
        rm.title = 'Забыть устройство';
        rm.onclick = () => api('POST', '/api/bluetooth/remove', { mac: d.mac }).then(scanBluetooth);
        li.append(rm);
      }
      return li;
    }));
  } catch {
    $('bt-status').textContent = `⚠ поиск не удался в ${new Date().toLocaleTimeString().slice(0, 5)} — повторите (сразу после загрузки Pi Bluetooth поднимается ~1 мин)`;
  } finally {
    btn.disabled = false;
  }
}

// ---------- старт ----------

// Собака может быть ещё не включена: ждём конфиг, не ломая страницу.
async function loadConfig() {
  for (;;) {
    try {
      const r = await fetch(API + '/api/config');
      if (r.ok) return await r.json();
    } catch { /* нет связи */ }
    renderLink();
    await new Promise(r => setTimeout(r, 2000));
  }
}

$('dog-host').value = DOG_HOST;
$('dog-form').onsubmit = e => {
  e.preventDefault();
  const host = $('dog-host').value.trim().replace(/^https?:\/\//, '').replace(/\/+$/, '');
  if (host) location.search = `?dog=${encodeURIComponent(host)}`;
};
renderLink();
config = await loadConfig();
buildStatic();
bindControls();
setupCamera();
setupPatrol();
setupFirmware();
dog = new Dog3D($('scene'), config, { onPick: select });
dog.loadModels().then(src => { $('scene').title = `Модель: ${src}`; });

// Переключатель плоскости рамы; выбор запоминается в браузере.
const PLANE_KEY = 'edog.showPlane';
const planeBox = $('show-plane');
try { planeBox.checked = localStorage.getItem(PLANE_KEY) === '1'; } catch { /* без хранилища */ }
const applyPlane = () => {
  const ok = dog.showFramePlane(planeBox.checked);
  planeBox.disabled = !ok;
  planeBox.parentElement.title = ok
    ? `Низ перекладины рамы: ${config.frame_guard.plane_mm} мм от оси бедра. Узел колена держится ниже: до ${config.frame_guard.limit_mm} мм (запас)`
    : 'Защита от упора в раму выключена на собаке (нет leg_hull.json)';
};
planeBox.onchange = () => {
  try { localStorage.setItem(PLANE_KEY, planeBox.checked ? '1' : '0'); } catch { /* без хранилища */ }
  applyPlane();
};
applyPlane();

// Режим стенда: корпус висит неподвижно, двигаются только ноги. Выбор запоминается.
const STAND_KEY = 'edog.staticBody';
const standBox = $('static-body');
try { standBox.checked = localStorage.getItem(STAND_KEY) === '1'; } catch { /* без хранилища */ }
dog.setStaticBody(standBox.checked);
standBox.onchange = () => {
  try { localStorage.setItem(STAND_KEY, standBox.checked ? '1' : '0'); } catch { /* без хранилища */ }
  dog.setStaticBody(standBox.checked);
};
connect();
