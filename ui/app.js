import { Dog3D } from './dog3d.js';

const $ = id => document.getElementById(id);
const LEG_NAMES = { FL: 'перед. левая', FR: 'перед. правая', RL: 'задн. левая', RR: 'задн. правая' };
const JOINT_NAMES = { hip: 'бедро', knee: 'колено' };
const POSE_NAMES = { stand: 'Стойка', sit: 'Сидеть', lie: 'Лечь', high: 'Высоко', mount: 'Монтаж' };
const MOVE_NAMES = { jump: 'Прыжок', joy: 'Радость' };
const DPAD_LABELS = { up: 'крестовина ↑', down: 'крестовина ↓', left: 'крестовина ←', right: 'крестовина →' };
const ACTION_NAMES = {
  stand: 'стойка', sit: 'сидеть', lie: 'лечь', high: 'высоко', mount: 'монтаж', jump: 'прыжок', joy: 'радость',
  torque_toggle: 'серво вкл/выкл', estop: 'аварийный стоп',
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

  for (const [k, v] of Object.entries(s.body)) {
    const input = $(k);
    if (input && document.activeElement !== input) input.value = v;
    $(`o-${k}`).textContent = k === 'height' ? `${v.toFixed(0)} мм` : `${v.toFixed(0)}°`;
  }
  const d = s.drive;
  $('drive-info').textContent = `vx ${d.vx.toFixed(2)} · поворот ${d.turn.toFixed(2)} · ${d.source === 'web' ? 'портал' : 'геймпад'}`;
  $('overlay').textContent = `${s.enabled ? 'серво включены' : 'серво отключены'} · ${{ stand: 'стойка', manual: 'ручной', demo: 'демо' }[s.mode]}`;
  renderDemo(s);
  renderGait(s.gait);

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
    $('cal-zero').value = sv.zero;
    $('cal-dir').value = sv.dir;
    $('cal-min').value = sv.min_deg;
    $('cal-max').value = sv.max_deg;
  }
  renderDetail();
}

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
    ['zero / dir', `${sv.zero}° / ${sv.dir > 0 ? '+1' : '−1'}`],
    ['Пределы', `${sv.min_deg}° … ${sv.max_deg}°`],
    ['Рама', !state.frame_guard ? '⚠ защита от упора выключена (нет leg_hull.json)'
      : sv.frame_blocked ? '⚠ УПОР: колено дошло до плоскости рамы, дальше не пойдёт' : 'колено ниже плоскости рамы'],
  ].map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join('');
  $('wz-zero-hint').textContent = sv.joint === 'hip'
    ? `бедро смотрело вертикально вниз (${sv.ref_deg}°)` : `голень стояла под ${sv.ref_deg}° к бедру`;
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
    zero: Number($('cal-zero').value), dir: Number($('cal-dir').value),
    min_deg: Number($('cal-min').value), max_deg: Number($('cal-max').value),
  });
  $('cal-set').onclick = () => api('POST', `/api/servos/${selected}/calibrate`, { joint: Number($('cal-joint').value) })
    .then(s => { calibDirty = true; select(selected); toast(`zero = ${s.servos.find(x => x.id === selected).zero}°`); });
  $('cal-flip').onclick = () => patch({ dir: -current().dir });
  $('cal-min-cur').onclick = () => patch({ min_deg: Math.round(current().current * 2) / 2 });
  $('cal-max-cur').onclick = () => patch({ max_deg: Math.round(current().current * 2) / 2 });
  $('cal-save').onclick = () => api('POST', '/api/calibration/save')
    .then(r => { calibDirty = false; toast(`Сохранено: ${r.saved}`); });
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
