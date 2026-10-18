import { Dog3D } from './dog3d.js';

const $ = id => document.getElementById(id);
const LEG_NAMES = { FL: 'перед. левая', FR: 'перед. правая', RL: 'задн. левая', RR: 'задн. правая' };
const JOINT_NAMES = { hip: 'бедро', knee: 'колено' };
const POSE_NAMES = { stand: 'Стойка', sit: 'Сидеть', lie: 'Лечь', high: 'Высоко' };
const ACTION_NAMES = {
  stand: 'стойка', sit: 'сидеть', lie: 'лечь', high: 'высоко', walk_toggle: 'ходьба вкл/выкл',
  torque_toggle: 'серво вкл/выкл', estop: 'аварийный стоп',
};
const AXIS_NAMES = { drive: 'вперёд/назад', turn: 'поворот', pitch: 'тангаж', roll: 'крен', height: 'высота (крестовина)' };
const BUTTON_LABELS = {
  BTN_SOUTH: 'A / ✕', BTN_EAST: 'B / ○', BTN_WEST: 'X / □', BTN_NORTH: 'Y / △',
  BTN_START: 'Start / Options', BTN_SELECT: 'Back / Share', BTN_TL: 'LB / L1', BTN_TR: 'RB / R1',
};
const AXIS_LABELS = {
  ABS_X: 'левый стик ↔', ABS_Y: 'левый стик ↕', ABS_RX: 'правый стик ↔', ABS_RY: 'правый стик ↕',
  ABS_Z: 'ось Z', ABS_RZ: 'ось RZ', ABS_HAT0Y: 'крестовина ↕', ABS_HAT0X: 'крестовина ↔',
};

let config, state, ws, dog;
let selected = null;
let calibDirty = false;
const rows = {};

// ---------- API ----------

async function api(method, path, body) {
  const r = await fetch(path, {
    method,
    headers: body ? { 'Content-Type': 'application/json' } : {},
    body: body ? JSON.stringify(body) : undefined,
  });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) {
    toast(typeof data.detail === 'string' ? data.detail : `Ошибка ${r.status}`);
    throw new Error(r.status);
  }
  return data;
}

function send(msg) {
  if (ws?.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

let toastTimer;
function toast(text) {
  const el = $('toast');
  el.textContent = text;
  el.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.hidden = true; }, 3000);
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
  ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onopen = () => setPill('st-link', 'связь', 'ok');
  ws.onclose = () => { setPill('st-link', 'нет связи', 'bad'); setTimeout(connect, 1000); };
  ws.onmessage = e => {
    const msg = JSON.parse(e.data);
    if (msg.type === 'state') render(msg);
    else if (msg.type === 'error') toast(msg.message);
  };
}

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
}

function render(s) {
  state = s;
  dog?.update(s);

  $('torque').checked = s.enabled;
  for (const b of $('modes').children) b.classList.toggle('active', b.dataset.mode === s.mode);
  for (const b of $('poses').children) b.classList.toggle('active', config.poses[[...$('poses').children].indexOf(b)] === s.pose);

  if (s.driver.error) setPill('st-driver', 'нет PCA9685', 'bad');
  else setPill('st-driver', s.driver.kind === 'mock' ? 'заглушка' : 'PCA9685', s.driver.kind === 'mock' ? 'warn' : 'ok');
  $('st-driver').title = s.driver.error ?? '';
  setPill('st-pad', s.gamepad.connected ? 'геймпад' : 'нет геймпада', s.gamepad.connected ? 'ok' : '');
  renderGamepad(s.gamepad);

  for (const [k, v] of Object.entries(s.body)) {
    const input = $(k);
    if (input && document.activeElement !== input) input.value = v;
    $(`o-${k}`).textContent = k === 'height' ? `${v.toFixed(0)} мм` : `${v.toFixed(0)}°`;
  }
  const d = s.drive;
  $('drive-info').textContent = s.mode === 'walk'
    ? `vx ${d.vx.toFixed(2)} · поворот ${d.turn.toFixed(2)} · ${d.source === 'web' ? 'портал' : 'геймпад'}`
    : 'Для движения включите режим «Ходьба».';
  $('overlay').textContent = `${s.enabled ? 'серво включены' : 'серво отключены'} · ${{ stand: 'стойка', walk: 'ходьба', manual: 'ручной' }[s.mode]}`;

  for (const sv of s.servos) {
    const tr = rows[sv.id];
    if (!tr) continue;
    const cell = k => tr.querySelector(`[data-k="${k}"]`);
    cell('installed').checked = sv.installed;
    tr.classList.toggle('off', !sv.installed);
    cell('joint').textContent = sv.joint_current.toFixed(1);
    cell('current').textContent = sv.current.toFixed(1);
    cell('target').textContent = sv.target.toFixed(1);
    cell('target').classList.toggle('clamped', sv.clamped);
    cell('pulse').textContent = s.enabled && sv.installed ? sv.pulse_us : '—';
    cell('limits').textContent = `${sv.min_deg}…${sv.max_deg}`;
  }
  renderDetail();
}

function renderGamepad(gp) {
  const label = gp.connected ? 'подключён' : gp.unavailable ? 'недоступен' : 'не подключён';
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
  const signal = !sv.installed ? 'не подключено' : !state.enabled ? 'серво выключены' : `${sv.pulse_us} мкс`;
  $('sd-kv').innerHTML = [
    ['Канал PCA9685', sv.channel],
    ['Угол сустава', `${sv.joint_current.toFixed(1)}° (цель ${sv.joint_target.toFixed(1)}°)`],
    ['Угол серво', `${sv.current.toFixed(1)}° (цель ${sv.target.toFixed(1)}°)`],
    ['Сигнал', signal],
    ['zero / dir', `${sv.zero}° / ${sv.dir > 0 ? '+1' : '−1'}`],
    ['Пределы', `${sv.min_deg}° … ${sv.max_deg}°`],
  ].map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join('');
  $('wz-zero-hint').textContent = sv.joint === 'hip'
    ? `бедро смотрело вертикально вниз (${sv.ref_deg}°)` : `голень стояла под ${sv.ref_deg}° к бедру`;
  $('wz-dir-hint').textContent = sv.joint === 'hip'
    ? 'бедро должно качнуться вперёд, к голове' : 'голень должна повернуться вперёд, к голове';
  $('sd-sweep').textContent = sv.sweeping ? 'Остановить проход' : 'Проход по ходу';
  $('sd-sweep').dataset.test = sv.sweeping ? 'stop' : 'sweep';
  $('cal-dirty').textContent = calibDirty ? 'есть несохранённые изменения' : '';
  const slider = $('sd-angle');
  Object.assign(slider, { min: sv.min_deg, max: sv.max_deg });
  if (document.activeElement !== slider) slider.value = sv.target;
  $('o-sd-angle').textContent = `${sv.target.toFixed(1)}°`;
}

// ---------- управление ----------

function bindControls() {
  $('torque').onchange = e => api('POST', '/api/enable', { enabled: e.target.checked });
  $('estop').onclick = () => api('POST', '/api/estop');
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
    b.onclick = () => api('POST', `/api/servos/${selected}/test`, { action: 'nudge', delta: Number(b.dataset.delta) });
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
    const { devices } = await api('GET', '/api/bluetooth/scan');
    $('bt-status').textContent = devices.length ? '' : 'ничего не найдено';
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
    $('bt-status').textContent = 'Bluetooth недоступен';
  } finally {
    btn.disabled = false;
  }
}

// ---------- старт ----------

config = await api('GET', '/api/config');
buildStatic();
bindControls();
dog = new Dog3D($('scene'), config, { onPick: select });
dog.loadModels().then(src => { $('scene').title = `Модель: ${src}`; });
connect();
