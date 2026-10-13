// SIL WebUI のフロント。
// 入力: キーボード + 仮想スティック -> POST /input （30 Hz）
// 出力: SSE /stream （50 Hz）
import { Viz3D } from '/viz3d.js';

const D = 180 / Math.PI;
const view = document.getElementById('view');
const charts = document.getElementById('charts');
const viz = new Viz3D(view);

function fitCanvas(c) {
  const r = c.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  c.width = Math.round(r.width * dpr);
  c.height = Math.round(r.height * dpr);
  c.getContext('2d').setTransform(dpr, 0, 0, dpr, 0, 0);
  c.width = Math.round(r.width * dpr); c.height = Math.round(r.height * dpr);
}
function fitAll() { fitCanvas(view); fitCanvas(charts); }
window.addEventListener('resize', fitAll);
fitAll();

// ---------------------------------------------------------------- 入力
const input = { throttle: 0, roll: 0, pitch: 0, yaw: 0 };
const keys = new Set();
const RATE = { throttle: 0.55, roll: 2.2, pitch: 2.2, yaw: 2.2 };   // 押しっぱなし時 [毎秒]
const STEP = { throttle: 0.02, roll: 0.10, pitch: 0.10, yaw: 0.10 }; // 1 タップあたり
const CENTERING = 3.2;                                              // 離すと戻る速さ

// キーは「押しっぱなし」と「タップ」の両方で効かせる。
// 押しっぱなしだけだと、素早い操作や自動テストで一切動かない。
const TAP = { throttle: 0, roll: 0, pitch: 0, yaw: 0 };
const TAP_KEYS = {
  KeyW: ['throttle', +1], KeyS: ['throttle', -1],
  ArrowRight: ['roll', +1], ArrowLeft: ['roll', -1],
  ArrowUp: ['pitch', +1], ArrowDown: ['pitch', -1],
  KeyD: ['yaw', +1], KeyA: ['yaw', -1],
};

window.addEventListener('keydown', (e) => {
  if (e.code === 'Space') { e.preventDefault(); post({ arm: true }); return; }
  if (e.code === 'KeyR') { post({ reset: true }); return; }
  if (e.code === 'KeyP') { paused = !paused; post({ paused }); return; }
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) e.preventDefault();
  if (!e.repeat && TAP_KEYS[e.code]) {
    const [axis, sign] = TAP_KEYS[e.code];
    TAP[axis] += sign * STEP[axis];          // タップ分を次の更新で反映
  }
  keys.add(e.code);
});
window.addEventListener('keyup', (e) => keys.delete(e.code));
window.addEventListener('blur', () => keys.clear());

let lastT = performance.now();
function updateInput() {
  const now = performance.now();
  const dt = Math.min(0.1, (now - lastT) / 1000);
  lastT = now;

  // スロットルは保持（離しても戻らない）
  if (keys.has('KeyW')) input.throttle += RATE.throttle * dt;
  if (keys.has('KeyS')) input.throttle -= RATE.throttle * dt;
  input.throttle += TAP.throttle; TAP.throttle = 0;
  input.throttle = Math.max(0, Math.min(1, input.throttle));
  if (thrSlider) { thrSlider.value = input.throttle; thrVal.textContent = input.throttle.toFixed(3); }

  // 舵は離すとセンタリング
  const axis = (name, plus, minus) => {
    let v = input[name];
    const p = keys.has(plus), m = keys.has(minus);
    if (p) v += RATE[name] * dt;
    else if (m) v -= RATE[name] * dt;
    else v -= Math.sign(v) * Math.min(Math.abs(v), CENTERING * dt);
    v += TAP[name]; TAP[name] = 0;
    input[name] = Math.max(-1, Math.min(1, v));
  };
  axis('roll', 'ArrowRight', 'ArrowLeft');
  axis('pitch', 'ArrowUp', 'ArrowDown');
  axis('yaw', 'KeyD', 'KeyA');
  drawSticks();
}

let paused = false;
async function post(extra) {
  try {
    await fetch('/input', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ...input, ...(extra || {}) }),
    });
  } catch (_) { /* 切断中 */ }
}
setInterval(() => { updateInput(); post(); }, 33);   // 30 Hz

// 仮想スティック（マウス/タッチ）
function bindStick(el, onMove) {
  const knob = el.querySelector('i');
  let active = false;
  const upd = (ev) => {
    const r = el.getBoundingClientRect();
    const cx = (ev.clientX - r.left) / r.width * 2 - 1;
    const cy = (ev.clientY - r.top) / r.height * 2 - 1;
    const x = Math.max(-1, Math.min(1, cx)), y = Math.max(-1, Math.min(1, cy));
    knob.style.left = `${(x + 1) / 2 * 100}%`;
    knob.style.top = `${(y + 1) / 2 * 100}%`;
    onMove(x, -y);
  };
  el.addEventListener('pointerdown', (e) => { active = true; el.setPointerCapture(e.pointerId); upd(e); });
  el.addEventListener('pointermove', (e) => { if (active) upd(e); });
  el.addEventListener('pointerup', (e) => { active = false; el.releasePointerCapture(e.pointerId); });
}
bindStick(document.getElementById('stickL'), (x, y) => {
  input.yaw = x; input.throttle = (y + 1) / 2;
});
bindStick(document.getElementById('stickR'), (x, y) => {
  input.roll = x; input.pitch = y;
});
function drawSticks() {
  const set = (id, x, y) => {
    const k = document.querySelector(`#${id} > i`);
    k.style.left = `${(x + 1) / 2 * 100}%`;
    k.style.top = `${(1 - y) / 2 * 100}%`;
  };
  set('stickL', input.yaw, input.throttle * 2 - 1);
  set('stickR', input.roll, input.pitch);
}

// --- スロットルの数値指定 ------------------------------------------------
// ホバー窓が非常に狭い（ホバー 0.129 / 18deg 傾斜時 0.135 / 上限飽和 0.428）ため、
// キー操作だけでは水平飛行を保つのが難しい。数値で入れられるようにする。
const thrSlider = document.getElementById('thrSlider');
const thrVal = document.getElementById('thrVal');
function setThrottle(v) {
  input.throttle = Math.max(0, Math.min(1, v));
  thrSlider.value = input.throttle;
  thrVal.textContent = input.throttle.toFixed(3);
  post();
}
thrSlider.oninput = () => setThrottle(+thrSlider.value);
document.getElementById('btnHover').onclick = () => setThrottle(0.129);
document.getElementById('btnHover2').onclick = () => setThrottle(0.136);

document.getElementById('btnArm').onclick = () => post({ arm: true });
document.getElementById('btnTrail').onclick = () => post({ clear_trail: true });
document.querySelectorAll('button[data-mode]').forEach((b) => {
  b.onclick = () => post({ mode: +b.dataset.mode });
});
const viewSpan = document.getElementById('viewSpan');
viewSpan.onchange = () => { viz.metersPerView = +viewSpan.value; viz.zoom = 1.0; };
viz.metersPerView = +viewSpan.value;
document.getElementById('follow').onchange = (e) => { viz.follow = e.target.checked; };
document.getElementById('btnReset').onclick = () => post({ reset: true });
document.getElementById('btnPause').onclick = () => { paused = !paused; post({ paused }); };
const speedEl = document.getElementById('speed');
speedEl.oninput = () => {
  document.getElementById('speedVal').textContent = `${(+speedEl.value).toFixed(2)}x`;
  post({ speed: +speedEl.value });
};

// ---------------------------------------------------------------- 出力
const MOTORS = ['FR', 'FL', 'MR', 'ML', 'RR', 'RL'];
const dutyEl = document.getElementById('duty');
MOTORS.forEach((m) => {
  dutyEl.insertAdjacentHTML('beforeend',
    `<span>${m}</span><span class="bar"><i id="bar${m}" style="width:0%"></i></span><span id="val${m}">0.000</span>`);
});

const hist = [];
const HIST_SEC = 20;

const es = new EventSource('/stream');
es.onmessage = (ev) => {
  const st = JSON.parse(ev.data);
  render(st);
};
es.onerror = () => {
  document.getElementById('status').textContent = '切断されました（サーバを確認）';
};

function render(st) {
  const s = document.getElementById('status');
  s.textContent = `${st.status}  |  t=${st.t.toFixed(1)}s  arm=${st.arm}${st.paused ? '  [一時停止]' : ''}`;
  s.className = st.arm === 2 ? 'flying' : '';

  viz.draw(st);

  const rows = [
    ['ロール (真値 / 推定)', `${(st.phi * D).toFixed(2)}° / ${(st.phi_est * D).toFixed(2)}°`],
    ['ピッチ (真値 / 推定)', `${(st.theta * D).toFixed(2)}° / ${(st.theta_est * D).toFixed(2)}°`],
    ['ヨー', `${(st.psi * D).toFixed(2)}°`],
    ['目標 ロール / ピッチ', `${(st.phi_ref * D).toFixed(1)}° / ${(st.theta_ref * D).toFixed(1)}°`],
    ['角速度 p / q / r', `${(st.p * D).toFixed(0)} / ${(st.q * D).toFixed(0)} / ${(st.r * D).toFixed(0)} °/s`],
    ['高度 (開ループ)', `${st.alt.toFixed(2)} m`],
    ['対地速度 / 上昇率', `${(st.gs || 0).toFixed(2)} / ${(st.climb || 0).toFixed(2)} m/s`],
    ['位置 N / E', `${(st.pos ? st.pos[0] : 0).toFixed(1)} / ${(st.pos ? st.pos[1] : 0).toFixed(1)} m`],
    ['T_ref', st.T_ref.toFixed(3)],
    ['common_duty', st.common_duty.toFixed(3)],
    ['スロットル入力', `${(st.input.throttle * 100).toFixed(0)} %`],
  ];
  document.getElementById('tbl').innerHTML =
    rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join('');

  MOTORS.forEach((m, i) => {
    const d = st.duty[i];
    document.getElementById(`bar${m}`).style.width = `${d * 100}%`;
    document.getElementById(`val${m}`).textContent = d.toFixed(3);
  });

  hist.push({ t: st.t, phi: st.phi * D, theta: st.theta * D,
              phi_ref: st.phi_ref * D, theta_ref: st.theta_ref * D });
  while (hist.length && st.t - hist[0].t > HIST_SEC) hist.shift();
  drawCharts();
}

function drawCharts() {
  const ctx = charts.getContext('2d');
  const W = charts.width, H = charts.height;
  ctx.clearRect(0, 0, W, H);
  if (hist.length < 2) return;
  const t0 = hist[0].t, t1 = hist[hist.length - 1].t;
  const span = Math.max(1e-3, t1 - t0);
  let lim = 5;
  for (const h of hist) lim = Math.max(lim, Math.abs(h.phi), Math.abs(h.theta));
  lim = Math.ceil(lim / 5) * 5;

  const X = (t) => ((t - t0) / span) * (W - 50) + 42;
  const Y = (v) => H / 2 - (v / lim) * (H / 2 - 12);

  ctx.strokeStyle = '#e2e8f0'; ctx.lineWidth = 1;
  [-lim, -lim / 2, 0, lim / 2, lim].forEach((v) => {
    ctx.beginPath(); ctx.moveTo(42, Y(v)); ctx.lineTo(W - 8, Y(v)); ctx.stroke();
    ctx.fillStyle = '#94a3b8'; ctx.font = '10px system-ui'; ctx.textAlign = 'right';
    ctx.fillText(v.toFixed(0), 38, Y(v) + 3);
  });

  const line = (key, color, dash) => {
    ctx.beginPath(); ctx.setLineDash(dash || []);
    hist.forEach((h, i) => (i ? ctx.lineTo(X(h.t), Y(h[key])) : ctx.moveTo(X(h.t), Y(h[key]))));
    ctx.strokeStyle = color; ctx.lineWidth = 1.6; ctx.stroke(); ctx.setLineDash([]);
  };
  line('phi_ref', '#93c5fd', [4, 3]);
  line('theta_ref', '#fca5a5', [4, 3]);
  line('phi', '#2563eb');
  line('theta', '#dc2626');

  ctx.font = '11px system-ui'; ctx.textAlign = 'left';
  ctx.fillStyle = '#2563eb'; ctx.fillText('ロール', 48, 14);
  ctx.fillStyle = '#dc2626'; ctx.fillText('ピッチ', 96, 14);
  ctx.fillStyle = '#94a3b8'; ctx.fillText('破線 = 目標 [deg]', 148, 14);
}
