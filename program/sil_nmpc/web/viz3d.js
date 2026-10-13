// ヘキサコプタの簡易 3D 描画。
//
// three.js を CDN から読むとオフラインで壊れるので、Canvas 2D に
// 自前の投影を書く。ワイヤ + 面塗りを painter's algorithm で重ねるだけ。

const HEX = {
  // 実機のモータ配置 [m]（nmpc_model.cpp kXm/kYm）。x=前, y=右, z=下(NED)
  motors: [
    { name: 'FR', x: 0.085, y: 0.050, cw: false },
    { name: 'FL', x: 0.085, y: -0.050, cw: true },
    { name: 'MR', x: 0.0, y: 0.100, cw: true },
    { name: 'ML', x: 0.0, y: -0.100, cw: false },
    { name: 'RR', x: -0.085, y: 0.050, cw: false },
    { name: 'RL', x: -0.085, y: -0.050, cw: true },
  ],
  propR: 0.038,
  bodyR: 0.030,
};

function rotBodyToWorld(phi, theta, psi) {
  // C_body_from_ned の転置（機体 -> NED）
  const cp = Math.cos(phi), sp = Math.sin(phi);
  const ct = Math.cos(theta), st = Math.sin(theta);
  const cy = Math.cos(psi), sy = Math.sin(psi);
  return [
    [ct * cy, sp * st * cy - cp * sy, cp * st * cy + sp * sy],
    [ct * sy, sp * st * sy + cp * cy, cp * st * sy - sp * cy],
    [-st, sp * ct, cp * ct],
  ];
}

function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function scale(v, k) { return [v[0] * k, v[1] * k, v[2] * k]; }

function mul(R, v) {
  return [
    R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
    R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
    R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
  ];
}

export class Viz3D {
  constructor(canvas) {
    this.c = canvas;
    this.ctx = canvas.getContext('2d');
    this.azim = -0.9;      // 視点方位 [rad]
    this.elev = 0.45;      // 視点仰角 [rad]
    this.zoom = 1.0;
    this.trail = [];
    this._epoch = -1;
    this.follow = true;      // カメラが機体を追う
    this.center = [0, 0, 0];
    this.metersPerView = 0.34;   // 画面に収める範囲 [m]。軌跡表示時は広げる
    this._bindDrag();
  }

  _bindDrag() {
    let dragging = false, lx = 0, ly = 0;
    this.c.addEventListener('mousedown', (e) => { dragging = true; lx = e.clientX; ly = e.clientY; });
    window.addEventListener('mouseup', () => { dragging = false; });
    window.addEventListener('mousemove', (e) => {
      if (!dragging) return;
      this.azim -= (e.clientX - lx) * 0.008;
      this.elev = Math.max(-1.3, Math.min(1.3, this.elev + (e.clientY - ly) * 0.008));
      lx = e.clientX; ly = e.clientY;
    });
    this.c.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.zoom = Math.max(0.3, Math.min(4, this.zoom * (e.deltaY > 0 ? 0.9 : 1.1)));
    }, { passive: false });
  }

  // 軌跡に点を足す（NED [m]）
  pushTrail(pos, epoch) {
    if (epoch !== this._epoch) { this.trail = []; this._epoch = epoch; }
    const last = this.trail[this.trail.length - 1];
    if (!last || Math.hypot(pos[0] - last[0], pos[1] - last[1], pos[2] - last[2]) > 0.05) {
      this.trail.push(pos.slice());
      if (this.trail.length > 4000) this.trail.shift();
    }
  }

  // ワールド(NED, m) -> 画面座標。カメラ中心 (this.center) からの相対で描く。
  project(w) {
    const c = this.center || [0, 0, 0];
    w = [w[0] - c[0], w[1] - c[1], w[2] - c[2]];
    const ca = Math.cos(this.azim), sa = Math.sin(this.azim);
    const ce = Math.cos(this.elev), se = Math.sin(this.elev);
    // NED -> 見やすい右手系（東, 北, 上）
    const x = w[1], y = w[0], z = -w[2];
    const xr = x * ca - y * sa;
    const yr = x * sa + y * ca;
    const sx = xr;
    const sy = yr * se + z * ce;
    const depth = yr * ce - z * se;
    const s = this.scale;
    return [this.cx + sx * s, this.cy - sy * s, depth];
  }

  draw(st) {
    const ctx = this.ctx;
    const W = this.c.width, H = this.c.height;
    this.cx = W / 2; this.cy = H * 0.60;

    // 機体のワールド位置（NED）。カメラはこれを追う。
    const pos = st.pos || [0, 0, 0];
    this.pushTrail(pos, st.trail_epoch || 0);
    this.center = this.follow ? pos : [0, 0, 0];

    // 機体は 0.17 m しかないので、広い表示範囲では点になって見えない。
    // 画面上で常に一定の大きさ（表示範囲の約 9%）に見えるよう誇張する。
    const span = this.metersPerView / this.zoom;
    this.modelScale = Math.max(1.0, (span * 0.09) / 0.17);
    // 機体は 0.17 m 幅しかないので、その実寸を画面に合わせて拡大する。
    // MODEL_SPAN_M を基準に「これが画面の何割を占めるか」で決める。
    this.scale = (Math.min(W, H) * 0.78 / this.metersPerView) * this.zoom;

    ctx.clearRect(0, 0, W, H);
    this._grid(ctx);
    this._trail(ctx);

    const R = rotBodyToWorld(st.phi, st.theta, st.psi);
    const faces = [];

    // アーム
    for (let i = 0; i < 6; i++) {
      const m = HEX.motors[i];
      const p0 = this.project(add(pos, scale(mul(R, [0, 0, 0]), this.modelScale)));
      const p1 = this.project(add(pos, scale(mul(R, [m.x, m.y, 0]), this.modelScale)));
      faces.push({ z: (p0[2] + p1[2]) / 2, kind: 'arm', p0, p1 });
    }
    // プロペラ円 + duty
    for (let i = 0; i < 6; i++) {
      const m = HEX.motors[i];
      const pts = [];
      for (let k = 0; k < 16; k++) {
        const a = (k / 16) * Math.PI * 2;
        pts.push(this.project(add(pos, scale(mul(R, [
          m.x + HEX.propR * Math.cos(a),
          m.y + HEX.propR * Math.sin(a), 0]), this.modelScale))));
      }
      const zc = pts.reduce((s, p) => s + p[2], 0) / pts.length;
      faces.push({ z: zc, kind: 'prop', pts, duty: st.duty ? st.duty[i] : 0, name: m.name, cw: m.cw });
    }
    // 機首マーカ
    const nose0 = this.project(add(pos, scale(mul(R, [0, 0, 0]), this.modelScale)));
    const nose1 = this.project(add(pos, scale(mul(R, [0.14, 0, 0]), this.modelScale)));
    faces.push({ z: (nose0[2] + nose1[2]) / 2, kind: 'nose', p0: nose0, p1: nose1 });

    // 現在地の地面投影（高度が分かるように縦線を引く）
    const gp = this.project([pos[0], pos[1], 0]);
    const ap = this.project(pos);
    ctx.strokeStyle = "rgba(148,163,184,0.6)"; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(ap[0], ap[1]); ctx.lineTo(gp[0], gp[1]); ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(gp[0], gp[1], 3, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(148,163,184,0.8)"; ctx.fill();

    faces.sort((a, b) => b.z - a.z);
    for (const f of faces) {
      if (f.kind === 'arm') {
        ctx.strokeStyle = '#6b7280'; ctx.lineWidth = 5; ctx.lineCap = 'round';
        ctx.beginPath(); ctx.moveTo(f.p0[0], f.p0[1]); ctx.lineTo(f.p1[0], f.p1[1]); ctx.stroke();
      } else if (f.kind === 'prop') {
        const d = Math.max(0, Math.min(1, f.duty));
        const hue = 210 - 210 * d;                 // 低=青, 高=赤
        ctx.beginPath();
        f.pts.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
        ctx.closePath();
        ctx.fillStyle = `hsla(${hue}, 85%, 55%, ${0.25 + 0.55 * d})`;
        ctx.fill();
        ctx.strokeStyle = `hsl(${hue}, 80%, 45%)`; ctx.lineWidth = 2; ctx.stroke();
        const c = f.pts[0];
        ctx.fillStyle = '#111827'; ctx.font = '11px system-ui'; ctx.textAlign = 'center';
        ctx.fillText(f.name, (f.pts[4][0] + f.pts[12][0]) / 2, (f.pts[0][1] + f.pts[8][1]) / 2 - 8);
      } else if (f.kind === 'nose') {
        ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 3;
        ctx.beginPath(); ctx.moveTo(f.p0[0], f.p0[1]); ctx.lineTo(f.p1[0], f.p1[1]); ctx.stroke();
      }
    }
    this._attitudeText(ctx, st);
  }

  _trail(ctx) {
    if (this.trail.length < 2) return;
    ctx.beginPath();
    this.trail.forEach((p, i) => {
      const s = this.project(p);
      i ? ctx.lineTo(s[0], s[1]) : ctx.moveTo(s[0], s[1]);
    });
    ctx.strokeStyle = "rgba(37,99,235,0.55)";
    ctx.lineWidth = 2;
    ctx.stroke();
    // 地面への投影（影）
    ctx.beginPath();
    this.trail.forEach((p, i) => {
      const s = this.project([p[0], p[1], 0]);
      i ? ctx.lineTo(s[0], s[1]) : ctx.moveTo(s[0], s[1]);
    });
    ctx.strokeStyle = "rgba(100,116,139,0.30)";
    ctx.lineWidth = 1.2;
    ctx.stroke();
  }

  _grid(ctx) {
    // グリッド間隔を表示範囲に合わせる（1/6 刻み）
    const step = niceStep(this.metersPerView / this.zoom / 6);
    const N = 6;
    const c = this.center || [0, 0, 0];
    const gx = Math.round(c[0] / step) * step;
    const gy = Math.round(c[1] / step) * step;
    ctx.strokeStyle = '#e5e7eb'; ctx.lineWidth = 1;
    for (let i = -N; i <= N; i++) {
      const a = this.project([gx + i * step, gy - N * step, 0]);
      const b = this.project([gx + i * step, gy + N * step, 0]);
      const c = this.project([gx - N * step, gy + i * step, 0]);
      const d = this.project([gx + N * step, gy + i * step, 0]);
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(c[0], c[1]); ctx.lineTo(d[0], d[1]); ctx.stroke();
    }
    // 北向き（機体の前方基準ではなくワールド）
    const o = this.project([gx, gy, 0]);
    const n = this.project([gx + N * step, gy, 0]);
    ctx.strokeStyle = '#9ca3af'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(n[0], n[1]); ctx.stroke();
    ctx.fillStyle = '#9ca3af'; ctx.font = '12px system-ui'; ctx.textAlign = 'left';
    ctx.fillText('N', n[0] + 4, n[1]);
  }

  _attitudeText(ctx, st) {
    const D = 180 / Math.PI;
    ctx.fillStyle = '#374151'; ctx.font = '13px ui-monospace, monospace'; ctx.textAlign = 'left';
    const lines = [
      `roll  ${(st.phi * D).toFixed(2).padStart(7)}°   (推定 ${(st.phi_est * D).toFixed(2)}°)`,
      `pitch ${(st.theta * D).toFixed(2).padStart(7)}°   (推定 ${(st.theta_est * D).toFixed(2)}°)`,
      `yaw   ${(st.psi * D).toFixed(2).padStart(7)}°`,
      `alt   ${st.alt.toFixed(2).padStart(7)} m`,
      `対地速度 ${(st.gs !== undefined ? st.gs : 0).toFixed(2).padStart(6)} m/s   上昇 ${(st.climb !== undefined ? st.climb : 0).toFixed(2)} m/s`,
      `位置  N ${(st.pos ? st.pos[0] : 0).toFixed(1)}  E ${(st.pos ? st.pos[1] : 0).toFixed(1)} m`,
      `表示範囲 ${(this.metersPerView / this.zoom).toFixed(1)} m`,
    ];
    lines.forEach((l, i) => ctx.fillText(l, 12, 20 + i * 17));
  }
}

export { HEX };


// 1, 2, 5 x 10^n の「きりのいい」間隔を返す
function niceStep(v) {
  if (!(v > 0)) return 0.05;
  const e = Math.pow(10, Math.floor(Math.log10(v)));
  const m = v / e;
  return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * e;
}
