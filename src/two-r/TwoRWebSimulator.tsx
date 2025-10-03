import React, { useEffect, useMemo, useRef, useState } from "react";
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from "recharts";

const COLORS = {
  sheet: "#0f172a",
  margin: "#94a3b8",
  path: "#94a3b8",
  trail: "#10B981",
  arm: "#111827",
  joints: "#ef4444",
  union: "#22c55e",
  gridMinor: "#e5e7eb",
  gridMajor: "#cbd5e1",
  axis: "#334155",
  workspaceOuter: "#f97316",
  workspaceInner: "#f59e0b",
  square: "#a78bfa",
  angle1: "#0ea5e9", // θ1 arc (sky)
  angle2: "#f43f5e"  // θ2 arc (rose)
};

const clamp = (x: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, x));
const deg = (rad: number) => (rad * 180) / Math.PI;

function almostMultiple(value: number, step: number, eps = 1e-6) {
  return Math.abs(value / step - Math.round(value / step)) < eps;
}

function finiteDiff(arr: number[], dt: number) {
  const n = arr.length;
  const out = new Array<number>(n).fill(0);
  if (n >= 2) {
    out[0] = (arr[1] - arr[0]) / dt;
    for (let i = 1; i < n - 1; i++) out[i] = (arr[i + 1] - arr[i - 1]) / (2 * dt);
    out[n - 1] = (arr[n - 1] - arr[n - 2]) / dt;
  }
  return out;
}

function fk2R(theta1: number, theta2: number, l1: number, l2: number) {
  const x = l1 * Math.cos(theta1) + l2 * Math.cos(theta1 + theta2);
  const y = l1 * Math.sin(theta1) + l2 * Math.sin(theta1 + theta2);
  return { x, y };
}

function ik2R(x: number, y: number, l1: number, l2: number, elbow: "up" | "down" = "up") {
  const r2 = x * x + y * y;
  let c2 = (r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  if (c2 < -1 - 1e-9 || c2 > 1 + 1e-9) return null;
  c2 = clamp(c2, -1, 1);
  const s2abs = Math.sqrt(Math.max(0, 1 - c2 * c2));
  const s2 = elbow === "up" ? s2abs : -s2abs;
  const theta2 = Math.atan2(s2, c2);
  const k1 = l1 + l2 * c2;
  const k2 = l2 * s2;
  const theta1 = Math.atan2(y, x) - Math.atan2(k2, k1);
  return { theta1, theta2 };
}

function superformula(phi: number, m: number, a = 1, b = 1, n1 = 0.35, n2 = 0.35, n3 = 0.35) {
  const t1 = Math.pow(Math.abs(Math.cos((m * phi) / 4) / a), n2);
  const t2 = Math.pow(Math.abs(Math.sin((m * phi) / 4) / b), n3);
  return Math.pow(t1 + t2, -1 / n1);
}

function cloverXY({ nPetals = 4, scale = 1, samples = 2400, rotationDeg = 0, n1 = 2.2, n2 = 4, n3 = 0, pinch = 0.15 }:
  { nPetals?: number; scale?: number; samples?: number; rotationDeg?: number; n1?: number; n2?: number; n3?: number; pinch?: number; }) {
  const m = 2 * nPetals;
  const rot = (rotationDeg * Math.PI) / 180;
  const x = new Array<number>(samples);
  const y = new Array<number>(samples);
  for (let i = 0; i < samples; i++) {
    const phi = (i / (samples - 1)) * 2 * Math.PI + rot;
    let r = superformula(phi, m, 1, 1, n1, n2, n3);
    if (pinch > 0) {
      const sep = 1 - pinch * Math.pow(Math.cos(nPetals * phi), 2);
      r *= sep;
    }
    x[i] = scale * r * Math.cos(phi);
    y[i] = scale * r * Math.sin(phi);
  }
  return { x, y } as { x: number[]; y: number[] };
}

function roseCurve(n = 3, num = 2000) {
  return cloverXY({ nPetals: n, scale: 1, samples: num, rotationDeg: 0, n1: 2.2, n2: 4, n3: 0, pinch: 0.18 });
}

function bbox(x: number[], y: number[]) {
  let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
  for (let i = 0; i < x.length; i++) {
    if (x[i] < xmin) xmin = x[i];
    if (x[i] > xmax) xmax = x[i];
    if (y[i] < ymin) ymin = y[i];
    if (y[i] > ymax) ymax = y[i];
  }
  return { xmin, xmax, ymin, ymax };
}

function rotateXY(x: number[], y: number[], rotDeg: number) {
  const th = (rotDeg * Math.PI) / 180;
  const c = Math.cos(th), s = Math.sin(th);
  const xr = new Array<number>(x.length);
  const yr = new Array<number>(y.length);
  for (let i = 0; i < x.length; i++) {
    xr[i] = x[i] * c - y[i] * s;
    yr[i] = x[i] * s + y[i] * c;
  }
  return { x: xr, y: yr } as { x: number[]; y: number[] };
}

function findLeftmostIndex(x: number[], y: number[]) {
  let idx = 0; let bestX = x[0]; let bestY = y[0];
  for (let i = 1; i < x.length; i++) {
    if (x[i] < bestX || (x[i] === bestX && y[i] < bestY)) { idx = i; bestX = x[i]; bestY = y[i]; }
  }
  return idx;
}
function rotateStartAtIndex<T>(arr: T[], i0: number): T[] { return arr.slice(i0).concat(arr.slice(0, i0)); }

function placeRoseInsidePage({ n, L_min, esc, rot_deg, W, H, center, margin = 0 }:
  { n: number; L_min: number; esc: number; rot_deg: number; W: number; H: number; center: [number, number]; margin?: number; }) {
  const esc_cmd = Math.min(esc, 1.2);
  const rot_cmd = clamp(rot_deg, -45, 45);
  const base = roseCurve(n, 4000);
  const bb0 = bbox(base.x, base.y);
  const L_cuadrado_base = Math.max(bb0.xmax - bb0.xmin, bb0.ymax - bb0.ymin);
  const L_req = L_min * esc_cmd;
  const s0 = L_req / L_cuadrado_base;
  const xs = base.x.map((v) => v * s0);
  const ys = base.y.map((v) => v * s0);
  const rot = rotateXY(xs, ys, rot_cmd);
  const bbR = bbox(rot.x, rot.y);
  const wR = bbR.xmax - bbR.xmin, hR = bbR.ymax - bbR.ymin;
  const wAllow = Math.max(1e-9, W - 2 * margin), hAllow = Math.max(1e-9, H - 2 * margin);
  const f_page = Math.min(1, wAllow / wR, hAllow / hR);
  const xr2 = rot.x.map((v) => v * f_page + center[0]);
  const yr2 = rot.y.map((v) => v * f_page + center[1]);
  const bbF = bbox(xr2, yr2);
  const L_square_eff = Math.max(bbF.xmax - bbF.xmin, bbF.ymax - bbF.ymin);
  const esc_eff = L_square_eff / L_min;
  const info = { L_req, L_eff: L_square_eff, esc_cmd, esc_eff, rot_cmd, margin, f_page, w_final: bbF.xmax - bbF.xmin, h_final: bbF.ymax - bbF.ymin, L_square_eff };
  return { x: xr2, y: yr2, info } as { x: number[]; y: number[]; info: any };
}

function resampleConstantSpeed(x: number[], y: number[], v = 0.1, dt = 0.02) {
  const n = x.length;
  const ds = new Array<number>(n - 1);
  for (let i = 0; i < n - 1; i++) ds[i] = Math.hypot(x[i + 1] - x[i], y[i + 1] - y[i]);
  const s = new Array<number>(n); s[0] = 0; for (let i = 1; i < n; i++) s[i] = s[i - 1] + ds[i - 1];
  const total = s[n - 1]; if (total <= 0) return { xn: x.slice(), yn: y.slice(), t: [0] } as any;
  const step = v * dt; const nSteps = Math.max(2, Math.ceil(total / step));
  const sNew = new Array<number>(nSteps); for (let i = 0; i < nSteps; i++) sNew[i] = (i * total) / (nSteps - 1);
  const xn = new Array<number>(nSteps), yn = new Array<number>(nSteps);
  let j = 0;
  for (let i = 0; i < nSteps; i++) {
    const si = sNew[i]; while (j < n - 1 && s[j + 1] < si) j++;
    const s0 = s[j], s1 = s[j + 1] ?? s[j]; const w = s1 > s0 ? (si - s0) / (s1 - s0) : 0;
    xn[i] = x[j] * (1 - w) + x[j + 1] * w; yn[i] = y[j] * (1 - w) + y[j + 1] * w;
  }
  const t = new Array<number>(nSteps); for (let i = 0; i < nSteps; i++) t[i] = i * dt;
  return { xn, yn, t } as { xn: number[]; yn: number[]; t: number[] };
}

function jacobian2R(theta1: number, theta2: number, l1: number, l2: number) {
  const s1 = Math.sin(theta1), c1 = Math.cos(theta1);
  const s12 = Math.sin(theta1 + theta2), c12 = Math.cos(theta1 + theta2);
  return [
    [-l1 * s1 - l2 * s12, -l2 * s12],
    [l1 * c1 + l2 * c12, l2 * c12],
  ];
}
function manipulabilityIndex(J: number[][]) {
  const a = J[0][0], b = J[0][1], c = J[1][0], d = J[1][1];
  const JJT00 = a * a + b * b, JJT01 = a * c + b * d, JJT11 = c * c + d * d;
  const det = JJT00 * JJT11 - JJT01 * JJT01; return Math.sqrt(Math.max(0, det));
}
function condNumber(J: number[][]) {
  const a = J[0][0], b = J[0][1], c = J[1][0], d = J[1][1];
  const JTJ00 = a * a + c * c, JTJ01 = a * b + c * d, JTJ11 = b * b + d * d;
  const tr = JTJ00 + JTJ11, det = JTJ00 * JTJ11 - JTJ01 * JTJ01;
  const disc = Math.max(0, tr * tr - 4 * det);
  const s1 = Math.sqrt((tr + Math.sqrt(disc)) / 2), s2 = Math.sqrt((tr - Math.sqrt(disc)) / 2);
  if (s2 <= 1e-12) return Infinity; return s1 / s2;
}

function gravityTorques(theta1: number, theta2: number, l1: number, l2: number, m1: number, m2: number, m_s2: number, m_tip: number, g = 9.81) {
  const c1 = Math.cos(theta1), c12 = Math.cos(theta1 + theta2);
  const tau_g2 = -g * ((m2 * (l2 / 2) + m_tip * l2) * c12);
  const term1 = (m1 * (l1 / 2) + (m2 + m_s2 + m_tip) * l1) * c1;
  const term2 = (m2 * (l2 / 2) + m_tip * l2) * c12;
  const tau_g1 = -g * (term1 + term2); return [tau_g1, tau_g2];
}
function inertiaMatrixSimplified(theta2: number, l1: number, l2: number, m1: number, m2: number, m_tip: number) {
  const I1 = (m1 * l1 * l1) / 3, I2 = (m2 * l2 * l2) / 3, c2 = Math.cos(theta2);
  const M11 = I1 + I2 + m2 * (l1 * l1 + (l2 * l2) / 4 + (l1 * l2 * c2) / 2) + m_tip * (l1 * l1 + l2 * l2 + 2 * l1 * l2 * c2);
  const M12 = I2 + m2 * ((l2 * l2) / 4 + (l1 * l2 * c2) / 2) + m_tip * (l2 * l2 + l1 * l2 * c2);
  const M22 = I2 + m2 * ((l2 * l2) / 4) + m_tip * (l2 * l2);
  return [[M11, M12], [M12, M22]];
}
function mulMatVec(M: number[][], v: number[]) { return [M[0][0] * v[0] + M[0][1] * v[1], M[1][0] * v[0] + M[1][1] * v[1]]; }

// Build a filled wedge path for an angle arc between a0 and a1 around (cx,cy)
function arcWedgeD(cx: number, cy: number, r: number, a0: number, a1: number, steps = 32) {
  if (!Number.isFinite(a0 + a1) || r <= 0) return '';
  // Normalize to shortest signed sweep from a0 to a1
  let dth = a1 - a0;
  while (dth > Math.PI) { a0 += 2 * Math.PI; dth = a1 - a0; }
  while (dth < -Math.PI) { a0 -= 2 * Math.PI; dth = a1 - a0; }
  const n = Math.max(2, Math.min(steps, Math.ceil(Math.abs(dth) / (Math.PI / 32))));
  const pts: [number, number][] = [];
  for (let i = 0; i <= n; i++) {
    const t = i / n; const a = a0 + dth * t; pts.push([cx + r * Math.cos(a), cy + r * Math.sin(a)]);
  }
  let d = `M ${cx},${cy} L ${pts[0][0]},${pts[0][1]} `;
  for (let i = 1; i < pts.length; i++) d += `L ${pts[i][0]},${pts[i][1]} `;
  d += 'Z';
  return d;
}

function angleLabelPos(cx: number, cy: number, r: number, a0: number, a1: number) {
  // Mid-angle, adjusted for wrap
  let m = (a0 + a1) / 2; let dth = a1 - a0; if (dth > Math.PI) m += Math.PI; if (dth < -Math.PI) m -= Math.PI;
  const x = cx + r * Math.cos(m), y = cy + r * Math.sin(m);
  return { x, y };
}

function buildScenario(params: any) {
  const { hoja, L_min, esc, rot_deg, n_pet, dx, dy, l1, l2, v_tip, dt, elbow, margin, masas } = params;
  const hl = hoja.toLowerCase();
  const W = hl === "a3" ? 0.420 : hl === "a4" ? 0.297 : 0.279;
  const H = hl === "a3" ? 0.297 : hl === "a4" ? 0.210 : 0.216;
  const center: [number, number] = [dx + W / 2, dy + H / 2];

  const placed = placeRoseInsidePage({ n: n_pet, L_min, esc, rot_deg, W, H, center, margin });

  const idxLeft = findLeftmostIndex(placed.x, placed.y);
  const xLeftStart = rotateStartAtIndex(placed.x, idxLeft);
  const yLeftStart = rotateStartAtIndex(placed.y, idxLeft);

  const reach = l1 + l2 - 0.005;
  const homeGap = Math.max(0.01, (margin || 0) + 0.02);
  let homeX = dx - homeGap; homeX = Math.max(homeX, -reach);
  const homeY = yLeftStart[0];

  const trajGuide = xLeftStart.map((x, i) => ({ x, y: yLeftStart[i] }));
  const fullX = [homeX, ...xLeftStart];
  const fullY = [homeY, ...yLeftStart];
  const { xn, yn, t } = resampleConstantSpeed(fullX, fullY, v_tip, dt);
  const N = xn.length;

  const th1 = new Array<number>(N).fill(NaN), th2 = new Array<number>(N).fill(NaN);
  let valid = 0; for (let i = 0; i < N; i++) { const sol = ik2R(xn[i], yn[i], l1, l2, elbow); if (sol) { th1[i] = sol.theta1; th2[i] = sol.theta2; valid++; } }

  const dth1 = finiteDiff(th1, dt), dth2 = finiteDiff(th2, dt);
  const ddth1 = finiteDiff(dth1, dt), ddth2 = finiteDiff(dth2, dt);

  const tau1 = new Array<number>(N).fill(NaN), tau2 = new Array<number>(N).fill(NaN);
  const mu = new Array<number>(N).fill(NaN), kappa = new Array<number>(N).fill(NaN);
  for (let i = 0; i < N; i++) {
    if (Number.isFinite(th1[i] + th2[i])) {
      const J = jacobian2R(th1[i], th2[i], l1, l2);
      mu[i] = manipulabilityIndex(J); kappa[i] = condNumber(J);
      const [g1, g2] = gravityTorques(th1[i], th2[i], l1, l2, masas.m1, masas.m2, masas.m_s2, masas.m_tip);
      const M = inertiaMatrixSimplified(th2[i], l1, l2, masas.m1, masas.m2, masas.m_tip);
      const tor = mulMatVec(M, [ddth1[i], ddth2[i]]);
      tau1[i] = g1 + tor[0]; tau2[i] = g2 + tor[1];
    }
  }

  const bb = bbox(xn.concat([0]), yn.concat([0]));
  const pad = 0.04; // más padding general
  // Más margen a la izquierda para evitar cortes y facilitar centrado visual
  const extraLeft = 0.06;
  const view = {
    xmin: Math.min(bb.xmin, dx) - pad - extraLeft,
    xmax: Math.max(bb.xmax, dx + W) + pad,
    ymin: Math.min(bb.ymin, dy) - pad,
    ymax: Math.max(bb.ymax, dy + H) + pad
  };

  return { W, H, center, placed, traj: xn.map((x: number, i: number) => ({ x, y: yn[i] })), trajGuide, t, th1, th2, dth1, dth2, ddth1, ddth2, tau1, tau2, mu, kappa, view, validPoints: valid, meta: { idxLeft, home: { x: homeX, y: homeY } } };
}

function nearly(a: number, b: number, eps = 1e-6) { return Math.abs(a - b) <= eps; }
function runUnitTests(p: any) {
  const results: { name: string; pass: boolean; details?: string }[] = [];
  const f1 = fk2R(0, 0, p.l1, p.l2);
  results.push({ name: "FK(0,0) en x = l1+l2, y = 0", pass: nearly(f1.x, p.l1 + p.l2, 1e-9) && nearly(f1.y, 0, 1e-9), details: `x=${f1.x.toFixed(4)}, y=${f1.y.toFixed(4)}` });
  const ik = ik2R(p.l1 + p.l2, 0, p.l1, p.l2, "up");
  results.push({ name: "IK(l1+l2,0) devuelve θ1≈0, θ2≈0", pass: !!ik && Math.abs(ik.theta1) < 1e-6 && Math.abs(ik.theta2) < 1e-6, details: ik ? `θ1=${ik.theta1}, θ2=${ik.theta2}` : "sin solución" });
  const hl = p.hoja.toLowerCase();
  const W = hl === "a3" ? 0.420 : hl === "a4" ? 0.297 : 0.279;
  const H = hl === "a3" ? 0.297 : hl === "a4" ? 0.210 : 0.216;
  const placed = placeRoseInsidePage({ n: p.n_pet, L_min: p.L_min, esc: p.esc, rot_deg: p.rot_deg, W, H, center: [p.dx + W / 2, p.dy + H / 2], margin: p.margin });
  results.push({ name: "placeRoseInsidePage usa esc_cmd ≤ 1.2", pass: (placed as any).info.esc_cmd <= 1.2, details: `esc_cmd=${(placed as any).info.esc_cmd}` });
  const rr = resampleConstantSpeed([0, 1], [0, 0], 0.1, 0.02); const mono = rr.t.every((v: number, i: number, a: number[]) => i === 0 || v >= a[i - 1]);
  results.push({ name: "resampleConstantSpeed genera t no decreciente", pass: mono, details: `len=${rr.t.length}` });
  const J = jacobian2R(0.1, -0.2, p.l1, p.l2); results.push({ name: "Jacobian 2x2 válido", pass: (J as any).length === 2 && (J as any)[0].length === 2 && (J as any)[1].length === 2 });
  const xx = [0.2, 0.1, -0.3, 0.4, -0.1], yy = [0, 0.05, 0.01, -0.02, 0.02];
  const idxL = findLeftmostIndex(xx, yy); const xrot = rotateStartAtIndex(xx, idxL);
  results.push({ name: "Arranque a la izquierda (min x primero)", pass: xrot[0] === Math.min(...xx), details: `idx=${idxL}, x0=${xrot[0]}` });
  return results;
}

const DEFAULTS = { hoja: "A4", L_min: 0.15, esc: 1.0, rot_deg: 45, n_pet: 3, dx: 0.05, dy: 0.13, l1: 0.22, l2: 0.22, v_tip: 0.05, dt: 0.02, elbow: "down" as "up" | "down", margin: 0.005, masas: { m1: 0.05, m2: 0.045, m_s2: 0.055, m_tip: 0.05 }, };
type Defaults = typeof DEFAULTS;

function Controls({ p, setP, onRun, running }: { p: Defaults; setP: (nv: any) => void; onRun: ()=>void; running: boolean }) {
  return (
    <div className="mx-auto max-w-[1100px] bg-white border rounded-2xl shadow-sm p-4">
      <div className="grid grid-cols-1 md:grid-cols-4 gap-3">
        <div className="md:col-span-2">
          <div className="section-title">Hoja y forma</div>
          <div className="flex items-center gap-2">
            <label className="flex flex-col items-start gap-2">Hoja
              <select className="input input-bordered select select-sm rounded-xl border p-2" value={p.hoja} onChange={(e) => setP({ ...p, hoja: (e.target as HTMLSelectElement).value })}>
                <option value="A3">A3</option>
                <option value="A4">A4</option>
                <option value="Carta">Carta</option>
              </select>
            </label>
            <div className="flex flex-col items-start">
              <Num label="n pétalos" v={p.n_pet} f={(v) => setP({ ...p, n_pet: parseInt(v || "0") })} />
            </div>
          </div>
        </div>

        <div className="md:col-span-1">
          <div className="section-title">Escala/rotación</div>
          <Num label="L_min [m]" v={p.L_min} f={(v) => setP({ ...p, L_min: parseFloat(v) })} />
          <Num label="esc (≤1.2)" v={p.esc} f={(v) => setP({ ...p, esc: parseFloat(v) })} />
          <Num label="rot [°] (±45)" v={p.rot_deg} f={(v) => setP({ ...p, rot_deg: parseFloat(v) })} />
        </div>

        <div className="md:col-span-1">
          <div className="section-title">Posición hoja</div>
          <Num label="dx [m]" v={p.dx} f={(v) => setP({ ...p, dx: parseFloat(v) })} />
          <Num label="dy [m]" v={p.dy} f={(v) => setP({ ...p, dy: parseFloat(v) })} />
          <Num label="margen [m]" v={p.margin} f={(v) => setP({ ...p, margin: parseFloat(v) })} />
        </div>

        <div className="md:col-span-1">
          <div className="section-title">Geometría 2R</div>
          <Num label="l1 [m]" v={p.l1} f={(v) => setP({ ...p, l1: parseFloat(v) })} />
          <Num label="l2 [m]" v={p.l2} f={(v) => setP({ ...p, l2: parseFloat(v) })} />
          <label className="flex items-center gap-2">Codo
            <select className="input input-bordered select select-sm rounded-xl border p-2" value={p.elbow} onChange={(e) => setP({ ...p, elbow: (e.target as HTMLSelectElement).value as "up" | "down" })}>
              <option value="up">up</option>
              <option value="down">down</option>
            </select>
          </label>
        </div>

        <div className="md:col-span-1">
          <div className="section-title">Dinámica</div>
          <Num label="v_tip [m/s]" v={p.v_tip} f={(v) => setP({ ...p, v_tip: parseFloat(v) })} />
          <Num label="dt [s]" v={p.dt} f={(v) => setP({ ...p, dt: parseFloat(v) })} />
        </div>

        <div className="md:col-span-4">
          <div className="section-title">Masas (kg)</div>
          <div className="flex items-center gap-3 flex-wrap">
            <Num label="m1" v={p.masas.m1} f={(v) => setP({ ...p, masas: { ...p.masas, m1: parseFloat(v) } })} />
            <Num label="m2" v={p.masas.m2} f={(v) => setP({ ...p, masas: { ...p.masas, m2: parseFloat(v) } })} />
            <Num label="m_s2" v={p.masas.m_s2} f={(v) => setP({ ...p, masas: { ...p.masas, m_s2: parseFloat(v) } })} />
            <Num label="m_tip" v={p.masas.m_tip} f={(v) => setP({ ...p, masas: { ...p.masas, m_tip: parseFloat(v) } })} />
          </div>
        </div>

      </div>
      <div className="flex gap-3 mt-4 justify-center">
        <button onClick={onRun} disabled={running} className="btn btn-primary disabled:opacity-50">{running ? "Simulando..." : "Simular"}</button>
        <button onClick={() => setP({ ...DEFAULTS })} className="btn">Reset</button>
      </div>
    </div>
  );
}

function Num({ label, v, f }: { label: string; v: number | string; f: (val: string)=>void }) {
  return (
    <label className="flex items-center gap-2">
      <span className="text-sm whitespace-nowrap">{label}</span>
      <input className="border rounded-xl p-2 w-28" type="number" step="any" value={v as any} onChange={(e) => f((e.target as HTMLInputElement).value)} />
    </label>
  );
}

function ChartBox({ title, data, lines }: { title: string; data: any[]; lines: React.ReactNode }) {
  return (
    <div className="w-full h-64 rounded-2xl border p-3 shadow-sm bg-white">
      <div className="font-semibold mb-2">{title}</div>
      <ResponsiveContainer width="100%" height="100%">
        <LineChart data={data} margin={{ top: 10, right: 20, bottom: 10, left: 0 }}>
          <CartesianGrid strokeDasharray="3 3" />
          <XAxis dataKey="t" tickFormatter={(v:any)=>Number(v).toFixed(1)} />
          <YAxis />
          <Tooltip formatter={(v:any)=>typeof v==="number"?v.toFixed(3):v} />
          <Legend />
          {lines}
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}

export default function TwoRWebSimulator() {
  const [p, setP] = useState(DEFAULTS);
  const [sim, setSim] = useState<any>(null);
  const [running, setRunning] = useState(false);
  const [paused, setPaused] = useState(false);
  const [theme, setTheme] = useState<string>(() => (localStorage.getItem("theme") || "light"));
  const [frame, setFrame] = useState(0);
  const rafRef = useRef<number>(0);
  const startTs = useRef<number>(0);
  const pausedElapsed = useRef<number>(0); // segundos acumulados antes de pausar

  const built = useMemo(() => (sim ? sim : null), [sim]);

  useEffect(() => {
    document.documentElement.setAttribute("data-theme", theme === "dark" ? "dark" : "light");
    localStorage.setItem("theme", theme);
  }, [theme]);

  const viewBox = useMemo(() => {
    if (!built) return "0 0 1 1";
    const v = built.view; const w = v.xmax - v.xmin; const h = v.ymax - v.ymin;
    return `${v.xmin} ${-v.ymax} ${w} ${h}`;
  }, [built]);

  function runOnce() {
    if (!(p.v_tip >= 0.01 && p.v_tip <= 0.1)) { alert("v_tip debe estar entre 0.01 y 0.10 m/s"); return; }
    const scenario = buildScenario(p);
    setSim(scenario);
    setFrame(0);
    setRunning(true);
    setPaused(false);
    startTs.current = 0;
    pausedElapsed.current = 0;
  }

  function togglePause() {
    if (!sim || !running) return;
    if (!paused) {
      // Entrar en pausa: memoriza tiempo y detiene RAF
      setPaused(true);
      pausedElapsed.current = sim.t[Math.min(frame, sim.t.length - 1)] || 0;
      cancelAnimationFrame(rafRef.current);
    } else {
      // Reanudar: reinicia startTs con compensación en el próximo tick
      setPaused(false);
      startTs.current = 0;
      rafRef.current = requestAnimationFrame(tick);
    }
  }

  function tick(ts: number) {
    if (!sim || !running) return;
    if (paused) return;
    const { t } = sim; const N = t.length;
    if (!startTs.current) startTs.current = ts - pausedElapsed.current * 1000;
    const elapsed = (ts - startTs.current) / 1000;
    pausedElapsed.current = elapsed;
    let lo = 0, hi = N - 1;
    while (lo < hi) { const mid = (lo + hi) >> 1; if (t[mid] < elapsed) lo = mid + 1; else hi = mid; }
    const i = lo; setFrame(i);
    if (i >= N - 1) { setRunning(false); cancelAnimationFrame(rafRef.current); return; }
    rafRef.current = requestAnimationFrame(tick);
  }

  useEffect(() => {
    if (!running || !built) return;
    if (!paused) {
      rafRef.current = requestAnimationFrame(tick);
    }
    return () => cancelAnimationFrame(rafRef.current);
  }, [running, built, paused]);

  const chartsData = useMemo(() => {
    if (!built) return null as any; const n = built.t.length; const data = new Array(n);
    for (let i = 0; i < n; i++) {
      data[i] = { t: built.t[i], th1: deg(built.th1[i] || 0), th2: deg(built.th2[i] || 0), w1: deg(built.dth1[i] || 0), w2: deg(built.dth2[i] || 0), a1: deg(built.ddth1[i] || 0), a2: deg(built.ddth2[i] || 0), mu: built.mu[i] || 0, kappa: built.kappa[i] || 0, tau1: built.tau1[i] || 0, tau2: built.tau2[i] || 0 };
    }
    return data as any[];
  }, [built]);

  // Pruebas removidas

  const gridNodes = useMemo(() => {
    if (!built) return null as any;
    const v = built.view;
    const nodes: React.ReactNode[] = [];
    const spacingMinor = 0.02;
    const spacingMajor = 0.10;

    const startX = Math.floor(v.xmin / spacingMinor) * spacingMinor;
    for (let x = startX; x <= v.xmax + 1e-9; x += spacingMinor) {
      const major = almostMultiple(x, spacingMajor);
      nodes.push(
        <line key={`gv-${x.toFixed(3)}`} x1={x} y1={v.ymin} x2={x} y2={v.ymax} stroke={major ? COLORS.gridMajor : COLORS.gridMinor} strokeWidth={major ? 0.0012 : 0.0006} />
      );
    }
    const startY = Math.floor(v.ymin / spacingMinor) * spacingMinor;
    for (let y = startY; y <= v.ymax + 1e-9; y += spacingMinor) {
      const major = almostMultiple(y, spacingMajor);
      nodes.push(
        <line key={`gh-${y.toFixed(3)}`} x1={v.xmin} y1={y} x2={v.xmax} y2={y} stroke={major ? COLORS.gridMajor : COLORS.gridMinor} strokeWidth={major ? 0.0012 : 0.0006} />
      );
    }

    nodes.push(<line key="axis-x" x1={v.xmin} y1={0} x2={v.xmax} y2={0} stroke={COLORS.axis} strokeWidth={0.002} />);
    nodes.push(<line key="axis-y" x1={0} y1={v.ymin} x2={0} y2={v.ymax} stroke={COLORS.axis} strokeWidth={0.002} />);

    const tickLen = 0.004;
    for (let x = Math.ceil(v.xmin / spacingMajor) * spacingMajor; x <= v.xmax + 1e-9; x += spacingMajor) {
      nodes.push(<line key={`tx-${x.toFixed(2)}`} x1={x} y1={-tickLen} x2={x} y2={tickLen} stroke={COLORS.axis} strokeWidth={0.002} />);
      nodes.push(
        <g key={`txtx-${x.toFixed(2)}`} transform="scale(1,-1)">
          <text x={x} y={-0.012} fontSize={0.012} textAnchor="middle" fill="#334155">{x.toFixed(2)}</text>
        </g>
      );
    }
    for (let y = Math.ceil(v.ymin / spacingMajor) * spacingMajor; y <= v.ymax + 1e-9; y += spacingMajor) {
      nodes.push(<line key={`ty-${y.toFixed(2)}`} x1={-tickLen} y1={y} x2={tickLen} y2={y} stroke={COLORS.axis} strokeWidth={0.002} />);
      if (Math.abs(y) > 1e-9) {
        nodes.push(
          <g key={`txty-${y.toFixed(2)}`} transform="scale(1,-1)">
            <text x={-0.015} y={-y - 0.002} fontSize={0.012} textAnchor="end" fill="#334155">{y.toFixed(2)}</text>
          </g>
        );
      }
    }

    nodes.push(
      <g key="lbl" transform="scale(1,-1)">
        <text x={v.xmax - 0.02} y={0.02} fontSize={0.014} textAnchor="end" fill="#334155">x [m]</text>
        <text x={0.02} y={-v.ymax + 0.02} fontSize={0.014} textAnchor="start" fill="#334155">y [m]</text>
      </g>
    );

    return nodes;
  }, [built]);

  return (
    <div className="p-4 md:p-8 space-y-6 text-gray-900 min-h-screen">
      <div className="hero rounded-2xl shadow-md p-4 md:p-6 mx-auto max-w-[1100px]">
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-2xl md:text-3xl font-bold">Proyecto - Grupo ServoSapiens</h1>
            <p className="text-sm text-gray-600">Simulador 2R: la hoja se ubica por (dx, dy). La trayectoria se ajusta dentro de la hoja (esc ≤ 1.2, rot ±45°) y se remuestrea a velocidad de punta constante. El trazo inicia desde HOME a la izquierda y entra por el punto más a la izquierda.</p>
          </div>
          <button className="btn" onClick={() => setTheme(theme === "dark" ? "light" : "dark")}>{theme === "dark" ? "Modo claro" : "Modo oscuro"}</button>
        </div>
      </div>

      <Controls p={p} setP={setP} onRun={runOnce} running={running} />

  <div className="rounded-2xl border p-4 shadow-sm bg-white mx-auto max-w-[1100px]" style={{ background: '#ffffff' }}>
        <div className="flex items-center justify-between mb-2">
          <div className="font-semibold">Vista del mecanismo</div>
          {built && (
            <div className="text-sm text-gray-600">t = {built.t[Math.min(frame, built.t.length - 1)].toFixed(2)} s · puntos IK válidos: {built.validPoints}/{built.t.length}</div>
          )}
        </div>
        <div className="w-full overflow-auto relative">
          <div className="absolute right-3 top-3 z-10 glass backdrop-blur-md rounded-xl border shadow-sm p-2 text-xs space-y-1">
            <div className="flex items-center gap-2"><span className="inline-block w-3 h-3 rounded-sm" style={{background: COLORS.trail}} /> Rastro</div>
            <div className="flex items-center gap-2"><span className="inline-block w-3 h-3 rounded-sm" style={{background: COLORS.path}} /> Trayectoria guía</div>
            <div className="flex items-center gap-2"><span className="inline-block w-3 h-3 rounded-sm" style={{background: COLORS.sheet}} /> Hoja</div>
            <div className="flex items-center gap-2"><span className="inline-block w-3 h-3 rounded-sm" style={{background: COLORS.margin}} /> Margen</div>
            <div className="flex items-center gap-2"><span className="inline-block w-3 h-3 rounded-sm" style={{background: COLORS.square}} /> Cuadrado circunscrito</div>
            <div className="flex items-center gap-2"><span className="inline-block w-3 h-3 rounded-sm" style={{background: COLORS.workspaceOuter}} /> Área de trabajo</div>
          </div>

          {running && (
            <div className="absolute left-3 top-3 z-10">
              <button onClick={togglePause} className="btn btn-secondary">{paused ? 'Continuar' : 'Pausa'}</button>
            </div>
          )}

          <svg className="w-full max-w-[1000px]" style={{ display: 'block', margin: '0 auto' }} viewBox={built ? viewBox : "0 0 1 1"}>
            <g transform="scale(1,-1)">
              {built && (
                <rect x={built.view.xmin} y={built.view.ymin} width={built.view.xmax - built.view.xmin} height={built.view.ymax - built.view.ymin} fill="#ffffff" />
              )}

              {built && gridNodes}

              {built && (
                <g>
                  <circle cx={0} cy={0} r={p.l1 + p.l2} fill="none" stroke={COLORS.workspaceOuter} strokeWidth={0.0012} strokeDasharray="0.008 0.008" />
                  <circle cx={0} cy={0} r={Math.abs(p.l1 - p.l2)} fill="none" stroke={COLORS.workspaceInner} strokeWidth={0.0012} strokeDasharray="0.008 0.008" />
                </g>
              )}

              {built && (
                <rect x={p.dx} y={p.dy} width={built.W} height={built.H} fill="none" strokeWidth={0.002} stroke={COLORS.sheet} />
              )}

              {built && p.margin > 0 && (
                <rect x={p.dx + p.margin} y={p.dy + p.margin} width={built.W - 2 * p.margin} height={built.H - 2 * p.margin} fill="none" strokeWidth={0.001} stroke={COLORS.margin} strokeDasharray="0.006 0.006" />
              )}

              {built && (
                (() => {
                  const L = built.placed.info.L_square_eff as number;
                  const [cx, cy] = built.center as [number, number];
                  return <rect x={cx - L / 2} y={cy - L / 2} width={L} height={L} fill="none" stroke={COLORS.square} strokeWidth={0.0012} strokeDasharray="0.006 0.006" />;
                })()
              )}

              {built && (
                <polyline fill="none" stroke={COLORS.path} strokeWidth={0.0018} points={built.trajGuide.map((pt: any) => `${pt.x},${pt.y}`).join(" ")} />
              )}

              {built && (
                <polyline fill="none" stroke={COLORS.trail} strokeWidth={0.003} strokeLinecap="round" points={built.traj.slice(0, frame + 1).map((pt: any) => `${pt.x},${pt.y}`).join(" ")} />
              )}

              {built && (
                <circle cx={built.meta.home.x} cy={built.meta.home.y} r={0.0045} fill="#10B981" />
              )}

              {built && (
                (() => {
                  const i = Math.min(frame, built.traj.length - 1);
                  const th1 = built.th1[i]; const th2 = built.th2[i]; if (!Number.isFinite(th1 + th2)) return null;
                  const x1 = p.l1 * Math.cos(th1), y1 = p.l1 * Math.sin(th1);
                  const x2 = x1 + p.l2 * Math.cos(th1 + th2), y2 = y1 + p.l2 * Math.sin(th1 + th2);
                  return (
                    <g>
                      <circle cx={0} cy={0} r={0.0045} fill={COLORS.arm} />
                      <line x1={0} y1={0} x2={x1} y2={y1} stroke={COLORS.arm} strokeWidth={0.0045} strokeLinecap="round" />
                      <circle cx={x1} cy={y1} r={0.0055} fill={COLORS.joints} />
                      <line x1={x1} y1={y1} x2={x2} y2={y2} stroke={COLORS.arm} strokeWidth={0.0045} strokeLinecap="round" />
                      <circle cx={x2} cy={y2} r={0.0045} fill={COLORS.arm} />

                      {/* Angle wedges and labels */}
                      {(() => {
                        const r1 = Math.max(0.012, Math.min(0.20 * p.l1, 0.04));
                        const r2 = Math.max(0.010, Math.min(0.20 * p.l2, 0.035));
                        const d1 = arcWedgeD(0, 0, r1, 0, th1);
                        const d2 = arcWedgeD(x1, y1, r2, th1, th1 + th2);
                        const m1 = angleLabelPos(0, 0, r1 * 0.8, 0, th1);
                        const m2 = angleLabelPos(x1, y1, r2 * 0.85, th1, th1 + th2);
                        return (
                          <>
                            {d1 && <path d={d1} fill={COLORS.angle1} fillOpacity={0.18} stroke={COLORS.angle1} strokeOpacity={0.5} strokeWidth={0.0012} />}
                            {d2 && <path d={d2} fill={COLORS.angle2} fillOpacity={0.18} stroke={COLORS.angle2} strokeOpacity={0.5} strokeWidth={0.0012} />}
                            <g transform="scale(1,-1)">
                              <text x={m1.x} y={-m1.y} fontSize={0.012} textAnchor="middle" fill="#334155">θ1 {deg(th1).toFixed(1)}°</text>
                              <text x={m2.x} y={-m2.y} fontSize={0.012} textAnchor="middle" fill="#334155">θ2 {deg(th2).toFixed(1)}°</text>
                            </g>
                          </>
                        );
                      })()}
                    </g>
                  );
                })()
              )}
            </g>
          </svg>
        </div>
        {running && <div className="text-xs text-gray-500 mt-2">Simulando en tiempo real… al terminar verás las gráficas.</div>}
      </div>

      {built && !running && chartsData && (
        <div className="space-y-6 mx-auto max-w-[1100px]">
          <div className="grid md:grid-cols-2 gap-4">
            <ChartBox title="Ángulos articulares [deg] vs t" data={chartsData} lines={<><Line type="monotone" dataKey="th1" name="θ1" dot={false} stroke={COLORS.angle1} /><Line type="monotone" dataKey="th2" name="θ2" dot={false} stroke={COLORS.angle2} /></>} />
            <ChartBox title="Velocidades angulares [deg/s] vs t" data={chartsData} lines={<><Line type="monotone" dataKey="w1" name="ω1" dot={false} stroke={COLORS.angle1} /><Line type="monotone" dataKey="w2" name="ω2" dot={false} stroke={COLORS.angle2} /></>} />
            <ChartBox title="Aceleraciones angulares [deg/s²] vs t" data={chartsData} lines={<><Line type="monotone" dataKey="a1" name="α1" dot={false} stroke={COLORS.angle1} /><Line type="monotone" dataKey="a2" name="α2" dot={false} stroke={COLORS.angle2} /></>} />
            <ChartBox title="Pares articulares [N·m] vs t" data={chartsData} lines={<><Line type="monotone" dataKey="tau1" name="τ1" dot={false} stroke={COLORS.angle1} /><Line type="monotone" dataKey="tau2" name="τ2" dot={false} stroke={COLORS.angle2} /></>} />
          </div>

          <div className="text-sm text-gray-600">Consejo: si alguna sección de la trayectoria no es alcanzable (IK devuelve NaN), prueba bajar <b>esc</b>, cambiar <b>dx/dy</b>, o alternar <b>codo</b> (up/down).</div>
        </div>
      )}
    </div>
  );
}
