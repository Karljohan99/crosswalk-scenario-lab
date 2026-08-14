'use strict';

/* ---------------- state ---------------- */

const QUANTITIES = [
  { id: 'approach_angle', desc: 'ped heading vs direction to nearest path point (deg, 0–180; 0 = straight at the path)' },
  { id: 'trajectory_approach_angle', desc: 'same angle at the predicted trajectory’s crosswalk entry point; None if prediction misses the crosswalk' },
  { id: 'crosswalk_angle', desc: 'crossing axis vs road direction (deg, 0–90; 90 = perpendicular crosswalk)' },
  { id: 'heading_to_crosswalk_angle', desc: 'ped heading vs crossing axis direction (deg, 0–180)' },
  { id: 'distance_to_path', desc: 'ped footprint to ego path centerline (m)' },
  { id: 'distance_to_crosswalk', desc: 'ped footprint to crosswalk polygon (m, 0 if touching)' },
  { id: 'on_crosswalk', desc: 'ped footprint intersects the crosswalk polygon (bool)' },
  { id: 'prediction_hits_crosswalk', desc: 'predicted trajectory buffer intersects the crosswalk (bool)' },
  { id: 'prediction_length', desc: 'speed × horizon (m)' },
  { id: 'speed', desc: 'pedestrian speed (m/s)' },
  { id: 'wide_safety_box_width', desc: 'config param, default 3.1 m — departing peds within half this of the path still block' },
  { id: 'is_pedestrian', desc: 'True for the pedestrian, False for the other vehicle — the rule runs once per object (bool)' },
];

const DEFAULT_RULE = `# Current production rule (collision_checker.py: _crosswalk_is_approaching_or_departing)
# The real checker applies it to approach_angle when the ped is ON the
# crosswalk, and to trajectory_approach_angle when only its prediction
# crosses it.

if on_crosswalk:
    angle = approach_angle
elif prediction_hits_crosswalk and trajectory_approach_angle is not None:
    angle = trajectory_approach_angle
else:
    return False

if angle < 60:
    return True
if 180 - angle < 60 and distance_to_path < wide_safety_box_width / 2:
    return True
return False`;

const DEFAULT_PARAMS = {
  curvature: 0, cwDist: 30, cwAngleDeg: 0, cwLen: 12, cwWid: 4,
  pedX: 30, pedY: 7, pedHeadingDeg: -90, speed: 1.4, horizon: 3.0, wideBox: 3.1,
  vehEnabled: false, vehX: 50, vehY: 3.5, vehHeadingDeg: 180, vehSpeed: 8,
};

const STARTER_SCENARIOS = [
  { name: 'Approaching, straight road', expected: true,
    params: { ...DEFAULT_PARAMS } },
  { name: 'Walking parallel on sidewalk', expected: false,
    params: { ...DEFAULT_PARAMS, pedHeadingDeg: 0 } },
  { name: 'Departed, far side, walking away', expected: false,
    params: { ...DEFAULT_PARAMS, pedY: -7 } },
  { name: 'Departing but still on the road', expected: true,
    params: { ...DEFAULT_PARAMS, pedY: -1 } },
  { name: 'Approaching on curved road', expected: true,
    params: { ...DEFAULT_PARAMS, curvature: 0.02, pedX: 24.28, pedY: 14.51, pedHeadingDeg: -55.6 } },
  { name: 'Cutting diagonally over angled crosswalk', expected: true,
    params: { ...DEFAULT_PARAMS, cwAngleDeg: 30, pedX: 26.5, pedY: 6.5, pedHeadingDeg: -60 } },
  { name: 'Loitering on crosswalk, walking along road', expected: true,
    params: { ...DEFAULT_PARAMS, pedX: 30, pedY: 1.5, pedHeadingDeg: 0, speed: 1.0 } },
  { name: 'Vehicle driving through, ped on sidewalk', expected: false,
    params: { ...DEFAULT_PARAMS, pedHeadingDeg: 0, vehEnabled: true } },
];

let params = { ...DEFAULT_PARAMS };
let varNames = Object.fromEntries(QUANTITIES.map(q => [q.id, q.id]));
let scenarios = STARTER_SCENARIOS.map(s => ({ name: s.name, expected: s.expected, params: { ...s.params } }));
let rules = [{ name: 'production rule', code: DEFAULT_RULE }];
let selectedRule = 0;  // the editor starts with the production rule loaded
let selectedScenario = -1;
let workingParams = null;  // snapshot of the unsaved working scene while a scenario is loaded
let ruleAst = null;
let ruleError = null;
let scene = null;
let ruleResult = null;  // current verdict: true/false, or null on rule error
let expectedForSave = true;

/* ---------------- controls ---------------- */

const CONTROLS = [
  { section: 'Road' },
  { key: 'curvature', label: 'curvature (1/m)', min: -0.05, max: 0.05, step: 0.001, fmt: v => v.toFixed(3) },
  { section: 'Crosswalk' },
  { key: 'cwDist', label: 'distance along path (m)', min: 5, max: 90, step: 0.5, fmt: v => v.toFixed(1) },
  { key: 'cwAngleDeg', label: 'angle vs road normal (°)', min: -45, max: 45, step: 1, fmt: v => v.toFixed(0) },
  { key: 'cwLen', label: 'length across road (m)', min: 6, max: 24, step: 0.5, fmt: v => v.toFixed(1) },
  { key: 'cwWid', label: 'width along road (m)', min: 2, max: 8, step: 0.5, fmt: v => v.toFixed(1) },
  { section: 'Pedestrian' },
  { key: 'pedX', label: 'x (m)', number: true },
  { key: 'pedY', label: 'y (m)', number: true },
  { key: 'pedHeadingDeg', label: 'heading (°)', min: -180, max: 180, step: 1, fmt: v => v.toFixed(0) },
  { key: 'speed', label: 'speed (m/s)', min: 0, max: 3, step: 0.1, fmt: v => v.toFixed(1) },
  { key: 'horizon', label: 'prediction horizon (s)', min: 0, max: 8, step: 0.5, fmt: v => v.toFixed(1) },
  { section: 'Other vehicle (naive prediction)' },
  { key: 'vehEnabled', label: 'enabled', checkbox: true },
  { key: 'vehX', label: 'x (m)', number: true },
  { key: 'vehY', label: 'y (m)', number: true },
  { key: 'vehHeadingDeg', label: 'heading (°)', min: -180, max: 180, step: 1, fmt: v => v.toFixed(0) },
  { key: 'vehSpeed', label: 'speed (m/s)', min: 0, max: 15, step: 0.5, fmt: v => v.toFixed(1) },
  { section: 'Checker params' },
  { key: 'wideBox', label: 'wide_safety_box_width (m)', min: 0, max: 8, step: 0.1, fmt: v => v.toFixed(1) },
];

const controlEls = {};

function buildControls() {
  const host = document.getElementById('controls');
  for (const c of CONTROLS) {
    if (c.section) {
      const h = document.createElement('div');
      h.className = 'hint';
      h.style.fontWeight = '650';
      h.textContent = c.section;
      host.appendChild(h);
      continue;
    }
    const row = document.createElement('div');
    row.className = 'ctl';
    const label = document.createElement('label');
    label.textContent = c.label;
    row.appendChild(label);
    if (c.checkbox) {
      const inp = document.createElement('input');
      inp.type = 'checkbox';
      inp.style.justifySelf = 'start';
      inp.addEventListener('change', () => { params[c.key] = inp.checked; update(); });
      row.appendChild(inp);
      controlEls[c.key] = { set: v => { inp.checked = !!v; } };
    } else if (c.number) {
      const inp = document.createElement('input');
      inp.type = 'number';
      inp.step = '0.1';
      inp.style.gridColumn = '2 / 4';
      inp.addEventListener('input', () => {
        const v = parseFloat(inp.value);
        if (Number.isFinite(v)) { params[c.key] = v; update(); }
      });
      row.appendChild(inp);
      controlEls[c.key] = { set: v => { if (document.activeElement !== inp) inp.value = v.toFixed(2); } };
    } else {
      const inp = document.createElement('input');
      inp.type = 'range';
      inp.min = c.min; inp.max = c.max; inp.step = c.step;
      const val = document.createElement('input');
      val.type = 'number';
      val.min = c.min; val.max = c.max; val.step = c.step;
      inp.addEventListener('input', () => { params[c.key] = parseFloat(inp.value); update(); });
      val.addEventListener('input', () => {
        const v = parseFloat(val.value);
        if (Number.isFinite(v)) { params[c.key] = v; update(); }
      });
      row.appendChild(inp);
      row.appendChild(val);
      controlEls[c.key] = { set: v => {
        inp.value = v;
        if (document.activeElement !== val) val.value = c.fmt(v);
      } };
    }
    host.appendChild(row);
  }
}

function syncControls() {
  for (const c of CONTROLS) if (c.key) controlEls[c.key].set(params[c.key]);
}

/* ---------------- angle visualization toggles ---------------- */

const ANGLE_VIZ = [
  { id: 'approach_angle', color: '--ink2' },
  { id: 'trajectory_approach_angle', color: '--cw-hit' },
  { id: 'crosswalk_angle', color: '--angle-cw' },
  { id: 'heading_to_crosswalk_angle', color: '--angle-hcw' },
];
const angleViz = Object.fromEntries(ANGLE_VIZ.map(a => [a.id, true]));
const angleLabelEls = {};
let angleVizObject = 'ped';   // whose angles the overlays show: 'ped' | 'veh'
let vehRadioEl = null;

function buildAngleToggles() {
  const host = document.getElementById('angleToggles');

  const objRow = document.createElement('div');
  objRow.className = 'tog';
  const caption = document.createElement('span');
  caption.textContent = 'show for:';
  caption.style.font = 'inherit';
  objRow.appendChild(caption);
  for (const [value, text] of [['ped', 'pedestrian'], ['veh', 'other vehicle']]) {
    const lab = document.createElement('label');
    lab.className = 'tog';
    lab.style.margin = '0';
    const radio = document.createElement('input');
    radio.type = 'radio';
    radio.name = 'angleVizObject';
    radio.checked = angleVizObject === value;
    radio.addEventListener('change', () => { angleVizObject = value; draw(); });
    if (value === 'veh') vehRadioEl = radio;
    const span = document.createElement('span');
    span.textContent = text;
    lab.appendChild(radio);
    lab.appendChild(span);
    objRow.appendChild(lab);
  }
  host.appendChild(objRow);
  for (const a of ANGLE_VIZ) {
    const row = document.createElement('label');
    row.className = 'tog';
    const cb = document.createElement('input');
    cb.type = 'checkbox';
    cb.checked = angleViz[a.id];
    cb.addEventListener('change', () => { angleViz[a.id] = cb.checked; draw(); });
    const swatch = document.createElement('span');
    swatch.className = 'swatch';
    swatch.style.background = `var(${a.color})`;
    const name = document.createElement('span');
    angleLabelEls[a.id] = name;
    row.appendChild(cb);
    row.appendChild(swatch);
    row.appendChild(name);
    host.appendChild(row);
  }

}

/* ---------------- variables panel ---------------- */

const IDENT_RE = /^[A-Za-z_]\w*$/;
const varValueEls = {};

function buildVarRows() {
  const host = document.getElementById('varRows');
  for (const q of QUANTITIES) {
    const tr = document.createElement('tr');
    const tdName = document.createElement('td');
    tdName.style.width = '55%';
    const inp = document.createElement('input');
    inp.value = varNames[q.id];
    inp.title = q.desc;
    inp.spellcheck = false;
    inp.addEventListener('input', () => {
      const v = inp.value.trim();
      const taken = QUANTITIES.some(o => o.id !== q.id && varNames[o.id] === v);
      if (IDENT_RE.test(v) && !PY_KEYWORDS.has(v) && !PY_RESERVED.has(v) && !taken) {
        inp.classList.remove('invalid');
        varNames[q.id] = v;
        update();
      } else {
        inp.classList.add('invalid');
      }
    });
    const desc = document.createElement('div');
    desc.className = 'desc';
    desc.textContent = q.desc;
    tdName.appendChild(inp);
    tdName.appendChild(desc);
    const tdVal = document.createElement('td');
    tdVal.className = 'val';
    varValueEls[q.id] = tdVal;
    tr.appendChild(tdName);
    tr.appendChild(tdVal);
    host.appendChild(tr);
  }
}

function syncVarInputs() {
  const inputs = document.querySelectorAll('#varRows input');
  QUANTITIES.forEach((q, i) => { inputs[i].value = varNames[q.id]; inputs[i].classList.remove('invalid'); });
}

function fmtValue(v) {
  if (v === null) return 'None';
  if (v === true) return 'True';
  if (v === false) return 'False';
  return v.toFixed(1);
}

/* ---------------- rule handling ---------------- */

function compileRule() {
  const src = document.getElementById('ruleCode').value;
  try {
    ruleAst = pyParse(src);
    ruleError = null;
  } catch (e) {
    ruleAst = null;
    ruleError = e instanceof PyError ? `line ${e.pyLine}: ${e.message}` : e.message;
  }
}

function bindingsFor(values) {
  const b = {};
  for (const q of QUANTITIES) b[varNames[q.id]] = values[q.id];
  return b;
}

function objectValuesList(sc) {
  return sc.vehValues ? [sc.values, sc.vehValues] : [sc.values];
}

// run the rule once per object; the crosswalk is blocked if any object triggers
function evalRuleAll(sc, ast = ruleAst) {
  if (!ast) return { error: ruleError || 'no rule' };
  let blocked = false, warning = null;
  for (const values of objectValuesList(sc)) {
    try {
      const r = runRule(ast, bindingsFor(values));
      blocked = blocked || r.result;
      warning = warning || r.warning;
    } catch (e) {
      if (e instanceof PyError) return { error: e.pyLine ? `line ${e.pyLine}: ${e.message}` : e.message };
      return { error: e.message };
    }
  }
  return { result: blocked, warning };
}

/* ---------------- scenarios ---------------- */

function renderScenarios() {
  const host = document.getElementById('scenList');
  host.innerHTML = '';

  if (workingParams) {
    const back = document.createElement('div');
    back.className = 'scen-row working';
    back.textContent = '◂ back to working scene (unsaved)';
    back.title = 'Restore the scene you were editing before loading a scenario';
    back.addEventListener('click', () => {
      params = { ...workingParams };
      workingParams = null;
      selectedScenario = -1;
      update();
      fitView();
    });
    host.appendChild(back);
  }

  const scenarioScenes = scenarios.map(s => computeScene(s.params));
  let passCount = 0, total = scenarios.length;
  scenarios.forEach((s, idx) => {
    const res = evalRuleAll(scenarioScenes[idx]);
    const pass = !res.error && res.result === s.expected;
    if (pass) passCount++;

    const row = document.createElement('div');
    row.className = 'scen-row' + (idx === selectedScenario ? ' selected' : '');
    row.addEventListener('click', () => loadScenario(idx));

    const top = document.createElement('div');
    top.className = 'scen-top';
    row.appendChild(top);

    const mark = document.createElement('span');
    mark.className = 'mark ' + (pass ? 'pass' : 'fail');
    mark.textContent = pass ? '✓' : '✗';
    top.appendChild(mark);

    const name = document.createElement('span');
    name.className = 'name';
    name.textContent = s.name;
    name.title = s.name;
    top.appendChild(name);

    const upd = document.createElement('button');
    upd.textContent = '⟳';
    upd.title = 'Overwrite this scenario with the current scene';
    upd.addEventListener('click', e => { e.stopPropagation(); s.params = { ...params }; update(); });
    top.appendChild(upd);

    const del = document.createElement('button');
    del.textContent = '✕';
    del.title = 'Delete scenario';
    del.addEventListener('click', e => {
      e.stopPropagation();
      scenarios.splice(idx, 1);
      if (selectedScenario === idx) selectedScenario = -1;
      else if (selectedScenario > idx) selectedScenario--;
      update();
    });
    top.appendChild(del);

    const status = document.createElement('div');
    status.className = 'scen-status';
    row.appendChild(status);

    const exp = document.createElement('span');
    exp.className = 'pill ' + (s.expected ? 'blocked' : 'clear');
    exp.textContent = 'exp ' + (s.expected ? 'BLOCK' : 'CLEAR');
    exp.title = 'Expected outcome — click to toggle';
    exp.addEventListener('click', e => { e.stopPropagation(); s.expected = !s.expected; update(); });
    status.appendChild(exp);

    const act = document.createElement('span');
    act.className = 'pill ' + (res.error ? 'err' : res.result ? 'blocked' : 'clear');
    act.textContent = res.error ? 'ERR' : res.result ? 'got BLOCK' : 'got CLEAR';
    if (res.error) act.title = res.error;
    status.appendChild(act);

    host.appendChild(row);
  });

  const cls = !total ? '' : passCount === total ? 'all-pass' : 'has-fail';
  const score = document.getElementById('score');
  score.textContent = total ? `${passCount}/${total} scenarios pass` : 'no scenarios';
  score.className = cls;
  const scenScore = document.getElementById('scenScore');
  scenScore.textContent = total ? `${passCount}/${total} pass` : '';
  scenScore.className = cls;

  renderRules(scenarioScenes);
}

function renderRules(scenarioScenes) {
  const host = document.getElementById('ruleList');
  host.innerHTML = '';
  rules.forEach((r, idx) => {
    let score = null, err = null;
    try {
      const ast = pyParse(r.code);
      score = 0;
      scenarios.forEach((s, i) => {
        const res = evalRuleAll(scenarioScenes[i], ast);
        if (!res.error && res.result === s.expected) score++;  // errors count as a miss
      });
    } catch (e) {
      err = e instanceof PyError ? `line ${e.pyLine}: ${e.message}` : e.message;
    }

    const row = document.createElement('div');
    row.className = 'scen-row' + (idx === selectedRule ? ' selected' : '');
    row.title = 'Load this rule into the editor';
    row.addEventListener('click', () => loadRule(idx));

    const top = document.createElement('div');
    top.className = 'scen-top';
    row.appendChild(top);

    const name = document.createElement('span');
    name.className = 'name';
    name.textContent = r.name;
    name.title = r.name;
    top.appendChild(name);

    const scoreEl = document.createElement('span');
    scoreEl.className = 'rule-score ' + (err ? 'err' : scenarios.length && score === scenarios.length ? 'all-pass' : 'has-fail');
    scoreEl.textContent = err ? 'ERR' : `${score}/${scenarios.length}`;
    if (err) scoreEl.title = err;
    top.appendChild(scoreEl);

    const upd = document.createElement('button');
    upd.textContent = '⟳';
    upd.title = 'Overwrite this rule with the editor contents';
    upd.addEventListener('click', e => {
      e.stopPropagation();
      r.code = document.getElementById('ruleCode').value;
      selectedRule = idx;
      update();
    });
    top.appendChild(upd);

    const del = document.createElement('button');
    del.textContent = '✕';
    del.title = 'Delete rule';
    del.addEventListener('click', e => {
      e.stopPropagation();
      rules.splice(idx, 1);
      if (selectedRule === idx) selectedRule = -1;
      else if (selectedRule > idx) selectedRule--;
      update();
    });
    top.appendChild(del);

    host.appendChild(row);
  });
}

function loadRule(idx) {
  selectedRule = idx;
  document.getElementById('ruleCode').value = rules[idx].code;
  document.getElementById('ruleName').value = rules[idx].name;
  compileRule();
  update();
}

function loadScenario(idx) {
  if (selectedScenario === -1) workingParams = { ...params };
  selectedScenario = idx;
  params = { ...scenarios[idx].params };
  document.getElementById('scenName').value = scenarios[idx].name;
  update();
  fitView();
}

/* ---------------- export / import ---------------- */

function exportJson() {
  return JSON.stringify({
    version: 1,
    rule: document.getElementById('ruleCode').value,
    variable_names: { ...varNames },
    rules: rules.map(r => ({ name: r.name, code: r.code })),
    scenarios: scenarios.map(s => ({ name: s.name, expected: s.expected, params: { ...s.params } })),
  }, null, 2);
}

function importJson(text) {
  const data = JSON.parse(text);
  if (typeof data.rule === 'string') document.getElementById('ruleCode').value = data.rule;
  if (data.variable_names) {
    for (const q of QUANTITIES) {
      const v = data.variable_names[q.id];
      if (typeof v === 'string' && IDENT_RE.test(v)) varNames[q.id] = v;
    }
  }
  if (Array.isArray(data.scenarios)) {
    scenarios = data.scenarios.map(s => ({
      name: String(s.name || 'unnamed'),
      expected: !!s.expected,
      params: { ...DEFAULT_PARAMS, ...s.params },
    }));
  }
  if (Array.isArray(data.rules)) {
    rules = data.rules.map(r => ({ name: String(r.name || 'unnamed'), code: String(r.code || '') }));
  }
  selectedRule = -1;
  selectedScenario = -1;
  workingParams = null;
  syncVarInputs();
  compileRule();
  update();
}

/* ---------------- canvas rendering ---------------- */

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const view = { cx: 40, cy: 0, scale: 10 };  // world center + px per meter
let dpr = 1;

function cssVar(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

function w2s(p) {
  return {
    x: canvas.clientWidth / 2 + (p.x - view.cx) * view.scale,
    y: canvas.clientHeight / 2 - (p.y - view.cy) * view.scale,
  };
}
function s2w(sx, sy) {
  return {
    x: view.cx + (sx - canvas.clientWidth / 2) / view.scale,
    y: view.cy - (sy - canvas.clientHeight / 2) / view.scale,
  };
}

function fitView() {
  const d = scene.draw;
  const xs = [], ys = [];
  for (const p of d.pathPts) { xs.push(p.x); ys.push(p.y); }
  for (const p of d.cwPoly) { xs.push(p.x); ys.push(p.y); }
  xs.push(d.ped.x - 5, d.ped.x + 5);
  ys.push(d.ped.y - 5, d.ped.y + 5);
  if (d.veh) {
    xs.push(d.veh.center.x - 6, d.veh.center.x + 6);
    ys.push(d.veh.center.y - 6, d.veh.center.y + 6);
  }
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);
  view.cx = (minX + maxX) / 2;
  view.cy = (minY + maxY) / 2;
  const sx = canvas.clientWidth / Math.max(1, maxX - minX + 14);
  const sy = canvas.clientHeight / Math.max(1, maxY - minY + 14);
  view.scale = Math.max(2, Math.min(sx, sy));
  draw();
}

function polyPath(poly) {
  ctx.beginPath();
  poly.forEach((p, i) => {
    const s = w2s(p);
    i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y);
  });
  ctx.closePath();
}

function linePath(pts) {
  ctx.beginPath();
  pts.forEach((p, i) => {
    const s = w2s(p);
    i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y);
  });
}

function drawArrowHead(tip, heading, sizePx, color) {
  const s = w2s(tip);
  const a = -heading;  // screen y is flipped
  ctx.save();
  ctx.translate(s.x, s.y);
  ctx.rotate(a);
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(-sizePx, -sizePx * 0.45);
  ctx.lineTo(-sizePx, sizePx * 0.45);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
  ctx.restore();
}

function drawRay(origin, heading, lenM, color, dashed) {
  ctx.save();
  if (dashed) ctx.setLineDash([4, 4]);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.2;
  linePath([origin, { x: origin.x + lenM * Math.cos(heading), y: origin.y + lenM * Math.sin(heading) }]);
  ctx.stroke();
  ctx.restore();
}

// arc between two world headings around a world point, radius in meters, with label
function drawAngleArc(center, h1, h2, radiusM, label, color, labelOffsetM = 1.0) {
  let d = (h2 - h1) % (2 * Math.PI);
  if (d > Math.PI) d -= 2 * Math.PI;
  if (d < -Math.PI) d += 2 * Math.PI;
  ctx.beginPath();
  const steps = 24;
  for (let i = 0; i <= steps; i++) {
    const a = h1 + d * i / steps;
    const s = w2s({ x: center.x + radiusM * Math.cos(a), y: center.y + radiusM * Math.sin(a) });
    i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y);
  }
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.stroke();
  const mid = h1 + d / 2;
  const lp = w2s({ x: center.x + (radiusM + labelOffsetM) * Math.cos(mid), y: center.y + (radiusM + labelOffsetM) * Math.sin(mid) });
  ctx.font = '12px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.strokeStyle = cssVar('--surface');
  ctx.lineWidth = 3;
  ctx.strokeText(label, lp.x, lp.y);
  ctx.fillStyle = color;
  ctx.fillText(label, lp.x, lp.y);
}

function offsetPath(pts, k, offset) {
  // offset each sample along its normal (heading + 90°)
  return pts.map(p => ({ x: p.x - Math.sin(p.h) * offset, y: p.y + Math.cos(p.h) * offset }));
}

function draw() {
  if (!scene) return;
  const cw = canvas.clientWidth, ch = canvas.clientHeight;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cw, ch);
  const d = scene.draw;
  const LANE_W = 3.5;  // ego path = right lane center; road spans -LANE_W/2 .. +1.5*LANE_W around it

  // grid (10 m)
  ctx.strokeStyle = cssVar('--grid');
  ctx.lineWidth = 1;
  const w0 = s2w(0, ch), w1 = s2w(cw, 0);
  for (let gx = Math.ceil(w0.x / 10) * 10; gx <= w1.x; gx += 10) {
    const s = w2s({ x: gx, y: 0 });
    ctx.beginPath(); ctx.moveTo(s.x, 0); ctx.lineTo(s.x, ch); ctx.stroke();
  }
  for (let gy = Math.ceil(w0.y / 10) * 10; gy <= w1.y; gy += 10) {
    const s = w2s({ x: 0, y: gy });
    ctx.beginPath(); ctx.moveTo(0, s.y); ctx.lineTo(cw, s.y); ctx.stroke();
  }

  // two-lane road surface; ego drives on the right lane
  const left = offsetPath(d.pathPts, params.curvature, LANE_W * 1.5);
  const right = offsetPath(d.pathPts, params.curvature, -LANE_W / 2);
  ctx.beginPath();
  left.forEach((p, i) => { const s = w2s(p); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); });
  right.slice().reverse().forEach(p => { const s = w2s(p); ctx.lineTo(s.x, s.y); });
  ctx.closePath();
  ctx.fillStyle = cssVar('--road');
  ctx.fill();
  ctx.strokeStyle = cssVar('--road-edge');
  ctx.lineWidth = 1.5;
  linePath(left); ctx.stroke();
  linePath(right); ctx.stroke();

  // dashed center marking between the lanes
  ctx.setLineDash([10, 10]);
  ctx.strokeStyle = cssVar('--zebra');
  ctx.lineWidth = 2;
  linePath(offsetPath(d.pathPts, params.curvature, LANE_W / 2));
  ctx.stroke();
  ctx.setLineDash([]);

  // local path drawn at safety-corridor width, light green like autoware_mini
  ctx.save();
  ctx.lineJoin = 'round';
  ctx.strokeStyle = cssVar('--safety');
  ctx.lineWidth = params.wideBox * view.scale;
  linePath(d.pathPts);
  ctx.stroke();
  ctx.restore();

  // local path centerline
  ctx.strokeStyle = cssVar('--path');
  ctx.lineWidth = 2;
  linePath(d.pathPts);
  ctx.stroke();

  // crosswalk zebra
  const cs = w2s(d.cwPose);
  ctx.save();
  ctx.translate(cs.x, cs.y);
  ctx.rotate(-d.axisH);
  const L = params.cwLen * view.scale, W = params.cwWid * view.scale;
  ctx.fillStyle = cssVar('--road');
  ctx.fillRect(-L / 2, -W / 2, L, W);
  ctx.fillStyle = cssVar('--zebra');
  const stripe = 0.5 * view.scale;
  for (let x = -L / 2 + stripe / 2; x + stripe <= L / 2 + 1; x += stripe * 2) {
    ctx.fillRect(x, -W / 2 + 2, stripe, W - 4);
  }
  ctx.restore();
  // crosswalk outline (highlight when prediction hits or ped on it)
  polyPath(d.cwPoly);
  const blocked = ruleResult === true;
  ctx.strokeStyle = blocked ? cssVar('--bad') : cssVar('--road-edge');
  ctx.lineWidth = blocked ? 2.5 : 1.5;
  ctx.stroke();

  // ego vehicle at path start (front bumper at s = 0)
  const ego = pathPose(params.curvature, -2.25);
  const es = w2s(ego);
  ctx.save();
  ctx.translate(es.x, es.y);
  ctx.rotate(-ego.h);
  const carL = 4.5 * view.scale, carW = 1.9 * view.scale;
  ctx.fillStyle = cssVar('--ink2');
  ctx.beginPath();
  ctx.roundRect(-carL / 2, -carW / 2, carL, carW, 3);
  ctx.fill();
  ctx.fillStyle = cssVar('--surface');
  ctx.font = `${Math.max(9, Math.min(13, view.scale * 1.1))}px system-ui, sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('EGO', 0, 0);
  ctx.restore();

  // path direction arrow
  const tip = d.pathPts[d.pathPts.length - 1];
  drawArrowHead(tip, tip.h, 9, cssVar('--path'));

  // other vehicle + its naive prediction
  if (d.veh) {
    const vcol = cssVar('--veh');
    if (d.veh.buffer) {
      polyPath(d.veh.buffer);
      ctx.fillStyle = cssVar('--veh-pred');
      ctx.fill();
      ctx.strokeStyle = vcol;
      ctx.lineWidth = 1.8;
      linePath(d.veh.predSeg);
      ctx.stroke();
      drawArrowHead(d.veh.predSeg[1], d.veh.heading, 8, vcol);
    }
    if (d.veh.entry) {
      const eS = w2s(d.veh.entry);
      ctx.beginPath();
      ctx.arc(eS.x, eS.y, 4, 0, 2 * Math.PI);
      ctx.fillStyle = vcol;
      ctx.fill();
      ctx.strokeStyle = cssVar('--surface');
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
    const vs = w2s(d.veh.center);
    ctx.save();
    ctx.translate(vs.x, vs.y);
    ctx.rotate(-d.veh.heading);
    ctx.fillStyle = vcol;
    ctx.beginPath();
    ctx.roundRect(-VEH_LENGTH / 2 * view.scale, -VEH_WIDTH / 2 * view.scale, VEH_LENGTH * view.scale, VEH_WIDTH * view.scale, 3);
    ctx.fill();
    ctx.restore();
    const vHandle = vehHandlePos();
    const vFront = { x: d.veh.center.x + Math.cos(d.veh.heading) * VEH_LENGTH / 2,
                     y: d.veh.center.y + Math.sin(d.veh.heading) * VEH_LENGTH / 2 };
    ctx.strokeStyle = vcol;
    ctx.lineWidth = 2;
    linePath([vFront, vHandle]);
    ctx.stroke();
    ctx.fillStyle = cssVar('--surface');
    ctx.font = `${Math.max(9, Math.min(13, view.scale * 1.1))}px system-ui, sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('VEH', vs.x, vs.y);
    drawArrowHead(vHandle, d.veh.heading, 10, vcol);
    const vhS = w2s(vHandle);
    ctx.beginPath();
    ctx.arc(vhS.x, vhS.y, 5, 0, 2 * Math.PI);
    ctx.fillStyle = cssVar('--surface');
    ctx.strokeStyle = vcol;
    ctx.lineWidth = 2;
    ctx.fill();
    ctx.stroke();
  }

  // which object's angles are visualized (falls back to the ped when the vehicle is off)
  const useVeh = angleVizObject === 'veh' && d.veh;
  const subj = useVeh
    ? { center: d.veh.center, heading: d.veh.heading, np: d.veh.np, towardPathH: d.veh.towardPathH,
        entry: d.veh.entry, entryNp: d.veh.entryNp, entryTowardPathH: d.veh.entryTowardPathH,
        clip: d.veh.clip, values: scene.vehValues }
    : { center: d.ped, heading: d.pedH, np: d.np, towardPathH: d.towardPathH,
        entry: d.entry, entryNp: d.entryNp, entryTowardPathH: d.entryTowardPathH,
        clip: d.clip, values: scene.values };

  // crosswalk angle: road direction vs crossing axis at the crosswalk center
  if (angleViz.crosswalk_angle) {
    const col = cssVar('--angle-cw');
    let axisDir = d.axisH;
    if (angDiffDeg(axisDir, d.cwPose.h) > 90) axisDir += Math.PI;  // pick the axis end forming the folded 0-90 angle
    drawRay(d.cwPose, d.cwPose.h, params.cwWid / 2 + 4, col, true);
    drawRay(d.cwPose, axisDir, params.cwLen / 2 + 2, col, true);
    drawAngleArc(d.cwPose, d.cwPose.h, axisDir, params.cwWid / 2 + 2.5,
      `${varNames.crosswalk_angle}=${scene.values.crosswalk_angle.toFixed(0)}°`, col);
  }

  // heading vs crossing axis at the subject object
  if (angleViz.heading_to_crosswalk_angle) {
    const col = cssVar('--angle-hcw');
    drawRay(subj.center, d.axisH, 4.2, col, true);
    drawAngleArc(subj.center, subj.heading, d.axisH, 3.4,
      `${varNames.heading_to_crosswalk_angle}=${subj.values.heading_to_crosswalk_angle.toFixed(0)}°`, col);
  }

  // nearest-point ray + approach angle arc
  if (angleViz.approach_angle) {
    const muted = cssVar('--muted');
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = muted;
    ctx.lineWidth = 1.2;
    linePath([subj.center, subj.np]);
    ctx.stroke();
    ctx.setLineDash([]);
    const npS = w2s(subj.np);
    ctx.beginPath();
    ctx.arc(npS.x, npS.y, 3, 0, 2 * Math.PI);
    ctx.fillStyle = muted;
    ctx.fill();
    drawAngleArc(subj.center, subj.heading, subj.towardPathH, 2.2,
      `${varNames.approach_angle}=${subj.values.approach_angle.toFixed(0)}°`, cssVar('--ink2'));
  }

  // prediction
  if (d.predSeg) {
    polyPath(d.buffer);
    ctx.fillStyle = cssVar('--pred');
    ctx.fill();
    ctx.strokeStyle = cssVar('--ped');
    ctx.lineWidth = 1.8;
    linePath(d.predSeg);
    ctx.stroke();
    drawArrowHead(d.predSeg[1], d.pedH, 8, cssVar('--ped'));
  }

  // trajectory approach angle at the subject's crosswalk entry point: the angle between
  // the trajectory direction and the ray from the entry point to the nearest path point
  if (angleViz.trajectory_approach_angle && subj.entry) {
    const col = cssVar('--cw-hit');
    // prediction ∩ crosswalk overlap area — what makes this crosswalk "hit"
    if (subj.clip && subj.clip.length >= 3) {
      ctx.save();
      ctx.globalAlpha = 0.3;
      polyPath(subj.clip);
      ctx.fillStyle = col;
      ctx.fill();
      ctx.restore();
    }
    // leg 1: trajectory direction at the entry point
    const ahead = { x: subj.entry.x + 2.4 * Math.cos(subj.heading), y: subj.entry.y + 2.4 * Math.sin(subj.heading) };
    ctx.strokeStyle = col;
    ctx.lineWidth = 1.5;
    linePath([subj.entry, ahead]);
    ctx.stroke();
    drawArrowHead(ahead, subj.heading, 7, col);
    // leg 2: dashed ray to the nearest point on the ego path
    drawRay(subj.entry, subj.entryTowardPathH, Math.hypot(subj.entryNp.x - subj.entry.x, subj.entryNp.y - subj.entry.y), col, true);
    const enS = w2s(subj.entryNp);
    ctx.beginPath();
    ctx.arc(enS.x, enS.y, 3, 0, 2 * Math.PI);
    ctx.fillStyle = col;
    ctx.fill();
    // push the label further out when the entry point sits close to the subject's own arc
    const nearSubj = Math.hypot(subj.entry.x - subj.center.x, subj.entry.y - subj.center.y) < 3.5;
    drawAngleArc(subj.entry, subj.heading, subj.entryTowardPathH, 1.4,
      `${varNames.trajectory_approach_angle}=${subj.values.trajectory_approach_angle.toFixed(0)}°`, col,
      nearSubj ? 2.8 : 1.0);
  }

  // pedestrian's entry point marker (the vehicle's is drawn in its own section)
  if (d.entry) {
    const eS = w2s(d.entry);
    ctx.beginPath();
    ctx.arc(eS.x, eS.y, 4, 0, 2 * Math.PI);
    ctx.fillStyle = cssVar('--cw-hit');
    ctx.fill();
    ctx.strokeStyle = cssVar('--surface');
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  // pedestrian footprint + heading arrow with drag handle
  const pedS = w2s(d.ped);
  ctx.beginPath();
  ctx.arc(pedS.x, pedS.y, Math.max(4, PED_RADIUS * view.scale), 0, 2 * Math.PI);
  ctx.fillStyle = cssVar('--ped');
  ctx.fill();
  ctx.strokeStyle = cssVar('--surface');
  ctx.lineWidth = 2;
  ctx.stroke();
  const handle = headingHandlePos();
  ctx.strokeStyle = cssVar('--ped');
  ctx.lineWidth = 2;
  linePath([d.ped, handle]);
  ctx.stroke();
  drawArrowHead(handle, d.pedH, 10, cssVar('--ped'));
  ctx.beginPath();
  const hS = w2s(handle);
  ctx.arc(hS.x, hS.y, 5, 0, 2 * Math.PI);
  ctx.fillStyle = cssVar('--surface');
  ctx.strokeStyle = cssVar('--ped');
  ctx.lineWidth = 2;
  ctx.fill();
  ctx.stroke();
}

function headingHandlePos() {
  const len = Math.max(1.6, 24 / view.scale);
  return {
    x: params.pedX + Math.cos(rad(params.pedHeadingDeg)) * len,
    y: params.pedY + Math.sin(rad(params.pedHeadingDeg)) * len,
  };
}

function vehHandlePos() {
  const len = VEH_LENGTH / 2 + Math.max(1.2, 20 / view.scale);
  return {
    x: params.vehX + Math.cos(rad(params.vehHeadingDeg)) * len,
    y: params.vehY + Math.sin(rad(params.vehHeadingDeg)) * len,
  };
}

/* canvas interaction */
let drag = null;

canvas.addEventListener('mousedown', e => {
  const rect = canvas.getBoundingClientRect();
  const sx = e.clientX - rect.left, sy = e.clientY - rect.top;
  const pedS = w2s({ x: params.pedX, y: params.pedY });
  const hS = w2s(headingHandlePos());
  const vhS = params.vehEnabled ? w2s(vehHandlePos()) : null;
  if (Math.hypot(sx - hS.x, sy - hS.y) < 12) drag = { type: 'heading' };
  else if (Math.hypot(sx - pedS.x, sy - pedS.y) < Math.max(12, PED_RADIUS * view.scale + 4)) drag = { type: 'ped' };
  else if (vhS && Math.hypot(sx - vhS.x, sy - vhS.y) < 12) drag = { type: 'vehHeading' };
  else if (scene.draw.veh && distPointToConvexPoly(s2w(sx, sy), scene.draw.veh.poly) < 6 / view.scale) drag = { type: 'veh' };
  else drag = { type: 'pan', sx, sy, cx: view.cx, cy: view.cy };
  canvas.style.cursor = drag.type === 'pan' ? 'grabbing' : 'move';
});

window.addEventListener('mousemove', e => {
  if (!drag) return;
  const rect = canvas.getBoundingClientRect();
  const sx = e.clientX - rect.left, sy = e.clientY - rect.top;
  if (drag.type === 'pan') {
    view.cx = drag.cx - (sx - drag.sx) / view.scale;
    view.cy = drag.cy + (sy - drag.sy) / view.scale;
    draw();
  } else if (drag.type === 'ped') {
    const w = s2w(sx, sy);
    params.pedX = Math.round(w.x * 100) / 100;
    params.pedY = Math.round(w.y * 100) / 100;
    update();
  } else if (drag.type === 'heading') {
    const w = s2w(sx, sy);
    params.pedHeadingDeg = Math.round(deg(Math.atan2(w.y - params.pedY, w.x - params.pedX)));
    update();
  } else if (drag.type === 'veh') {
    const w = s2w(sx, sy);
    params.vehX = Math.round(w.x * 100) / 100;
    params.vehY = Math.round(w.y * 100) / 100;
    update();
  } else if (drag.type === 'vehHeading') {
    const w = s2w(sx, sy);
    params.vehHeadingDeg = Math.round(deg(Math.atan2(w.y - params.vehY, w.x - params.vehX)));
    update();
  }
});

window.addEventListener('mouseup', () => { drag = null; canvas.style.cursor = 'grab'; });

canvas.addEventListener('wheel', e => {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const sx = e.clientX - rect.left, sy = e.clientY - rect.top;
  const before = s2w(sx, sy);
  view.scale *= e.deltaY < 0 ? 1.15 : 1 / 1.15;
  view.scale = Math.max(1, Math.min(120, view.scale));
  const after = s2w(sx, sy);
  view.cx += before.x - after.x;
  view.cy += before.y - after.y;
  draw();
}, { passive: false });

function resizeCanvas() {
  dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(canvas.clientWidth * dpr);
  canvas.height = Math.round(canvas.clientHeight * dpr);
  draw();
}

/* ---------------- main update ---------------- */

function update() {
  scene = computeScene(params);
  syncControls();

  for (const q of QUANTITIES) {
    const el = varValueEls[q.id];
    el.textContent = scene.vehValues
      ? `${fmtValue(scene.values[q.id])} · ${fmtValue(scene.vehValues[q.id])}`
      : fmtValue(scene.values[q.id]);
    el.title = scene.vehValues ? 'pedestrian · vehicle' : '';
  }
  for (const a of ANGLE_VIZ) angleLabelEls[a.id].textContent = varNames[a.id];
  vehRadioEl.disabled = !params.vehEnabled;
  vehRadioEl.parentElement.style.opacity = params.vehEnabled ? '' : '0.45';

  const res = evalRuleAll(scene);
  ruleResult = res.error ? null : res.result;
  const verdict = document.getElementById('verdict');
  const msg = document.getElementById('ruleMsg');
  if (res.error) {
    verdict.className = 'error';
    verdict.textContent = 'RULE ERROR';
    msg.className = 'err';
    msg.textContent = res.error;
  } else {
    verdict.className = res.result ? 'blocked' : 'clear';
    verdict.textContent = res.result ? 'BLOCKED' : 'NOT BLOCKED';
    msg.className = 'warn';
    msg.textContent = res.warning || '';
  }

  renderScenarios();
  draw();
}

/* ---------------- wiring ---------------- */

buildControls();
buildAngleToggles();
buildVarRows();
document.getElementById('ruleCode').value = DEFAULT_RULE;
compileRule();

document.getElementById('ruleCode').addEventListener('input', () => { selectedRule = -1; compileRule(); update(); });

document.getElementById('ruleSave').addEventListener('click', () => {
  const name = document.getElementById('ruleName').value.trim() || `rule ${rules.length + 1}`;
  rules.push({ name, code: document.getElementById('ruleCode').value });
  selectedRule = rules.length - 1;
  update();
});
document.getElementById('ruleCode').addEventListener('keydown', e => {
  if (e.key === 'Tab') {
    e.preventDefault();
    const el = e.target;
    const start = el.selectionStart;
    el.value = el.value.slice(0, start) + '    ' + el.value.slice(el.selectionEnd);
    el.selectionStart = el.selectionEnd = start + 4;
    compileRule();
    update();
  }
});

document.getElementById('scenExpected').addEventListener('click', () => {
  expectedForSave = !expectedForSave;
  const el = document.getElementById('scenExpected');
  el.textContent = expectedForSave ? 'expect BLOCKED' : 'expect CLEAR';
  el.className = 'pill ' + (expectedForSave ? 'blocked' : 'clear');
});

document.getElementById('scenSave').addEventListener('click', () => {
  const name = document.getElementById('scenName').value.trim() || `scenario ${scenarios.length + 1}`;
  scenarios.push({ name, expected: expectedForSave, params: { ...params } });
  selectedScenario = scenarios.length - 1;
  document.getElementById('scenName').value = '';
  update();
});

document.getElementById('saveFileBtn').addEventListener('click', () => {
  const blob = new Blob([exportJson() + '\n'], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'crosswalk_scenarios.json';
  a.click();
  URL.revokeObjectURL(a.href);
  const m = document.getElementById('ioMsg');
  m.className = 'ok';
  m.textContent = 'saved crosswalk_scenarios.json';
});

document.getElementById('loadFileBtn').addEventListener('click', () =>
  document.getElementById('loadFileInput').click());

document.getElementById('loadFileInput').addEventListener('change', async e => {
  const file = e.target.files[0];
  if (!file) return;
  const m = document.getElementById('ioMsg');
  try {
    importJson(await file.text());
    m.className = 'ok';
    m.textContent = `loaded ${scenarios.length} scenarios from ${file.name}`;
  } catch (err) {
    m.className = 'err';
    m.textContent = `load failed: ${err.message}`;
  }
  e.target.value = '';
});

document.getElementById('exportBtn').addEventListener('click', () => {
  document.getElementById('ioText').value = exportJson();
  const m = document.getElementById('ioMsg');
  m.className = 'ok'; m.textContent = 'exported';
});

document.getElementById('copyBtn').addEventListener('click', async () => {
  const t = document.getElementById('ioText');
  if (!t.value) t.value = exportJson();
  t.select();
  try { await navigator.clipboard.writeText(t.value); } catch (_) { document.execCommand('copy'); }
  const m = document.getElementById('ioMsg');
  m.className = 'ok'; m.textContent = 'copied to clipboard';
});

document.getElementById('importBtn').addEventListener('click', () => {
  const m = document.getElementById('ioMsg');
  try {
    importJson(document.getElementById('ioText').value);
    m.className = 'ok';
    m.textContent = `imported ${scenarios.length} scenarios`;
  } catch (e) {
    m.className = 'err';
    m.textContent = `import failed: ${e.message}`;
  }
});

document.getElementById('fitBtn').addEventListener('click', fitView);

const themes = ['auto', 'light', 'dark'];
let themeIdx = 0;
document.getElementById('themeBtn').addEventListener('click', () => {
  themeIdx = (themeIdx + 1) % 3;
  const t = themes[themeIdx];
  if (t === 'auto') document.documentElement.removeAttribute('data-theme');
  else document.documentElement.setAttribute('data-theme', t);
  document.getElementById('themeBtn').textContent = { auto: '◐ Auto', light: '☀ Light', dark: '● Dark' }[t];
  draw();
});
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => draw());

new ResizeObserver(resizeCanvas).observe(canvas);
resizeCanvas();
update();
fitView();
