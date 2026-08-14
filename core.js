'use strict';

/* ---------------- geometry ----------------
 * Mirrors autoware_mini semantics:
 *  - approach angle  = get_angle_between_two_headings(heading, get_heading_towards_path(pos))   [geometry.py:158, path.py:416]
 *  - prediction      = naive_predictor.py: straight segment from the object's front, length speed*horizon
 *  - trajectory buffer half-width = dimensions.y / 2                                            [collision.py:175]
 *  - trajectory approach angle taken at the nearest point of (buffer ∩ crosswalk) along the trajectory
 *    [collision_checker.py:806-817]
 */

const PED_RADIUS = 0.4;   // pedestrian footprint radius = dimensions/2 (0.8 m square-ish person)
const VEH_LENGTH = 4.5;   // other-vehicle footprint (m)
const VEH_WIDTH = 1.9;
const PATH_LENGTH = 100;  // local path length, m
const PATH_STEP = 0.5;    // path sampling step, m

const rad = d => d * Math.PI / 180;
const deg = r => r * 180 / Math.PI;

// pose on a constant-curvature path at arc length s (path starts at origin heading +x)
function pathPose(k, s) {
  if (Math.abs(k) < 1e-9) return { x: s, y: 0, h: 0 };
  const th = s * k;
  return { x: Math.sin(th) / k, y: (1 - Math.cos(th)) / k, h: th };
}

function samplePath(k) {
  const pts = [];
  for (let s = 0; s <= PATH_LENGTH + 1e-9; s += PATH_STEP) pts.push(pathPose(k, s));
  return pts;
}

// abs difference between two headings, wrapped to [0, 180] deg (geometry.py get_angle_between_two_headings)
function angDiffDeg(h1, h2) {
  let d = Math.abs(h1 - h2) % (2 * Math.PI);
  if (d > Math.PI) d = 2 * Math.PI - d;
  return deg(d);
}

function projectOnSegment(a, b, p) {  // returns {t: clamped distance along ab, d2: squared dist to closest point}
  const abx = b.x - a.x, aby = b.y - a.y;
  const len2 = abx * abx + aby * aby;
  let t = len2 > 0 ? ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  const cx = a.x + t * abx, cy = a.y + t * aby;
  const dx = p.x - cx, dy = p.y - cy;
  return { t: t * Math.sqrt(len2), d2: dx * dx + dy * dy, x: cx, y: cy };
}

// nearest point on the sampled path polyline (shapely linestring.interpolate(project()) equivalent)
function nearestOnPolyline(pts, p) {
  let best = null;
  for (let i = 0; i < pts.length - 1; i++) {
    const r = projectOnSegment(pts[i], pts[i + 1], p);
    if (best === null || r.d2 < best.d2) best = r;
  }
  return { x: best.x, y: best.y, dist: Math.sqrt(best.d2) };
}

// path.py get_heading_towards_path: heading from a position toward its closest point on the path
function headingTowardPath(pts, p) {
  const np = nearestOnPolyline(pts, p);
  return Math.atan2(np.y - p.y, np.x - p.x);
}

// rectangle polygon (CCW) centered at c, long axis along heading h: len along axis, wid across
function rectPoly(c, h, len, wid) {
  const ux = Math.cos(h), uy = Math.sin(h);       // axis direction
  const nx = -uy, ny = ux;                        // normal
  const hl = len / 2, hw = wid / 2;
  return [
    { x: c.x + ux * hl + nx * hw, y: c.y + uy * hl + ny * hw },
    { x: c.x - ux * hl + nx * hw, y: c.y - uy * hl + ny * hw },
    { x: c.x - ux * hl - nx * hw, y: c.y - uy * hl - ny * hw },
    { x: c.x + ux * hl - nx * hw, y: c.y + uy * hl - ny * hw },
  ];
}

// flat-capped buffer of segment a→b with half-width r (CCW rectangle)
function segBufferPoly(a, b, r) {
  const dx = b.x - a.x, dy = b.y - a.y;
  const len = Math.hypot(dx, dy) || 1;
  const nx = -dy / len * r, ny = dx / len * r;
  return [
    { x: a.x + nx, y: a.y + ny },
    { x: a.x - nx, y: a.y - ny },
    { x: b.x - nx, y: b.y - ny },
    { x: b.x + nx, y: b.y + ny },
  ];
}

function polyArea(poly) {
  let s = 0;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    s += a.x * b.y - b.x * a.y;
  }
  return s / 2;
}

function ensureCCW(poly) { return polyArea(poly) < 0 ? poly.slice().reverse() : poly; }

// Sutherland–Hodgman clip of convex subject against convex clip polygon (both CCW)
function clipConvex(subject, clip) {
  let out = subject.slice();
  for (let i = 0; i < clip.length && out.length; i++) {
    const a = clip[i], b = clip[(i + 1) % clip.length];
    const inp = out;
    out = [];
    const inside = p => (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) >= 0;
    const cross = (p, q) => {
      const d1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
      const d2 = (b.x - a.x) * (q.y - a.y) - (b.y - a.y) * (q.x - a.x);
      const t = d1 / (d1 - d2);
      return { x: p.x + t * (q.x - p.x), y: p.y + t * (q.y - p.y) };
    };
    for (let j = 0; j < inp.length; j++) {
      const p = inp[j], q = inp[(j + 1) % inp.length];
      if (inside(p)) {
        out.push(p);
        if (!inside(q)) out.push(cross(p, q));
      } else if (inside(q)) {
        out.push(cross(p, q));
      }
    }
  }
  return out;
}

// distance from point to convex polygon boundary; 0 if inside
function distPointToConvexPoly(p, poly) {
  let inside = true;
  let minD2 = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    if ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) < 0) inside = false;
    const r = projectOnSegment(a, b, p);
    if (r.d2 < minD2) minD2 = r.d2;
  }
  return inside ? 0 : Math.sqrt(minD2);
}

// distance from a convex polygon to a polyline; 0 if they touch
// (min distance is attained at a vertex of one of the two)
function distPolyToPolyline(poly, pts) {
  let min = Infinity;
  for (const c of poly) min = Math.min(min, nearestOnPolyline(pts, c).dist);
  for (const p of pts) {
    min = Math.min(min, distPointToConvexPoly(p, poly));
    if (min === 0) break;
  }
  return min;
}

// distance between two convex polygons; 0 if they intersect
function distPolyToPoly(a, b) {
  const clip = clipConvex(a, b);
  if (clip.length >= 3 && Math.abs(polyArea(clip)) > 1e-12) return 0;
  let min = Infinity;
  for (const c of a) min = Math.min(min, distPointToConvexPoly(c, b));
  for (const c of b) min = Math.min(min, distPointToConvexPoly(c, a));
  return min;
}

// naive straight prediction from an object's front point, buffered, clipped
// against the crosswalk; entry point = nearest point of the overlap along the
// trajectory (collision_checker.py:806-817)
function computePrediction(pathPts, cwPoly, p0, heading, predLen, halfWidth) {
  const out = { predSeg: null, buffer: null, clip: null, entry: null, entryNp: null, entryTowardPathH: null, hits: false, trajAngle: null };
  if (predLen <= 1e-6) return out;
  const dir = { x: Math.cos(heading), y: Math.sin(heading) };
  const p1 = { x: p0.x + dir.x * predLen, y: p0.y + dir.y * predLen };
  out.predSeg = [p0, p1];
  out.buffer = segBufferPoly(p0, p1, halfWidth);
  out.clip = clipConvex(out.buffer, cwPoly);
  if (out.clip.length >= 3 && Math.abs(polyArea(out.clip)) > 1e-9) {
    out.hits = true;
    let minT = Infinity;
    for (const v of out.clip) minT = Math.min(minT, projectOnSegment(p0, p1, v).t);
    out.entry = { x: p0.x + dir.x * minT, y: p0.y + dir.y * minT };
    out.entryNp = nearestOnPolyline(pathPts, out.entry);
    out.entryTowardPathH = Math.atan2(out.entryNp.y - out.entry.y, out.entryNp.x - out.entry.x);
    out.trajAngle = angDiffDeg(heading, out.entryTowardPathH);  // trajectory heading is constant (straight prediction)
  }
  return out;
}

/* compute all quantities + drawing geometry for a scene parameter set */
function computeScene(p) {
  const pathPts = samplePath(p.curvature);

  const cwPose = pathPose(p.curvature, p.cwDist);
  const axisH = cwPose.h + Math.PI / 2 + rad(p.cwAngleDeg);   // crossing direction
  const cwPoly = ensureCCW(rectPoly(cwPose, axisH, p.cwLen, p.cwWid));

  const ped = { x: p.pedX, y: p.pedY };
  const pedH = rad(p.pedHeadingDeg);

  const np = nearestOnPolyline(pathPts, ped);
  const towardPathH = Math.atan2(np.y - ped.y, np.x - ped.x);
  const approachAngle = angDiffDeg(pedH, towardPathH);
  const distToPath = Math.max(0, np.dist - PED_RADIUS);

  const dCwCenter = distPointToConvexPoly(ped, cwPoly);
  const onCrosswalk = dCwCenter <= PED_RADIUS;
  const distToCrosswalk = Math.max(0, dCwCenter - PED_RADIUS);

  const predLen = p.speed * p.horizon;
  const pedFront = { x: ped.x + Math.cos(pedH) * PED_RADIUS, y: ped.y + Math.sin(pedH) * PED_RADIUS };  // trajectory starts at object front
  const pred = computePrediction(pathPts, cwPoly, pedFront, pedH, predLen, PED_RADIUS);

  let cwAngle = angDiffDeg(axisH, cwPose.h);
  if (cwAngle > 90) cwAngle = 180 - cwAngle;                   // axis is undirected vs road: fold to 0-90

  // optional other vehicle, also with a naive straight prediction
  let veh = null, vehValues = null;
  if (p.vehEnabled) {
    const c = { x: p.vehX, y: p.vehY };
    const h = rad(p.vehHeadingDeg);
    const poly = ensureCCW(rectPoly(c, h, VEH_LENGTH, VEH_WIDTH));
    const vehNp = nearestOnPolyline(pathPts, c);
    const vehTowardH = Math.atan2(vehNp.y - c.y, vehNp.x - c.x);
    const vehPredLen = p.vehSpeed * p.horizon;
    const front = { x: c.x + Math.cos(h) * VEH_LENGTH / 2, y: c.y + Math.sin(h) * VEH_LENGTH / 2 };
    const vehPred = computePrediction(pathPts, cwPoly, front, h, vehPredLen, VEH_WIDTH / 2);
    const dCw = distPolyToPoly(poly, cwPoly);
    vehValues = {
      approach_angle: angDiffDeg(h, vehTowardH),
      trajectory_approach_angle: vehPred.trajAngle,
      crosswalk_angle: cwAngle,
      heading_to_crosswalk_angle: angDiffDeg(h, axisH),
      distance_to_path: distPolyToPolyline(poly, pathPts),
      distance_to_crosswalk: dCw,
      on_crosswalk: dCw === 0,
      prediction_hits_crosswalk: vehPred.hits,
      prediction_length: vehPredLen,
      speed: p.vehSpeed,
      wide_safety_box_width: p.wideBox,
      is_pedestrian: false,
    };
    veh = { center: c, heading: h, poly, np: vehNp, towardPathH: vehTowardH, ...vehPred };
  }

  return {
    values: {
      approach_angle: approachAngle,
      trajectory_approach_angle: pred.trajAngle,               // null (None) when prediction misses the crosswalk
      crosswalk_angle: cwAngle,
      heading_to_crosswalk_angle: angDiffDeg(pedH, axisH),
      distance_to_path: distToPath,
      distance_to_crosswalk: distToCrosswalk,
      on_crosswalk: onCrosswalk,
      prediction_hits_crosswalk: pred.hits,
      prediction_length: predLen,
      speed: p.speed,
      wide_safety_box_width: p.wideBox,
      is_pedestrian: true,
    },
    vehValues,
    draw: { pathPts, cwPose, axisH, cwPoly, ped, pedH, np, towardPathH,
            predSeg: pred.predSeg, buffer: pred.buffer, clip: pred.clip,
            entry: pred.entry, entryNp: pred.entryNp, entryTowardPathH: pred.entryTowardPathH,
            veh },
  };
}

/* ---------------- mini-Python interpreter ---------------- */

class PyError extends Error {
  constructor(msg, line) { super(msg); this.pyLine = line; }
}

const PY_KEYWORDS = new Set(['if', 'elif', 'else', 'return', 'and', 'or', 'not', 'True', 'False', 'None', 'is', 'pass']);
const PY_RESERVED = new Set(['for', 'while', 'def', 'class', 'import', 'from', 'in', 'lambda', 'with', 'try', 'except',
  'raise', 'global', 'nonlocal', 'del', 'yield', 'assert', 'break', 'continue', 'match']);

function pyTokenize(src) {
  const tokens = [];
  const indents = [0];
  const lines = src.split('\n');
  let lastLine = 1;
  for (let li = 0; li < lines.length; li++) {
    let line = lines[li];
    const hash = line.indexOf('#');
    if (hash >= 0) line = line.slice(0, hash);
    if (!line.trim()) continue;
    const lineNo = li + 1;
    lastLine = lineNo;
    let i = 0, indent = 0;
    while (i < line.length && (line[i] === ' ' || line[i] === '\t')) { indent += line[i] === '\t' ? 4 : 1; i++; }
    if (indent > indents[indents.length - 1]) { indents.push(indent); tokens.push({ type: 'INDENT', line: lineNo }); }
    else while (indent < indents[indents.length - 1]) { indents.pop(); tokens.push({ type: 'DEDENT', line: lineNo }); }
    if (indent !== indents[indents.length - 1]) throw new PyError('inconsistent indentation', lineNo);
    while (i < line.length) {
      const ch = line[i];
      if (ch === ' ' || ch === '\t') { i++; continue; }
      let m;
      if ((m = line.slice(i).match(/^(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?/)) && m[0]) {
        tokens.push({ type: 'NUMBER', value: parseFloat(m[0]), line: lineNo }); i += m[0].length; continue;
      }
      if ((m = line.slice(i).match(/^[A-Za-z_]\w*/))) {
        const w = m[0];
        if (PY_RESERVED.has(w)) throw new PyError(`'${w}' is not supported in rule snippets`, lineNo);
        tokens.push({ type: PY_KEYWORDS.has(w) ? w : 'NAME', value: w, line: lineNo });
        i += w.length; continue;
      }
      const two = line.substr(i, 2);
      if (['**', '//', '<=', '>=', '==', '!='].includes(two)) { tokens.push({ type: 'OP', value: two, line: lineNo }); i += 2; continue; }
      if ('+-*/%<>=():,.;'.includes(ch)) { tokens.push({ type: 'OP', value: ch, line: lineNo }); i++; continue; }
      throw new PyError(`unexpected character '${ch}'`, lineNo);
    }
    tokens.push({ type: 'NEWLINE', line: lineNo });
  }
  while (indents.length > 1) { indents.pop(); tokens.push({ type: 'DEDENT', line: lastLine }); }
  tokens.push({ type: 'EOF', line: lastLine });
  return tokens;
}

function pyParse(src) {
  const toks = pyTokenize(src);
  let pos = 0;
  const peek = () => toks[pos];
  const next = () => toks[pos++];
  const at = (type, value) => peek().type === type && (value === undefined || peek().value === value);
  const eat = (type, value) => { if (at(type, value)) return next(); return null; };
  const expect = (type, value, what) => {
    const t = eat(type, value);
    if (!t) throw new PyError(`expected ${what || value || type}, got ${describe(peek())}`, peek().line);
    return t;
  };
  const describe = t => t.type === 'EOF' ? 'end of code' : t.type === 'NEWLINE' ? 'end of line' : `'${t.value ?? t.type}'`;

  function parseBlockStmts(stopAtDedent) {
    const stmts = [];
    while (!at('EOF') && !(stopAtDedent && at('DEDENT'))) stmts.push(parseStmt());
    return stmts;
  }

  function parseStmt() {
    const t = peek();
    if (at('if')) return parseIf();
    return parseSimpleLine();
  }

  function parseSimpleLine() {
    const stmts = [parseSimpleStmt()];
    while (eat('OP', ';')) {
      if (at('NEWLINE')) break;
      stmts.push(parseSimpleStmt());
    }
    expect('NEWLINE', undefined, 'end of line');
    return stmts.length === 1 ? stmts[0] : { type: 'group', stmts, line: stmts[0].line };
  }

  function parseSimpleStmt() {
    const t = peek();
    if (eat('return')) {
      const value = (at('NEWLINE') || at('OP', ';')) ? null : parseExpr();
      return { type: 'return', value, line: t.line };
    }
    if (eat('pass')) return { type: 'pass', line: t.line };
    if (at('NAME') && toks[pos + 1] && toks[pos + 1].type === 'OP' && toks[pos + 1].value === '=') {
      const name = next().value;
      next(); // '='
      return { type: 'assign', name, value: parseExpr(), line: t.line };
    }
    return { type: 'exprstmt', value: parseExpr(), line: t.line };
  }

  function parseIf() {
    const t = expect('if');
    const branches = [{ cond: parseExpr(), body: parseSuite() }];
    let orelse = [];
    while (true) {
      if (at('elif')) { next(); branches.push({ cond: parseExpr(), body: parseSuite() }); }
      else if (at('else')) { next(); orelse = parseSuite(); break; }
      else break;
    }
    return { type: 'if', branches, orelse, line: t.line };
  }

  function parseSuite() {
    expect('OP', ':', "':'");
    if (eat('NEWLINE')) {
      expect('INDENT', undefined, 'an indented block');
      const stmts = parseBlockStmts(true);
      expect('DEDENT', undefined, 'end of block');
      return stmts;
    }
    const s = parseSimpleLine();
    return s.type === 'group' ? s.stmts : [s];
  }

  /* expressions */
  function parseExpr() { return parseTernary(); }

  function parseTernary() {
    const value = parseOr();
    if (at('if')) {
      const t = next();
      const cond = parseOr();
      expect('else', undefined, "'else' of conditional expression");
      const orelse = parseTernary();
      return { type: 'ternary', cond, value, orelse, line: t.line };
    }
    return value;
  }

  function parseOr() {
    let node = parseAnd();
    while (at('or')) { const t = next(); node = { type: 'or', left: node, right: parseAnd(), line: t.line }; }
    return node;
  }

  function parseAnd() {
    let node = parseNot();
    while (at('and')) { const t = next(); node = { type: 'and', left: node, right: parseNot(), line: t.line }; }
    return node;
  }

  function parseNot() {
    if (at('not')) { const t = next(); return { type: 'not', value: parseNot(), line: t.line }; }
    return parseComparison();
  }

  function parseComparison() {
    const first = parseArith();
    const ops = [], rest = [];
    while (true) {
      let op = null;
      if (at('OP') && ['<', '<=', '>', '>=', '==', '!='].includes(peek().value)) op = next().value;
      else if (at('is')) { next(); op = eat('not') ? 'is not' : 'is'; }
      else break;
      ops.push(op);
      rest.push(parseArith());
    }
    if (!ops.length) return first;
    return { type: 'compare', first, ops, rest, line: first.line };
  }

  function parseArith() {
    let node = parseTerm();
    while (at('OP', '+') || at('OP', '-')) { const t = next(); node = { type: 'binop', op: t.value, left: node, right: parseTerm(), line: t.line }; }
    return node;
  }

  function parseTerm() {
    let node = parseUnary();
    while (at('OP') && ['*', '/', '//', '%'].includes(peek().value)) {
      const t = next();
      node = { type: 'binop', op: t.value, left: node, right: parseUnary(), line: t.line };
    }
    return node;
  }

  function parseUnary() {
    if (at('OP', '-') || at('OP', '+')) { const t = next(); return { type: 'unary', op: t.value, value: parseUnary(), line: t.line }; }
    return parsePower();
  }

  function parsePower() {
    const base = parsePostfix();
    if (at('OP', '**')) { const t = next(); return { type: 'binop', op: '**', left: base, right: parseUnary(), line: t.line }; }
    return base;
  }

  function parsePostfix() {
    let node = parseAtom();
    while (true) {
      if (eat('OP', '.')) {
        const name = expect('NAME', undefined, 'attribute name');
        node = { type: 'attr', obj: node, name: name.value, line: name.line };
      } else if (at('OP', '(')) {
        const t = next();
        const args = [];
        if (!at('OP', ')')) {
          args.push(parseExpr());
          while (eat('OP', ',')) args.push(parseExpr());
        }
        expect('OP', ')', "')'");
        node = { type: 'call', func: node, args, line: t.line };
      } else break;
    }
    return node;
  }

  function parseAtom() {
    const t = peek();
    if (at('NUMBER')) { next(); return { type: 'num', value: t.value, line: t.line }; }
    if (at('NAME')) { next(); return { type: 'name', name: t.value, line: t.line }; }
    if (eat('True')) return { type: 'const', value: true, line: t.line };
    if (eat('False')) return { type: 'const', value: false, line: t.line };
    if (eat('None')) return { type: 'const', value: null, line: t.line };
    if (eat('OP', '(')) {
      const e = parseExpr();
      expect('OP', ')', "')'");
      return e;
    }
    throw new PyError(`unexpected ${describe(t)}`, t.line);
  }

  const body = parseBlockStmts(false);
  expect('EOF');
  return { type: 'module', body };
}

/* evaluation */
const pyRepr = v => v === null ? 'None' : v === true ? 'True' : v === false ? 'False' : typeof v === 'number' ? String(v) : String(v);
const pyTypeName = v => v === null ? 'NoneType' : typeof v === 'boolean' ? 'bool' : typeof v === 'number' ? 'float' : typeof v;
const pyTruthy = v => !(v === null || v === false || v === 0);

function asNumber(v, opDesc, line) {
  if (typeof v === 'number') return v;
  if (typeof v === 'boolean') return v ? 1 : 0;
  throw new PyError(`unsupported operand type for ${opDesc}: '${pyTypeName(v)}'`, line);
}

const PY_BUILTINS = {
  abs: { fn: (args) => Math.abs(args[0]), arity: [1, 1] },
  min: { fn: (args) => Math.min(...args), arity: [1, Infinity] },
  max: { fn: (args) => Math.max(...args), arity: [1, Infinity] },
  round: { fn: (args) => { const n = args.length > 1 ? args[1] : 0; const f = 10 ** n; return Math.round(args[0] * f) / f; }, arity: [1, 2] },
};
const PY_MATH_FNS = ['degrees', 'radians', 'sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'hypot', 'sqrt', 'fabs', 'floor', 'ceil', 'exp', 'log'];
const PY_MATH = {
  degrees: a => deg(a[0]), radians: a => rad(a[0]),
  sin: a => Math.sin(a[0]), cos: a => Math.cos(a[0]), tan: a => Math.tan(a[0]),
  asin: a => Math.asin(a[0]), acos: a => Math.acos(a[0]), atan: a => Math.atan(a[0]),
  atan2: a => Math.atan2(a[0], a[1]), hypot: a => Math.hypot(...a), sqrt: a => Math.sqrt(a[0]),
  fabs: a => Math.abs(a[0]), floor: a => Math.floor(a[0]), ceil: a => Math.ceil(a[0]),
  exp: a => Math.exp(a[0]), log: a => Math.log(a[0]),
};

function pyEval(node, env) {
  switch (node.type) {
    case 'num': case 'const': return node.type === 'num' ? node.value : node.value;
    case 'name': {
      if (env.has(node.name)) return env.get(node.name);
      if (node.name === 'math') return { __module: 'math' };
      if (node.name in PY_BUILTINS) return { __builtin: node.name };
      throw new PyError(`name '${node.name}' is not defined`, node.line);
    }
    case 'attr': {
      const obj = pyEval(node.obj, env);
      if (obj && obj.__module === 'math') {
        if (node.name === 'pi') return Math.PI;
        if (node.name === 'e') return Math.E;
        if (PY_MATH_FNS.includes(node.name)) return { __mathfn: node.name };
        throw new PyError(`module 'math' has no attribute '${node.name}'`, node.line);
      }
      throw new PyError(`'${pyTypeName(obj)}' object has no attribute '${node.name}'`, node.line);
    }
    case 'call': {
      const fn = pyEval(node.func, env);
      const args = node.args.map(a => {
        const v = pyEval(a, env);
        return asNumber(v, 'function argument', node.line);
      });
      if (fn && fn.__builtin) {
        const b = PY_BUILTINS[fn.__builtin];
        if (args.length < b.arity[0] || args.length > b.arity[1])
          throw new PyError(`${fn.__builtin}() takes ${b.arity[0]}${b.arity[1] === Infinity ? '+' : `..${b.arity[1]}`} arguments (${args.length} given)`, node.line);
        return b.fn(args);
      }
      if (fn && fn.__mathfn) return PY_MATH[fn.__mathfn](args);
      throw new PyError(`'${pyTypeName(fn)}' object is not callable`, node.line);
    }
    case 'ternary': return pyTruthy(pyEval(node.cond, env)) ? pyEval(node.value, env) : pyEval(node.orelse, env);
    case 'or': { const l = pyEval(node.left, env); return pyTruthy(l) ? l : pyEval(node.right, env); }
    case 'and': { const l = pyEval(node.left, env); return pyTruthy(l) ? pyEval(node.right, env) : l; }
    case 'not': return !pyTruthy(pyEval(node.value, env));
    case 'unary': {
      const v = asNumber(pyEval(node.value, env), `unary ${node.op}`, node.line);
      return node.op === '-' ? -v : v;
    }
    case 'binop': {
      const l = asNumber(pyEval(node.left, env), `'${node.op}'`, node.line);
      const r = asNumber(pyEval(node.right, env), `'${node.op}'`, node.line);
      switch (node.op) {
        case '+': return l + r;
        case '-': return l - r;
        case '*': return l * r;
        case '/': if (r === 0) throw new PyError('division by zero', node.line); return l / r;
        case '//': if (r === 0) throw new PyError('division by zero', node.line); return Math.floor(l / r);
        case '%': if (r === 0) throw new PyError('division by zero', node.line); return ((l % r) + r) % r;
        case '**': return l ** r;
      }
      break;
    }
    case 'compare': {
      let left = pyEval(node.first, env);
      for (let i = 0; i < node.ops.length; i++) {
        const right = pyEval(node.rest[i], env);
        const op = node.ops[i];
        let ok;
        if (op === 'is') ok = left === right;
        else if (op === 'is not') ok = left !== right;
        else if (op === '==') ok = left === right;
        else if (op === '!=') ok = left !== right;
        else {
          if (left === null || right === null)
            throw new PyError(`'${op}' not supported between instances of '${pyTypeName(left)}' and '${pyTypeName(right)}' — guard with 'is not None'`, node.line);
          const l = asNumber(left, `'${op}'`, node.line), r = asNumber(right, `'${op}'`, node.line);
          ok = op === '<' ? l < r : op === '<=' ? l <= r : op === '>' ? l > r : l >= r;
        }
        if (!ok) return false;
        left = right;
      }
      return true;
    }
  }
  throw new PyError(`internal: unknown node '${node.type}'`, node.line || 0);
}

class ReturnValue { constructor(v) { this.value = v; } }

function pyExec(stmts, env) {
  for (const s of stmts) {
    switch (s.type) {
      case 'pass': break;
      case 'group': pyExec(s.stmts, env); break;
      case 'assign': env.set(s.name, pyEval(s.value, env)); break;
      case 'exprstmt': pyEval(s.value, env); break;
      case 'return': throw new ReturnValue(s.value === null ? null : pyEval(s.value, env));
      case 'if': {
        let done = false;
        for (const br of s.branches) {
          if (pyTruthy(pyEval(br.cond, env))) { pyExec(br.body, env); done = true; break; }
        }
        if (!done) pyExec(s.orelse, env);
        break;
      }
    }
  }
}

/* run a parsed rule against a {name: value} object; returns {result, warning} or throws PyError */
function runRule(ast, bindings) {
  const env = new Map(Object.entries(bindings));
  try {
    pyExec(ast.body, env);
  } catch (e) {
    if (e instanceof ReturnValue) {
      const v = e.value;
      const result = pyTruthy(v);
      const warning = typeof v === 'boolean' ? null : `rule returned ${pyRepr(v)} — interpreted as ${result ? 'True' : 'False'}`;
      return { result, warning };
    }
    throw e;
  }
  throw new PyError('rule finished without returning a value — add a return statement', 0);
}
