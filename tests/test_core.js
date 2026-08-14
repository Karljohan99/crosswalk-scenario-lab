// Test core.js:
//  1. geometry vs Python/shapely reference (reference.json, regenerate with reference.py)
//  2. the mini-Python interpreter
//  3. the default rule against the starter scenarios' situations
// Run: node tests/test_core.js
'use strict';
const fs = require('fs');
const path = require('path');

const coreSrc = fs.readFileSync(path.join(__dirname, '..', 'core.js'), 'utf8');
const { computeScene, pyParse, runRule, PyError } =
  eval(coreSrc + '\n;({ computeScene, pyParse, runRule, PyError })');

let failures = 0;
function check(label, cond, detail) {
  if (cond) console.log(`  PASS ${label}`);
  else { failures++; console.error(`  FAIL ${label}${detail ? ' — ' + detail : ''}`); }
}
function close(a, b, tol) {
  if (a === null || b === null) return a === b;
  return Math.abs(a - b) <= tol;
}

/* ---- 1. geometry vs Python reference ---- */
console.log('geometry vs shapely/repo reference:');
const ref = JSON.parse(fs.readFileSync(path.join(__dirname, 'reference.json'), 'utf8'));
ref.forEach((tc, i) => {
  const v = computeScene(tc.params).values;
  const e = tc.expected;
  for (const key of ['approach_angle', 'trajectory_approach_angle', 'crosswalk_angle',
                     'heading_to_crosswalk_angle', 'distance_to_path', 'distance_to_crosswalk',
                     'prediction_length']) {
    const tol = key.includes('angle') ? 0.15 : 0.02;  // buffer polygons: shapely rounds corners of the clip, we don't
    check(`config ${i} ${key}`, close(v[key], e[key], tol), `js=${v[key]} py=${e[key]}`);
  }
  for (const key of ['on_crosswalk', 'prediction_hits_crosswalk']) {
    check(`config ${i} ${key}`, v[key] === e[key], `js=${v[key]} py=${e[key]}`);
  }
});

/* ---- 2. interpreter ---- */
console.log('interpreter:');
function run(src, bindings) { return runRule(pyParse(src), bindings || {}); }

check('return True', run('return True').result === true);
check('return False', run('return False').result === false);
check('chained comparison', run('return 0 < a < 60', { a: 30 }).result === true);
check('chained comparison false', run('return 0 < a < 60', { a: 80 }).result === false);
check('and/or/not', run('return not (a and b) or c', { a: true, b: false, c: false }).result === true);
check('arithmetic', run('return (3 + 4 * 2) / 2 == 5.5').result === true);
check('floor div / mod', run('return 7 // 2 == 3 and -7 % 3 == 2').result === true);
check('power', run('return 2 ** 3 ** 2 == 512').result === true);
check('math functions', run('return abs(math.degrees(math.pi) - 180) < 1e-9').result === true);
check('math.atan2', run('return math.atan2(1, 1) == math.pi / 4').result === true);
check('min/max/round', run('return min(3, 1, 2) == 1 and max(3, 1) == 3 and round(2.567, 1) == 2.6').result === true);
check('ternary', run('return 1 if a > 5 else 0', { a: 10 }).result === true);
check('if/elif/else block', run(
`if a < 10:
    return False
elif a < 20:
    return True
else:
    return False`, { a: 15 }).result === true);
check('nested if + assignment', run(
`x = a * 2
if x > 10:
    y = x - 10
    if y > 3:
        return True
return False`, { a: 8 }).result === true);
check('inline suite', run('if a: return True\nreturn False', { a: true }).result === true);
check('is None', run('return x is None', { x: null }).result === true);
check('is not None', run('return x is not None and x < 60', { x: 30 }).result === true);
check('pass statement', run('if a:\n    pass\nreturn True', { a: true }).result === true);
check('comment handling', run('# comment\nreturn True  # trailing').result === true);
check('non-bool return warns', (() => { const r = run('return 5'); return r.result === true && !!r.warning; })());

function throws(src, bindings, msgPart) {
  try { run(src, bindings); return false; }
  catch (e) { return e instanceof PyError && (!msgPart || e.message.includes(msgPart)); }
}
check('undefined name error', throws('return foo', {}, "name 'foo' is not defined"));
check('None comparison error', throws('return x < 5', { x: null }, 'NoneType'));
check('no return error', throws('x = 1', {}, 'without returning'));
check('for loop rejected', throws('for i in x: return True', {}, 'not supported'));
check('division by zero', throws('return 1 / 0', {}, 'division by zero'));
check('syntax error has line', (() => {
  try { run('x = 1\nreturn ((1 + 2'); return false; }
  catch (e) { return e instanceof PyError && e.pyLine === 2; }
})());

/* ---- vehicle object ---- */
console.log('vehicle object:');
const VBASE = { curvature: 0, cwDist: 30, cwAngleDeg: 0, cwLen: 12, cwWid: 4,
  pedX: 30, pedY: 7, pedHeadingDeg: -90, speed: 1.4, horizon: 3.0, wideBox: 3.1,
  vehEnabled: true, vehX: 50, vehY: 1.75, vehHeadingDeg: 180, vehSpeed: 8 };
const vs1 = computeScene(VBASE);
check('veh values present when enabled', !!vs1.vehValues);
check('veh is_pedestrian false / ped true', vs1.vehValues.is_pedestrian === false && vs1.values.is_pedestrian === true);
check('veh driving through: prediction hits crosswalk', vs1.vehValues.prediction_hits_crosswalk === true);
check('veh driving along road: approach ~90', Math.abs(vs1.vehValues.approach_angle - 90) < 1,
  `got ${vs1.vehValues.approach_angle}`);
check('veh distance_to_path = lane offset - half width', Math.abs(vs1.vehValues.distance_to_path - 0.8) < 0.01,
  `got ${vs1.vehValues.distance_to_path}`);
const vs2 = computeScene({ ...VBASE, vehX: 30, vehY: 0, vehSpeed: 0 });
check('veh parked on crosswalk: on_crosswalk', vs2.vehValues.on_crosswalk === true);
check('veh parked: no prediction', vs2.vehValues.prediction_hits_crosswalk === false && vs2.vehValues.trajectory_approach_angle === null);
check('veh disabled: no vehValues', computeScene({ ...VBASE, vehEnabled: false }).vehValues === null);

/* ---- 3. default rule against starter scenarios ---- */
console.log('starter scenarios with bundled rule:');
const starter = JSON.parse(fs.readFileSync(path.join(__dirname, '..', 'scenarios', 'starter.json'), 'utf8'));
const starterAst = pyParse(starter.rule);
const KNOWN_FAILING = new Set(['Loitering on crosswalk, walking along road']);  // deliberate miss of the current rule
for (const s of starter.scenarios) {
  const sc = computeScene(s.params);
  const objectValues = sc.vehValues ? [sc.values, sc.vehValues] : [sc.values];
  const blocked = objectValues.some(v => runRule(starterAst, v).result);  // rule runs per object
  const shouldPass = !KNOWN_FAILING.has(s.name);
  check(`starter: ${s.name}${shouldPass ? '' : ' (expected to fail)'}`,
    (blocked === s.expected) === shouldPass, `rule returned ${blocked}, scenario expects ${s.expected}`);
}

console.log(failures ? `\n${failures} FAILURES` : '\nall tests passed');
process.exit(failures ? 1 : 0);
