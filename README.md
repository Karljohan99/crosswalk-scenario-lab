# Crosswalk Scenario Lab

Interactive sandbox for designing the **crosswalk-blocking rule** of an
autonomous-vehicle planner: should the ego vehicle treat a crosswalk as blocked,
given how a pedestrian (or another vehicle) is positioned and moving?

**Live demo:** https://karljohan99.github.io/crosswalk-scenario-lab/

Or open **`index.html`** directly in a browser — no build step, no server, no
network access needed (plain scripts, so `file://` works).

## What it does

- Draws a 2D top-down scene: a two-lane road (configurable **curvature**) with
  dashed center marking, the ego on the right lane with its local path drawn
  as a light-green band at safety-corridor width, a **crosswalk** (position
  along the path, angle vs the road, dimensions), a **pedestrian** (position,
  heading, speed, prediction horizon), and optionally an **other vehicle**
  (4.5 × 1.9 m, own position/heading/speed) with the same naive straight
  prediction. The rule runs once per object and the crosswalk is blocked if
  any object triggers; the `is_pedestrian` variable lets a rule distinguish
  them.
- Computes the geometric quantities a rule-based checker works with, live, and
  visualizes the angles on the canvas (toggleable per angle, switchable
  between pedestrian and vehicle).
- Lets you write the blocking decision as a **Python snippet** that must
  `return True` (blocked) or `False`. A built-in Python-subset interpreter runs
  it — no server, no Pyodide.
- Lets you save **scenarios** with an expected outcome (blocked / clear) and
  shows how many scenarios the current rule decides correctly.
- Lets you save multiple **rules** and compare them: each saved rule shows how
  many scenarios it decides correctly, so candidate rules can be ranked at a
  glance.
- **Export / Import** serializes scenarios + rules + variable names as JSON,
  via text box or file (`scenarios/starter.json` is such a file).

Drag the pedestrian or vehicle to move it, drag the arrow tip to rotate its
heading, scroll to zoom, drag the background to pan.

## The model

- Each object's prediction is **naive**: a straight constant-velocity segment
  starting at the object's front, of length `speed × horizon`, buffered to the
  object's width with flat caps.
- The **approach angle** is the angle between the object's heading and the
  direction from the object toward its nearest point on the ego path
  (0–180°; 0 = heading straight at the path).
- The **trajectory approach angle** is the same angle taken at the prediction's
  crosswalk **entry point** — the nearest point of (prediction buffer ∩
  crosswalk polygon) along the trajectory. It is `None` when the prediction
  misses the crosswalk.
- The **endpoint approach angle** is the object's heading vs the direction
  from the crosswalk **centerline endpoint nearest to the object** toward that
  endpoint's nearest point on the ego path. Unlike the entry-point anchor, it
  is fixed per crosswalk side — stable against prediction wiggle and against
  the ego path curving under the crosswalk (autoware_mini branch
  `crosswalk_centerline_endpoint_anchor`).
- Two rules are bundled: the **production rule** (entry-point anchor) and the
  **endpoint-anchored rule** (same thresholds, endpoint anchor in the
  trajectory branch). Both block when the relevant angle is under 60°, or
  when the object is departing (`180 − angle < 60°`) but still within half of
  `wide_safety_box_width` of the path — so pedestrians walking parallel to the
  road, and vehicles driving along it, do not block.

Not modeled: temporal filtering (the tool evaluates a single frame) and ego
motion / braking dynamics — the tool answers *should this crosswalk be treated
as blocked*, not *can we stop in time*.

The geometry is cross-validated against a Python/shapely reference (angles
agree to < 0.15° over straight, curved, and angled-crosswalk configurations).

## Variables available in the rule

Each variable can be **renamed** in the Variables panel; the rule sees the
names you chose. When the other vehicle is enabled, the rule runs once per
object with that object's values bound.

| default name | meaning |
|---|---|
| `approach_angle` | object heading vs direction to nearest point on ego path, 0–180° (0 = straight at the path) |
| `trajectory_approach_angle` | same angle taken at the prediction's crosswalk entry point; `None` if the prediction misses the crosswalk |
| `endpoint_approach_angle` | object heading vs direction from the nearest crosswalk-centerline endpoint to its nearest path point, 0–180° |
| `crosswalk_angle` | crossing axis vs road direction, 0–90° (90 = perpendicular crosswalk) |
| `heading_to_crosswalk_angle` | object heading vs crossing-axis direction, 0–180° |
| `distance_to_path` | object footprint to ego path centerline (m) |
| `distance_to_crosswalk` | object footprint to crosswalk polygon (m, 0 if touching) |
| `on_crosswalk` | object footprint intersects the crosswalk polygon (bool) |
| `prediction_hits_crosswalk` | prediction buffer intersects the crosswalk polygon (bool) |
| `prediction_length` | speed × horizon (m) |
| `speed` | object speed (m/s) |
| `wide_safety_box_width` | tunable parameter (default 3.1 m); departing objects within half of it still block |
| `is_pedestrian` | `True` for the pedestrian, `False` for the other vehicle (bool) |

## Python subset supported in the rule

`if / elif / else` (indent- or single-line), assignments, `return`, `pass`,
`and / or / not`, chained comparisons (`0 < a < 60`), `is None / is not None`,
conditional expressions, arithmetic (`+ - * / // % **`), `abs min max round`,
and `math.` (`degrees radians sin cos tan asin acos atan atan2 hypot sqrt fabs
floor ceil exp log pi e`). No loops, strings, or containers — the rule is a
pure decision function.

## Layout

| file | contents |
|---|---|
| `index.html` | markup only |
| `style.css` | styling, light/dark theme tokens |
| `core.js` | pure logic, no DOM: scene geometry + the mini-Python interpreter |
| `ui.js` | controls, canvas rendering, scenarios, export/import |
| `scenarios/starter.json` | bundled starter scenarios (importable via the UI) |
| `tests/` | test suite for `core.js` (see below) |

## Tests

```bash
node tests/test_core.js
```

Checks the geometry against a Python/shapely reference (`tests/reference.json`),
exercises the interpreter (syntax, semantics, error cases), and runs the bundled
rule against the starter scenarios. Regenerate the reference values with
`python3 tests/reference.py > tests/reference.json` after changing the scene
model (requires Python with numpy and shapely).
