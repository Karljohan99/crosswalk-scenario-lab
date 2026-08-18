'use strict';

/* Bundled starter data: default scene parameters, the bundled rules, and the starter
 * scenarios with expected outcomes. Loaded by index.html before ui.js — a JS file rather
 * than JSON so the lab also works from file:// — and by tests/test_core.js. STARTER has
 * the same shape as an Export from the UI; scenario params are overrides applied on top
 * of DEFAULT_PARAMS, like the Import format. */

const DEFAULT_PARAMS = {
  curvature: 0, cwDist: 30, cwAngleDeg: 0, cwLen: 12, cwWid: 4, cwOffset: 0,
  pedX: 30, pedY: 8.5, pedHeadingDeg: -90, speed: 1.4, horizon: 3.0, wideBox: 3.1,
  vehEnabled: false, vehX: 50, vehY: 3.5, vehHeadingDeg: 180, vehSpeed: 8,
};

const PRODUCTION_RULE = `# Current production rule (collision_checker.py: _crosswalk_is_approaching_or_departing)
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

const COMBINED_RULE = `# Combined-gate rule (branch crosswalk_centerline_endpoint_anchor)
# Trajectory branch: block when the prediction crosses the ego lane
# transversally at the conflict point (path_crossing_angle > 30), the
# object holds either crossing certificate — moving along the crossing
# axis (within 60 deg) or crosswise to the road where it currently is
# (local_crossing_angle > 30) — and it heads toward the conflict point,
# or is departing but still within half of wide_safety_box_width.

if on_crosswalk:
    angle = approach_angle
    if angle < 60:
        return True
    return 180 - angle < 60 and distance_to_path < wide_safety_box_width / 2

if not prediction_hits_crosswalk or path_crossing_angle is None:
    return False

axis_alignment = min(heading_to_crosswalk_angle, 180 - heading_to_crosswalk_angle)
if path_crossing_angle > 30 and (axis_alignment < 60 or local_crossing_angle > 30):
    if moving_toward_conflict:
        return True
    return distance_to_path < wide_safety_box_width / 2
return False`;

const AXIS_PICK_RULE = `# Axis-pick anchor rule (branch crosswalk_axis_pick_anchor)
# Production thresholds, but the trajectory branch measures the heading
# against the towards-path direction of a fixed anchor instead of the
# prediction's crosswalk entry point: at the crossing end on the object's
# side (side split where the path crosses the crossing), the endpoint or
# boundary corner whose towards-path direction is closest to the crossing
# axis — the least-distorted projection onto the local path.

if on_crosswalk:
    angle = approach_angle
elif prediction_hits_crosswalk and anchor_approach_angle is not None:
    angle = anchor_approach_angle
else:
    return False

if angle < 60:
    return True
if 180 - angle < 60 and distance_to_path < wide_safety_box_width / 2:
    return True
return False`;

const STARTER = {
  version: 1,
  rule: PRODUCTION_RULE,   // the editor opens with the production rule loaded
  rules: [
    { name: 'production rule', code: PRODUCTION_RULE },
    { name: 'combined-gate rule', code: COMBINED_RULE },
    { name: 'axis-pick anchor rule', code: AXIS_PICK_RULE },
  ],
  scenarios: [
    { name: 'Approaching, straight road', expected: true,
      params: {} },
    { name: 'Walking parallel on sidewalk', expected: false,
      params: { pedHeadingDeg: 0 } },
    { name: 'Departed, far side, walking away', expected: false,
      params: { pedY: -6 } },
    { name: 'Departing but still on the road', expected: true,
      params: { pedY: -1 } },
    { name: 'Approaching on curved road', expected: true,
      params: { curvature: 0.02, pedX: 23.43, pedY: 15.75, pedHeadingDeg: -55.6 } },
    { name: 'Cutting diagonally over angled crosswalk', expected: true,
      params: { cwAngleDeg: 30, pedX: 26.25, pedY: 8.25, pedHeadingDeg: -60 } },
    { name: 'Loitering on crosswalk, walking along road', expected: true,
      params: { pedX: 30, pedY: 1.5, pedHeadingDeg: 0, speed: 1.0 } },
    { name: 'Vehicle driving through, ped on sidewalk', expected: false,
      params: { pedHeadingDeg: 0, vehEnabled: true } },
    { name: 'Vehicle parked on crosswalk', expected: false,
      params: { pedHeadingDeg: 0, vehEnabled: true, vehX: 30.5, vehY: 1.2, vehHeadingDeg: 0, vehSpeed: 0 } },
    { name: 'Vehicle cutting diagonally toward ego lane', expected: true,
      params: { pedHeadingDeg: 0, vehEnabled: true, vehX: 35, vehY: 10, vehHeadingDeg: -135, vehSpeed: 5 } },
    // #417 signature: fast object rides along a curving road, its straight prediction clips
    // the crosswalk far from it and the entry-point anchor reads "approaching" (~58°);
    // the crossing gate clears it (72° off the crossing axis)
    { name: 'Scooter cutting the curve, prediction clips crosswalk', expected: false,
      params: { curvature: 0.04, cwAngleDeg: -20, pedX: 16.96, pedY: 10.98,
                pedHeadingDeg: 30.4, speed: 8, horizon: 3.0 } },
    // genuine crosser walking diagonally along a long crosswalk mapped asymmetrically around
    // the road; anchor-based angles read this as departing, the crossing gate blocks it
    { name: 'Diagonal crosser along long offset crosswalk', expected: true,
      params: { curvature: -0.05, cwAngleDeg: -12, cwLen: 20, cwOffset: 8,
                pedX: 26.51, pedY: -22.23, pedHeadingDeg: 127.06, speed: 1.6, horizon: 8 } },
    // MR 349 legacy miss: >60 deg off the crossing axis of a heavily skewed crosswalk, yet
    // crossing the road perpendicularly — the local-transversality certificate blocks it
    { name: 'Perpendicular crosser on strongly skewed crosswalk', expected: true,
      params: { cwAngleDeg: 62, cwLen: 26, cwWid: 6,
                pedX: 31.5, pedY: 8.5, pedHeadingDeg: -90, speed: 1.4, horizon: 6 } },
    // fast walker on the inside of a tight curve, heading nearly parallel to the road
    // where it stands — prediction clips the wide crosswalk without genuinely crossing
    { name: 'Tight curve, pedestrian almost aligned with road', expected: false,
      params: { curvature: -0.09, cwDist: 17.5, cwAngleDeg: -5, cwLen: 10.5, cwWid: 7.5,
                cwOffset: -1.5, pedX: 4.63, pedY: -13.81, pedHeadingDeg: 75, speed: 2.7 } },
    // crosser already on the roadway of a skewed crosswalk, cutting toward the ego lane
    { name: 'Skewed crosswalk, pedestrian on road', expected: true,
      params: { cwDist: 36, cwAngleDeg: 37, cwLen: 10.5, cwWid: 3.5,
                pedX: 32.51, pedY: 2.5, pedHeadingDeg: -25, speed: 2.7 } },
  ],
};
