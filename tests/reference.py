"""Reference values for crosswalk-scenario-lab geometry, computed independently
with shapely, so the JS implementation in core.js can be checked against it.

Usage: python3 reference.py > reference.json
Requires numpy and shapely (>= 2.0).
"""
import json, math

import numpy as np
import shapely
from shapely.geometry import LineString, Polygon


def get_angle_between_two_headings(angle1, angle2):
    """Smallest absolute difference between two headings (radians), 0..pi."""
    diff = np.abs(angle1 - angle2)
    return np.where(diff > np.pi, 2 * np.pi - diff, diff)

PED_RADIUS = 0.4
PATH_LENGTH = 100
PATH_STEP = 0.5
LANE_WIDTH = 3.5  # crosswalk is centered on the road, half a lane left of the ego path


def path_pose(k, s):
    if abs(k) < 1e-9:
        return s, 0.0, 0.0
    th = s * k
    return math.sin(th) / k, (1 - math.cos(th)) / k, th


def sample_path(k):
    return [path_pose(k, s) for s in np.arange(0, PATH_LENGTH + 1e-9, PATH_STEP)]


def rect_poly(cx, cy, h, length, width):
    ux, uy = math.cos(h), math.sin(h)
    nx, ny = -uy, ux
    hl, hw = length / 2, width / 2
    return Polygon([
        (cx + ux * hl + nx * hw, cy + uy * hl + ny * hw),
        (cx - ux * hl + nx * hw, cy - uy * hl + ny * hw),
        (cx - ux * hl - nx * hw, cy - uy * hl - ny * hw),
        (cx + ux * hl - nx * hw, cy + uy * hl - ny * hw),
    ])


def compute(p):
    pts = sample_path(p['curvature'])
    path_ls = LineString([(x, y) for x, y, _ in pts])

    cwx, cwy, cwh = path_pose(p['curvature'], p['cwDist'])
    axis_h = cwh + math.pi / 2 + math.radians(p['cwAngleDeg'])
    cwx += -math.sin(cwh) * LANE_WIDTH / 2 + math.cos(axis_h) * p.get('cwOffset', 0)
    cwy += math.cos(cwh) * LANE_WIDTH / 2 + math.sin(axis_h) * p.get('cwOffset', 0)
    cw_poly = rect_poly(cwx, cwy, axis_h, p['cwLen'], p['cwWid'])

    ped = shapely.Point(p['pedX'], p['pedY'])
    ped_h = math.radians(p['pedHeadingDeg'])

    # path.py get_heading_towards_path
    np_pt = path_ls.interpolate(path_ls.project(ped))
    toward_h = math.atan2(np_pt.y - ped.y, np_pt.x - ped.x)
    approach = math.degrees(get_angle_between_two_headings(ped_h, toward_h))
    dist_path = max(0.0, ped.distance(path_ls) - PED_RADIUS)

    d_center = cw_poly.exterior.distance(ped) if not cw_poly.contains(ped) else 0.0
    on_cw = d_center <= PED_RADIUS
    dist_cw = max(0.0, d_center - PED_RADIUS)

    pred_len = p['speed'] * p['horizon']
    traj_angle = None
    hits = False
    if pred_len > 1e-6:
        dx, dy = math.cos(ped_h), math.sin(ped_h)
        p0 = (ped.x + dx * PED_RADIUS, ped.y + dy * PED_RADIUS)
        p1 = (p0[0] + dx * pred_len, p0[1] + dy * pred_len)
        traj_ls = LineString([p0, p1])
        buffer = traj_ls.buffer(PED_RADIUS, cap_style=2)  # flat caps, like create_trajectory_buffers
        inter = cw_poly.intersection(buffer)
        if not inter.is_empty and inter.area > 1e-9:
            hits = True
            coords = shapely.get_coordinates(inter)
            dists = traj_ls.project(shapely.points(coords))
            entry = traj_ls.interpolate(np.min(dists))
            np2 = path_ls.interpolate(path_ls.project(entry))
            toward2 = math.atan2(np2.y - entry.y, np2.x - entry.x)
            traj_angle = math.degrees(get_angle_between_two_headings(ped_h, toward2))

    cw_angle = math.degrees(get_angle_between_two_headings(axis_h, cwh))
    if cw_angle > 90:
        cw_angle = 180 - cw_angle

    # crossing gate (branch crosswalk_centerline_endpoint_anchor): conflict point = middle
    # path sample inside the crosswalk (same quantization as core.js pathConflict), path
    # tangent there is analytic; toward uses the chord from the crosswalk entry to it
    inside = [pt for pt in pts if cw_poly.covers(shapely.Point(pt[0], pt[1]))]
    crossing_angle = None
    toward = None
    if inside:
        conflict_x, conflict_y, conflict_th = inside[len(inside) // 2]
        angle = math.degrees(get_angle_between_two_headings(ped_h, conflict_th))
        crossing_angle = min(angle, 180 - angle)
        if hits:
            toward_h = math.atan2(conflict_y - entry.y, conflict_x - entry.x)
            toward = math.degrees(get_angle_between_two_headings(ped_h, toward_h)) < 90

    # axis-pick anchor (branch crosswalk_axis_pick_anchor): towards-path heading of the
    # anchor (crossing endpoint + boundary corners) at the crossing end on the object's
    # side (side split at the conflict point) whose direction is closest to the crossing axis
    anchor_angle = None
    if inside:
        ax, ay = math.cos(axis_h), math.sin(axis_h)
        nx, ny = -ay, ax
        endpoints = [(cwx + ax * p['cwLen'] / 2, cwy + ay * p['cwLen'] / 2),
                     (cwx - ax * p['cwLen'] / 2, cwy - ay * p['cwLen'] / 2)]
        toward_obj_h = math.atan2(ped.y - conflict_y, ped.x - conflict_x)
        side = 0 if math.degrees(get_angle_between_two_headings(toward_obj_h, axis_h)) < 90 else 1
        ex, ey = endpoints[side]
        anchors = [(ex, ey),
                   (ex + nx * p['cwWid'] / 2, ey + ny * p['cwWid'] / 2),
                   (ex - nx * p['cwWid'] / 2, ey - ny * p['cwWid'] / 2)]
        headings = []
        for x, y in anchors:
            np_a = path_ls.interpolate(path_ls.project(shapely.Point(x, y)))
            headings.append(math.atan2(np_a.y - y, np_a.x - x))
        devs = np.degrees(get_angle_between_two_headings(np.array(headings), axis_h))
        devs = np.minimum(devs, 180 - devs)
        anchor_angle = math.degrees(get_angle_between_two_headings(ped_h, headings[np.argmin(devs)]))

    # local crossing angle: heading vs path tangent at the object's nearest path sample
    # (same quantization as core.js localPathTangent)
    local_th = min(pts, key=lambda pt: (pt[0] - ped.x) ** 2 + (pt[1] - ped.y) ** 2)[2]
    local_angle = math.degrees(get_angle_between_two_headings(ped_h, local_th))
    local_crossing_angle = min(local_angle, 180 - local_angle)

    return {
        'approach_angle': approach,
        'trajectory_approach_angle': traj_angle,
        'anchor_approach_angle': anchor_angle,
        'path_crossing_angle': crossing_angle,
        'local_crossing_angle': local_crossing_angle,
        'moving_toward_conflict': toward,
        'crosswalk_angle': cw_angle,
        'heading_to_crosswalk_angle': math.degrees(get_angle_between_two_headings(ped_h, axis_h)),
        'distance_to_path': dist_path,
        'distance_to_crosswalk': dist_cw,
        'on_crosswalk': on_cw,
        'prediction_hits_crosswalk': hits,
        'prediction_length': pred_len,
    }


BASE = dict(curvature=0, cwDist=30, cwAngleDeg=0, cwLen=12, cwWid=4, cwOffset=0,
            pedX=30, pedY=7, pedHeadingDeg=-90, speed=1.4, horizon=3.0, wideBox=3.1)

CONFIGS = [
    dict(BASE),                                                                  # straight approach
    dict(BASE, curvature=0.02, pedX=24, pedY=14, pedHeadingDeg=-70, speed=1.5),  # curved road
    dict(BASE, cwAngleDeg=30, pedX=25, pedY=8, pedHeadingDeg=-45),               # angled crosswalk
    dict(BASE, curvature=-0.03, pedX=35, pedY=-9, pedHeadingDeg=100, horizon=5), # right curve, far side
    dict(BASE, pedX=30, pedY=-1, pedHeadingDeg=-90),                             # departing on road
    dict(BASE, speed=0),                                                         # no prediction
    dict(BASE, curvature=-0.05, cwAngleDeg=-12, cwLen=20, cwOffset=8,            # long offset crosswalk,
         pedX=26.51, pedY=-22.23, pedHeadingDeg=127.06, speed=1.6, horizon=8),   # diagonal crosser
    dict(BASE, cwOffset=-14, cwLen=12),                                          # crosswalk shifted off the path
    dict(BASE, cwAngleDeg=62, cwLen=26, cwWid=6, pedX=31.5, pedY=8.5,            # strongly skewed crosswalk,
         horizon=6),                                                             # perpendicular crosser
]

print(json.dumps([{'params': c, 'expected': compute(c)} for c in CONFIGS], indent=1))
