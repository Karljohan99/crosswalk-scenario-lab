"""Reference values for crosswalk-scenario-lab geometry, computed with the
autoware_mini repo's own helpers (get_angle_between_two_headings) and shapely —
same mechanics as collision_checker.py check_pedestrian_crosswalk.

Usage: python3 reference.py > reference.json
Requires the autoware_mini repo and ROS Noetic python packages (paths below).
"""
import sys, json, math
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
sys.path.insert(0, '/home/pilve/autoware_mini_ws/src/autoware_mini/src')

import numpy as np
import shapely
from shapely.geometry import LineString, Polygon
from autoware_mini.geometry import get_angle_between_two_headings

PED_RADIUS = 0.4
PATH_LENGTH = 100
PATH_STEP = 0.5


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

    return {
        'approach_angle': approach,
        'trajectory_approach_angle': traj_angle,
        'crosswalk_angle': cw_angle,
        'heading_to_crosswalk_angle': math.degrees(get_angle_between_two_headings(ped_h, axis_h)),
        'distance_to_path': dist_path,
        'distance_to_crosswalk': dist_cw,
        'on_crosswalk': on_cw,
        'prediction_hits_crosswalk': hits,
        'prediction_length': pred_len,
    }


BASE = dict(curvature=0, cwDist=30, cwAngleDeg=0, cwLen=12, cwWid=4,
            pedX=30, pedY=7, pedHeadingDeg=-90, speed=1.4, horizon=3.0, wideBox=3.1)

CONFIGS = [
    dict(BASE),                                                                  # straight approach
    dict(BASE, curvature=0.02, pedX=24, pedY=14, pedHeadingDeg=-70, speed=1.5),  # curved road
    dict(BASE, cwAngleDeg=30, pedX=25, pedY=8, pedHeadingDeg=-45),               # angled crosswalk
    dict(BASE, curvature=-0.03, pedX=35, pedY=-9, pedHeadingDeg=100, horizon=5), # right curve, far side
    dict(BASE, pedX=30, pedY=-1, pedHeadingDeg=-90),                             # departing on road
    dict(BASE, speed=0),                                                         # no prediction
]

print(json.dumps([{'params': c, 'expected': compute(c)} for c in CONFIGS], indent=1))
