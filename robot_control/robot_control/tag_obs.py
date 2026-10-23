"""AprilTag detections -> 2D observations in base_link (no ROS).

The tag's orientation is described by its outward normal (pointing from the
tag face towards the viewer). apriltag's tag z axis is +normal or -normal;
z_sign (+1 / -1) selects which, checked on the robot.
"""
import math
import statistics
from dataclasses import dataclass

from robot_control.se2 import angle_between, circular_mean, quat_to_matrix

MIN_DIST = 0.25
MAX_DIST = 3.0
MAX_OBLIQUE = math.radians(60.0)
MIN_COUNT = 3
MAX_SPREAD = 0.02


@dataclass(frozen=True)
class TagObs:
    id: int
    stamp: float        # s, image time
    x: float            # tag centre in base_link (m)
    y: float
    psi: float          # outward normal direction in base_link (rad)
    dist: float         # hypot(x, y)
    oblique: float      # angle between normal and tag->robot ray (rad); 0 = face-on
    count: int = 1      # frames averaged into this observation


def make_obs(tag_id, stamp, x, y, psi, count=1):
    oblique = angle_between(math.cos(psi), math.sin(psi), -x, -y)
    return TagObs(tag_id, stamp, x, y, psi, math.hypot(x, y), oblique, count)


def obs_from_transform(tag_id, stamp, t, q, z_sign):
    """t = (x, y, z), q = (x, y, z, w) of the tag in base_link."""
    R = quat_to_matrix(*q)
    nx, ny = z_sign * R[0][2], z_sign * R[1][2]
    return make_obs(tag_id, stamp, t[0], t[1], math.atan2(ny, nx))


def is_usable(obs, min_dist=MIN_DIST, max_dist=MAX_DIST, max_oblique=MAX_OBLIQUE):
    return min_dist <= obs.dist <= max_dist and obs.oblique <= max_oblique


def group_by_id(obs_list):
    groups = {}
    for o in obs_list:
        groups.setdefault(o.id, []).append(o)
    return groups


def summarize(group, min_count=MIN_COUNT, max_spread=MAX_SPREAD):
    """Frames of one tag -> (median observation, None) or (None, reason)."""
    if len(group) < min_count:
        return None, f'only {len(group)} frames'
    xs = [o.x for o in group]
    ys = [o.y for o in group]
    spread = max(statistics.pstdev(xs), statistics.pstdev(ys))
    if spread > max_spread:
        return None, f'unstable (std {spread:.3f} m)'
    obs = make_obs(group[0].id, max(o.stamp for o in group),
                   statistics.median(xs), statistics.median(ys),
                   circular_mean([o.psi for o in group]), len(group))
    return obs, None


def aggregate(obs_list, min_count=MIN_COUNT, max_spread=MAX_SPREAD):
    """Window of frames -> ({id: TagObs}, {id: reason rejected})."""
    accepted, rejected = {}, {}
    for tag_id, group in group_by_id(obs_list).items():
        obs, reason = summarize(group, min_count, max_spread)
        if obs is None:
            rejected[tag_id] = reason
        else:
            accepted[tag_id] = obs
    return accepted, rejected


def filter_window(raw, known_ids, limits):
    """One look window -> ({id: TagObs}, {id: reason}).

    limits: the `look:` section of nav.yaml (angles in degrees).
    """
    usable, rejected = [], {}
    max_oblique = math.radians(limits['max_oblique_deg'])
    for o in raw:
        if o.id not in known_ids:
            rejected[o.id] = 'not in map'
        elif is_usable(o, limits['min_dist'], limits['max_dist'], max_oblique):
            usable.append(o)
        else:
            rejected.setdefault(o.id, f'out of range (d {o.dist:.2f} m, '
                                      f'oblique {math.degrees(o.oblique):.0f} deg)')
    accepted, unstable = aggregate(usable, limits['min_count'], limits['max_spread'])
    rejected.update(unstable)
    for tag_id in accepted:
        rejected.pop(tag_id, None)      # some frames were out of range, enough were fine
    return accepted, rejected
