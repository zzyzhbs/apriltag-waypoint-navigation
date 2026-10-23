"""Stop-look-correct waypoint control (no ROS).

After every primitive the robot is still: look at tags -> EKF update ->
re-plan the next primitive from the new estimate -> execute -> EKF predict.
Moves use SI units (spin: signed rad, ccw > 0; drive: m); Steps (HW1) use
positive m / deg plus a direction.
"""
import math
from dataclasses import dataclass, field

from robot_control import ekf
from robot_control.se2 import angle_between, wrap_angle
from robot_control.step_timing import min_step, pick_level


@dataclass(frozen=True)
class NavParams:
    pos_tol: float
    ang_tol: float          # rad
    align_min: float        # rad
    align_max: float        # rad
    max_iters: int
    max_searches: int
    search_max_dist: float
    max_oblique: float      # rad
    view_dist: float        # m, tags farther than this are not expected to be seen
    view_half: float        # rad
    min_split: float        # rad
    max_drive: float        # m
    undershoot_k: float
    undershoot_c: float     # m
    undershoot_min: float   # m
    drive_levels: tuple
    spin_levels: tuple


def params_from_config(cfg):
    """nav.yaml (whole file) -> NavParams."""
    t, c, lv, lk = cfg['tolerance'], cfg['control'], cfg['levels'], cfg['look']
    r = math.radians
    return NavParams(
        pos_tol=t['pos'], ang_tol=r(t['ang_deg']),
        align_min=r(c['align_min_deg']), align_max=r(c['align_max_deg']),
        max_iters=int(c['max_iters']), max_searches=int(c['max_searches']),
        search_max_dist=float(c['search_max_dist']),
        max_oblique=r(lk['max_oblique_deg']), view_dist=float(lk['max_dist']),
        view_half=r(c['view_half_deg']), min_split=r(c['min_split_deg']),
        max_drive=float(c['max_drive']), undershoot_k=float(c['undershoot_k']),
        undershoot_c=float(c['undershoot_c']), undershoot_min=float(c['undershoot_min']),
        drive_levels=tuple(lv['drive']), spin_levels=tuple(lv['spin']))


@dataclass(frozen=True)
class Move:
    kind: str       # 'drive' | 'spin'
    value: float    # m, or rad (ccw > 0)


class NavigationHalt(RuntimeError):
    """Tags keep disagreeing with the estimate: map, extrinsic or tag frame is wrong."""


# ---------- planning ----------

def align_tol(dist, p):
    """Heading error allowed before driving; larger when the goal is close."""
    return min(max(math.atan2(p.pos_tol, dist), p.align_min), p.align_max)


def drive_length(dist, p):
    """Planned drive towards a goal `dist` ahead: capped, and short of it when long."""
    if dist >= p.undershoot_min:
        dist -= min(p.undershoot_k * dist + p.undershoot_c, dist / 2)
    return min(dist, p.max_drive)


def next_move(pose, goal, p):
    """Turn towards the goal, drive to it, then turn to its heading. None = reached."""
    dx, dy = goal.x - pose.x, goal.y - pose.y
    dist = math.hypot(dx, dy)
    if dist > p.pos_tol:
        bearing = wrap_angle(math.atan2(dy, dx) - pose.theta)
        if abs(bearing) > align_tol(dist, p):
            return Move('spin', bearing)
        return Move('drive', drive_length(dist * math.cos(bearing), p))
    turn = wrap_angle(goal.theta - pose.theta)
    if abs(turn) > p.ang_tol:
        return Move('spin', turn)
    return None


def tag_oblique_from(pose, lm):
    """Viewing obliquity of a tag from the robot position (independent of heading)."""
    return angle_between(math.cos(lm.yaw), math.sin(lm.yaw), pose.x - lm.x, pose.y - lm.y)


def tag_bearing(pose, lm):
    """Bearing of a tag relative to the robot heading."""
    return wrap_angle(math.atan2(lm.y - pose.y, lm.x - pose.x) - pose.theta)


def search_move(pose, landmarks, p, exclude=()):
    """Smallest turn that faces a tag close and frontal enough to be seen.

    Returns (Move, tag id), or (None, None). Tags in `exclude` were already searched.
    """
    best, best_id = None, None
    for lm in landmarks:
        if lm.id in exclude:
            continue
        if math.hypot(lm.x - pose.x, lm.y - pose.y) > p.search_max_dist:
            continue
        if tag_oblique_from(pose, lm) > p.max_oblique:
            continue
        turn = tag_bearing(pose, lm)
        if abs(turn) >= p.align_min and (best is None or abs(turn) < abs(best)):
            best, best_id = turn, lm.id
    return (None, None) if best is None else (Move('spin', best), best_id)


def expect_visible(pose, lm, p):
    """Should the camera see this tag from the (estimated) pose?"""
    return (math.hypot(lm.x - pose.x, lm.y - pose.y) <= p.view_dist
            and abs(tag_bearing(pose, lm)) <= p.view_half
            and tag_oblique_from(pose, lm) <= p.max_oblique)


def split_spin(pose, move, landmarks, p):
    """Shorten a turn that would end with no tag in view.

    Turn only until the last visible tag reaches the edge of the view, so the
    next look still corrects the heading; the rest of the turn is then shorter.
    """
    if move.kind != 'spin':
        return move
    after = pose._replace(theta=wrap_angle(pose.theta + move.value))
    if any(expect_visible(after, lm, p) for lm in landmarks):
        return move
    best = 0.0
    for lm in landmarks:
        if not expect_visible(pose, lm, p):
            continue
        b = tag_bearing(pose, lm)
        t = b + p.view_half if move.value > 0 else b - p.view_half
        if t * move.value > 0 and abs(t) < abs(move.value):
            best = t if abs(t) > abs(best) else best
    return Move('spin', best) if abs(best) >= p.min_split else move


def move_to_primitive(move):
    """Move -> (kind, direction, positive amount in m or deg)."""
    if move.kind == 'drive':
        return 'drive', 'fwd', move.value
    return 'spin', 'ccw' if move.value > 0 else 'cw', math.degrees(abs(move.value))


def to_step(move, calib, p):
    kind, direction, amount = move_to_primitive(move)
    levels = p.drive_levels if kind == 'drive' else p.spin_levels
    return pick_level(calib, kind, direction, amount, levels)


def below_min_step(move, calib, p):
    """True when the move is smaller than any calibrated level can make."""
    kind, direction, amount = move_to_primitive(move)
    smallest = min_step(calib, kind, direction,
                        p.drive_levels if kind == 'drive' else p.spin_levels)
    return smallest is not None and amount < smallest


def step_motion(step):
    """Executed Step -> Move with its nominal (signed, SI) amount."""
    if step.kind == 'drive':
        return Move('drive', step.amount)
    sign = 1.0 if step.dir == 'ccw' else -1.0
    return Move('spin', sign * math.radians(step.amount))


# ---------- the loop ----------

class NavLog:
    """Hooks for logging; the ROS node overrides them (all no-ops here)."""

    def on_look(self, goal_index, records, pose, P):
        pass

    def on_step(self, goal_index, step, pose_before, pose_after, P, timing=None):
        """timing = (t_start, t_stop) of the motion if execute() returned it."""
        pass

    def on_waypoint(self, result):
        pass


@dataclass
class TagRecord:
    obs: object         # TagObs
    accepted: bool
    d2: float
    reason: str = ''


@dataclass
class WaypointResult:
    index: int
    goal: object        # Pose2
    reached: bool
    tag_based: bool     # had at least one tag update during this waypoint
    pose: object        # estimated Pose2 when done
    sigmas: tuple
    iters: int
    tags_used: list = field(default_factory=list)
    note: str = ''


class Navigator:
    """look() -> (accepted {id: TagObs}, rejected {id: reason});
    execute(step) runs it and may return its (t_start, t_stop)."""

    def __init__(self, landmark_map, calib, params, noise, start_pose, look, execute,
                 log=None):
        self.map = landmark_map
        self.calib = calib
        self.p = params
        self.noise = noise
        self.x = ekf.state_from_pose(start_pose)
        self.P = ekf.initial_covariance(noise)
        self.look = look
        self.execute = execute
        self.log = log or NavLog()
        self.all_rejected_looks = 0

    def pose(self):
        return ekf.pose_from_state(self.x)

    def update_from_look(self, goal_index):
        """One look -> EKF updates, nearest tag first. Returns ids that were used."""
        accepted, rejected = self.look()
        records = [TagRecord(None, False, 0.0, f'tag {i}: {why}') for i, why in rejected.items()]
        used = []
        for obs in sorted(accepted.values(), key=lambda o: o.dist):
            lm = self.map.get(obs.id)
            if lm is None:
                records.append(TagRecord(obs, False, 0.0, 'not in map'))
                continue
            r = ekf.update(self.x, self.P, obs, lm, self.noise)
            records.append(TagRecord(obs, r.accepted, r.d2, '' if r.accepted else 'gate'))
            if r.accepted:
                self.x, self.P = r.x, r.P
                used.append(obs.id)
        self.check_consistency(accepted, used)
        self.log.on_look(goal_index, records, self.pose(), self.P)
        return used

    def check_consistency(self, accepted, used):
        if accepted and not used:
            self.all_rejected_looks += 1
        elif used:
            self.all_rejected_looks = 0
        if self.all_rejected_looks >= 2:
            raise NavigationHalt('two looks in a row with every tag rejected by the gate')

    def predict(self, move):
        if move.kind == 'drive':
            self.x, self.P = ekf.predict_drive(self.x, self.P, move.value, self.noise)
        else:
            self.x, self.P = ekf.predict_spin(self.x, self.P, move.value, self.noise)

    def act(self, goal_index, move):
        """Time, execute and predict one move. Returns the Step, or None if it has no T."""
        step = to_step(move, self.calib, self.p)
        if step.T is None:
            return None
        before = self.pose()
        timing = self.execute(step)
        self.predict(step_motion(step))
        self.log.on_step(goal_index, step, before, self.pose(), self.P, timing)
        return step

    def choose_move(self, goal, tag_based, searched):
        """Next move, and the id of the tag it searches for (None if not a search)."""
        pose, landmarks = self.pose(), self.map.landmarks.values()
        move = next_move(pose, goal, self.p)
        if move is None and not tag_based and len(searched) < self.p.max_searches:
            return search_move(pose, landmarks, self.p, searched)
        if move is None:
            return None, None
        return split_spin(pose, move, landmarks, self.p), None

    def result(self, index, goal, reached, tag_based, iters, used, note=''):
        r = WaypointResult(index, goal, reached, tag_based, self.pose(), ekf.sigmas(self.P),
                           iters, sorted(used), note)
        self.log.on_waypoint(r)
        return r

    def run_goal(self, index, goal):
        used, searched = set(), set()
        for it in range(1, self.p.max_iters + 1):
            used.update(self.update_from_look(index))
            move, search_id = self.choose_move(goal, bool(used), searched)
            if move is None:
                note = '' if used else 'no tag seen; reached by prediction only'
                return self.result(index, goal, True, bool(used), it, used, note)
            if below_min_step(move, self.calib, self.p):
                return self.result(index, goal, True, bool(used), it, used,
                                   f'remaining {move.kind} is below the smallest step')
            if self.act(index, move) is None:
                return self.result(index, goal, False, bool(used), it, used,
                                   f'no calibrated level for {move}')
            if search_id is not None:
                searched.add(search_id)
        return self.result(index, goal, False, bool(used), self.p.max_iters, used,
                           'max_iters reached')

    def run(self, goals):
        return [self.run_goal(i, g) for i, g in enumerate(goals, 1)]
