"""Summaries and text tables for hw2_probe (no ROS)."""
import math
import statistics

from robot_control.se2 import circular_mean, wrap_angle


def fmt_pose(p):
    return f'({p.x:+.3f}, {p.y:+.3f}, {math.degrees(p.theta):+7.2f} deg)'


def circular_std(angles):
    """Std of angles around their circular mean (rad)."""
    m = circular_mean(angles)
    return statistics.pstdev([wrap_angle(a - m) for a in angles])


def tag_stats(group, duration_s):
    """Raw frames of one tag -> dict of rate, mean and std."""
    xs, ys = [o.x for o in group], [o.y for o in group]
    psis = [o.psi for o in group]
    return {
        'n': len(group),
        'rate': len(group) / duration_s,
        'dist': statistics.mean(o.dist for o in group),
        'x': statistics.mean(xs), 'x_std': statistics.pstdev(xs),
        'y': statistics.mean(ys), 'y_std': statistics.pstdev(ys),
        'psi': circular_mean(psis), 'psi_std': circular_std(psis),
        'oblique': statistics.mean(o.oblique for o in group),
    }


def format_stats(tag_id, s):
    return (f'tag {tag_id}: n {s["n"]:3d} ({s["rate"]:4.1f} Hz)  d {s["dist"]:.3f} m  '
            f'x {s["x"]:+.3f}±{s["x_std"] * 1000:.1f}mm  '
            f'y {s["y"]:+.3f}±{s["y_std"] * 1000:.1f}mm  '
            f'psi {math.degrees(s["psi"]):+7.2f}±{math.degrees(s["psi_std"]):.2f}deg  '
            f'oblique {math.degrees(s["oblique"]):.1f}deg')


def format_latency(latencies):
    if not latencies:
        return 'latency: no data'
    return (f'latency (receive - image stamp): mean {statistics.mean(latencies) * 1000:.0f} ms, '
            f'max {max(latencies) * 1000:.0f} ms')


def survey_diff(landmark, tag_pose):
    """Measured minus map: (dx, dy, distance, dyaw)."""
    dx, dy = tag_pose.x - landmark.x, tag_pose.y - landmark.y
    return dx, dy, math.hypot(dx, dy), wrap_angle(tag_pose.theta - landmark.yaw)


def format_survey_row(landmark, tag_pose):
    dx, dy, d, dyaw = survey_diff(landmark, tag_pose)
    return (f'tag {landmark.id}: map ({landmark.x:+.3f}, {landmark.y:+.3f}, '
            f'{math.degrees(landmark.yaw):+7.2f})  measured {fmt_pose(tag_pose)}  '
            f'diff {d * 100:.1f} cm ({dx * 100:+.1f}, {dy * 100:+.1f}), '
            f'{math.degrees(dyaw):+.1f} deg')


def survey_yaml(tag_size, tag_poses, robot_pose):
    """Measured tag poses in the landmarks.yaml format."""
    return {
        'frame': 'world',
        'tag_size': tag_size,
        'surveyed_from': [round(robot_pose.x, 4), round(robot_pose.y, 4),
                          round(math.degrees(robot_pose.theta), 2)],
        'landmarks': [{'id': i, 'x': round(p.x, 4), 'y': round(p.y, 4),
                       'yaw_deg': round(math.degrees(p.theta), 2)}
                      for i, p in sorted(tag_poses.items())],
    }
