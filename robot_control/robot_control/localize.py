"""Pose from tags: single-tag solutions and a simple weighted fusion (no ROS).

world_T_base = world_T_tag ⊕ inverse(base_T_tag)    (localize: map known)
world_T_tag  = world_T_base ⊕ base_T_tag             (survey: robot pose known)
"""
import math

from robot_control.se2 import Pose2, compose, inverse


def obs_pose(obs):
    """Tag pose in base_link (theta = outward normal)."""
    return Pose2(obs.x, obs.y, obs.psi)


def pose_from_tag(obs, landmark):
    """Robot pose in world from one tag whose map pose is known."""
    return compose(landmark.pose(), inverse(obs_pose(obs)))


def tag_from_pose(obs, robot_pose):
    """Tag pose in world from a robot pose that is known."""
    return compose(robot_pose, obs_pose(obs))


def tag_weight(obs, min_dist=0.3):
    """Closer tags are trusted more (position noise grows ~ d^2)."""
    return 1.0 / max(obs.dist, min_dist) ** 2


def fuse_poses(poses, weights):
    """Weighted mean of poses (circular mean for theta)."""
    total = sum(weights)
    x = sum(w * p.x for p, w in zip(poses, weights)) / total
    y = sum(w * p.y for p, w in zip(poses, weights)) / total
    s = sum(w * math.sin(p.theta) for p, w in zip(poses, weights))
    c = sum(w * math.cos(p.theta) for p, w in zip(poses, weights))
    return Pose2(x, y, math.atan2(s, c))
