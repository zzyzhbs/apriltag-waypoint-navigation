"""Single-tag pose solutions (no ROS).

world_T_base = world_T_tag ⊕ inverse(base_T_tag)    (localize: map known)
world_T_tag  = world_T_base ⊕ base_T_tag             (survey: robot pose known)
"""
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
