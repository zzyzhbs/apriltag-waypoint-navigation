"""EKF for the robot pose [x, y, theta] in world (numpy, no ROS).

Predict once per primitive (drive d metres / spin a radians), update once per
tag seen in a look. Observation of a tag in base_link: z = [x_b, y_b, psi].
"""
import math
from dataclasses import dataclass

import numpy as np

from robot_control.se2 import Pose2, wrap_angle


@dataclass(frozen=True)
class EkfNoise:
    init_xy: float
    init_theta: float
    k_d: float
    c_d: float
    k_lat: float
    k_hd: float         # rad per m
    k_s: float
    c_s: float          # rad
    obs_a: float
    obs_b: float
    obs_c: float        # rad
    obs_e: float        # rad per m
    oblique: float      # rad
    oblique_factor: float
    gate: float


def noise_from_config(ekf):
    """`ekf:` section of nav.yaml -> EkfNoise (degrees -> radians)."""
    m, o, i = ekf['motion'], ekf['obs'], ekf['init_sigma']
    r = math.radians
    return EkfNoise(
        init_xy=i['xy'], init_theta=r(i['theta_deg']),
        k_d=m['k_d'], c_d=m['c_d'], k_lat=m['k_lat'], k_hd=r(m['k_hd_deg']),
        k_s=m['k_s'], c_s=r(m['c_s_deg']),
        obs_a=o['a'], obs_b=o['b'], obs_c=r(o['c_deg']), obs_e=r(o['e_deg']),
        oblique=r(o['oblique_deg']), oblique_factor=o['oblique_factor'],
        gate=ekf['gate'])


def state_from_pose(pose):
    return np.array([pose.x, pose.y, pose.theta], dtype=float)


def pose_from_state(x):
    return Pose2(float(x[0]), float(x[1]), wrap_angle(float(x[2])))


def initial_covariance(noise):
    return np.diag([noise.init_xy ** 2, noise.init_xy ** 2, noise.init_theta ** 2])


def sigmas(P):
    """Std of x, y, theta."""
    return tuple(float(math.sqrt(max(v, 0.0))) for v in np.diag(P))


# ---------- predict ----------

def drive_model(x, d):
    c, s = math.cos(x[2]), math.sin(x[2])
    return np.array([x[0] + d * c, x[1] + d * s, x[2]])


def drive_jacobian(x, d):
    F = np.eye(3)
    F[0, 2] = -d * math.sin(x[2])
    F[1, 2] = d * math.cos(x[2])
    return F


def drive_noise(x, d, noise):
    """Along-track, cross-track and heading noise, rotated into world."""
    ad = abs(d)
    body = np.diag([(noise.k_d * ad + noise.c_d) ** 2, (noise.k_lat * ad) ** 2,
                    (noise.k_hd * ad) ** 2])
    c, s = math.cos(x[2]), math.sin(x[2])
    G = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return G @ body @ G.T


def spin_noise(a, noise):
    Q = np.zeros((3, 3))
    Q[2, 2] = (noise.k_s * abs(a) + noise.c_s) ** 2
    return Q


def predict_drive(x, P, d, noise):
    F = drive_jacobian(x, d)
    return drive_model(x, d), F @ P @ F.T + drive_noise(x, d, noise)


def predict_spin(x, P, a, noise):
    x_new = x.copy()
    x_new[2] = wrap_angle(x[2] + a)
    return x_new, P + spin_noise(a, noise)


# ---------- update ----------

def expected_obs(x, lm):
    """Where the tag should appear in base_link if the robot is at x."""
    dx, dy = lm.x - x[0], lm.y - x[1]
    c, s = math.cos(x[2]), math.sin(x[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap_angle(lm.yaw - x[2])])


def obs_jacobian(x, lm):
    h = expected_obs(x, lm)
    c, s = math.cos(x[2]), math.sin(x[2])
    return np.array([[-c, -s, h[1]],
                     [s, -c, -h[0]],
                     [0.0, 0.0, -1.0]])


def obs_noise(obs, noise):
    s_xy = noise.obs_a + noise.obs_b * obs.dist ** 2
    s_psi = noise.obs_c + noise.obs_e * obs.dist
    if obs.oblique > noise.oblique:
        s_psi *= noise.oblique_factor
    return np.diag([s_xy ** 2, s_xy ** 2, s_psi ** 2])


def innovation(z, h):
    nu = z - h
    nu[2] = wrap_angle(nu[2])
    return nu


@dataclass
class UpdateResult:
    x: np.ndarray
    P: np.ndarray
    accepted: bool
    d2: float           # squared Mahalanobis distance of the innovation


def update(x, P, obs, lm, noise):
    """One tag observation. Rejected (state unchanged) when d2 > noise.gate."""
    z = np.array([obs.x, obs.y, obs.psi])
    H = obs_jacobian(x, lm)
    R = obs_noise(obs, noise)
    nu = innovation(z, expected_obs(x, lm))
    S = H @ P @ H.T + R
    d2 = float(nu @ np.linalg.solve(S, nu))
    if d2 > noise.gate:
        return UpdateResult(x, P, False, d2)
    K = P @ H.T @ np.linalg.inv(S)
    x_new = x + K @ nu
    x_new[2] = wrap_angle(x_new[2])
    I_KH = np.eye(3) - K @ H
    P_new = I_KH @ P @ I_KH.T + K @ R @ K.T      # Joseph form keeps P symmetric PD
    return UpdateResult(x_new, P_new, True, d2)
