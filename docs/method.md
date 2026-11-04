# Method details

This page gives the equations and rules behind the [README](../README.md). Parameter values come from
[`nav.yaml`](../robot_control/config/hw2/nav.yaml); the code is in
[`robot_control/robot_control/`](../robot_control/robot_control).

## Frames and state

The world frame has its origin at the start point, x along the robot's start heading, and y to the
left. `base_link` is the robot centre. The camera is 0.075 m ahead of the centre and 0.125 m above the
floor, measured by hand (`hw2_perception_launch.py`).

The filter state is the planar pose $`\mathbf{x} = [x, y, \theta]^\top`$ with covariance $`P`$. It starts at
$`(0, 0, 0)`$ with standard deviations of 1 cm and 1°.

## Motion primitives and timing

A primitive is a fixed wheel command held for a duration $`T`$: a straight drive of length $`d`$, or an
in-place spin of angle $`a`$. There is no feedback during a primitive.

For each direction and command level, hand-measured trials give a line fit (`ol_model.py`):

```math
\text{amount} = \text{rate} \cdot T + \text{offset}.
```

The planner inverts this line. For spins it then multiplies $`T`$ by a direction-specific `time_scale`
(0.83 for CCW and 0.77 for CW in `study_wood2_hw2_ext.yaml`). A level's smallest step is the fit
evaluated at $`\max(T_{\min}, T_{\min} / \text{time\_scale})`$ (`step_timing.min_amount`).

`step_timing.pick_level` chooses the level, trying fast before slow:

1. the fastest level whose $`T`$ is within its calibrated range, with 20 % allowed outside it;
2. otherwise, for a long move, the fastest level;
3. otherwise the slowest level.

Spins above `fine_max_deg` (20°) may use only the coarse level (`stop_look.to_step`).

## EKF prediction

The filter predicts once per primitive, using the nominal amount (`ekf.py`). The EKF follows the
standard form [Thrun et al., ch. 3 and 7].

**Drive of length $`d`$:**

```math
\mathbf{x}^+ = \mathbf{x} + \begin{bmatrix} d\cos\theta \\ d\sin\theta \\ 0 \end{bmatrix}, \qquad F = \begin{bmatrix} 1 & 0 & -d\sin\theta \\ 0 & 1 & d\cos\theta \\ 0 & 0 & 1 \end{bmatrix},
```

```math
Q = G(\theta)\,\mathrm{diag}\big((k_d|d| + c_d)^2,\ (k_\text{lat}|d|)^2,\ (k_\text{hd}|d|)^2\big)\,G(\theta)^\top,
```

where $`G(\theta)`$ rotates along-track and cross-track noise into the world frame.

**Spin of angle $`a`$:**

```math
\theta^+ = \theta + a, \qquad F = I, \qquad Q_{\theta\theta} = (k_s|a| + c_s)^2 .
```

In both cases $`P^+ = F P F^\top + Q`$.

## Tag observations

`tag_listener.py` looks up each detected tag in `base_link` through TF, using the image timestamp. An
observation is:

```math
\mathbf{z} = [x_b,\ y_b,\ \psi]^\top,
```

where $`(x_b, y_b)`$ is the tag centre in `base_link` and $`\psi`$ is the direction of the tag's outward
normal in `base_link`.

One look is a 1 s window with the robot stopped, after a 1 s settle. `tag_obs.filter_window` then
handles each tag as follows:

| Check | Rule | Logged reason if it fails |
|---|---|---|
| Known tag | id is in the map | `not in map` |
| Geometry, per frame | $`0.25 \le d \le 3`$ m and obliquity $`\le 60^\circ`$ | `out of range` |
| Frame count | at least 3 usable frames | `only N frames` |
| Spread | larger of the x and y standard deviations $`\le 2`$ cm | `unstable` |

A tag that passes becomes one observation: the median of $`x_b`$ and $`y_b`$ and the circular mean of $`\psi`$.

## EKF update

For a landmark at $`\mathbf{p}_l`$ with normal direction $`\psi_l`$, and $`d_x = x_l - x`$, $`d_y = y_l - y`$:

```math
h(\mathbf{x}) = \begin{bmatrix} \cos\theta\, d_x + \sin\theta\, d_y \\ -\sin\theta\, d_x + \cos\theta\, d_y \\ \psi_l - \theta \end{bmatrix}, \qquad H = \begin{bmatrix} -\cos\theta & -\sin\theta & h_2 \\ \sin\theta & -\cos\theta & -h_1 \\ 0 & 0 & -1 \end{bmatrix}.
```

Measurement noise depends on the distance $`d`$ to the tag:

```math
R = \mathrm{diag}(\sigma_{xy}^2,\ \sigma_{xy}^2,\ \sigma_\psi^2), \qquad \sigma_{xy} = a + b\,d^2, \qquad \sigma_\psi = c + e\,d .
```

$`\sigma_\psi`$ is doubled when the obliquity exceeds 30°.

With innovation $`\boldsymbol{\nu} = \mathbf{z} - h(\mathbf{x})`$ (heading wrapped) and
$`S = HPH^\top + R`$, the update is:

```math
K = PH^\top S^{-1}, \qquad \mathbf{x}^+ = \mathbf{x} + K\boldsymbol{\nu}, \qquad P^+ = (I - KH)P(I - KH)^\top + KRK^\top .
```

Tags in one look are applied one at a time, nearest first, each using the state left by the previous
update.

## Chi-square gate

Before each update the filter computes the squared Mahalanobis distance of the innovation:

```math
d^2 = \boldsymbol{\nu}^\top S^{-1} \boldsymbol{\nu}.
```

The observation is rejected when $`d^2 > 11.34`$, the 99 % point of a chi-square distribution with 3
degrees of freedom [Bar-Shalom et al.; Bar-Shalom and Fortmann].

`Navigator.check_consistency` raises `NavigationHalt` after two consecutive looks in which tags passed
the look checks and the gate rejected all of them. A look with no usable tag neither counts toward this
nor resets it. The exception is raised before the look is written to `observations.csv`, so the last
look of a halted run is not in the log.

## Waypoint control

`stop_look.next_move` plans from the current estimate. With distance $`D`$ to the goal and bearing error
$`\beta`$:

1. If $`D`$ exceeds 3 cm and a drive of length $`D`$ is executable, align and then drive:
   - Turn first when $`|\beta|`$ exceeds the alignment tolerance
     $`\min(\max(\operatorname{atan2}(0.03, D),\ 3^\circ),\ 15^\circ)`$ and that turn is executable.
   - Otherwise drive $`D\cos\beta`$. A distance $`L \ge 0.3`$ m is shortened by
     $`\min(0.03L + 0.08,\ L/2)`$ and capped at 1.2 m, because the robot has no reverse primitive and
     an overshoot would mean turning around.
2. If no drive is planned and the heading error exceeds 3°, turn to the goal heading.
3. Otherwise no move remains.

Two rules then adjust the plan (`Navigator.choose_move`):

- **Search.** If no move remains and this waypoint has had no tag update, turn toward the nearest
  reachable tag not yet searched for, at most twice.
- **Split.** A turn that would end with no tag expected in view is shortened so the last visible tag
  stops at the edge of the view (35° from the heading), provided the shortened turn is at least 15°.

## How a waypoint ends

`Navigator.run_goal` allows 15 looks per waypoint. This is what the `reached` column in `waypoints.csv`
means:

| Condition | `reached` | `note` |
|---|---|---|
| No move remains | 1 | empty, or `no tag seen; reached by prediction only` |
| Planned move is below the smallest step | 1 | `remaining drive/spin is below the smallest step` |
| No level can execute the move | 0 | `no calibrated level for ...` |
| 15 looks used | 0 | `max_iters reached` |

So `reached = 1` does not mean the estimate is within tolerance.

## Tag placement

Each tag stands on the extension of one leg of the route, 0.5 m past the leg's end, and faces back along
the leg, so the robot looks at it while driving that leg (`landmarks.yaml`).

| Tag | Leg | Position (m) | Normal direction |
|---|---|---|---|
| 0 | (0, 0) to (1, 0) | (1.5, 0) | 180° |
| 1 | (1, 0) to (1, 2) | (1, 2.5) | −90° |
| 2 | (1, 2) to (0, 0) | (−0.224, −0.447) | 63.43° |

At WP2 the goal heading of 180° faces no tag. The split rule stops the turn at about 125°, which keeps
tag 1 at the edge of the view, so the last 55° or so is not corrected.

## Parameters

| Parameter | Value | Meaning |
|---|---|---|
| `k_d`, `c_d` | 0.08, 0.01 m | along-track standard deviation per metre, and per drive |
| `k_lat` | 0.03 | cross-track standard deviation per metre |
| `k_hd_deg` | 2.0° | heading standard deviation per metre driven |
| `k_s`, `c_s_deg` | 0.05, 2.0° | heading standard deviation per unit of spin, and per spin |
| `a`, `b` | 0.005 m, 0.01 | $`\sigma_{xy} = a + b\,d^2`$ |
| `c_deg`, `e_deg` | 1.5°, 1.5° per m | $`\sigma_\psi = c + e\,d`$ |
| `oblique_deg`, `oblique_factor` | 30°, 2 | obliquity above which $`\sigma_\psi`$ is scaled, and the scale |
| `gate` | 11.34 | chi-square threshold |
| `pos`, `ang_deg` | 0.03 m, 3° | waypoint targets |
| `max_iters` | 15 | looks per waypoint |

## References

See the [README](../README.md#references) for full citations.
