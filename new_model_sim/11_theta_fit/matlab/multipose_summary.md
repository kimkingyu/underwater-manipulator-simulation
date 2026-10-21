# Multi-pose check of the theta model (2026-10-22)

Data: q = 0 set (22 cases, `theta_dataset.json`) + three extra poses, 8 steady cases each
(±x/±y/±z at 0.8 m/s, +x/+y at 0.3 m/s):

| pose | q [rad] | cases | notes |
|---|---|---|---|
| qb8 | [0.45, -0.50, -0.40] | 8 | T_pz_08 capped at 2500 it., tail mean |
| qc | [0, -0.80, 0] | 8 | all converged |
| qd | [0, 0, -0.80] | 8 | T_nz_08 capped at 2500 it., tail mean; mesh needed 0.3 mm on 3 pin4 sliver faces |

Joint range found first (`13_pose_q1/joint_limits.json`): q = 0 is the positive mechanical stop of
J2 and J3, only [-1.57, 0] is reachable there; J1 is free over ±1.57.

All errors: RMS(pred - CFD) / q = 0 data-set RMS of the group (one fixed scale for every row).

## Leave-one-pose-out (train on q = 0 + the other two poses), `scan_link_terms_pose.m`

| link terms | F | M | tau | link | linkM |
|---|---|---|---|---|---|
| none | 0.244 | 0.480 | 0.870 | 0.849 | 1.489 |
| cop_moment (current) | 0.244 | 0.481 | 0.813 | 0.850 | 1.053 |
| cross_force | 0.245 | 0.477 | 0.775 | 0.823 | 1.489 |
| cross_force_l3 | 0.244 | 0.476 | 0.799 | 0.848 | 1.489 |
| cop + cross_l3 | 0.244 | 0.479 | 0.802 | 0.848 | 1.052 |

q = 0 theta, no refit, on the same poses (`eval_theta_poses.m`): tau 0.574 / 0.715 / 1.027
(qb8 / qc / qd), RMS 0.79 -- about the same as the multi-pose leave-one-out above. Adding poses to
the fit does not improve prediction at an unseen pose.

## Per joint, RMS torque error / RMS CFD torque [N m], q = 0 theta

| pose | J1 | J2 | J3 |
|---|---|---|---|
| qb8 | 0.09 / 0.77 | 0.15 / 0.25 | 0.14 / 0.30 |
| qc | 0.07 / 0.73 | 0.22 / 0.19 | 0.16 / 0.19 |
| qd | 0.20 / 0.64 | 0.29 / 0.42 | 0.19 / 0.35 |

J1 is predicted well at every pose (error 10-30 % of the torque). J2 and J3 errors are as large as the
torques themselves: the model has no skill there. None of the link-term forms changes this (J2/J3
leave-one-pose-out errors stay within ±0.05 N m across all five variants).

## Why

J2/J3 torques change sign with the pose for the same current, e.g. +y at 0.8 m/s:
J3 -0.22 (q0), -0.50 (qb8), -0.07 (qc), -0.07 (qd); -z: J2 +0.26, +0.44, +0.26, +0.84 and
J3 -0.20, -0.16, +0.25, -0.74. Every link term in the model is a fixed coefficient times the link's
own velocity, so the arm's links and the hull shadowing each other (which depends on the pose) cannot
be represented. A per-pose parameter set, or a q-dependent term fitted on a pose grid, would be needed;
three poses are not enough to identify the latter.

## Decision

The q = 0 theta (`theta_fit.mat`, cop_moment) stays the default. Its J2/J3 torques should be read as
order-of-magnitude only away from q = 0.
