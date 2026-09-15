# Passive LTV observer

This directory implements only the first, linear time-varying observer from
Wang and Tayebi, *Pose, Velocity and Landmark Position Estimation Using IMU
and Bearing Measurements* (ACC 2025).

The state is ordered as `[body_landmarks, body_velocity, body_gravity]`.
`LtvObserver::propagateImu()` implements equations (10), (14), (15), and the
propagation part of (17). `LtvObserver::updateFeatures()` implements modified
bearing output (8) and the camera correction terms of (15)--(17). Camera
correction uses adaptive explicit-Euler substeps while holding the current
measurement fixed; this is an engineering discretization, not a discrete
equation given by the paper.

VINS supplies `R_BC=RIC[0]`, `p_BC=TIC[0]`, normalized camera0 coordinates,
and the latest completed accelerometer/gyroscope bias estimates. The observer
is a passive one-way branch and does not alter preintegration, reprojection,
optimization, marginalization, or feature tracking.

The practical implementation caps and dynamically replaces landmarks, so the
paper's fixed-landmark and persistence-of-excitation assumptions do not apply
unchanged. If observer snapshots are later used as optimizer factors, they are
also correlated with the existing VINS measurements; their covariance must not
be treated as that of an independent sensor and the combined system does not
inherit the paper's GES/AGAS guarantees.
