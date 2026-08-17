# PID Gain Tuning Report — straight_line_assistant

**Date:** 2026-08-17
**Platform:** TurtleBot3 Burger (RPi 4, Ubuntu, ROS 2 Humble, `ROS_DOMAIN_ID=30`)
**Node:** `straight_line_assistant` (heading-hold PID, 20 Hz, parallel form)
**Result:** Gains changed from `kp=1.2, ki=0.01, kd=0.40` → **`kp=1.4, ki=0.30, kd=0.10`**, validated on hardware up to full speed (0.22 m/s).

---

## 1. Objective

Tune the heading-hold PID (yaw error → `angular.z` correction) analytically
instead of by trial-and-error: identify a plant model from logged data, design
gains with stability-margin targets, then validate incrementally on the real
robot.

## 2. Plant identification

Source data: `ident_data.csv` — open-loop chirp run (`ident_mode`, amplitude
0.4 rad/s, 0.05→1 Hz sweep, 20 s @ 20 Hz), input = commanded angular velocity,
output = EKF-filtered yaw from `/odometry/filtered`.

Two independent fits (frequency-response fit on angular rate; time-domain
least-squares on yaw) agree:

```
        yaw_filtered(s)         1.0 · e^(−0.10 s)
P(s) = -----------------  =  --------------------        R² = 0.95
          ω_cmd(s)             s · (0.17 s + 1)
```

- Gain K ≈ 1.0 — robot tracks commanded angular rate accurately.
- τ = 0.17 s lag + L = 0.10 s effective delay: motor/driver + OpenCR odometry
  + **EKF filtering lag** + 20 Hz zero-order hold. Total parasitic lag ≈ 0.27 s.
- Consequence: loop crossover must stay below ~1/(3·(τ+L)) ≈ 1.5 rad/s.

The 1/s is kinematics (yaw = ∫ω dt) — an **integrating plant**, so classic
Ziegler–Nichols reaction-curve / Cohen–Coon rules do not apply.

## 3. Diagnosis of the original gains (kp=1.2, ki=0.01, kd=0.4)

- Phase margin ≈ 93° — stable but over-damped; sluggish correction.
- **ki = 0.01 ≈ no integral action**: a constant wheel imbalance (~0.05 rad/s)
  leaves a ~2° permanent heading error that never converges (confirmed on
  hardware: `i_term` ≈ 0.0001 at end of run).
- **kd = 0.4 too large**: differentiates 20 Hz-quantized EKF yaw steps, causing
  control-output chatter at ~2.8 Hz (measured on hardware, peaks ±0.09 rad/s).

## 4. Gain design

Pole placement on `K/(s(τs+1))` with PD control gives
`kp = τωn²/K`, `kd = (2ζωnτ − 1)/K`. With ωn = 3.0 rad/s, ζ = 1.0:
kp ≈ 1.55, kd ≈ 0.05. Integral gain chosen from a phase-margin sweep
(ki = 0.3 → PM 65°; ki = 0.6 → PM 57°; ki = 1.0 → PM 47°, rejected).
All candidates verified in a discrete simulation replicating the node exactly
(20 Hz, ±2.84 rad/s saturation, integral clamp ±1.0, delay buffer).

Design targets: PM ≥ 50°, GM ≥ 8 dB, ωc ≈ 1.5 rad/s, no saturation in normal
operation, `ki · integral_limit ≤ 0.5 rad/s`.

MATLAB artifacts (for reproduction/refinement): `matlab/fit_plant.m`
(re-identification from any chirp CSV), `matlab/tune_pid.m` (margin table,
pole placement, `pidtune` cross-check), `matlab/sim_loop.m` (node-faithful
simulation + Simulink recipe).

## 5. Experimental validation (SSH-driven, key injection via tmux)

Protocol per run: engage heading-hold at 0.15 m/s for 9 s, log `/pid_debug`
at 20 Hz (~200 samples/run), one gain set per run.

| Run     | kp / ki / kd   | MAE (mrad) | Steady-state (mrad) | Output chatter | Peak correction |
|---------|----------------|-----------:|--------------------:|----------------|----------------:|
| baseline| 1.2 / 0.01 / 0.40 | 5.7 | 5.7  | **~2.8 Hz chatter** | 0.060 rad/s |
| stage 1 | 0.8 / 0.10 / 0.00 | 6.7 | 10.7 (creeping) | smooth | 0.014 rad/s |
| **stage 2** | **1.4 / 0.30 / 0.10** | 5.9 | 6.2 | smooth | 0.027 rad/s |
| stage 3 | 1.7 / 0.50 / 0.20 | 7.4 | 4.6 | mildly busy | 0.046 rad/s |

Full-speed check of stage 2 @ **0.22 m/s**: MAE 7.4 mrad (0.42°),
steady-state 3.3 mrad (0.19°), peak correction 0.032 rad/s — far from the
±2.84 rad/s saturation limit, no wag, no hunting.

Interpretation:
- Baseline's low MAE masked constant D-term chatter (wasted motor effort,
  audible jitter) and a dead integral term.
- Stage 1 was too soft: steady-state error grew through the run (ki too weak).
- Stage 2 matched baseline MAE with ~50% smoother control effort and a live
  integrator (nonzero `i_term` correcting a real heading bias).
- Stage 3 reduced steady-state error slightly but increased control activity
  with no MAE benefit — not worth the reduced margin (PM ≈ 55°).

**Selected gains: kp = 1.4, ki = 0.30, kd = 0.10, integral_limit = 1.0.**
Safe ceilings established from the plant model: kp ≤ 2.2, ki ≤ 0.8, kd ≤ 0.3.

## 6. Changes applied

- Robot: live parameters set to tuned values; `config/params.yaml` updated in
  both `src/` and `install/` trees (backups: `params.yaml.bak`).
- Repo: `scripts/apply_gains.sh` (staged gain presets), `matlab/` tuning
  scripts, this report; local `params.yaml` annotated with validation results.

## 7. Incidents

- **Pi rebooted mid-session (~17:04)** — no OOM in logs; likely battery
  voltage sag under combined load (cartographer + rviz + driving on a
  1.8 GB RPi 4). Recommend checking battery health and considering whether
  rviz needs to run on the robot.
- Relaunch required `LDS_MODEL=LDS-01` and sourcing `~/turtlebot3_ws` before
  `~/ros2_ws`; worth adding to the robot runbook.
- Cumulative test driving ≈ 7.5 m of straight-line runs.

## 8. Recommendations / next steps

1. Long-horizon drift run (30–60 s) and carpet-transition test; if residual
   bias persists, raise ki toward 0.5 (ceiling 0.8) watching for slow hunting.
2. Do **not** add an error deadband — EKF yaw noise floor is only 0.05 mrad,
   and a deadband on an integrating plant guarantees a standing error.
3. Consider gating heading-hold on IMU freshness (the EKF already detects
   IMU-stale and inflates yaw covariance; the PID currently only gates on
   odom staleness).
4. Re-run `ident_mode` on different surfaces and re-fit with
   `matlab/fit_plant.m` if the robot is deployed somewhere new.
