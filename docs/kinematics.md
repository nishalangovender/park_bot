# 4WS Kinematics Reference

This document is the single written source of truth for the `park_bot` 4WS kinematics. Both the Python module `park_bot/kinematics.py` and the TypeScript module `src/lib/park-bot/kinematics.ts` in the [`nishalangovender.com`](https://nishalangovender.com/projects/park-bot/demo) portfolio demo implement the equations below. A shared fixture table (`docs/kinematic_fixtures.md`) keeps both implementations in lockstep — drift surfaces as a failing test on either side.

## Vehicle model

Rectangular chassis with four independently-steered, independently-driven wheels at the corners. All dimensions in metres, angles in radians, velocities in m/s and rad/s.

| Symbol | Meaning                                         | Default |
|--------|-------------------------------------------------|---------|
| `L`    | Wheelbase (front-axle to rear-axle, longitudinal) | 0.40    |
| `W`    | Track (left wheel to right wheel, lateral)        | 0.30    |
| `r`    | Wheel radius                                    | 0.05    |

Wheel indexing: `FL` front-left, `FR` front-right, `RL` rear-left, `RR` rear-right.

Body frame: `x̂` forward, `ŷ` left, `ẑ` up. Body twist `(vₓ, vᵧ, ω)` expressed in the body frame.

## Modes

Four discrete kinematic modes. Each mode is a map from a desired body twist `(vₓ, vᵧ, ω)` to a target steering angle and linear speed for each wheel.

### 1. Ackermann

Rear wheels straight. Front wheels steer to achieve the commanded yaw rate at the commanded forward speed.

```
Turning radius:    R = vₓ / ω          (for ω ≠ 0)
δ_FL = atan2(L, R - W/2)
δ_FR = atan2(L, R + W/2)
δ_RL = 0
δ_RR = 0

wheel speed (each): |v_i| = |vₓ| + |ω| · r_i, where r_i is the wheel's distance from the instantaneous centre of rotation (ICR).
```

If `ω = 0`: all four δ = 0; all four wheel speeds = `vₓ / r`.

Ackermann rejects lateral velocity commands (`vᵧ ≠ 0` is clamped to 0 with a warning).

### 2. Crab

All four wheels steer the same direction. The body translates along the commanded `(vₓ, vᵧ)` vector without rotating. Yaw commands are clamped to 0.

```
heading φ = atan2(vᵧ, vₓ)
δ_FL = δ_FR = δ_RL = δ_RR = φ
speed s = hypot(vₓ, vᵧ)
wheel speed (each) = s / r
```

### 3. Counter-steer

Front wheels and rear wheels steer in opposite directions. Turning radius about half of the Ackermann equivalent.

```
half-L = L / 2
R = vₓ / ω
δ_FL = atan2(half-L, R - W/2)
δ_FR = atan2(half-L, R + W/2)
δ_RL = -atan2(half-L, R - W/2)
δ_RR = -atan2(half-L, R + W/2)
```

### 4. Pivot

Zero translation; pure yaw about the chassis centre. All four wheels steer tangent to a circle centred on the chassis.

```
δ_FL = atan2( L/2, -W/2)       // points to chassis centre
δ_FR = atan2( L/2,  W/2)
δ_RL = atan2(-L/2, -W/2)
δ_RR = atan2(-L/2,  W/2)

radius ρ_i = hypot(L/2, W/2)    // identical for all four corners
wheel speed magnitude = |ω| · ρ_i / r
wheel direction: +ω (CCW) drives FL and RR forward (in their steered frame), FR and RL backward.
```

Pivot rejects `vₓ ≠ 0` or `vᵧ ≠ 0` commands (clamped to 0).

## Mode dispatch API

```
compute_wheel_targets(twist, mode) → WheelTargets
```

where

```
twist         = (vx, vy, omega)                         # m/s, m/s, rad/s
mode          ∈ {ACKERMANN, CRAB, COUNTER_STEER, PIVOT}
WheelTargets  = ((δFL, δFR, δRL, δRR),                  # rad
                 (ωFL, ωFR, ωRL, ωRR))                  # wheel angular velocity, rad/s
```

## Inverse mapping (forward kinematics)

For completeness (used for unit-testing round-trip), given wheel targets and the active mode, `recover_twist(wheel_targets, mode) → twist` reverses the equations above. Round-trip `recover_twist(compute_wheel_targets(t, m), m) ≈ t` within 1e-9 for all modes except the singular cases (ω = 0 under Ackermann/Counter-steer, or `twist = 0` under Pivot).

## Edge cases

- **Zero twist.** `(0, 0, 0)`: all wheel speeds 0; steering angles retained at last committed values (Ackermann and Crab hold; Pivot and Counter-steer snap to their characteristic geometry).
- **Pivot singularity.** A pivot with `|ω| < 1e-6` produces a zero wheel-speed result but retains the tangent steering geometry.
- **Ackermann straight.** `ω = 0`: all δ = 0, wheel speeds equal.
- **Mode switches.** Modes are discrete; there is no continuous blend. A mode switch is expected to be preceded by a brief `(0, 0, 0)` twist so the wheels can re-angle before motion resumes.

## Sign conventions

- Positive `vₓ` = forward.
- Positive `vᵧ` = left.
- Positive `ω` = CCW about `ẑ`.
- Positive δ = wheel steers towards `+ŷ` (left).
- Positive wheel angular velocity = rolling forward in the wheel's currently-steered direction.
