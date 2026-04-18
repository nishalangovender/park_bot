# Kinematic Fixtures

Fixture values used by both `test/test_kinematics.py` and `src/lib/park-bot/__tests__/kinematics.test.ts`. All values are computed from the equations in `docs/kinematics.md` with `L=0.40`, `W=0.30`, `r=0.05`.

Angles in radians, rounded to 6 decimal places. Wheel angular velocities in rad/s, rounded to 4 decimal places. A test passes if every asserted component is within `1e-5` of the table value.

## Ackermann

| Case | `(vx, vy, ω)` | δFL | δFR | δRL | δRR | ωFL | ωFR | ωRL | ωRR |
|------|---------------|-----|-----|-----|-----|-----|-----|-----|-----|
| A1 straight forward | `(1.0, 0.0, 0.0)` | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 20.0000 | 20.0000 | 20.0000 | 20.0000 |
| A2 zero twist | `(0.0, 0.0, 0.0)` | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| A3 right turn | `(1.0, 0.0, 0.5)` | 0.218669 | 0.196295 | 0.000000 | 0.000000 | 20.8327 | 19.1673 | 18.5000 | 21.5000 |

## Crab

| Case | `(vx, vy, ω)` | δFL | δFR | δRL | δRR | ωFL | ωFR | ωRL | ωRR |
|------|---------------|-----|-----|-----|-----|-----|-----|-----|-----|
| C1 forward | `(1.0, 0.0, 0.0)` | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 20.0000 | 20.0000 | 20.0000 | 20.0000 |
| C2 sideways left | `(0.0, 1.0, 0.0)` | 1.570796 | 1.570796 | 1.570796 | 1.570796 | 20.0000 | 20.0000 | 20.0000 | 20.0000 |
| C3 diagonal | `(1.0, 1.0, 0.0)` | 0.785398 | 0.785398 | 0.785398 | 0.785398 | 28.2843 | 28.2843 | 28.2843 | 28.2843 |

## Counter-steer

| Case | `(vx, vy, ω)` | δFL | δFR | δRL | δRR | ωFL | ωFR | ωRL | ωRR |
|------|---------------|-----|-----|-----|-----|-----|-----|-----|-----|
| S1 tight right | `(1.0, 0.0, 1.0)` | 0.110657 | 0.099669 | -0.110657 | -0.099669 | 20.2237 | 19.7763 | 20.2237 | 19.7763 |

## Pivot

| Case | `(vx, vy, ω)` | δFL | δFR | δRL | δRR | ωFL | ωFR | ωRL | ωRR |
|------|---------------|-----|-----|-----|-----|-----|-----|-----|-----|
| P1 CCW | `(0.0, 0.0, 1.0)` | 2.214297 | 0.927295 | -2.214297 | -0.927295 | 5.0000 | -5.0000 | -5.0000 | 5.0000 |

## Round-trip cases

For each fixture above, `recover_twist(compute_wheel_targets(twist, mode), mode) == twist` within 1e-9 — except A2 (trivially zero) and P1 where steering angles are input-independent.
