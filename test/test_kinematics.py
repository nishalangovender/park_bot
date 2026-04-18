"""Unit tests for park_bot.kinematics.

Reference: docs/kinematics.md (equations), docs/kinematic_fixtures.md (values).
Tests assert components are within 1e-5 of fixture values (6 dp rad / 4 dp rad/s).
"""

import math

import pytest

from park_bot.kinematics import KinematicMode, compute_wheel_targets, recover_twist

TOL = 1e-5
RT_TOL = 1e-9


def _assert_targets(got, expected_deltas, expected_omegas):
    deltas, omegas = got
    for i, (g, e) in enumerate(zip(deltas, expected_deltas)):
        assert abs(g - e) < TOL, f"delta[{i}] {g} != {e}"
    for i, (g, e) in enumerate(zip(omegas, expected_omegas)):
        assert abs(g - e) < TOL, f"omega[{i}] {g} != {e}"


# --- Ackermann ---

def test_ackermann_straight_forward():
    got = compute_wheel_targets((1.0, 0.0, 0.0), KinematicMode.ACKERMANN)
    _assert_targets(got, (0.0, 0.0, 0.0, 0.0), (20.0, 20.0, 20.0, 20.0))


def test_ackermann_zero_twist():
    got = compute_wheel_targets((0.0, 0.0, 0.0), KinematicMode.ACKERMANN)
    _assert_targets(got, (0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))


def test_ackermann_right_turn():
    got = compute_wheel_targets((1.0, 0.0, 0.5), KinematicMode.ACKERMANN)
    _assert_targets(
        got,
        (0.212938, 0.183943, 0.0, 0.0),
        (18.92749, 21.86893, 18.5, 21.5),
    )


# --- Crab ---

def test_crab_forward():
    got = compute_wheel_targets((1.0, 0.0, 0.0), KinematicMode.CRAB)
    _assert_targets(got, (0.0, 0.0, 0.0, 0.0), (20.0, 20.0, 20.0, 20.0))


def test_crab_sideways_left():
    got = compute_wheel_targets((0.0, 1.0, 0.0), KinematicMode.CRAB)
    half_pi = math.pi / 2
    _assert_targets(
        got,
        (half_pi, half_pi, half_pi, half_pi),
        (20.0, 20.0, 20.0, 20.0),
    )


def test_crab_diagonal():
    got = compute_wheel_targets((1.0, 1.0, 0.0), KinematicMode.CRAB)
    quarter_pi = math.pi / 4
    _assert_targets(
        got,
        (quarter_pi, quarter_pi, quarter_pi, quarter_pi),
        (28.28427, 28.28427, 28.28427, 28.28427),
    )


# --- Counter-steer ---

def test_counter_steer_tight_right():
    got = compute_wheel_targets((1.0, 0.0, 1.0), KinematicMode.COUNTER_STEER)
    _assert_targets(
        got,
        (0.231091, 0.172191, -0.231091, -0.172191),
        (17.46425, 23.34524, 17.46425, 23.34524),
    )


# --- Pivot ---

def test_pivot_ccw():
    got = compute_wheel_targets((0.0, 0.0, 1.0), KinematicMode.PIVOT)
    _assert_targets(
        got,
        (2.214297, 0.927295, -2.214297, -0.927295),
        (5.0, -5.0, -5.0, 5.0),
    )


# --- Edge cases ---

def test_ackermann_rejects_lateral():
    # Lateral velocity clamped to 0 for Ackermann.
    got = compute_wheel_targets((1.0, 0.5, 0.0), KinematicMode.ACKERMANN)
    _assert_targets(got, (0.0, 0.0, 0.0, 0.0), (20.0, 20.0, 20.0, 20.0))


def test_pivot_rejects_translation():
    got = compute_wheel_targets((1.0, 1.0, 0.0), KinematicMode.PIVOT)
    # Steering holds pivot geometry; all wheel speeds zero.
    assert got[1] == (0.0, 0.0, 0.0, 0.0)


def test_pivot_singularity_small_omega():
    got = compute_wheel_targets((0.0, 0.0, 1e-9), KinematicMode.PIVOT)
    assert all(abs(o) < 1e-7 for o in got[1])


# --- Round-trip ---

@pytest.mark.parametrize(
    "twist,mode",
    [
        ((1.0, 0.0, 0.0), KinematicMode.ACKERMANN),
        ((1.0, 0.0, 0.5), KinematicMode.ACKERMANN),
        ((1.0, 0.0, 0.0), KinematicMode.CRAB),
        ((0.0, 1.0, 0.0), KinematicMode.CRAB),
        ((1.0, 1.0, 0.0), KinematicMode.CRAB),
        ((1.0, 0.0, 1.0), KinematicMode.COUNTER_STEER),
        ((0.0, 0.0, 1.0), KinematicMode.PIVOT),
    ],
)
def test_round_trip(twist, mode):
    targets = compute_wheel_targets(twist, mode)
    got = recover_twist(targets, mode)
    for g, t in zip(got, twist):
        assert abs(g - t) < RT_TOL, f"round-trip {mode.name}: {got} != {twist}"
