"""Standalone 4WS kinematics.

Pure-Python. No ROS2 dependency. Implements docs/kinematics.md:

    compute_wheel_targets(twist, mode) -> (deltas, omegas)
    recover_twist(wheel_targets, mode) -> twist

Both halves of the mapping are tested against docs/kinematic_fixtures.md.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Tuple

# Vehicle parameters (metres / metres / metres).
L = 0.40
W = 0.30
R = 0.05

# Flat tuple aliases — keeps signatures simple for the tests.
Twist = Tuple[float, float, float]                    # (vx, vy, omega)
WheelAngles = Tuple[float, float, float, float]       # (FL, FR, RL, RR)
WheelSpeeds = Tuple[float, float, float, float]       # (FL, FR, RL, RR), rad/s
WheelTargets = Tuple[WheelAngles, WheelSpeeds]


class KinematicMode(Enum):
    ACKERMANN = "ackermann"
    CRAB = "crab"
    COUNTER_STEER = "counter_steer"
    PIVOT = "pivot"


# --- Forward mapping (twist -> wheel targets) ---

def compute_wheel_targets(twist: Twist, mode: KinematicMode) -> WheelTargets:
    vx, vy, omega = twist
    if mode is KinematicMode.ACKERMANN:
        return _ackermann(vx, omega)
    if mode is KinematicMode.CRAB:
        return _crab(vx, vy)
    if mode is KinematicMode.COUNTER_STEER:
        return _counter_steer(vx, omega)
    if mode is KinematicMode.PIVOT:
        return _pivot(omega)
    raise ValueError(f"unknown mode: {mode!r}")


def _ackermann(vx: float, omega: float) -> WheelTargets:
    # Clamp lateral velocity (vy ignored by caller).
    if abs(omega) < 1e-9:
        speed = vx / R
        return (0.0, 0.0, 0.0, 0.0), (speed, speed, speed, speed)
    radius = vx / omega  # signed turning radius
    d_fl = math.atan2(L, radius - W / 2)
    d_fr = math.atan2(L, radius + W / 2)
    r_fl = math.hypot(L, radius - W / 2)
    r_fr = math.hypot(L, radius + W / 2)
    r_rl = abs(radius - W / 2)
    r_rr = abs(radius + W / 2)
    w_fl = math.copysign(omega * r_fl / R, vx)
    w_fr = math.copysign(omega * r_fr / R, vx)
    w_rl = math.copysign(omega * r_rl / R, vx)
    w_rr = math.copysign(omega * r_rr / R, vx)
    return (d_fl, d_fr, 0.0, 0.0), (w_fl, w_fr, w_rl, w_rr)


def _crab(vx: float, vy: float) -> WheelTargets:
    speed = math.hypot(vx, vy)
    phi = math.atan2(vy, vx) if speed > 0 else 0.0
    w = speed / R
    return (phi, phi, phi, phi), (w, w, w, w)


def _counter_steer(vx: float, omega: float) -> WheelTargets:
    if abs(omega) < 1e-9:
        speed = vx / R
        return (0.0, 0.0, 0.0, 0.0), (speed, speed, speed, speed)
    half_l = L / 2
    radius = vx / omega
    d_fl = math.atan2(half_l, radius - W / 2)
    d_fr = math.atan2(half_l, radius + W / 2)
    r_fl = math.hypot(half_l, radius - W / 2)
    r_fr = math.hypot(half_l, radius + W / 2)
    w_fl = math.copysign(omega * r_fl / R, vx)
    w_fr = math.copysign(omega * r_fr / R, vx)
    return (d_fl, d_fr, -d_fl, -d_fr), (w_fl, w_fr, w_fl, w_fr)


def _pivot(omega: float) -> WheelTargets:
    half_l = L / 2
    half_w = W / 2
    # Steering angles point tangent to the circle centred on chassis origin.
    d_fl = math.atan2(half_l, -half_w)
    d_fr = math.atan2(half_l, half_w)
    d_rl = math.atan2(-half_l, -half_w)
    d_rr = math.atan2(-half_l, half_w)
    rho = math.hypot(half_l, half_w)
    speed = omega * rho / R
    # Sign pattern: +omega (CCW) drives FL and RR forward, FR and RL backward.
    return (d_fl, d_fr, d_rl, d_rr), (speed, -speed, -speed, speed)


# --- Inverse mapping (targets -> twist) ---

def recover_twist(targets: WheelTargets, mode: KinematicMode) -> Twist:
    deltas, omegas = targets
    if mode is KinematicMode.ACKERMANN:
        return _recover_ackermann(deltas, omegas)
    if mode is KinematicMode.CRAB:
        return _recover_crab(deltas, omegas)
    if mode is KinematicMode.COUNTER_STEER:
        return _recover_counter_steer(deltas, omegas)
    if mode is KinematicMode.PIVOT:
        return _recover_pivot(omegas)
    raise ValueError(f"unknown mode: {mode!r}")


def _recover_ackermann(deltas, omegas):
    d_fl, d_fr, _, _ = deltas
    w_fl, w_fr, w_rl, w_rr = omegas
    if abs(d_fl) < 1e-12 and abs(d_fr) < 1e-12:
        vx = w_fl * R
        return vx, 0.0, 0.0
    # Derive body speed from average rear-wheel speed.
    v_rear = ((w_rl + w_rr) / 2) * R
    # Recover turning radius from front-left steer angle.
    # d_fl = atan2(L, radius - W/2) => radius - W/2 = L/tan(d_fl)
    radius = L / math.tan(d_fl) + W / 2
    omega = v_rear / radius
    vx = v_rear
    return vx, 0.0, omega


def _recover_crab(deltas, omegas):
    phi = deltas[0]
    speed = omegas[0] * R
    return speed * math.cos(phi), speed * math.sin(phi), 0.0


def _recover_counter_steer(deltas, omegas):
    d_fl = deltas[0]
    w_fl = omegas[0]
    if abs(d_fl) < 1e-12:
        return w_fl * R, 0.0, 0.0
    half_l = L / 2
    # d_fl = atan2(half_l, radius - W/2) => radius - W/2 = half_l/tan(d_fl)
    radius = half_l / math.tan(d_fl) + W / 2
    r_fl = math.hypot(half_l, radius - W / 2)
    v_fl = w_fl * R
    omega = v_fl / r_fl
    vx = omega * radius
    return vx, 0.0, omega


def _recover_pivot(omegas):
    rho = math.hypot(L / 2, W / 2)
    # FL wheel speed = omega * rho / R (positive for +omega)
    omega = omegas[0] * R / rho
    return 0.0, 0.0, omega
