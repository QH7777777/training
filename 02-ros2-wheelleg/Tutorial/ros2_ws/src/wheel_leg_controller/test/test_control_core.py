"""Verify motor conventions, saturation recovery, and VMC geometry."""

import math

import pytest

from wheel_leg_controller.control_core import ControlCore, PID


def joints(right_speed=0.0, left_speed=0.0):
    """Return symmetric leg state with independently specified wheel speeds."""
    return {'right1': (0.0, 0.0), 'right2': (0.0, 0.0),
            'left1': (0.0, 0.0), 'left2': (0.0, 0.0),
            'rightwheel': (0.0, right_speed), 'leftwheel': (0.0, left_speed)}


def test_wheel_direction_and_radius():
    """Positive forward velocity needs opposite native motor signs."""
    core = ControlCore('wheel_pi')
    cmd, values = core.compute(joints(-5.0, 5.0), (0, 0, 0, 1), 0, 0.5, 0.12)
    assert values[:2] == pytest.approx([0.3, 0.3])
    assert cmd[:4] == [0.0] * 4
    assert -2.0 <= cmd[4] < 0.0 < cmd[5] <= 2.0
    core = ControlCore('wheel_pi')
    cmd, _ = core.compute(joints(), (0, 0, 0, 1), 0, -0.5, 0.12)
    assert cmd[4] > 0.0 > cmd[5]


def test_integrator_recovers_after_saturation():
    """A long unreachable command must not accumulate hidden integral."""
    pid = PID(1.0, 2.0, 0.0, 2.0)
    for _ in range(5000):
        assert pid.update(100.0) == 2.0
    assert pid.integral == 0.0
    assert pid.update(-0.1) < 0.0


def test_vmc_symmetry_and_native_leg_signs():
    """Identical leg geometry yields mirrored motor torques within VMC limits."""
    core = ControlCore('vmc')
    cmd, values = core.compute(joints(), (0, 0, 0, 1), 0, 0, 0.12)
    expected = 0.075 + math.sqrt(0.14**2 - 0.04**2)
    assert values[2:4] == pytest.approx([expected, expected])
    assert cmd[0] == pytest.approx(-cmd[2])
    assert cmd[1] == pytest.approx(-cmd[3])
    assert all(abs(t) <= 1.8 for t in cmd[:4])
    # Preserve original vmc_cal.py derivative behavior, as the assignment asks.
    assert core.legs[0].d_L_0 == core.legs[0].d_phi_0 == 0.0


def test_lqr_position_reference_is_frozen_while_stationary():
    """Drift must remain visible to the position loop instead of being erased."""
    core = ControlCore('lqr')
    state = joints()
    state['rightwheel'] = (-1.0, 0.0)
    state['leftwheel'] = (1.0, 0.0)
    core.compute(state, (0, 0, 0, 1), 0, 0, 0.10)
    assert all(lqr.x_b_set == pytest.approx(0.06) for lqr in core.lqrs)
    state['rightwheel'] = (-2.0, 0.0)
    state['leftwheel'] = (2.0, 0.0)
    core.compute(state, (0, 0, 0, 1), 0, 0, 0.10)
    for lqr in core.lqrs:
        assert lqr.x_b == pytest.approx(0.12)
        assert lqr.x_b_set == pytest.approx(0.06)


def test_lqr_states_match_mirrored_encoders_and_imu():
    """Both LQR legs must see common forward speed and the original pitch sign."""
    core = ControlCore('lqr')
    pitch = 0.05
    q = (0, math.sin(pitch / 2), 0, math.cos(pitch / 2))
    cmd, measured = core.compute(joints(-5.0, 5.0), q, 0.2, 0, 0.10)
    for lqr in core.lqrs:
        assert lqr.v_b_whole == pytest.approx(0.3)
        assert lqr.phi == pytest.approx(-pitch)
        assert lqr.d_phi == pytest.approx(-0.2)
        assert any(abs(k) > 1e-6 for k in lqr.K)
    assert measured[6] == pytest.approx(-pitch)
    assert all(abs(t) <= 1.8 for t in cmd[:4])
    assert all(abs(t) <= 2.0 for t in cmd[4:])
    assert all(wheel.previous is None for wheel in core.wheels)


def test_lqr_position_reference_follows_motion_then_freezes():
    """Move and stop without resetting the position target every later step."""
    core = ControlCore('lqr')
    state = joints()
    core.compute(state, (0, 0, 0, 1), 0, 0.5, 0.10)
    assert core.velocity_setpoint == pytest.approx(0.8 * 0.002)
    state['rightwheel'] = (-2.0, 0.0)
    state['leftwheel'] = (2.0, 0.0)
    core.compute(state, (0, 0, 0, 1), 0, 0.5, 0.10)
    assert core.lqrs[0].x_b_set == pytest.approx(0.12)
    for _ in range(10):
        core.compute(state, (0, 0, 0, 1), 0, 0, 0.10)
    assert core.velocity_setpoint == 0.0
    state['rightwheel'] = (-3.0, 0.0)
    state['leftwheel'] = (3.0, 0.0)
    core.compute(state, (0, 0, 0, 1), 0, 0, 0.10)
    assert core.lqrs[0].x_b_set == pytest.approx(0.12)


def test_lqr_impact_recovery_slows_and_requires_stable_attitude():
    """A landing disturbance lowers speed demand until posture is recaptured."""
    core = ControlCore('lqr')
    state = joints(-5.0, 5.0)
    for _ in range(250):
        core.compute(state, (0, 0, 0, 1), 0, 0.3, 0.10)
    assert core.recovery_armed
    cruising_setpoint = core.velocity_setpoint

    core.compute(state, (0, 0, 0, 1), 1.0, 0.3, 0.10)
    assert core.recovery_active
    assert core.velocity_setpoint < cruising_setpoint

    q_tilted = (0, math.sin(math.radians(10.0) / 2), 0,
                math.cos(math.radians(10.0) / 2))
    core.compute(state, q_tilted, 0.0, 0.3, 0.10)
    assert core.recovery_active

    for _ in range(149):
        core.compute(state, (0, 0, 0, 1), 0.0, 0.3, 0.10)
    assert core.recovery_active
    core.compute(state, (0, 0, 0, 1), 0.0, 0.3, 0.10)
    assert not core.recovery_active
    assert core.recovery_cooldown > 0.0


def test_lqr_traction_assist_is_bounded_and_clears_on_stop():
    """Sustained uphill stall adds forward torque without stale drive at rest."""
    core = ControlCore('lqr')
    state = joints()
    first, _ = core.compute(state, (0, 0, 0, 1), 0, 0.2, 0.10)
    for _ in range(500):
        moving, _ = core.compute(state, (0, 0, 0, 1), 0, 0.2, 0.10)
    assert 0.0 < core.traction_assist <= 1.0
    assert moving[4] > first[4]
    assert moving[5] < first[5]
    core.compute(state, (0, 0, 0, 1), 0, -0.2, 0.10)
    assert core.traction_direction == -1.0
    assert core.traction_assist < 0.01
    core.compute(state, (0, 0, 0, 1), 0, 0.0, 0.10)
    assert core.traction_assist == 0.0
