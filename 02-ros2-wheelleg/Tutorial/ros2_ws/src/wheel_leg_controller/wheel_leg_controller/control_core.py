"""Wheel PI, VMC and LQR control, independent of ROS and the simulator."""

import math

from .vmc_cal import five_links_param
from .lqr_update import lqr_controller
from .roll_leg_control import RollLegController


DT = 0.002
WHEEL_RADIUS = 0.06


def clip(value, lower, upper):
    """Limit a scalar to its physical bounds."""
    return max(lower, min(upper, value))


class PID:
    """Discrete PID with conditional integration and output saturation."""

    def __init__(self, kp, ki, kd, limit):
        """Initialize gains, output bounds, and persistent integral state."""
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self.integral = 0.0
        self.previous = None

    def update(self, error, feedforward=0.0):
        """Advance one fixed 2 ms control step."""
        derivative = 0.0 if self.previous is None else (error - self.previous) / DT
        self.previous = error
        candidate = self.integral + error * DT
        base = self.kp * error + self.kd * derivative + feedforward
        raw = base + self.ki * candidate
        if abs(raw) <= self.limit or error * raw < 0.0:
            self.integral = candidate
        return clip(base + self.ki * self.integral, -self.limit, self.limit)


class ControlCore:
    """Compute wheel PI, VMC leg PID, or coupled LQR balance commands."""

    def __init__(self, mode, wheel_kp=0.3, wheel_ki=1.5,
                 leg_kp=700.0, leg_ki=300.0, leg_kd=5.0):
        """Create separate controller and geometry state for both sides."""
        self.mode = mode
        self.wheels = [PID(wheel_kp, wheel_ki, 0.0, 2.0) for _ in range(2)]
        self.legs = [five_links_param(DT) for _ in range(2)]
        self.length_pids = [PID(leg_kp, leg_ki, leg_kd, 100.0) for _ in range(2)]
        self.lqrs = [lqr_controller(DT) for _ in range(2)]
        self.roll_controller = RollLegController()
        self.lqr_length_previous = [0.0, 0.0]
        self.position_initialized = False
        self.velocity_setpoint = 0.0
        self.recovery_active = False
        self.recovery_stable_time = 0.0
        self.recovery_cooldown = 0.0
        self.recovery_armed = False
        self.recovery_arm_time = 0.0
        self.traction_assist = 0.0
        self.traction_direction = 0.0

    def compute(self, joints, orientation, gyro_pitch, target_v, target_length):
        """Return six native motor torques and measured values with SI units."""
        x, y, z, w = orientation
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = (value / norm for value in orientation)
        # Keep the pitch sign convention of robot_control_main.py.
        pitch = -math.asin(clip(2.0 * (w*y - z*x), -1.0, 1.0))
        gyro_pitch = -gyro_pitch
        # Mirror-mounted wheels: positive forward is -right and +left.
        right_speed = -joints['rightwheel'][1] * WHEEL_RADIUS
        left_speed = joints['leftwheel'][1] * WHEEL_RADIUS
        if self.mode == 'lqr':
            disturbance = (abs(pitch) > math.radians(8.0)
                           or abs(gyro_pitch) > 0.85)
            if not self.recovery_armed:
                if (abs(pitch) < math.radians(5.0) and abs(gyro_pitch) < 0.6):
                    self.recovery_arm_time += DT
                    self.recovery_armed = self.recovery_arm_time >= 0.5
                else:
                    self.recovery_arm_time = 0.0
            if self.recovery_cooldown > 0.0:
                self.recovery_cooldown = max(0.0, self.recovery_cooldown - DT)
            if (self.recovery_armed and not self.recovery_active
                    and self.recovery_cooldown == 0.0 and disturbance):
                self.recovery_active = True
                self.recovery_stable_time = 0.0
            if self.recovery_active:
                if (abs(pitch) < math.radians(5.0) and abs(gyro_pitch) < 0.6):
                    self.recovery_stable_time += DT
                else:
                    self.recovery_stable_time = 0.0
                if self.recovery_stable_time >= 0.3:
                    self.recovery_active = False
                    self.recovery_cooldown = 0.5
            recovery_target_v = 0.8 * target_v if self.recovery_active else target_v
            return self.balance(joints, (x, y, z, w), pitch, gyro_pitch,
                                recovery_target_v, target_length,
                                right_speed, left_speed)
        right_torque = -self.wheels[0].update((target_v - right_speed) / WHEEL_RADIUS)
        left_torque = self.wheels[1].update((target_v - left_speed) / WHEEL_RADIUS)
        torques = [0.0, 0.0, 0.0, 0.0, right_torque, left_torque]
        for side, prefix in enumerate(('right', 'left')):
            leg = self.legs[side]
            leg.forward_kinematics_cal(
                side, joints[prefix + '1'][0], joints[prefix + '2'][0],
                pitch, gyro_pitch)
            if self.mode != 'vmc':
                continue
            support = 4.0 / max(0.2, math.cos(leg.theta))
            # The original VMC clips each leg motor at 1.8 N m. Convert that
            # bound back to virtual force so the length PID cannot wind up
            # behind a second, unobserved output saturation.
            j1, j2 = leg.VMC_torque_cal(1.0, 0.0)
            self.length_pids[side].limit = min(100.0, 1.8 / max(abs(j1), abs(j2), 1e-6))
            force = self.length_pids[side].update(target_length - leg.L_0, support)
            # Stage 3 supplies virtual axial force only. LQR supplies Tp in stage 4.
            t1, t2 = leg.VMC_torque_cal(force, 0.0)
            sign = 1.0 if side == 0 else -1.0
            torques[side * 2] = clip(float(sign * t1), -6.0, 6.0)
            torques[side * 2 + 1] = clip(float(sign * t2), -6.0, 6.0)
        return torques, [right_speed, left_speed,
                         self.legs[0].L_0, self.legs[1].L_0,
                         self.legs[0].theta, self.legs[1].theta, pitch]

    def balance(self, joints, orientation, pitch, gyro_pitch,
                target_v, target_length, right_speed, left_speed):
        """Rebuild the original control sequence from message-derived state."""
        x, y, z, w = orientation
        roll = -math.atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y))
        common_pos = (joints['leftwheel'][0] - joints['rightwheel'][0]) / 2.0
        common_speed = (joints['leftwheel'][1] - joints['rightwheel'][1]) / 2.0
        position = common_pos * WHEEL_RADIUS
        left_length, right_length = self.roll_controller.compute_leg_lengths(
            roll, 0.0, target_length)
        rate = 0.8 if abs(target_v) > abs(self.velocity_setpoint) else 1.4
        self.velocity_setpoint += clip(
            target_v - self.velocity_setpoint, -rate * DT, rate * DT)
        # Initialize once, follow during motion, freeze on stopping. Updating this
        # reference every stationary step would silently disable position control.
        if (not self.position_initialized
                or self.velocity_setpoint != 0.0 or target_v != 0.0):
            for lqr in self.lqrs:
                lqr.x_b_set = position
            self.position_initialized = True

        torques = [0.0] * 6
        for side, prefix in enumerate(('right', 'left')):
            leg, lqr = self.legs[side], self.lqrs[side]
            leg.forward_kinematics_cal(
                side, joints[prefix + '1'][0], joints[prefix + '2'][0],
                pitch, gyro_pitch)
            # states_update already reverses the right motor; mirror the common
            # encoder input just as the original per-leg functions do.
            encoder_sign = -1.0 if side == 0 else 1.0
            lqr.states_update(side, leg, pitch, gyro_pitch,
                              encoder_sign * common_pos, encoder_sign * common_speed,
                              WHEEL_RADIUS)
            lqr.calc_k_matrix_from_poly(leg.L_0)
            lqr.v_set = self.velocity_setpoint
            errors = (lqr.theta, lqr.d_theta, lqr.x_b - lqr.x_b_set,
                      lqr.v_b_whole - lqr.v_set, lqr.phi, lqr.d_phi)
            wheel_torque = sum(lqr.K[2*i] * error for i, error in enumerate(errors))
            tp = sum(lqr.K[2*i+1] * error for i, error in enumerate(errors))
            length_setpoint = right_length if side == 0 else left_length
            length_error = length_setpoint - leg.L_0
            # Preserve the original discrete length PD: derivative is delta-error,
            # not delta-error / dt. Stage 3's continuous PID is a different mode.
            length_force = clip(700.0 * length_error + 5.0 * (
                length_error - self.lqr_length_previous[side]), -150.0, 150.0)
            self.lqr_length_previous[side] = length_error
            cosine = math.cos(leg.theta)
            support = 4.0 / cosine if abs(cosine) > 1e-6 else 0.0
            force = clip(support + length_force, -100.0, 100.0)
            t1, t2 = leg.VMC_torque_cal(force, tp)
            sign = 1.0 if side == 0 else -1.0
            torques[side*2] = float(sign * t1)
            torques[side*2+1] = float(sign * t2)
            torques[4+side] = clip(float(sign * wheel_torque), -2.0, 2.0)
        # The flat-ground LQR does not supply enough sustained drive on an
        # incline. Integrate the common wheel-speed error with a hard bound;
        # release it during a fall or when the operator requests a stop.
        direction = 0.0 if abs(target_v) < 1e-6 else (1.0 if target_v > 0 else -1.0)
        if direction != self.traction_direction:
            self.traction_assist = 0.0
        self.traction_direction = direction
        if direction == 0.0:
            self.traction_assist = 0.0
        elif abs(pitch) > 0.25:
            self.traction_assist = max(0.0, self.traction_assist - 0.01)
        else:
            speed_error = abs(self.velocity_setpoint) - (
                right_speed + left_speed) * (1.0 if target_v > 0 else -1.0) / 2.0
            self.traction_assist = clip(
                self.traction_assist + 3.0 * speed_error * DT, 0.0, 1.0)
        torques[4] = clip(torques[4] + direction * self.traction_assist, -2.0, 2.0)
        torques[5] = clip(torques[5] - direction * self.traction_assist, -2.0, 2.0)
        return torques, [right_speed, left_speed,
                         self.legs[0].L_0, self.legs[1].L_0,
                         self.legs[0].theta, self.legs[1].theta, pitch,
                         position, self.lqrs[0].x_b_set, self.velocity_setpoint, roll,
                         gyro_pitch, float(self.recovery_active)]
