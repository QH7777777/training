"""Run isolated, realtime LQR acceptance and record the required ROS bag."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray


def main():
    """Verify pitch from raw IMU samples and save reviewable evidence."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', default='stage4_validation')
    parser.add_argument('--seconds', type=float, default=12.0)
    parser.add_argument('--length', type=float, default=0.10)
    args = parser.parse_args()
    steps = round(args.seconds / 0.002)
    if steps < 3000 or not 0.10 <= args.length <= 0.14:
        parser.error('Use at least 6 seconds and a leg length in [0.10, 0.14] m')
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    bag = output / 'balance_bag'
    if bag.exists():
        parser.error('balance_bag already exists; choose a new output directory')
    rclpy.init()
    probe = Node('stage4_verification_' + str(os.getpid()))
    namespace = '/stage4_' + str(os.getpid())
    imu_rows, joint_rows, telemetry_rows, commands = [], [], [], []

    def on_imu(msg):
        q = msg.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
        pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y-z*x)))))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        imu_rows.append([stamp, pitch])

    probe.create_subscription(Imu, namespace + '/imu', on_imu, 10000)
    probe.create_subscription(JointState, namespace + '/joint_states', joint_rows.append, 10000)
    probe.create_subscription(WheelLegCmd, namespace + '/actuator_cmd', commands.append, 10000)
    probe.create_subscription(Float64MultiArray, namespace + '/controller/telemetry',
                              lambda msg: telemetry_rows.append(list(msg.data)), 10000)
    processes, log_files = [], []

    def start(name, command):
        log = (output / (name + '.log')).open('w')
        log_files.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    try:
        recorder = start('recorder', ['ros2', 'bag', 'record', '-o', str(bag),
                                      namespace + '/imu', namespace + '/joint_states',
                                      namespace + '/actuator_cmd'])
        # Advertise the message types without publishing, so the recorder joins
        # every topic before the controller can trigger the first physics step.
        announcements = [probe.create_publisher(kind, namespace + '/' + topic, 10)
                         for kind, topic in ((Imu, 'imu'), (JointState, 'joint_states'),
                                             (WheelLegCmd, 'actuator_cmd'))]
        deadline = time.monotonic() + 10.0
        while (time.monotonic() < deadline
               and any(pub.get_subscription_count() < 2 for pub in announcements)):
            rclpy.spin_once(probe, timeout_sec=0.05)
        if any(pub.get_subscription_count() < 2 for pub in announcements):
            raise RuntimeError('Bag recorder did not subscribe to all three topics')
        for pub in announcements:
            probe.destroy_publisher(pub)
        controller = start('controller', [
            'ros2', 'run', 'wheel_leg_controller', 'wheel_leg_controller', '--ros-args',
            '-r', '__ns:=' + namespace, '-p', 'mode:=lqr',
            '-p', 'target_length:=' + str(args.length), '-p', 'max_steps:=' + str(steps)])
        simulator = start('simulator', [
            'ros2', 'run', 'wheel_leg_sim', 'wheel_leg_sim', '--ros-args',
            '-r', '__ns:=' + namespace, '-p', 'viewer:=false', '-p', 'realtime:=true',
            '-p', 'publish_clock:=true'])
        deadline = time.monotonic() + max(60.0, args.seconds * 3)
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.01)
            if any(p.poll() is not None for p in (recorder, controller, simulator)):
                raise RuntimeError('A verification process exited; inspect the saved logs')
            if (imu_rows and imu_rows[-1][0] >= steps * 0.002 - 1e-9
                    and len(commands) >= steps and len(telemetry_rows) >= steps):
                break
        if not imu_rows or imu_rows[-1][0] < steps * 0.002 - 1e-9:
            raise RuntimeError('Simulation did not finish the requested duration')
        tail = [r for r in imu_rows if r[0] >= args.seconds - 5]
        longest, beginning = 0.0, None
        for stamp, pitch in imu_rows:
            if abs(pitch) < 10:
                if beginning is None:
                    beginning = stamp
                longest = max(longest, stamp - beginning)
            else:
                beginning = None
        late_positions = [r[10] for r in telemetry_rows if r[0] >= args.seconds - 5]
        result = {
            'namespace': namespace, 'target_length_m': args.length,
            'duration_sim_s': imu_rows[-1][0], 'command_count': len(commands),
            'joint_sample_count': len(joint_rows),
            'longest_continuous_pitch_under_10deg_s': longest,
            'max_abs_pitch_deg_entire_run': max(abs(r[1]) for r in imu_rows),
            'max_abs_pitch_deg_last_5s': max(abs(r[1]) for r in tail),
            'mean_pitch_deg_last_5s': sum(r[1] for r in tail) / len(tail),
            'position_range_last_5s_m': [min(late_positions), max(late_positions)],
            'position_reference_range_m': [min(r[11] for r in telemetry_rows),
                                           max(r[11] for r in telemetry_rows)],
            'max_abs_leg_torque_Nm': max(abs(v) for c in commands for v in (
                c.right_phi1, c.right_phi4, c.left_phi1, c.left_phi4)),
            'max_abs_wheel_torque_Nm': max(abs(v) for c in commands
                                           for v in (c.right_wheel, c.left_wheel)),
        }
        result['passed'] = (longest >= 5.0 and len(commands) == steps
                            and result['max_abs_leg_torque_Nm'] <= 6.0
                            and result['max_abs_wheel_torque_Nm'] <= 2.0)
        (output / 'metrics.json').write_text(json.dumps(result, indent=2))
        with (output / 'pitch.csv').open('w') as file:
            writer = csv.writer(file)
            writer.writerow(['simulation_time_s', 'raw_imu_pitch_deg'])
            writer.writerows(imu_rows)
        with (output / 'telemetry.csv').open('w') as file:
            writer = csv.writer(file)
            writer.writerow(['time_s', 'target_v', 'target_L', 'v_right', 'v_left',
                             'L_right', 'L_left', 'theta_right', 'theta_left', 'pitch',
                             'position', 'position_ref', 'velocity_setpoint', 'roll',
                             'pitch_rate', 'recovery_active'])
            writer.writerows(telemetry_rows)
        print(json.dumps(result, indent=2))
        if not result['passed']:
            raise RuntimeError('LQR acceptance failed')
    finally:
        # Stop only processes created by this verification, allowing bag metadata
        # to flush through the normal SIGINT shutdown path.
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        for file in log_files:
            file.close()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
