"""Measure stage 2/3 against unmodified simulator in a private namespace."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd
from std_msgs.msg import Float64MultiArray

parser = argparse.ArgumentParser()
parser.add_argument('--mode', default='vmc')
parser.add_argument('--speed', type=float, default=0.0)
parser.add_argument('--length', type=float, default=0.1)
parser.add_argument('--steps', type=int, default=6000)
parser.add_argument('--tag', default='test')
parser.add_argument('--output-dir', default='stage23_validation')
parser.add_argument('--param', action='append', default=[])
parser.add_argument('--sim-param', action='append', default=[])
a = parser.parse_args()
base = Path(a.output_dir)
base.mkdir(parents=True, exist_ok=True)
rclpy.init()
probe = Node('stage23_probe_' + str(os.getpid()))
ns = '/stage23_' + str(os.getpid())
rows = []
torque_rows = []
probe.create_subscription(Float64MultiArray, ns + '/controller/telemetry',
                          lambda m: rows.append(list(m.data)), 10000)
probe.create_subscription(WheelLegCmd, ns + '/actuator_cmd',
                          lambda m: torque_rows.append([
                              m.right_phi1, m.right_phi4, m.left_phi1, m.left_phi4,
                              m.right_wheel, m.left_wheel]), 10000)
pub = probe.create_publisher(Twist, ns + '/cmd_vel', 10)
cmd = Twist()
cmd.linear.x = a.speed
probe.create_timer(0.02, lambda: pub.publish(cmd))
processes, logs = [], []
started = time.monotonic()
try:
    controller_args = ['-p', 'mode:=' + a.mode, '-p', 'max_steps:=' + str(a.steps),
                       '-p', 'target_length:=' + str(a.length)]
    for p in a.param:
        controller_args.extend(['-p', p])
    simulator_args = ['-p', 'viewer:=false', *sum(
        (['-p', value] for value in a.sim_param), [])]
    for package, extra in [('wheel_leg_controller', controller_args),
                           ('wheel_leg_sim', simulator_args)]:
        log = tempfile.TemporaryFile(mode='w+')
        logs.append((package, log))
        processes.append(subprocess.Popen(
            ['ros2', 'run', package, package, '--ros-args', '-r', '__ns:=' + ns, *extra],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    while time.monotonic() - started < 40:
        rclpy.spin_once(probe, timeout_sec=0.01)
        if rows and rows[-1][0] >= (a.steps - 1) * 0.002 - 1e-9:
            break
        if any(p.poll() is not None for p in processes):
            raise RuntimeError('Node exited')
    if not rows or rows[-1][0] < (a.steps - 1) * 0.002 - 1e-9:
        raise RuntimeError(f'Incomplete run: {len(rows)} samples')
    tail = [r for r in rows if r[0] >= max(0, a.steps * 0.002 - 3)]
    metrics = {'mode': a.mode, 'command_speed': a.speed, 'target_length': a.length,
               'samples': len(rows), 'duration': rows[-1][0], 'params': a.param,
               'sim_params': a.sim_param}
    for index, label in [(3, 'right_speed'), (4, 'left_speed'),
                         (5, 'right_length'), (6, 'left_length'), (9, 'pitch')]:
        values = [r[index] for r in tail]
        target = a.speed if index in (3, 4) else (a.length if index in (5, 6) else 0.0)
        metrics[label] = {'mean': sum(values) / len(values), 'min': min(values),
                          'max': max(values),
                          'max_error': max(abs(v-target) for v in values),
                          'mean_absolute_error': (sum(abs(v-target) for v in values)
                                                  / len(values)),
                          'relative_mae': (sum(abs(v-target) for v in values)
                                           / len(values) / abs(target)
                                           if index in (3, 4) and target else None)}
    if a.mode == 'lqr' and len(rows[0]) > 10:
        metrics['travel_distance_m'] = rows[-1][10] - rows[0][10]
        metrics['max_abs_pitch_deg'] = max(abs(r[9]) for r in rows) * 180.0 / 3.141592653589793
        metrics['stage5_pass'] = (
            abs(a.speed - 0.3) < 1e-9
            and metrics['travel_distance_m'] >= 3.0
            and metrics['max_abs_pitch_deg'] < 10.0)
    with (base / (a.tag + '.csv')).open('w') as f:
        writer = csv.writer(f)
        header = ['time_s', 'target_v', 'target_L', 'v_right', 'v_left',
                  'L_right', 'L_left', 'theta_right', 'theta_left', 'pitch']
        if a.mode == 'lqr':
            header += ['position', 'position_ref', 'velocity_setpoint', 'roll',
                       'pitch_rate', 'recovery_active']
        writer.writerow(header)
        writer.writerows(rows)
    import csv
    with (base / (a.tag + '_torques.csv')).open('w') as f:
        writer = csv.writer(f)
        writer.writerow(['right_phi1', 'right_phi4', 'left_phi1', 'left_phi4',
                         'right_wheel', 'left_wheel'])
        writer.writerows(torque_rows)
    (base / (a.tag + '.json')).write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2))
finally:
    for p in processes:
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
    for p in processes:
        try:
            p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            p.wait(timeout=4)
    for name, log in logs:
        log.seek(0)
        contents = log.read()
        (base / (a.tag + '_' + name + '.log')).write_text(contents)
        if 'Traceback' in contents:
            print(contents)
        log.close()
    probe.destroy_node()
    rclpy.shutdown()
