"""Independent ROS 2 controller for wheel PI, VMC and LQR balance."""

from collections import OrderedDict
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from .control_core import ControlCore, DT, clip


JOINT_NAMES = ('right1', 'right2', 'left1', 'left2', 'rightwheel', 'leftwheel')


class WheelLegController(Node):
    """Publish one control command for each complete sensor pair."""

    def __init__(self):
        """Load control parameters and establish the topic interfaces."""
        super().__init__('wheel_leg_controller')
        self.declare_parameter('mode', 'lqr')
        self.declare_parameter('test_right_wheel_torque', 0.1)
        self.declare_parameter('test_left_wheel_torque', 0.0)
        self.declare_parameter('max_steps', 0)
        self.declare_parameter('target_length', 0.10)
        self.declare_parameter('cmd_timeout', 1.0)
        gains = {'wheel_kp': 0.3, 'wheel_ki': 1.5, 'leg_kp': 700.0,
                 'leg_ki': 300.0, 'leg_kd': 5.0}
        for name, value in gains.items():
            self.declare_parameter(name, value)
            gains[name] = float(self.get_parameter(name).value)
        self.mode = self.get_parameter('mode').value
        self.target_length = float(self.get_parameter('target_length').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.right_torque = float(self.get_parameter('test_right_wheel_torque').value)
        self.left_torque = float(self.get_parameter('test_left_wheel_torque').value)
        self.max_steps = self.get_parameter('max_steps').value
        if (not all(math.isfinite(x) and abs(x) <= 2.0
                    for x in (self.right_torque, self.left_torque))
                or type(self.max_steps) is not int or self.max_steps < 0
                or self.mode not in ('link_test', 'wheel_pi', 'vmc', 'lqr')
                or not 0.10 <= self.target_length <= 0.14
                or not math.isfinite(self.cmd_timeout) or self.cmd_timeout <= 0.0
                or not all(math.isfinite(g) and g >= 0.0 for g in gains.values())):
            raise ValueError('参数无效：mode=link_test/wheel_pi/vmc/lqr，腿长 0.10–0.14 m，'
                             'max_steps ≥0，超时 >0，增益非负，轮力矩在 ±2 N·m 内')
        self.core = ControlCore(self.mode, **gains)
        self.target_v = 0.0
        self.last_velocity_command = float('-inf')

        self.pending = OrderedDict()
        self.steps = 0
        self.imu = None
        self.joints = {}
        self.cmd_pub = self.create_publisher(WheelLegCmd, 'actuator_cmd', 10)
        self.telemetry_pub = self.create_publisher(
            Float64MultiArray, 'controller/telemetry', 10)
        self.velocity_sub = self.create_subscription(Twist, 'cmd_vel', self.on_velocity, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.on_imu, 10)
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.on_joints, 10)
        self.get_logger().info(
            f'控制器启动：mode={self.mode}，目标腿长 {self.target_length:.3f} m，'
            f'max_steps={self.max_steps}（0=持续运行），'
            + ('LQR 平衡已启用。' if self.mode == 'lqr' else '当前模式不包含 LQR 平衡。'))

    def on_velocity(self, msg):
        """Accept forward speed; stale commands return to zero after timeout."""
        if not math.isfinite(msg.linear.x):
            self.get_logger().warning('忽略非有限速度指令')
            return
        self.target_v = clip(msg.linear.x, -1.0, 1.0)
        self.last_velocity_command = time.monotonic()
        if abs(msg.angular.z) > 1e-9:
            self.get_logger().warning('当前只处理 linear.x，暂不处理转向',
                                      throttle_duration_sec=2.0)

    def on_imu(self, msg):
        """Cache the IMU sample until its joint sample arrives."""
        self.accept_state('imu', msg)

    def on_joints(self, msg):
        """Cache the joint sample until its IMU sample arrives."""
        self.accept_state('joints', msg)

    def accept_state(self, kind, msg):
        """Match exact timestamps; bounded storage tolerates missing samples."""
        if self.max_steps and self.steps >= self.max_steps:
            return
        stamp = msg.header.stamp
        key = (stamp.sec, stamp.nanosec)
        pair = self.pending.setdefault(key, {})
        pair[kind] = msg
        while len(self.pending) > 10:
            self.pending.popitem(last=False)
        if 'imu' not in pair or 'joints' not in pair:
            return
        self.pending.pop(key, None)
        imu, js = pair['imu'], pair['joints']
        if (len(js.name) != len(set(js.name))
                or len(js.position) != len(js.name)
                or len(js.velocity) != len(js.name)
                or not set(JOINT_NAMES).issubset(js.name)):
            self.get_logger().warning('忽略不完整或关节名称重复的状态消息')
            return
        q, w = imu.orientation, imu.angular_velocity
        values = [q.x, q.y, q.z, q.w, w.x, w.y, w.z, *js.position, *js.velocity]
        if (not all(math.isfinite(x) for x in values)
                or sum(x * x for x in (q.x, q.y, q.z, q.w)) < 1e-12):
            self.get_logger().warning('忽略非有限状态或无效姿态四元数')
            return
        self.imu = imu
        self.joints = {
            name: (js.position[i], js.velocity[i]) for i, name in enumerate(js.name)
        }

        target_v = (self.target_v if time.monotonic() - self.last_velocity_command
                    <= self.cmd_timeout else 0.0)
        torques, measured = self.core.compute(
            self.joints, (q.x, q.y, q.z, q.w), w.y, target_v, self.target_length)
        if self.mode == 'link_test':
            torques = [0.0, 0.0, 0.0, 0.0, self.right_torque, self.left_torque]
        if not all(math.isfinite(value) for value in [*torques, *measured]):
            self.get_logger().error('控制输出无效，停止发布力矩')
            return
        cmd = WheelLegCmd()
        for name, value in zip(('right_phi1', 'right_phi4', 'left_phi1', 'left_phi4',
                                'right_wheel', 'left_wheel'), torques):
            setattr(cmd, name, value)
        telemetry = Float64MultiArray()
        telemetry.data = [self.steps * DT, target_v, self.target_length, *measured]
        label = ('time_s,target_v_m_s,target_L_m,v_right_m_s,v_left_m_s,'
                 'L_right_m,L_left_m,theta_right_rad,theta_left_rad,pitch_rad')
        if self.mode == 'lqr':
            label += (',position_m,position_ref_m,velocity_setpoint_m_s,roll_rad,'
                      'pitch_rate_rad_s,recovery_active')
        telemetry.layout.dim = [MultiArrayDimension(
            label=label,
            size=len(telemetry.data), stride=len(telemetry.data))]
        self.telemetry_pub.publish(telemetry)
        self.cmd_pub.publish(cmd)
        self.steps += 1
        if self.steps == 1:
            self.get_logger().info('已配齐首组状态并发送力矩，状态 → 指令回路开始运行')
        if self.steps % 250 == 0:
            right_speed = self.joints['rightwheel'][1]
            self.get_logger().info(
                f't={self.steps * DT:.3f}s，v目标={target_v:.2f}m/s，'
                f'右轮={right_speed:.2f}rad/s，'
                f'腿长右/左={measured[2]:.4f}/{measured[3]:.4f}m，'
                f'pitch={math.degrees(measured[6]):.2f}°')
        if self.max_steps and self.steps == self.max_steps:
            self.pending.clear()
            self.get_logger().info(
                '已达到 max_steps，停止发布；仿真处理完最后一条指令后将暂停。'
                '重新测试请重启仿真和控制器，Ctrl+C 退出。')


def main(args=None):
    """Run the standalone ROS 2 controller process."""
    rclpy.init(args=args)
    node = None
    try:
        node = WheelLegController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
