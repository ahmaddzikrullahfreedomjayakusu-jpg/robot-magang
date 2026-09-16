"""Bridges Nav2's /cmd_vel into this robot's existing /motor_rpm protocol.

Nav2's controller_server publishes geometry_msgs/Twist on /cmd_vel. This
robot's hardware bridge (see robot1.py / robotmaganglidar1.py in the parent
"Robot magang" folder) does not listen on /cmd_vel -- it listens on
/motor_rpm (std_msgs/Int16MultiArray, [kiri, kanan]) and forwards each byte
pair as a 5-byte packet (0xAA, kanan, kiri, checksum, 0x55) over TCP to the
phone bridge, which relays it over USB OTG serial to the STM32 driving the
hoverboard. The value convention there was empirically tuned on the real
robot and is reused as-is:

    127            = stop (neutral)
    127 - step ... = forward, larger step = faster
    127 + step ... = reverse

This node does not open any socket itself and does not duplicate robot1.py's
TCP connection -- it only republishes on /motor_rpm in the format robot1.py
already expects, so it can run on the same machine as robot1.py without
touching the existing hardware link.
"""

import time

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
import rclpy


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class CmdVelToMotorBridge(Node):

    def __init__(self):
        super().__init__("cmd_vel_to_motor_bridge")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motor_rpm_topic", "/motor_rpm")
        self.declare_parameter("wheel_base", 0.50)
        self.declare_parameter("motor_neutral", 127)
        self.declare_parameter("motor_step", 30)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("publish_rate", 20.0)
        # The 97/127/157 values in robotmaganglidar1.py were only ever
        # empirically tested at full speed -- small in-between deviations
        # from neutral may fall inside the motor's real dead-zone (friction)
        # and produce no actual wheel movement at all. Any non-zero command
        # gets floored to at least this deviation so Nav2's slow/careful
        # velocities (common near goals and obstacles) don't just stutter.
        self.declare_parameter("min_effective_step", 12)

        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.motor_neutral = int(self.get_parameter("motor_neutral").value)
        self.motor_step = int(self.get_parameter("motor_step").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.min_effective_step = int(self.get_parameter("min_effective_step").value)

        self.last_twist = Twist()
        self.last_cmd_time = 0.0

        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self.on_cmd_vel, 10
        )
        self.motor_pub = self.create_publisher(
            Int16MultiArray, self.get_parameter("motor_rpm_topic").value, 10
        )

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self.publish_motor_cmd)

        self.get_logger().info(
            f"cmd_vel_to_motor_bridge ready: neutral={self.motor_neutral}, "
            f"step={self.motor_step}, wheel_base={self.wheel_base}m -- "
            "calibrate wheel_base and max_linear_speed on the real robot"
        )

    def on_cmd_vel(self, msg: Twist) -> None:
        self.last_twist = msg
        self.last_cmd_time = time.monotonic()

    def publish_motor_cmd(self) -> None:
        timed_out = (time.monotonic() - self.last_cmd_time) > self.cmd_vel_timeout
        if self.last_cmd_time == 0.0 or timed_out:
            kiri = kanan = self.motor_neutral
        else:
            kiri, kanan = self._twist_to_wheel_bytes(self.last_twist)

        msg = Int16MultiArray()
        msg.data = [kiri, kanan]
        self.motor_pub.publish(msg)

    def _twist_to_wheel_bytes(self, twist: Twist):
        v = clamp(twist.linear.x, -self.max_linear_speed, self.max_linear_speed)
        w = twist.angular.z

        v_left = v - w * self.wheel_base * 0.5
        v_right = v + w * self.wheel_base * 0.5

        frac_left = clamp(v_left / self.max_linear_speed, -1.0, 1.0)
        frac_right = clamp(v_right / self.max_linear_speed, -1.0, 1.0)

        step_left = self._floor_effective_step(frac_left * self.motor_step)
        step_right = self._floor_effective_step(frac_right * self.motor_step)

        # Hardware convention: smaller byte = forward, larger byte = reverse.
        kiri = self.motor_neutral - step_left
        kanan = self.motor_neutral - step_right

        lo = self.motor_neutral - self.motor_step
        hi = self.motor_neutral + self.motor_step
        kiri = int(clamp(kiri, lo, hi))
        kanan = int(clamp(kanan, lo, hi))
        return kiri, kanan

    def _floor_effective_step(self, raw_step: float) -> int:
        # NOTE: flooring each wheel independently means a very gentle turn
        # commanded at low speed can floor both wheels to the same minimum
        # step, briefly losing the steering differential -- traded off
        # against the wheel not moving at all otherwise. Nav2 replans at
        # 20Hz off real odometry, so it corrects heading on the next cycle.
        step = round(raw_step)
        if step == 0:
            return 0
        if abs(step) < self.min_effective_step:
            step = self.min_effective_step if step > 0 else -self.min_effective_step
        return int(clamp(step, -self.motor_step, self.motor_step))


def main() -> None:
    rclpy.init()
    node = CmdVelToMotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
