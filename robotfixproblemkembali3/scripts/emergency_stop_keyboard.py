#!/usr/bin/env python3
"""Keyboard-driven emergency stop for robotfixproblemkembali3 (copied from pelcdx) -- run this in its own
terminal alongside start.sh.

  SPACE  -- toggle: stop immediately (publishes True on /emergency_stop,
            checked first thing every control tick in
            manual_waypoint_driver_node, overriding everything else) on
            the first press, resume (publishes False) on the next.
  Ctrl+C -- exit (leaves the stop state as it was)

This is a physical-cable-free alternative to unplugging the STM32 USB
to prevent a crash -- added per explicit user request (2026-10-01).
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class EmergencyStopKeyboard(Node):
    def __init__(self) -> None:
        super().__init__("emergency_stop_keyboard")
        self.declare_parameter("emergency_stop_topic", "/emergency_stop")
        topic = str(self.get_parameter("emergency_stop_topic").value)
        self.pub = self.create_publisher(Bool, topic, 10)
        self.stopped = False

    def publish_state(self) -> None:
        msg = Bool()
        msg.data = self.stopped
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = EmergencyStopKeyboard()
    old_settings = termios.tcgetattr(sys.stdin)
    print(
        "\n[emergency_stop_keyboard] SPASI = toggle stop/lanjut, Ctrl+C = exit\n"
        "Keep this terminal focused while the robot is driving.\n"
    )
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not readable:
                continue
            ch = sys.stdin.read(1)
            if ch == "\x03":
                break
            if ch == " ":
                node.stopped = not node.stopped
                node.publish_state()
                if node.stopped:
                    print("[STOP] Emergency stop ON -- tekan SPASI lagi untuk lanjut.")
                else:
                    print("[RESUME] Emergency stop OFF -- robot boleh jalan lagi.")
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
