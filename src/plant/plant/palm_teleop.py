#!/usr/bin/env python3
import math
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
POS_STEP = 0.002
ANG_STEP = 0.5 * DEG2RAD
RESET_POS = [0.55, 0.0, 0.5]


class PalmTeleop(Node):
    def __init__(self):
        super().__init__('palm_teleop')
        self.pub_pose = self.create_publisher(Float64MultiArray, '/palm_pose_cmd', 10)
        self.pose = [RESET_POS[0], RESET_POS[1], RESET_POS[2], 0.0, 0.0, 0.0]
        self.timer = self.create_timer(0.005, self._poll_keyboard)
        self._stdin_fd = sys.stdin.fileno()
        self._stdin_old = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        self._print_help()
        self._publish_pose()

    def destroy_node(self):
        termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._stdin_old)
        return super().destroy_node()

    def _print_help(self):
        self.get_logger().info(
            'Palm teleop keys: up/down +x/-x, left/right +y/-y, [/ ] +z/-z, O/P roll +10/-10 deg, U/I pitch +10/-10 deg, J/K yaw +10/-10 deg, R reset, Q quit'
        )

    def _publish_pose(self):
        msg = Float64MultiArray()
        msg.data = self.pose.copy()
        self.pub_pose.publish(msg)
        self.get_logger().info(
            f"palm pose: pos={[round(v, 3) for v in self.pose[:3]]} "
            f"rpy_deg={[round(v * RAD2DEG, 1) for v in self.pose[3:6]]}"
        )

    def _poll_keyboard(self):
        if select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1)
            changed = True

            if key == '\x1b':
                seq = sys.stdin.read(2)
                if seq == '[A':
                    self.pose[0] += POS_STEP
                elif seq == '[B':
                    self.pose[0] -= POS_STEP
                elif seq == '[C':
                    self.pose[1] -= POS_STEP
                elif seq == '[D':
                    self.pose[1] += POS_STEP
                else:
                    changed = False
            elif key == '[':
                self.pose[2] += POS_STEP
            elif key == ']':
                self.pose[2] -= POS_STEP
            elif key.lower() == 'o':
                self.pose[3] += ANG_STEP
            elif key.lower() == 'p':
                self.pose[3] -= ANG_STEP
            elif key.lower() == 'u':
                self.pose[4] += ANG_STEP
            elif key.lower() == 'i':
                self.pose[4] -= ANG_STEP
            elif key.lower() == 'j':
                self.pose[5] += ANG_STEP
            elif key.lower() == 'k':
                self.pose[5] -= ANG_STEP
            elif key.lower() == 'r':
                self.pose[:] = [RESET_POS[0], RESET_POS[1], RESET_POS[2], 0.0, 0.0, 0.0]
            elif key.lower() == 'q':
                raise KeyboardInterrupt
            else:
                changed = False

            if changed:
                self._publish_pose()


def main():
    rclpy.init()
    node = PalmTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
