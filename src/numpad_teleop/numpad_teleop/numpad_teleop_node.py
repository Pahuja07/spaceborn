#!/usr/bin/env python3
"""Numpad keyboard teleop for the A200 Husky (diff-drive).

Reads single keypresses in raw terminal mode and publishes geometry_msgs/Twist
to cmd_vel at a fixed rate. The A200 uses a diff_drive_controller, so only
linear.x (forward/back) and angular.z (turn) are commanded; there is no lateral
motion. The last command is republished continuously so twist_mux does not drop
it on its input timeout.

Numpad layout (NumLock ON):

        7  8  9        fwd-left   forward   fwd-right
        4  5  6        turn-left   STOP     turn-right
        1  2  3        back-left   back     back-right

Other keys:
    + / -   increase / decrease speed scale
    space   emergency stop (zero velocity)
    q       quit
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# key -> (linear_x_sign, angular_z_sign)
MOVE_BINDINGS = {
    '8': (1.0, 0.0),    # forward
    '2': (-1.0, 0.0),   # back
    '4': (0.0, 1.0),    # turn left (CCW)
    '6': (0.0, -1.0),   # turn right (CW)
    '7': (1.0, 1.0),    # forward + left
    '9': (1.0, -1.0),   # forward + right
    '1': (-1.0, -1.0),   # back + left
    '3': (-1.0, 1.0),  # back + right
    '5': (0.0, 0.0),    # stop
}

# Arrow-key escape sequences as a fallback for laptops without a numpad.
ARROW_BINDINGS = {
    '\x1b[A': (1.0, 0.0),    # up
    '\x1b[B': (-1.0, 0.0),   # down
    '\x1b[D': (0.0, 1.0),    # left
    '\x1b[C': (0.0, -1.0),   # right
}

SPEED_BINDINGS = {
    '+': 1.1,
    '=': 1.1,   # same physical key as '+' without shift
    '-': 0.9,
}

BANNER = """
Numpad teleop (A200 diff-drive) -> {topic}

        7  8  9      fwd-left   forward    fwd-right
        4  5  6      turn-left   STOP      turn-right
        1  2  3      back-left    back     back-right

  + / - : speed up / down     space : stop     q : quit
  (arrow keys also work)
"""


class NumpadTeleop(Node):
    def __init__(self):
        super().__init__('numpad_teleop')
        self.declare_parameter('cmd_vel_topic', '/a200_0000/cmd_vel')
        self.declare_parameter('linear_speed', 0.4)     # m/s
        self.declare_parameter('angular_speed', 0.25)    # rad/s
        self.declare_parameter('publish_rate', 20.0)    # Hz

        topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.pub = self.create_publisher(Twist, topic, 10)
        self.lin_sign = 0.0
        self.ang_sign = 0.0
        self.scale = 1.0
        self.topic = topic

        self.timer = self.create_timer(1.0 / rate, self._publish)

    def apply_key(self, key):
        """Update the current command from a key. Returns False to quit."""
        if key == 'q':
            return False
        if key == ' ':
            self.lin_sign = 0.0
            self.ang_sign = 0.0
        elif key in MOVE_BINDINGS:
            self.lin_sign, self.ang_sign = MOVE_BINDINGS[key]
        elif key in ARROW_BINDINGS:
            self.lin_sign, self.ang_sign = ARROW_BINDINGS[key]
        elif key in SPEED_BINDINGS:
            self.scale = max(0.1, min(3.0, self.scale * SPEED_BINDINGS[key]))
            self.get_logger().info(
                f'speed scale = {self.scale:.2f} '
                f'(lin {self.linear_speed * self.scale:.2f} m/s, '
                f'ang {self.angular_speed * self.scale:.2f} rad/s)')
        return True

    def _publish(self):
        msg = Twist()
        msg.linear.x = self.lin_sign * self.linear_speed * self.scale
        msg.angular.z = self.ang_sign * self.angular_speed * self.scale
        self.pub.publish(msg)

    def stop(self):
        self.lin_sign = 0.0
        self.ang_sign = 0.0
        self._publish()


def read_key(timeout=0.1):
    """Read one keypress (or escape sequence) from stdin, or '' on timeout."""
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return ''
    ch = sys.stdin.read(1)
    if ch == '\x1b':  # possible arrow-key escape sequence
        if select.select([sys.stdin], [], [], 0.01)[0]:
            ch += sys.stdin.read(1)
            if select.select([sys.stdin], [], [], 0.01)[0]:
                ch += sys.stdin.read(1)
    return ch


def main():
    rclpy.init()
    node = NumpadTeleop()
    print(BANNER.format(topic=node.topic))

    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(0.05)
            if key and not node.apply_key(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
