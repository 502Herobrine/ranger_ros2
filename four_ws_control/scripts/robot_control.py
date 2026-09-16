#!/usr/bin/env python3
# Copyright 2026 AgileX Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the AgileX Robotics nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Convert a body Twist to the four Ranger Mini V3 wheel commands."""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray


def wheel_targets(vx, vy, wz, wheel_base, wheel_separation, wheel_radius):
    """Return FL, FR, RL, RR steering angles and signed wheel angular speeds."""
    angles, speeds = [], []
    for x, y in ((wheel_base / 2, wheel_separation / 2),
                 (wheel_base / 2, -wheel_separation / 2),
                 (-wheel_base / 2, wheel_separation / 2),
                 (-wheel_base / 2, -wheel_separation / 2)):
        wx, wy = vx - wz * y, vy + wz * x
        angle, speed = math.atan2(wy, wx), math.hypot(wx, wy) / wheel_radius
        if angle > math.pi / 2:
            angle -= math.pi
            speed = -speed
        elif angle < -math.pi / 2:
            angle += math.pi
            speed = -speed
        angles.append(angle)
        speeds.append(speed)
    return angles, speeds


class Commander(Node):
    """Retain upstream topics and wheel ordering; use measured steering feedback."""

    def __init__(self):
        super().__init__('commander')
        self.wheel_separation = self.declare_parameter('wheel_separation', 0.38).value
        self.wheel_base = self.declare_parameter('wheel_base', 0.50).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.09).value
        self.timeout = self.declare_parameter('cmd_vel_timeout', 0.5).value
        if min(self.wheel_separation, self.wheel_base, self.wheel_radius, self.timeout) <= 0:
            raise ValueError('Wheel geometry and command timeout must be positive')
        self.target = (0.0, 0.0, 0.0)
        self.received = None
        self.angles = [0.0] * 4
        self.measured = {}
        self.feedback_received = None
        self.names = [p + '_steering_joint' for p in
                      ('front_left', 'front_right', 'rear_left', 'rear_right')]
        self.pub_pos = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.create_subscription(Twist, 'cmd_vel', self.command, 1)
        self.create_subscription(Joy, 'joy', self.joy, 10)
        self.create_subscription(JointState, 'joint_states', self.joints, 10)
        self.create_timer(0.02, self.update)

    def command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if all(math.isfinite(v) for v in values):
            self.target = values
            self.received = self.get_clock().now()

    def joy(self, msg):
        if len(msg.buttons) <= 5 or len(msg.axes) <= 3:
            return
        command = Twist()
        if msg.buttons[0]:
            command.linear.x = msg.axes[1] * 1.5
            command.linear.y = msg.axes[0] * 1.5
        elif msg.buttons[4]:
            command.linear.x = msg.axes[1] * 1.5
            command.angular.z = msg.axes[3] * 2.0
        elif msg.buttons[5]:
            command.angular.z = msg.axes[3] * 2.0
        self.command(command)

    def joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        # Standalone V3 uses fl/fr/rl/rr; composite robots use the long names.
        aliases = [p + '_steering_joint' for p in ('fl', 'fr', 'rl', 'rr')]
        measured = {name: positions.get(name, positions.get(alias, math.nan))
                    for name, alias in zip(self.names, aliases)}
        if all(math.isfinite(value) for value in measured.values()):
            self.measured = measured
            self.feedback_received = self.get_clock().now()
        else:
            self.feedback_received = None

    def update(self):
        target = self.target
        now = self.get_clock().now()
        if (self.received is None or self.feedback_received is None or
                not 0 <= (now - self.received).nanoseconds <= self.timeout * 1e9 or
                not 0 <= (now - self.feedback_received).nanoseconds <= self.timeout * 1e9):
            target = (0.0, 0.0, 0.0)
        speeds = [0.0] * 4
        if any(abs(v) > 1e-6 for v in target):
            self.angles, speeds = wheel_targets(
                *target, self.wheel_base, self.wheel_separation, self.wheel_radius)
            # Turn the wheels before driving; a common scale preserves the
            # requested curvature while steering catches up after mode changes.
            errors = [abs(a - self.measured.get(name, a + math.pi))
                      for name, a in zip(self.names, self.angles)]
            scale = max(0.0, math.cos(max(errors))) if max(errors) < 0.2 else 0.0
            speeds = [v * scale for v in speeds]
        # Stop wheel rotation while retaining steering, rather than scrubbing
        # all four tires back to zero on every stop/watchdog expiration.
        self.pub_pos.publish(Float64MultiArray(data=self.angles))
        self.pub_vel.publish(Float64MultiArray(data=speeds))


def main(args=None):
    rclpy.init(args=args)
    node = Commander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
