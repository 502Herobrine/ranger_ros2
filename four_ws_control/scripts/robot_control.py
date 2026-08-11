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

"""
Convert cmd_vel to Ranger four-wheel-steering controller commands.

Adapted for Ranger Mini V3 from AgileX Robotics' four_ws_control package in
ugv_gazebo_sim (humble branch, commit 8bf6a52a3139969d27548622e6144a90c2b80894).
"""

import math
import threading

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


vel_msg = Twist()
msg_vel = Twist()
mode_selection = 0  # 1: opposite phase, 2: in-phase, 3: pivot turn


class Commander(Node):
    """Publish steering-position and wheel-velocity command arrays."""

    def __init__(self):
        super().__init__('commander')
        self.wheel_separation = self.declare_parameter(
            'wheel_separation', 0.38).value
        self.wheel_base = self.declare_parameter('wheel_base', 0.50).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.09).value
        self.wheel_steering_y_offset = self.declare_parameter(
            'wheel_steering_y_offset', 0.0).value
        self.steering_track = (
            self.wheel_separation - 2 * self.wheel_steering_y_offset)

        self.pos = np.zeros(4, dtype=float)
        self.vel = np.zeros(4, dtype=float)
        self.pub_pos = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        global vel_msg, mode_selection, msg_vel
        vel_msg = msg_vel

        if mode_selection == 1:  # opposite phase
            steering_offset = (
                vel_msg.angular.z * self.wheel_steering_y_offset)
            direction = np.sign(vel_msg.linear.x)
            left_speed = direction * math.hypot(
                vel_msg.linear.x
                - vel_msg.angular.z * self.steering_track / 2,
                vel_msg.angular.z * self.wheel_base / 2,
            ) - steering_offset
            right_speed = direction * math.hypot(
                vel_msg.linear.x
                + vel_msg.angular.z * self.steering_track / 2,
                vel_msg.angular.z * self.wheel_base / 2,
            ) + steering_offset
            self.vel[:] = [left_speed, right_speed, left_speed, right_speed]

            self.pos[0] = math.atan2(
                vel_msg.angular.z * self.wheel_base,
                2 * vel_msg.linear.x
                + vel_msg.angular.z * self.steering_track,
            )
            self.pos[1] = math.atan2(
                vel_msg.angular.z * self.wheel_base,
                2 * vel_msg.linear.x
                - vel_msg.angular.z * self.steering_track,
            )
            self.pos[2] = -self.pos[0]
            self.pos[3] = -self.pos[1]

        elif mode_selection == 2:  # in-phase
            velocity = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
            direction = np.sign(vel_msg.linear.x)
            angle = math.atan2(vel_msg.linear.y, vel_msg.linear.x)
            if angle > math.pi / 2:
                angle -= math.pi
                direction = -direction
            elif angle < -math.pi / 2:
                angle += math.pi
                direction = -direction
            self.pos[:] = angle
            self.vel[:] = direction * velocity

        elif mode_selection == 3:  # pivot turn
            angle = math.atan2(self.wheel_base, self.steering_track)
            self.pos[:] = [-angle, angle, angle, -angle]
            pivot_speed = vel_msg.angular.z * math.hypot(
                self.wheel_base / 2, self.steering_track / 2)
            self.vel[:] = [
                -pivot_speed, pivot_speed, -pivot_speed, pivot_speed]

        else:
            self.pos[:] = 0.0
            self.vel[:] = 0.0

        # The official equations produce linear wheel speeds; ros2_control's
        # velocity interfaces expect angular velocity in radians per second.
        angular_wheel_velocity = self.vel / self.wheel_radius
        self.pub_pos.publish(Float64MultiArray(data=self.pos.tolist()))
        self.pub_vel.publish(
            Float64MultiArray(data=angular_wheel_velocity.tolist()))


class JoySubscriber(Node):
    """Select an official steering mode and velocity from a joystick."""

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy, 'joy', self.listener_callback, 10)

    def listener_callback(self, data):
        global vel_msg, mode_selection
        if len(data.buttons) > 5:
            if data.buttons[0] == 1:
                mode_selection = 2
            elif data.buttons[4] == 1:
                mode_selection = 1
            elif data.buttons[5] == 1:
                mode_selection = 3
            else:
                mode_selection = 0

        if len(data.axes) > 3:
            vel_msg.linear.x = data.axes[1] * 1.5
            vel_msg.linear.y = data.axes[0] * 1.5
            vel_msg.angular.z = data.axes[3] * 2.0


class TeleopSubscriber(Node):
    """Select a steering mode from cmd_vel and store the requested Twist."""

    def __init__(self):
        super().__init__('teleop_subscriber')
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10)

    def cmd_callback(self, msg):
        global msg_vel, mode_selection
        has_linear = abs(msg.linear.x) > 1e-6 or abs(msg.linear.y) > 1e-6
        has_yaw = abs(msg.angular.z) > 1e-6

        if has_yaw and has_linear:
            mode_selection = 1
        elif has_yaw:
            mode_selection = 3
        elif has_linear:
            mode_selection = 2
        else:
            mode_selection = 0

        msg_vel.linear.x = msg.linear.x
        msg_vel.linear.y = msg.linear.y
        msg_vel.angular.z = msg.angular.z


def main(args=None):
    rclpy.init(args=args)
    commander = Commander()
    joy_subscriber = JoySubscriber()
    teleop_subscriber = TeleopSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)
    executor.add_node(teleop_subscriber)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        commander.destroy_node()
        joy_subscriber.destroy_node()
        teleop_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
