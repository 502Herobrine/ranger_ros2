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

"""Regress wheel velocities, steering interlock and command/feedback watchdogs."""

import importlib.util
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray


spec = importlib.util.spec_from_file_location(
    'robot_control', Path(__file__).resolve().parents[1] / 'scripts/robot_control.py')
control = importlib.util.module_from_spec(spec)
spec.loader.exec_module(control)


@pytest.mark.parametrize('twist', [
    (0.0, 0.0, 0.0), (0.2, 0.0, 0.0), (-0.2, 0.0, 0.0),
    (0.0, 0.2, 0.0), (0.0, -0.2, 0.0), (0.0, 0.0, 0.8),
    (0.0, 0.0, -0.8), (0.15, 0.0, 0.4), (-0.15, 0.0, 0.4),
    (0.1, -0.1, 0.3)])
def test_wheel_velocities(twist):
    # Reconstruct physical rolling velocities, independently of steering modes.
    angles, speeds = control.wheel_targets(*twist, 0.5, 0.38, 0.09)
    centers = [(0.25, 0.19), (0.25, -0.19), (-0.25, 0.19), (-0.25, -0.19)]
    assert len(angles) == len(speeds) == 4
    for angle, speed, (x, y) in zip(angles, speeds, centers):
        assert -math.pi / 2 <= angle <= math.pi / 2
        assert speed * 0.09 * math.cos(angle) == pytest.approx(twist[0] - twist[2] * y)
        assert speed * 0.09 * math.sin(angle) == pytest.approx(twist[1] + twist[2] * x)


@pytest.fixture
def commander():
    rclpy.init()
    node = control.Commander()
    # Deterministic ROS time: watchdog checks need no wall-clock sleeps.
    node.get_clock()._set_ros_time_is_active(True)
    node.get_clock().set_ros_time_override(Time(seconds=10))
    for timer in node.timers:
        timer.cancel()
    observer = rclpy.create_node('command_observer')
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    output = {}
    observer.create_subscription(
        Float64MultiArray, '/forward_position_controller/commands',
        lambda msg: output.update(angles=list(msg.data)), 10)
    observer.create_subscription(
        Float64MultiArray, '/forward_velocity_controller/commands',
        lambda msg: output.update(speeds=list(msg.data)), 10)

    def sample():
        deadline = time.monotonic() + 5.0
        while (node.pub_pos.get_subscription_count() == 0 or
               node.pub_vel.get_subscription_count() == 0):
            assert time.monotonic() < deadline, 'Output subscribers unavailable'
            executor.spin_once(timeout_sec=0.02)
        output.clear()
        node.update()
        while len(output) < 2 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(output) == 2, 'Controller output topics unavailable'
        return output['angles'], output['speeds']

    try:
        yield node, sample
    finally:
        executor.shutdown()
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def lateral_command(node):
    msg = Twist()
    msg.linear.y = 0.2
    node.command(msg)


def feedback(node, angles):
    # JointState ordering is independent of the fixed controller array ordering.
    node.joints(JointState(name=node.names[::-1], position=list(angles)[::-1]))


def test_waits_for_complete_aligned_steering(commander):
    node, sample = commander
    lateral_command(node)
    assert sample()[1] == [0.0] * 4
    node.joints(JointState(name=node.names[:3], position=[math.pi / 2] * 3))
    assert sample()[1] == [0.0] * 4
    feedback(node, [0.0] * 4)
    assert sample()[1] == [0.0] * 4
    feedback(node, [math.pi / 2] * 4)
    angles, speeds = sample()
    assert angles == pytest.approx([math.pi / 2] * 4)
    assert speeds == pytest.approx([0.2 / 0.09] * 4)


@pytest.mark.parametrize('stop', ['zero', 'command_timeout', 'feedback_timeout', 'clock_reset'])
def test_stop_retains_steering(commander, stop):
    node, sample = commander
    lateral_command(node)
    feedback(node, [math.pi / 2] * 4)
    angles, speeds = sample()
    assert min(speeds) > 0
    if stop == 'zero':
        node.command(Twist())
    elif stop == 'clock_reset':
        node.get_clock().set_ros_time_override(Time(seconds=1))
    else:
        node.get_clock().set_ros_time_override(Time(seconds=11))
        if stop == 'command_timeout':
            feedback(node, angles)
        else:
            lateral_command(node)
    stopped_angles, stopped_speeds = sample()
    assert stopped_angles == angles
    assert stopped_speeds == [0.0] * 4


@pytest.mark.parametrize('button,expected', [
    (0, (0.75, -0.75, 0.0)), (4, (0.75, 0.0, 0.5)), (5, (0.0, 0.0, 0.5))])
def test_joystick_modes_and_release(commander, button, expected):
    node, _ = commander
    buttons = [0] * 6
    buttons[button] = 1
    node.joy(Joy(axes=[-0.5, 0.5, 0.0, 0.25], buttons=buttons))
    assert node.target == expected
    node.joy(Joy(axes=[-0.5, 0.5, 0.0, 0.25], buttons=[0] * 6))
    assert node.target == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('prefixes', [
    ('fl', 'fr', 'rl', 'rr'), ('front_left', 'front_right', 'rear_left', 'rear_right')])
def test_model_joint_names_and_incomplete_feedback(commander, prefixes):
    node, sample = commander
    msg = Twist()
    msg.linear.x, msg.angular.z = 0.15, 0.4
    node.command(msg)
    # Distinct wheel targets catch accidental ordering errors with either model.
    angles, expected = control.wheel_targets(0.15, 0.0, 0.4, 0.5, 0.38, 0.09)
    names = [prefix + '_steering_joint' for prefix in prefixes]
    node.joints(JointState(name=names[::-1], position=angles[::-1]))
    assert sample()[1] == pytest.approx(expected)
    # Unrelated/partial feedback must not refresh old steering measurements.
    node.joints(JointState(name=['arm_joint'], position=[0.0]))
    assert sample()[1] == [0.0] * 4
    node.joints(JointState(name=names, position=[math.nan] + angles[1:]))
    assert sample()[1] == [0.0] * 4
