#!/usr/bin/env python3
"""Closed-loop check of a generated controller (run with the Gazebo launch up).

1. a trajectory over the limits is rejected (the reference does not move);
2. one point (final state + time): +0.3 rad on every joint in 3 s, then hold;
3. several points with velocities, back to the start;
4. header.stamp = now (simulation time): one point, as 2;
5. header.stamp 1.5 s in the past, first point at 1 s: warning, the reference jumps onto the trajectory;
6. header.stamp 10 s in the past, last point at 2 s: rejected (ends in the past);
7. the motion of 2 and back with adaptation on: par_REG must change and stay finite.
Prints the tracking error of each phase.
"""
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.duration import Duration as RclDuration
from rclpy.parameter import Parameter
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def duration(t):
    return Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


class Checker(Node):
    def __init__(self, controller):
        # stamps on the controller manager's clock: simulation time in Gazebo
        super().__init__('check_controller', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.state = None
        self.par = None
        self.errors = []
        self.create_subscription(JointTrajectoryControllerState, f'/{controller}/controller_state', self.on_state, 10)
        self.create_subscription(Float64MultiArray, f'/{controller}/par_REG', self.on_par, 10)
        self.publisher = self.create_publisher(JointTrajectory, f'/{controller}/joint_trajectory', 10)
        self.adaptation = self.create_client(SetBool, f'/{controller}/set_adaptation')

    def on_state(self, msg):
        self.state = msg
        self.errors.append(np.array(msg.error.positions))

    def on_par(self, msg):
        self.par = np.array(msg.data)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def send(self, points, stamp_offset=None):
        msg = JointTrajectory()
        if stamp_offset is not None:   # header.stamp = now + offset [s]
            msg.header.stamp = (self.get_clock().now() + RclDuration(seconds=stamp_offset)).to_msg()
        msg.joint_names = list(self.state.joint_names)
        for q, dq, t in points:
            point = JointTrajectoryPoint(positions=list(q), time_from_start=duration(t))
            if dq is not None:
                point.velocities = list(dq)
            msg.points.append(point)
        self.publisher.publish(msg)

    def phase(self, name, seconds):
        self.errors = []
        self.spin_for(seconds)
        e = np.abs(np.array(self.errors))
        print(f'{name}: {len(e)} samples, max |e| per joint [mrad] {np.round(1e3 * e.max(axis=0), 2)}, '
              f'final |e| [mrad] {np.round(1e3 * e[-1], 2)}')
        return e


def main():
    rclpy.init()
    node = Checker(sys.argv[1] if len(sys.argv) > 1 else 'franka_controller')
    while node.state is None or node.par is None:
        node.spin_for(0.1)
    node.spin_for(2.0)   # settle after activation
    q0 = np.array(node.state.reference.positions)
    print('reference at start [rad]', np.round(q0, 4))

    # 1. over the limits: 1.5 rad in 0.5 s peaks at 5.6 rad/s and 34.6 rad/s^2
    node.send([(q0 + np.r_[1.5, np.zeros(len(q0) - 1)], None, 0.5)])
    node.spin_for(1.0)
    moved = np.abs(np.array(node.state.reference.positions) - q0).max()
    print(f'1. rejected trajectory: reference moved by {moved:.2e} rad (expected 0)')

    # 2. one point
    q1 = q0 + 0.3
    node.send([(q1, None, 3.0)])
    node.phase('2. one point, motion + 1 s hold', 4.0)
    print('   reference at the end - target [rad]', np.round(np.array(node.state.reference.positions) - q1, 6))

    # 3. several points with velocities, back to q0
    mid = 0.5 * (q0 + q1)
    node.send([(mid, -0.1 * np.ones(len(q0)), 2.0), (q0, None, 4.0)])
    node.phase('3. two points, motion + 1 s hold', 5.0)

    # 4. stamp = now
    node.send([(q1, None, 3.0)], stamp_offset=0.0)
    node.phase('4. stamp now, one point, motion + 1 s hold', 4.0)
    print('   reference at the end - target [rad]', np.round(np.array(node.state.reference.positions) - q1, 6))

    # 5. stamp 1.5 s ago, first point at 1 s: the reference jumps onto the second segment
    before = np.array(node.state.reference.positions)
    node.send([(q1 - 0.05, None, 1.0), (q0, None, 3.0)], stamp_offset=-1.5)
    node.spin_for(0.05)
    jump = np.abs(np.array(node.state.reference.positions) - before).max()
    print(f'5. stamp 1.5 s ago: reference moved by {jump:.4f} rad in 0.05 s (a jump of about 0.05 + quintic progress)')
    node.phase('   then motion + 1 s hold', 2.5)

    # 6. ends in the past
    before = np.array(node.state.reference.positions)
    node.send([(q1, None, 1.0), (q0, None, 2.0)], stamp_offset=-10.0)
    node.spin_for(1.0)
    print(f'6. ends in the past: reference moved by {np.abs(np.array(node.state.reference.positions) - before).max():.2e} rad (expected 0)')

    # 7. adaptation on, same motion forth and back
    par_before = node.par.copy()
    future = node.adaptation.call_async(SetBool.Request(data=True))
    rclpy.spin_until_future_complete(node, future)
    node.send([(q1, None, 3.0), (q0, None, 6.0)])
    node.phase('7. adaptation on, motion + 1 s hold', 7.0)
    change = np.abs(node.par - par_before)
    print(f'   par_REG: max |change| {change.max():.3e}, finite: {np.isfinite(node.par).all()}, '
          f'entries changed: {(change > 0).sum()} of {len(change)}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
