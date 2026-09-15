#!/usr/bin/env python3

import sys
import termios
import tty
import threading
import time
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class ArmTeleop(Node):

    def __init__(self):
        super().__init__('arm_teleop')

        self.joint_names = [
            'base_yaw_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
            'gripper_joint'
        ]

        self.joint_limits = {
            'base_yaw_joint': (-3.14, 3.14),
            'shoulder_joint': (-1.57, 1.57),
            'elbow_joint': (-2.50, 2.50),
            'wrist_pitch_joint': (-1.57, 1.57),
            'wrist_roll_joint': (-3.14, 3.14),
            'gripper_joint': (0.0, math.pi / 2.0)
        }

        self.key_to_joint = {
            'b': 'base_yaw_joint',
            's': 'shoulder_joint',
            'e': 'elbow_joint',
            'w': 'wrist_pitch_joint',
            'r': 'wrist_roll_joint',
            'g': 'gripper_joint'
        }

        self.declare_parameter('speed', 0.5)
        self.speed = float(self.get_parameter('speed').value)

        self.actual_positions = {}
        self.command_positions = {}

        self.selected_joint = None
        self.direction = 0

        self.initialized = False
        self.running = True

        self.lock = threading.Lock()

        # ---------------------------------------------------------
        # ROS
        # ---------------------------------------------------------

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # 100 Hz
        self.timer = self.create_timer(
            0.01,
            self.update
        )

        # ---------------------------------------------------------
        # Keyboard thread
        # ---------------------------------------------------------

        self.keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )

        self.keyboard_thread.start()

        self.get_logger().info('Teleop started.')
        self.get_logger().info(
            'B S E W R G = select joint'
        )
        self.get_logger().info(
            'UP / DOWN = move'
        )
        self.get_logger().info(
            'Q = quit'
        )

    # =============================================================
    # JOINT STATES
    # =============================================================

    def joint_state_callback(self, msg):

        with self.lock:

            for i, name in enumerate(msg.name):

                if name in self.joint_names:

                    if i < len(msg.position):
                        self.actual_positions[name] = msg.position[i]

            # Only initialize once
            if not self.initialized:

                if all(
                    name in self.actual_positions
                    for name in self.joint_names
                ):

                    self.command_positions = dict(
                        self.actual_positions
                    )

                    self.initialized = True

                    self.get_logger().info(
                        'Joint positions initialized.'
                    )

    # =============================================================
    # READ ONE KEY
    # =============================================================

    def read_key(self):

        fd = sys.stdin.fileno()

        old_settings = termios.tcgetattr(fd)

        try:

            tty.setraw(fd)

            key = sys.stdin.read(1)

            # Arrow key
            if key == '\x1b':

                key2 = sys.stdin.read(1)

                if key2 == '[':

                    key3 = sys.stdin.read(1)

                    if key3 == 'A':
                        return 'UP'

                    if key3 == 'B':
                        return 'DOWN'

            return key

        finally:

            termios.tcsetattr(
                fd,
                termios.TCSADRAIN,
                old_settings
            )

    # =============================================================
    # KEYBOARD
    # =============================================================

    def keyboard_loop(self):

        while self.running:

            key = self.read_key()

            # ---------------------------------------------
            # Quit
            # ---------------------------------------------

            if key.lower() == 'q':

                self.running = False

                with self.lock:
                    self.direction = 0

                rclpy.shutdown()

                return

            # ---------------------------------------------
            # Select joint
            # ---------------------------------------------

            if key.lower() in self.key_to_joint:

                with self.lock:

                    self.selected_joint = \
                        self.key_to_joint[key.lower()]

                    self.direction = 0

                print(
                    f'\nSelected: {self.selected_joint}',
                    flush=True
                )

                continue

            # ---------------------------------------------
            # UP
            # ---------------------------------------------

            if key == 'UP':

                with self.lock:

                    if self.selected_joint is not None:
                        self.direction = 1

                continue

            # ---------------------------------------------
            # DOWN
            # ---------------------------------------------

            if key == 'DOWN':

                with self.lock:

                    if self.selected_joint is not None:
                        self.direction = -1

                continue

    # =============================================================
    # UPDATE POSITION
    # =============================================================

    def update(self):

        if not self.initialized:
            return

        with self.lock:

            if self.selected_joint is None:
                return

            if self.direction == 0:
                return

            joint = self.selected_joint

            # 10 ms
            dt = 0.01

            self.command_positions[joint] += (
                self.speed *
                self.direction *
                dt
            )

            # Clamp
            lower, upper = self.joint_limits[joint]

            self.command_positions[joint] = max(
                lower,
                min(
                    upper,
                    self.command_positions[joint]
                )
            )

            positions = [
                self.command_positions[name]
                for name in self.joint_names
            ]

        # ---------------------------------------------------------
        # Publish
        # ---------------------------------------------------------

        msg = JointTrajectory()

        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        point.positions = positions

        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000

        msg.points.append(point)

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = ArmTeleop()

    executor = rclpy.executors.MultiThreadedExecutor(
        num_threads=2
    )

    executor.add_node(node)

    try:

        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:

        node.running = False

        if rclpy.ok():
            rclpy.shutdown()

        node.destroy_node()


if __name__ == '__main__':
    main()