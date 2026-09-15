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

        # =========================================================
        # JOINT NAMES
        # =========================================================

        self.joint_names = [
            'base_yaw_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
            'gripper_joint'
        ]

        # =========================================================
        # JOINT LIMITS
        # =========================================================

        self.joint_limits = {

            'base_yaw_joint':
                (-3.14, 3.14),

            'shoulder_joint':
                (-1.57, 1.57),

            'elbow_joint':
                (-2.50, 2.50),

            'wrist_pitch_joint':
                (-1.57, 1.57),

            'wrist_roll_joint':
                (-3.14, 3.14),

            'gripper_joint':
                (0.0, math.pi / 2.0)
        }

        # =========================================================
        # KEY -> JOINT
        # =========================================================

        self.key_to_joint = {

            'b': 'base_yaw_joint',
            's': 'shoulder_joint',
            'e': 'elbow_joint',
            'w': 'wrist_pitch_joint',
            'r': 'wrist_roll_joint',
            'g': 'gripper_joint'
        }

        # =========================================================
        # SPEED
        # =========================================================

        self.declare_parameter(
            'speed',
            0.5
        )

        self.speed = float(
            self.get_parameter('speed').value
        )

        # =========================================================
        # POSITION STATE
        # =========================================================

        self.command_positions = {
            name: 0.0
            for name in self.joint_names
        }

        self.actual_positions = {
            name: 0.0
            for name in self.joint_names
        }

        self.initialized = False

        # =========================================================
        # KEYBOARD STATE
        # =========================================================

        self.selected_joint = None

        # +1 = up
        # -1 = down
        #  0 = stopped

        self.direction = 0

        # Time at which the last movement key was received
        self.last_move_key_time = 0.0

        # If no arrow key is received for this long,
        # stop the arm.
        self.key_timeout = 0.15

        self.running = True

        self.lock = threading.Lock()

        # =========================================================
        # ROS SUBSCRIBER
        # =========================================================

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # =========================================================
        # ROS PUBLISHER
        # =========================================================

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # =========================================================
        # MOVEMENT TIMER
        # =========================================================

        # 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.update
        )

        # =========================================================
        # KEYBOARD THREAD
        # =========================================================

        self.keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )

        self.keyboard_thread.start()

        # =========================================================
        # LOGGING
        # =========================================================

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            'Arm Teleop Started'
        )

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            'B = base yaw'
        )

        self.get_logger().info(
            'S = shoulder'
        )

        self.get_logger().info(
            'E = elbow'
        )

        self.get_logger().info(
            'W = wrist pitch'
        )

        self.get_logger().info(
            'R = wrist roll'
        )

        self.get_logger().info(
            'G = gripper'
        )

        self.get_logger().info(
            'UP = move forward'
        )

        self.get_logger().info(
            'DOWN = move backward'
        )

        self.get_logger().info(
            'Q = quit'
        )

    # =============================================================
    # JOINT STATE CALLBACK
    # =============================================================

    def joint_state_callback(self, msg):

        with self.lock:

            for i, name in enumerate(msg.name):

                if name in self.joint_names:

                    if i < len(msg.position):

                        self.actual_positions[name] = \
                            msg.position[i]

            # Initialize command positions from simulator
            # only once.

            if not self.initialized:

                for name in self.joint_names:

                    if name in msg.name:

                        index = msg.name.index(name)

                        if index < len(msg.position):

                            self.command_positions[name] = \
                                msg.position[index]

                self.initialized = True

                self.get_logger().info(
                    'Joint positions initialized.'
                )

    # =============================================================
    # READ KEY
    # =============================================================

    def read_key(self):

        fd = sys.stdin.fileno()

        old_settings = termios.tcgetattr(fd)

        try:

            tty.setraw(fd)

            key = sys.stdin.read(1)

            # -----------------------------------------------------
            # Arrow keys
            #
            # UP:
            # ESC [ A
            #
            # DOWN:
            # ESC [ B
            # -----------------------------------------------------

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
    # KEYBOARD LOOP
    # =============================================================

    def keyboard_loop(self):

        while self.running:

            try:

                key = self.read_key()

            except Exception:

                break

            now = time.monotonic()

            # =====================================================
            # QUIT
            # =====================================================

            if key.lower() == 'q':

                with self.lock:

                    self.direction = 0
                    self.running = False

                return

            # =====================================================
            # JOINT SELECTION
            # =====================================================

            if key.lower() in self.key_to_joint:

                with self.lock:

                    self.selected_joint = \
                        self.key_to_joint[key.lower()]

                    # Stop previous movement when changing joint
                    self.direction = 0

                print(
                    f'\nSelected: '
                    f'{self.selected_joint}',
                    flush=True
                )

                continue

            # =====================================================
            # UP
            # =====================================================

            if key == 'UP':

                with self.lock:

                    if self.selected_joint is not None:

                        self.direction = 1

                        self.last_move_key_time = now

                continue

            # =====================================================
            # DOWN
            # =====================================================

            if key == 'DOWN':

                with self.lock:

                    if self.selected_joint is not None:

                        self.direction = -1

                        self.last_move_key_time = now

                continue

    # =============================================================
    # UPDATE
    # =============================================================

    def update(self):

        with self.lock:

            if not self.initialized:
                return

            if self.selected_joint is None:
                return

            # -----------------------------------------------------
            # AUTOMATIC STOP
            # -----------------------------------------------------

            # Terminal key-repeat stops when the key is released.
            # If we haven't received another arrow event recently,
            # assume the key has been released.

            if (
                time.monotonic() -
                self.last_move_key_time
                > self.key_timeout
            ):

                self.direction = 0

            # -----------------------------------------------------
            # If stopped, don't publish movement
            # -----------------------------------------------------

            if self.direction == 0:
                return

            joint = self.selected_joint

            # =====================================================
            # POSITION UPDATE
            # =====================================================

            dt = 0.05

            self.command_positions[joint] += (
                self.speed *
                self.direction *
                dt
            )

            # =====================================================
            # LIMIT
            # =====================================================

            lower, upper = self.joint_limits[joint]

            self.command_positions[joint] = max(
                lower,
                min(
                    upper,
                    self.command_positions[joint]
                )
            )

            # -----------------------------------------------------
            # If limit reached, stop movement
            # -----------------------------------------------------

            if (
                self.command_positions[joint] <= lower
                and self.direction < 0
            ):

                self.direction = 0

            if (
                self.command_positions[joint] >= upper
                and self.direction > 0
            ):

                self.direction = 0

            # =====================================================
            # COPY ALL POSITIONS
            # =====================================================

            positions = [
                self.command_positions[name]
                for name in self.joint_names
            ]

        # =========================================================
        # PUBLISH TRAJECTORY
        # =========================================================

        msg = JointTrajectory()

        msg.joint_names = list(
            self.joint_names
        )

        point = JointTrajectoryPoint()

        point.positions = positions

        # ---------------------------------------------------------
        # IMPORTANT:
        #
        # Very short trajectory prevents the controller from
        # trying to execute an old command for a long time.
        # ---------------------------------------------------------

        point.time_from_start.sec = 0

        point.time_from_start.nanosec = 50000000

        msg.points.append(point)

        self.publisher.publish(msg)


# =================================================================
# MAIN
# =================================================================

def main(args=None):

    rclpy.init(args=args)

    node = ArmTeleop()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.running = False

        if rclpy.ok():

            rclpy.shutdown()

        node.destroy_node()


if __name__ == '__main__':

    main()