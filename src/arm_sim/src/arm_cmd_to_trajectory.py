#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmCmdToTrajectory(Node):
    """Bridge the Kratos high-level /arm_cmd interface to ros2_control JTC."""

    JOINT_NAMES = [
        "base_yaw_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist_pitch_joint",
        "wrist_roll_joint",
        "gripper_joint",
    ]

    def __init__(self):
        super().__init__("arm_cmd_to_trajectory")

        self.declare_parameter("command_topic", "/arm_cmd")
        self.declare_parameter("trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("command_horizon", 0.10)

        command_topic = self.get_parameter("command_topic").value
        trajectory_topic = self.get_parameter("trajectory_topic").value
        self.command_horizon = float(self.get_parameter("command_horizon").value)

        self.publisher = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            command_topic,
            self.command_callback,
            10,
        )

        self.get_logger().info(
            f"Bridging {command_topic} -> {trajectory_topic} "
            f"({self.command_horizon:.2f}s horizon)"
        )

    def command_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 5:
            self.get_logger().warn(
                f"Expected at least 5 arm joint values, got {len(msg.data)}"
            )
            return

        values = [float(x) for x in msg.data[:5]]
        # The high-level stack normally supplies a 6th gripper value. If it is
        # absent, hold the gripper at zero rather than dropping the command.
        values.append(float(msg.data[5]) if len(msg.data) >= 6 else 0.0)

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = values
        point.time_from_start.sec = int(self.command_horizon)
        point.time_from_start.nanosec = int(
            (self.command_horizon - int(self.command_horizon)) * 1e9
        )

        trajectory.points.append(point)
        self.publisher.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = ArmCmdToTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
