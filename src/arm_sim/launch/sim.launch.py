from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os


def generate_launch_description():

    pkg_arm = get_package_share_directory("arm_sim")

    xacro_file = os.path.join(
        pkg_arm,
        "urdf",
        "arm_cad.urdf.xacro",
    )

    world_file = os.path.join(
        pkg_arm,
        "worlds",
        "empty.sdf",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file,
            " use_gazebo:=true",
            " use_mock_hardware:=false",
        ])
    }

    # Automatically set Gazebo real-time factor to 1.0
    set_rtf = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            """
            until gz service -s /world/empty/set_physics \
                --reqtype gz.msgs.Physics \
                --reptype gz.msgs.Boolean \
                --timeout 1000 \
                --req 'real_time_factor: 1.0'
            do
                echo "Waiting for Gazebo physics service..."
                sleep 1
            done
            echo "Gazebo real-time factor set to 1.0"
            """
        ],
        output="screen",
    )

    return LaunchDescription([

        # Gazebo
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world_file],
            output="screen",
        ),

        # Wait for Gazebo to start, then set RTF = 1.0
        TimerAction(
            period=3.0,
            actions=[
                set_rtf
            ]
        ),

        # /clock bridge
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ],
            output="screen",
        ),

        # Publish robot description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                robot_description,
                {"use_sim_time": True},
            ],
            output="screen",
        ),

        # Spawn robot
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic",
                "robot_description",
                "-name",
                "custom_arm",
                "-z",
                "1",
            ],
            output="screen",
        ),

        # Controllers
        TimerAction(
            period=10.0,
            actions=[
                joint_state_spawner
            ]
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[arm_spawner],
            )
        ),
    ])