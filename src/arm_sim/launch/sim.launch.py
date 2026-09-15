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

    return LaunchDescription([

        # Gazebo
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world_file],
            output="screen",
        ),


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

        # Spawn robot into Gazebo from robot_description
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