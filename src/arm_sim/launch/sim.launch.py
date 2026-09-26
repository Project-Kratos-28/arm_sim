import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as file:
        return yaml.safe_load(file)


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as file:
        return file.read()


def generate_launch_description():
    pkg_arm = get_package_share_directory("arm_sim")
    # Gazebo Sim needs the *parent* share directory for model://arm_sim/...
    # and the Jazzy library directory for gz_ros2_control.
    arm_install_share = os.path.dirname(pkg_arm)
    arm_install_lib = os.path.join(
        get_package_share_directory("arm_sim"), "..", "..", "lib"
    )
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    resource_path = os.pathsep.join(
        [p for p in [arm_install_share, existing_resource_path] if p]
    )
    plugin_path = os.pathsep.join(
        [p for p in ["/opt/ros/jazzy/lib", arm_install_lib, existing_plugin_path] if p]
    )

    xacro_file = os.path.join(pkg_arm, "urdf", "arm_cad.urdf.xacro")
    world_file = os.path.join(pkg_arm, "worlds", "pick_test.sdf")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_high_level = LaunchConfiguration("use_high_level")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    declare_use_high_level = DeclareLaunchArgument(
        "use_high_level",
        default_value="false",
        description="Launch the arm_high_level stack",
    )

    use_high_level = LaunchConfiguration("use_high_level")

    declare_use_high_level = DeclareLaunchArgument(
        "use_high_level",
        default_value="false",
        description="Launch the arm_high_level stack"
    )

    # arm_sim owns the robot description used by Gazebo and robot_state_publisher.
    # High-level control is completely separate and communicates through /arm_cmd.
    
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file, " use_gazebo:=true", " use_mock_hardware:=false"]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # Gazebo.
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=resource_path
    )
    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH", value=plugin_path
    )

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r" , "-s", world_file],
        output="screen",
    )

    # /clock bridge.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",

            "/elbow_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/elbow_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/gripper_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/gripper_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
    )
    

    # Robot state publisher uses the simulation robot description.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Spawn into Gazebo from robot_description.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "custom_arm",
            "-z", "0.3",
        ],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Simulation command bridge.
    # arm_high_level publishes Float64MultiArray on /arm_cmd.
    # This node converts that command into a JointTrajectory for ros2_control.
    # ros2_control's JointTrajectoryController expects JointTrajectory on
    # /arm_controller/joint_trajectory, so this small bridge connects them.
    command_bridge = Node(
        package="arm_sim",
        executable="arm_cmd_to_trajectory.py",
        name="arm_cmd_to_trajectory",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    high_level_launch_file = os.path.join(
        os.path.expanduser("~/arm_high_level"),
        "src",
        "arm_controller",
        "launch",
        "arm.launch.py",
    )

    high_level_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(high_level_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz": "false",
        }.items(),
        condition=IfCondition(use_high_level),
    )

    

    # Keep the existing RTF fix because it was part of the working sim setup.
    set_rtf = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "until gz service -s /world/pick_test/set_physics "
            "--reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 1000 "
            "--req 'real_time_factor: 1.0'; do "
            "echo 'Waiting for Gazebo physics service...'; sleep 1; done; "
            "echo 'Gazebo real-time factor set to 1.0'"
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_high_level, 
        set_gz_resource_path,
        set_gz_plugin_path,
        gazebo,
        clock_bridge,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[set_rtf]),
        TimerAction(period=8.0, actions=[joint_state_spawner]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[arm_spawner],
            )
        ),
        # Start high-level nodes after the controller manager has had time to appear.
        TimerAction(
            period=10.0,
            actions=[command_bridge],
        ),

        TimerAction(
            period=12.0,
            actions=[high_level_launch],
        ),

    ])
