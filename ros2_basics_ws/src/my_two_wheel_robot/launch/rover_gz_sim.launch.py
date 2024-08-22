from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import tempfile
import xacro


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_file_path = os.path.join(
        get_package_share_directory("my_two_wheel_robot"),
        "model",
        "world.sdf",
    )

    # Process xacro or urdf for robot_description
    xacro_file_path = os.path.join(
        get_package_share_directory("my_two_wheel_robot"),
        "model",
        # "two_wheel_robot.xacro",
        "rover.urdf.xacro",
    )

    urdf_file_path = os.path.join(
        get_package_share_directory("my_two_wheel_robot"),
        "model",
        "two_wheel_robot.urdf",
    )

    # method 1
    # robot_description_config = xacro.process_file(xacro_file_path)
    # robot_desc = robot_description_config.toxml()
    # method 2
    robot_desc = ParameterValue(Command(["xacro ", xacro_file_path]), value_type=str)

    # Xacro command to generate URDF
    xacro_command = [
        "ros2",
        "run",
        "xacro",
        "xacro",
        xacro_file_path,
        "-o",
        urdf_file_path,
    ]

    # # Convert Xacro to URDF
    # ExecuteProcess(cmd=xacro_command, output="screen"),
    # # Include the Gazebo launch file
    # IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             os.path.join(
    #                 get_package_share_directory("gazebo_ros"),
    #                 "launch",
    #                 "gazebo.launch.py",
    #             )
    #         ]
    #     ),
    # ),

    # Start Gazebo with plugin providing the robot spawning service
    # VERSION 1 (obsolete)
    # gazebo_cmd = Node(
    #     package="gazebo_ros",
    #     executable="spawn_entity.py",
    #     arguments=["-entity", "two_wheel_robot", "-file", urdf_file],
    #     output="screen",
    # )

    # VERSION 2
    # world_sdf = tempfile.mktemp(prefix="nav2_", suffix=".sdf")
    # world_sdf_xacro = ExecuteProcess(
    #     cmd=["xacro", "-o", world_sdf, ["headless:=", "False"], world]
    # )
    # start_gazebo_cmd = ExecuteProcess(
    #     cmd=["gz", "sim", "-r", "-s", world_sdf],
    #     output="screen",
    # )

    # Publish robot state
    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     # parameters=[{"robot_description": open(urdf_file).read()}],
    #     parameters=[
    #         {
    #             "use_sim_time": True,
    #             "robot_description": Command(["xacro", " ", xacro_file_path]),
    #         }
    #     ],
    # )

    temp_urdf_file = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".urdf")
    if os.path.exists(temp_urdf_file.name):
        os.remove(temp_urdf_file.name)
    urdf_file_path = temp_urdf_file.name

    # Convert XACRO to URDF and write to the temporary file
    command = f"xacro {xacro_file_path} > {urdf_file_path}"
    convert_xacro_to_urdf = ExecuteProcess(
        cmd=[command],
        shell=True,
        output="screen",
    )

    # Start Ignition Gazebo with an empty world
    # gz_sim = ExecuteProcess(
    #     cmd=["ign", "gazebo", world_file_path, "-v", "4"], output="screen"
    # )
    world = LaunchConfiguration("world")
    declare_world_cmd = DeclareLaunchArgument(
        "world", default_value="maze.sdf", description="World file to use in Gazebo"
    )
    gz_world_arg = PathJoinSubstitution(
        [get_package_share_directory("my_two_wheel_robot"), "model", world]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_world_arg}.items(),
    )

    # Spawn the URDF model into Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "rover",
            "-allow_renaming",
            "true",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
        ],
    )
    # gz_spawn_entity = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "run",
    #         # "ros_ign_gazebo",
    #         "ros_gz_sim",
    #         "create",
    #         "-name",
    #         "my_two_wheel_robot",
    #         # "-file",
    #         # urdf_file_path,
    #         "-topic",
    #         "/robot_description",
    #         # Make the base on the floor
    #         "-x",
    #         "0",
    #         "-y",
    #         "0",
    #         "-z",
    #         "0",
    #     ],
    #     output="screen",
    # )

    # Start the robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_desc}],
    )

    # Start RViz
    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("my_two_wheel_robot"),
                "rviz",
                "robot.rviz",
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_world_cmd)
    # ld.add_action(world_sdf_xacro)

    # Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)

    # Robot state publisher
    ld.add_action(robot_state_publisher_node)

    # RViz
    ld.add_action(rviz_cmd)

    return ld
