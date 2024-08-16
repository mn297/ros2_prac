from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the Xacro file
    xacro_file = os.path.join(
        get_package_share_directory("my_two_wheel_robot"),
        "model",
        "two_wheel_robot.xacro",
    )

    # Convert the Xacro file to a URDF file
    urdf_file = "/tmp/two_wheel_robot.urdf"
    urdf_file = os.path.join(
        get_package_share_directory("my_two_wheel_robot"),
        "model",
        "two_wheel_robot.urdf",
    )

    # Xacro command to generate URDF
    xacro_command = ["ros2", "run", "xacro", "xacro", xacro_file, "-o", urdf_file]

    return LaunchDescription(
        [
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
            # Spawn the entity in Gazebo
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity", "two_wheel_robot", "-file", urdf_file],
                output="screen",
            ),
            # Publish robot state
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": open(urdf_file).read()}],
            ),
            # Start RViz
            Node(
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
            ),
        ]
    )
