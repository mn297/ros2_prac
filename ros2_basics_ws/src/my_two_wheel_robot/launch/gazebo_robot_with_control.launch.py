from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        # os.getenv('HOME'), 'ros2_ws/src/my_two_wheel_robot/urdf/two_wheel_robot.xacro')
        get_package_share_directory('my_two_wheel_robot'), 'urdf/two_wheel_robot.xacro')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'two_wheel_robot', '-file', urdf_file],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                # os.getenv('HOME'), 'ros2_ws/src/my_two_wheel_robot/rviz/robot.rviz')]
                get_package_share_directory('my_two_wheel_robot'), 'rviz/robot.rviz')]
        )
    ])
