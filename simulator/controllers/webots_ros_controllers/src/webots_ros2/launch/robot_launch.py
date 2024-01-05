import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher

WEBOTS_CONTROLLER_URL = os.getenv('WEBOTS_CONTROLLER_URL')

def generate_launch_description():
    print(f"----------   Robot: {WEBOTS_CONTROLLER_URL} . ----------")
    
    package_dir = get_package_share_directory('webots_ros2')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_robot.urdf')).read_text()

    robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        additional_env={'WEBOTS_CONTROLLER_URL': WEBOTS_CONTROLLER_URL},
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return LaunchDescription([
        robot_driver
    ])