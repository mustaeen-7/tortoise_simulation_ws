from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('tortoise_bot')
    world_path = os.path.join(pkg_path, 'worlds', 'nxtwave_world.world')

    return LaunchDescription([
        # ✅ Make Gazebo aware of your package models
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', os.path.join(pkg_path, 'models')]
        ),

        # ✅ Launch Gazebo with your world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
    ])

