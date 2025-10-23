import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the model path from environment variable
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('tortoise_bot'),
        'models',
        model_folder,
        'model.sdf'  # ✅ make sure this file actually exists
    )

    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='8.087546')
    y_pose = LaunchConfiguration('y_pose', default='2.548081')
    roll  = LaunchConfiguration('roll',  default='0.000490')
    pitch = LaunchConfiguration('pitch', default='0.006094')
    yaw   = LaunchConfiguration('yaw',   default='-1.524284')

    # Node to spawn entity in Gazebo
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-file', urdf_path,       # ✅ use the actual URDF/SDF path here
            '-x', x_pose,
            '-y', y_pose,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(start_gazebo_ros_spawner_cmd)
    return ld

