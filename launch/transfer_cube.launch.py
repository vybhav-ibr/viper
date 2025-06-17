from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Dynamically get the full path to the SDF file in your package
    sdf_file_path = os.path.join(
        get_package_share_directory('viperX300_description'),
        'urdf',
        'box.sdf'
    )

    return LaunchDescription([
        # Delay the service call to give Gazebo time to start
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                cmd = [
    'ros2', 'service', 'call',
    '/world/empty/create',
    'ros_gz_interfaces/srv/SpawnEntity',
    f'{{entity_factory: {{name: "box", sdf_filename: "{sdf_file_path}", pose: {{position: {{x: 0.0, y: 0.0, z: 0.025}}, orientation: {{x: 0.0, y: 0.0, z: 0.1, w: 1.0}}}}}}}}'],
                    output='screen'
                )
            ]
        )
    ])

