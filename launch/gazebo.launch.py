
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
 
    share_dir = get_package_share_directory('viperX300_description')
    print(share_dir)
    xacro_file = os.path.join(share_dir, 'urdf', 'vx300s.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    # joint_state_publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Launch Gazebo Sim (gz sim)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-v 4 -s --headless-rendering -r --verbose','use_sim_time': 'true'}.items()
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/world/default/model/Example/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model', 
                   '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                  '/left_cam/image_raw@sensor_msgs/msg/Image[gz.msgs.Image', '/top_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                  '/right_cam/image_raw@sensor_msgs/msg/Image[gz.msgs.Image', '/right_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                  '/top_cam/image_raw@sensor_msgs/msg/Image[gz.msgs.Image', '/top_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',]
        )
    # Spawn entity in Gazebo Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'vx300s',
            '-string', robot_urdf,
            '-allow_renaming', 'true'
        ],
        output='screen'
    )
    
    left_robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('viperX300_description'),
            'config',
            'left_ros2_controller.yaml',
        ]
    )
    
    left_joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_joint_trajectory_controller',
            '--param-file',
            left_robot_controllers,
            ],
    )

    left_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_joint_state_broadcaster',
            '--param-file',
            left_robot_controllers,
        ],
    )
    
    right_robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('viperX300_description'),
            'config',
            'right_ros2_controller.yaml',
        ]
    )
    
    right_joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'right_joint_trajectory_controller',
            '--param-file',
            right_robot_controllers,
            ],
    )

    right_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'right_joint_state_broadcaster',
            '--param-file',
            right_robot_controllers,
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_sim,
        gz_ros2_bridge,
        spawn_entity,
        left_joint_state_broadcaster_spawner,
        left_joint_trajectory_controller_spawner,
        right_joint_state_broadcaster_spawner,
        right_joint_trajectory_controller_spawner
    ])
