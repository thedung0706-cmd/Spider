import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_name = 'spider_controller'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'spider.urdf')
    
    world_file = 'empty.sdf'
    brick_file = os.path.join(pkg_share, 'worlds', 'brick.sdf')
    controller_config_file = os.path.join(pkg_share, 'config', 'controllers.yaml')
    
    # Dùng Command chuẩn công nghiệp để ép hệ thống gọi engine Xacro biên dịch đường dẫn tuyệt đối
    robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'empty',
                   '-topic', 'robot_description',
                   '-name', 'hexapod_spider',
                   '-z', '0.1'],
        output='screen'
    )

    spawn_brick = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'empty',
                   '-file', brick_file,
                   '-name', 'obstacle_brick',
                   '-x', '0.5',
                   '-y', '0.0',
                   '-z', '0.025'],
        output='screen'
    )

    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", controller_config_file],
        output="screen",
    )

    # ĐIỂM MỚI: Gọi bộ điều khiển mô-men xoắn thay vì quỹ đạo vị trí
    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_command_controller", "--param-file", controller_config_file],
        output="screen",
    )

    return LaunchDescription([
        node_robot_state_publisher,
        start_gazebo_cmd,
        spawn_robot,
        spawn_brick,
        TimerAction(period=3.0, actions=[spawn_broadcaster]),
        TimerAction(period=5.0, actions=[spawn_controller])
    ])