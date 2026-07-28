import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'spider_controller'

    # 1. Bật Gazebo Sim (Hệ sinh thái mới)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    # 2. Đọc bản vẽ 3D
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'spider.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # 3. Node xuất trạng thái robot
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 4. Thả robot vào Gazebo Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'spider'],
        output='screen'
    )

    # 5. Khởi động các bộ điều khiển có độ trễ để chờ Gazebo và controller_manager sẵn sàng
    load_joint_state_broadcaster = TimerAction(
        period=3.0,  # Chờ 3 giây cho Gazebo khởi động xong
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    load_joint_trajectory_controller = TimerAction(
        period=4.5,  # Chờ thêm 1.5 giây nữa để bật bộ điều khiển quỹ đạo
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
    ])
# =====================================================================
    # 6. GỌI SPAWNER CHUẨN: Truyền trực tiếp file YAML vào từng bộ điều khiển
    # =====================================================================
    yaml_file_path = os.path.join(pkg_path, 'config', 'controllers.yaml')
    
    load_joint_state_broadcaster = TimerAction(
        period=3.5,  
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--param-file', yaml_file_path],
                output='screen'
            )
        ]
    )

    load_joint_trajectory_controller = TimerAction(
        period=5.0,  
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller', '--param-file', yaml_file_path],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        node_robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
    ])