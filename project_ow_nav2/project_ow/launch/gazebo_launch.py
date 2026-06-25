import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('project_ow')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'Dhruv.urdf')

    with open(urdf_file_path, 'r') as f:
        robot_desc = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # Launch gzserver exactly like the standalone command that works
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '/opt/ros/humble/share/gazebo_ros/worlds/empty.world',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_force_system.so'],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'Dhruv_robot'],
        output='screen'
    )

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        gzserver,
        TimerAction(period=3.0, actions=[gzclient, spawn_entity]),
        TimerAction(period=6.0, actions=[load_jsb, load_arm]),
    ])
