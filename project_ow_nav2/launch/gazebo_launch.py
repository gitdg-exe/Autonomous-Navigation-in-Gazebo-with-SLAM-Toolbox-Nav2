import os
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, TimerAction,
                            IncludeLaunchDescription, DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share   = get_package_share_directory('project_ow_nav2')
    nav2_dir    = get_package_share_directory('nav2_bringup')
    slam_dir    = get_package_share_directory('slam_toolbox')

    urdf_path   = os.path.join(pkg_share, 'urdf',   'Dhruv.urdf')
    world_path  = os.path.join(pkg_share, 'worlds', 'obstacles.world')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # ── Robot State Publisher ───────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # ── Gazebo server ───────────────────────────────────────────────
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_force_system.so'],
        output='screen'
    )

    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen')

    # ── Spawn robot ─────────────────────────────────────────────────
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'Dhruv_robot',
                   '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # ── SLAM Toolbox (builds map from /scan, publishes map→odom TF) ─
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time':   'true',
            'slam_params_file': slam_params,
        }.items()
    )

    # ── Nav2 bringup (no AMCL — SLAM provides the map TF) ──────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time':  'true',
            'params_file':   nav2_params,
            'use_composition': 'False',
        }.items()
    )

    # ── RViz2 with Nav2 default config ─────────────────────────────
    rviz_cfg = os.path.join(nav2_dir, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gzserver,

        # gzclient + robot spawn after gzserver is up
        TimerAction(period=3.0, actions=[gzclient, spawn]),

        # SLAM + Nav2 after robot exists and LIDAR publishes
        TimerAction(period=8.0, actions=[slam, nav2]),

        # RViz last so Nav2 topics are ready
        TimerAction(period=12.0, actions=[rviz]),
    ])
