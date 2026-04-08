from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_g1_sim_controller = FindPackageShare('g1_sim_controller')

    # Wall-clock time everywhere — the simulator runs near real-time and
    # all DDS timestamps (lidar, IMU) already use time.time().
    use_sim_time = False

    pc_config = PathJoinSubstitution([
        pkg_g1_sim_controller,
        "config",
        "pc_to_laserscan.yaml",
    ])

    slam_config = PathJoinSubstitution([
        pkg_g1_sim_controller,
        "config",
        "slam_toolbox.yaml",
    ])

    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='ens5',
        description='Network interface to use for Unitree SDK'
    )

    basic_telemetry_node = Node(
        package='g1_sim_controller',
        executable='basic_telemetry',
        name='basic_telemetry',
        output='screen',
        parameters=[
            {'network_interface': LaunchConfiguration('network_interface')},
            {'use_sim_time': use_sim_time},
        ]
    )

    # cmd_vel_bridge disabled – replaced by dds_ros2_bridge
    # cmd_vel_bridge_node = Node(
    #     package='g1_sim_controller',
    #     executable='cmd_vel_bridge',
    #     name='cmd_vel_bridge',
    #     output='screen',
    #     parameters=[{'network_interface': LaunchConfiguration('network_interface')}]
    # )

    dds_ros2_bridge_node = Node(
        package='g1_sim_controller',
        executable='dds_ros2_bridge',
        name='dds_ros2_bridge',
        output='screen',
        parameters=[
            {'network_interface': LaunchConfiguration('network_interface')},
            {'use_sim_time': use_sim_time},
        ]
    )

    camera_bridge_node = Node(
        package='g1_sim_controller',
        executable='camera_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    pc_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pc_to_ls",
        output="screen",
        parameters=[
            pc_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cloud_in", "/utlidar/cloud_livox_mid360"),
            ("scan",     "/scan"),
        ],
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0', '0', '0.46', '0', '0', '0', 'base_link', 'livox_frame'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        network_interface_arg,
        basic_telemetry_node,
        dds_ros2_bridge_node,
        camera_bridge_node,
        pc_to_laserscan_node,
        slam_node,
        static_tf_node
    ])
