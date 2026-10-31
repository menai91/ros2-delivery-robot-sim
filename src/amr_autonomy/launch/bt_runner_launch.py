from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tree = LaunchConfiguration('tree')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tree', default_value='delivery_queue',
            description='Tree name in the installed trees/ folder, or a path to a tree XML file'),

        # bt_runner resolves the name itself, so the argument is passed through unchanged.
        # use_sim_time is set inside bt_runner, so it is not set here.
        Node(
            package='amr_autonomy',
            executable='bt_runner',
            arguments=[tree],
            output='screen'),
    ])
