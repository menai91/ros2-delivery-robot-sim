import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
 
def generate_launch_description():
    
    pkg_share = get_package_share_directory('ros2_project')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
 
    # Read the URDF into a string (same as the display launch file)
    urdf_path = os.path.join(pkg_share, 'urdf', 'myrobot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
 
    # Use IncludeLaunchDescription to launch another launch.py
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
 
    # 2. Publish the URDF on /robot_description (and TF for the fixed joints)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
 
    # 3. Ask Gazebo to create a model from the /robot_description topic
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'myrobot',
            '-z', '0.02',   # drop from just above the ground
        ],
        output='screen',
    )
 
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
    ])
 