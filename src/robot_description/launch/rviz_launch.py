import os # used to build file path

from ament_index_python.packages import get_package_share_directory # get_package_share_directory returns the path to package's install/share folder
from launch import LaunchDescription # required since generate_launch_description returns a LaunchDescription
from launch_ros.actions import Node # required since we are launching a node


def generate_launch_description():
    
    # Get path of the robot_description package (install/ros2_project/share/)
    pkg_share = get_package_share_directory('ros2_project')

    """
    First gets the path to the urdf file, then saves the content of the urdf tile to robot_description.
    
    'with' ensures that the file is automatically closed after reading
    'r' means read only (which is the default)
    """
    urdf_path = os.path.join(pkg_share, 'urdf', 'myrobot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    

    # load rviz config (this is optional)
    rviz_config = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')
    # tenary condition in python is a if condition else b. Unlike c++'s "cond ? a : b"
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []


    # Create nodes that we want to launch via the launch file
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
 
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )
 
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=rviz_args,
        output='screen',
    )

    # Takes in a list of actions to launch.
    return LaunchDescription([
    robot_state_publisher,
    joint_state_publisher_gui,
    rviz,

    ])