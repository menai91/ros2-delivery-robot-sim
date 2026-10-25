import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
 
def generate_launch_description():
    
    pkg_share = get_package_share_directory('ros2_project')
    # Read the URDF into a string (same as the display launch file)
    urdf_path = os.path.join(pkg_share, 'urdf', 'myrobot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='my_world.sdf',
        description='Name of the SDF world file to load'
    )

    nav_arg = DeclareLaunchArgument(
        'use_nav',
        default_value='false',
        description='Start Nav2 with the saved map (use with slam:=false)'
    )
    
    slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Start slam_toolbox (set false when using Nav2 with a saved map)'
    )


    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        LaunchConfiguration('world')
    ])

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
 
    # Use IncludeLaunchDescription to launch another launch.py
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                '-r ',
                world_file,
            ]
        }.items(),
    )

    # 2. Publish the URDF on /robot_description (and TF for the fixed joints)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
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

    # 4. Create a bridge between ROS 2 and Gazebo so that it can send messages between one another
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # direction of where the message / topic goes
            # [ Gazebo to ROS (the robot’s outputs, like odometry)
            # ] ROS to Gazebo (commands into the robot, like /cmd_vel)
            # @ both ways
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            output='screen',
   )

    # 5. Load my rviz config 
    rviz_config = os.path.join(
        pkg_share, 'rviz', 'urdf_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 6. launch SLAM
    slam_params = os.path.join(
        pkg_share, 'config', 'slam_params.yaml')
    
    slam_map_file = PathJoinSubstitution([
        pkg_share,
        'maps',
        PythonExpression([
            "'", LaunchConfiguration('world'), "'.removesuffix('.sdf') + '_map.yaml'"
        ]),
    ])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam')))
    
    #7. Launch Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': slam_map_file,
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav')))
        
    return LaunchDescription([
        world_arg,
        slam_arg,
        nav_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        gazebo_bridge,
        rviz_node,
        slam,
        nav2,
    ])
 