import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dwa_planner')
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')


    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_description = Command(
        ['xacro ', os.path.join(get_package_share_directory('turtlebot3_description'),
         'urdf', 'turtlebot3_waffle.urdf')]
    )


    custom_world = os.path.join(
        get_package_share_directory('dwa_planner'),
        'worlds',
        'my_world.sdf'
    )



    urdf = os.path.join(pkg_gazebo, 'models', 'turtlebot3_waffle', 'model.sdf')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time,'world': custom_world}.items()
    )


    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'turtlebot3',
             '-file', urdf,
             '-x', '0.0',      # X position
             '-y', '0.0',      # Y position
             '-z', '0.0',     # Slightly above ground to avoid collision
             '-R', '0.0',      # Roll
             '-P', '0.0',      # Pitch
             '-Y', '0.0'      # Yaw (heading)
             ],
        output='screen'
    )



    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description}]
    )






    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    dwa_planner = Node(
        package='dwa_planner',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/scan', '/scan'),
            ('/goal_pose', '/goal_pose')
        ]
    )

    # teleop_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_keyboard',
    #     output='screen',
    #     parameters=[{'stamped': False}, {'use_sim_time': use_sim_time}],
    #     prefix='xterm -e',
    #     remappings=[('cmd_vel', 'cmd_vel')]
    # )

    transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_broadcaster',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    
    static_tf_base_scan = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_scan",
        arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "base_scan"]
    )


    return LaunchDescription([
        declare_use_sim_time_cmd,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        transform_publisher,
        static_tf_base_scan,
        rviz,
        dwa_planner,
        # teleop_node
    ])