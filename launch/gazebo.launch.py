from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription , AppendEnvironmentVariable, GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join 
from launch.substitutions import Command, PathJoinSubstitution
from pathlib import Path
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():

    # Start a simulation with the cafe world
    gz_sim_path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    sprayer_path = get_package_share_directory("sprayerbot")
    world_file = join(sprayer_path, "models", "worlds", "ag_world.sdf")
    
    #world_file="empty.sdf"
    gazebo_sim = IncludeLaunchDescription(gz_sim_path,
                                          launch_arguments=[("gz_args", world_file),
                                                             ("gz_version", "8")])

    # Create a robot in the world.
    # Steps: 
    # 1. Process a file using the xacro tool to get an xml file containing the robot description.
    # 2. Publish this robot description using a ros topic so all nodes can know about the joints of the robot. 
    # 3. Spawn a simulated robot in the gazebo simulation using the published robot description topic. 

    # Step 1. Process robot file. 
    robot_file = join(get_package_share_directory("sprayerbot"), "robot_description","sprayer.urdf.xacro")

    #Step 2. Publish robot file to ros topic /robot_description & static joint positions to /tf

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', str(robot_file)]), value_type=str
            )
        }]
    )

    # Step 3. Spawn a robot in gazebo by listening to the published topic.
    robot = Node(
        package='ros_gz_sim',
        executable="create",
        arguments=[
            "-topic", "/robot_description", 
            "-z", "0.5",
        ],
        name="spawn_robot",
        output="both"
    )

    # Gazebo Bridge: This brings data (sensors/clock) out of gazebo into ROS.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                   '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   ],
        output='screen'
        )

    # Step 5: Enable the ros2 controllers
    start_controllers  = Node(
                package="controller_manager",
                executable="spawner",
                arguments=['joint_state_broadcaster', 'diff_drive_base_controller'],
                output="screen",
            )
    
    # A gui tool for easy tele-operation.
    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
    )

    # Twist Stamper

    stamper = Node(package="twist_stamper", executable="twist_stamper", 
                   remappings=[("cmd_vel_in","/cmd_vel"),
                              ("cmd_vel_out", "/diff_drive_base_controller/cmd_vel")])
    
    # Adds the models to the path so Gazebo can find them. 
    model_path = AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", join(sprayer_path, "models"))

    robot_localization_dir = get_package_share_directory('robot_localization')
    parameters_file_dir = join(sprayer_path, 'config')
    parameters_file_path = join(sprayer_path, 'example_config.yaml')
    
    os.environ['FILE_PATH'] = str(parameters_file_dir)

   
    local_filter =  Node(
                package='robot_localization', 
                executable='ekf_node', 
                name='local_localization',
                output='screen',
                parameters=[join(parameters_file_dir, "example_config.yaml")],
                remappings=[('odometry/filtered', 'odometry/local')]    

            )
    global_filter =    Node(
                package='robot_localization', 
                executable='ekf_node', 
                name='global_localization',
                output='screen',
                parameters=[join(parameters_file_dir, "example_config.yaml")]                    
            )           
    nav_transform = Node(
                package='robot_localization', 
                executable='navsat_transform_node', 
                name='navsat_transform_node',
                output='screen',
                parameters=[join(parameters_file_dir, "navsat.yaml")],
                remappings=[
                            ('gps/fix', '/navsat'),
                            ('imu','/imu/data')
                            ]           

            )   


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([sprayer_path, 'config', 'rviz.rviz'])],
        parameters=[('use_sim_time',"True")]
    )

    return LaunchDescription([gazebo_sim,  robot, bridge, stamper,
                              robot_steering, robot_state_publisher,
                              start_controllers,  model_path, local_filter, global_filter, nav_transform, rviz])