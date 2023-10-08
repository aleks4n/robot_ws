# Copyright (c) 2023
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import yaml, xacro
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource


#https://github.com/bponsler/ros2-support

def generate_launch_description():
    
            
    entity_name_1 = "bot1"
    entity_name_2 = "bot2"
    
    package_path = get_package_share_directory("multi_robot_arm")
    robot_desc_path = os.path.join(get_package_share_directory("multi_robot_arm"), "urdf", "bcr_bot.xacro")
    xacro_path = os.path.join(get_package_share_directory('multi_robot_arm'),'urdf','bcr_bot.xacro')
    xacro_path1 = os.path.join(get_package_share_directory('multi_robot_arm'),'urdf','logotest.xacro')
    xacro_path2 = os.path.join(get_package_share_directory('multi_robot_arm'),'urdf','logotest1.xacro')
    urdf_path1 = os.path.join(get_package_share_directory('multi_robot_arm'),'urdf','bcr_bot.xacro')
    camera_enabled = LaunchConfiguration("camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=False)
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    python_commander_dir = get_package_share_directory('nav2_simple_commander')

    map_yaml_file = os.path.join('home', 'aliihsan', 'robot_ws', 'finishedmap.yaml')
    

    cloudy_v2_urdf_path = PathJoinSubstitution(
        [FindPackageShare('multi_robot_arm'), 'urdf', 'robots', 'cloudy_v2.urdf.xacro']
    )




    with open(robot_desc_path, 'r') as infp:
       robot_desc = infp.read()
    
    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time", default_value="true", description="Use simulator time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    robot_type = "ur5"  # ROBOT_MODEL  #LaunchConfiguration("robot")
    
    world = LaunchConfiguration("world")
    declare_world_path = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            package_path,
            "worlds",
            "warehouse.world"
        ),
        description="Full path to world model file to load",
    )

    declare_world_path = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            package_path,
            "worlds",
            "warehouse.world",
        ),
        description="Full path to world model file to load",
    )


    declare_camera = DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
    declare_lidar = DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled)

    declare_robot_type = DeclareLaunchArgument(
        name="robot_type", default_value=robot_type, description="Robot type"
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-u",
            "-s", "libgazebo_ros_factory.so",
            "-s", "libgazebo_ros_init.so",
            world,
        ],
        
        output="screen",
    )
    gazebo_client = ExecuteProcess(cmd=["gzclient"], output="screen")


  



    gazebo_models_path = os.path.join(package_path, 'models')
  #gazebo_models_path = os.path.join(pkg_share, '/home/aliihsan/new_ws/src/my_bot/urdf/ur/meshes/ur5/visual')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path


    robot_state_publisher1 = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=entity_name_1,
            parameters=[{'frame_prefix': entity_name_1+'/', 
                         'use_sim_time': use_sim_time, 
                         'robot_description': Command(['xacro ', xacro_path1, 
                                                       ' robot_name:=', entity_name_1,
                                                       ' sim_gazebo:=', "true",
                                                       ' odometry_source:=', "world",
                                                       ' robot_namespace:=', entity_name_1])}],
            output="screen"
        )



 

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', "bot1/robot_description",
            '-entity', "bot1", #default enitity name _bcr_bot
            '-z', "0.25",
            '-x', "0",
            '-y', "0",
            '-Y', "0"
        ]
    )

    robot_state_publisher2 = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=entity_name_2,
            parameters=[{'frame_prefix': entity_name_2+'/', 
                         'use_sim_time': use_sim_time, 
                         'robot_description': Command(['xacro ', xacro_path2,
                                                       ' robot_name:=', entity_name_2,
                                                       ' sim_gazebo:=', "true",
                                                       ' odometry_source:=', "world",
                                                       ' robot_name:=', entity_name_2])}],
            output="screen"
        )



 

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', "bot2/robot_description",
            '-entity', "bot2", #default enitity name _bcr_bot
            '-z', "0.25",
            '-x', "5",
            '-y', "-4",
            '-Y', "0"
        ]
    )









    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_path,'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    twist_mux_params = os.path.join(package_path,'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/bir/cmd_vel')]
        )
    


    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_path,'launch','online_async_launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_path,'launch','navigation_launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_path,'launch','multi_localization.launch.py'
                )]) 
    )


    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_yaml_file}.items())



    rviz2 = ExecuteProcess(cmd=["rviz2"], output="screen")



    demo_cmd = Node(
        package='nav2_simple_commander',
        executable='example_waypoint_follower',
        emulate_tty=True,
        output='screen')



    ld = LaunchDescription()
    DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
    DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled)
    DeclareLaunchArgument("odometry_source", default_value = odometry_source)
    ld.add_action(declare_world_path)
    ld.add_action(declare_robot_type)
    ld.add_action(declare_use_sim_time)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(robot_state_publisher1)
    ld.add_action(spawn_entity1)
    ld.add_action(robot_state_publisher2)
    ld.add_action(spawn_entity2)
    ld.add_action(localization)
    
 


    #ld.add_action(spawn_robot2)
    #ld.add_action(robot_state_publisher2)
    
    # Multiple ARMs in gazebo must be spawned in a serial fashion due to 
    # a global namespace dependency introduced by ros2_control.
    # robot_final_action is the last action from previous robot and 
    # new robot will only be spawned once previous action is completed
    



    

    return ld


