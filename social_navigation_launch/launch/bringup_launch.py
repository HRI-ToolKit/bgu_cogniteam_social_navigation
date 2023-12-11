# Copyright (c) 2018 Intel Corporation
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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace

from launch_ros.actions import Node



def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('social_navigation_launch')
    launch_dir = os.path.join(bringup_dir, 'launch')
    nav2_dir = os.path.join(bringup_dir, 'launch')


    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    
    # DeclareLaunchArgument('map_path',
    #     default_value='/home/yakir/bgu_cogniteam_social_navigation_ws/src/bgu_cogniteam_social_navigation/social_navigation_ui/'
    #     , description=''),    

        
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')



    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': '/home/yakir/bgu_cogniteam_social_navigation_ws/src/bgu_cogniteam_social_navigation/social_navigation_ui/map.yaml',
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'default_bt_xml_filename': default_bt_xml_filename,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),
        
        # Declare launch arguments for initial pose x and y
        DeclareLaunchArgument('initial_pose_x', default_value='-5.0',description=''),
        DeclareLaunchArgument('initial_pose_y', default_value='7.0',description=''),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                LaunchConfiguration('initial_pose_x'),
                LaunchConfiguration('initial_pose_y'),
                '0', '0', '0', '0', 'odom', 'base_footprint'
            ]
        )
        ,Node(
            name='camera_link_camera_color_optical_frame_tf_node',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0.0', '0', '0', '0', '0','map','odom']
        ),      

        Node(
            name='social_navigation_ui_node',
            package='social_navigation_ui',
            executable='social_navigation_ui_node',
            parameters=[
                {'initial_pose_x': LaunchConfiguration('initial_pose_x')},
                {'initial_pose_y': LaunchConfiguration('initial_pose_y')},
                {'map':'/home/yakir/bgu_cogniteam_social_navigation_ws/src/bgu_cogniteam_social_navigation/social_navigation_ui/'}]

        ),
        Node(
            name='social_navigation_manager_node',
            package='social_navigation_manager',
            executable='social_navigation_manager_node'            
        )    
                           
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
