#!/usr/bin/env python3
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
#
# Authors: Arshad Mehmood

import os
import yaml
import random

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from swarm_control.utils.namespace_utils import load_sdf_with_namespace, create_namespaced_bridge_yaml

def launch_setup(context, *args, **kwargs):
    random.seed(42)
    # Paths
    swarm_dir = get_package_share_directory('swarm_control')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # Simulation config
    world_path = os.path.join(swarm_dir, 'worlds', 'empty_world.world')
    
    actions = []
    # Launch Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v2 {world_path}', 'on_exit_shutdown': 'true'}.items()
    )
    actions.append(gzserver_cmd)

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items()
    )
    actions.append(gzclient_cmd)

    # Add GZ model path to env
    environment = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(swarm_dir, 'models')
    )
    actions.append(environment)

    # Read evaluated values
    use_sim_time = LaunchConfiguration('use_sim_time', default='true').perform(context)
    num_bots = int(LaunchConfiguration('num_bots').perform(context))

    # Load model and URDF
    TURTLEBOT3_MODEL = 'waffle'
    model_dir = f'turtlebot3_{TURTLEBOT3_MODEL}'
    # sdf_file_name = 'model.sdf'
    sdf_file_name = 'minimal_model.sdf'
    sdf_path = os.path.join(swarm_dir, 'models', model_dir, sdf_file_name)

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    # urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_file_name = 'minimal_urdf.urdf'
    urdf_path = os.path.join(swarm_dir, 'urdf', urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Spawn each bot
    for i in range(num_bots):
        namespace = f'bot{i + 1}'
        x_pose = round(random.uniform(-3.0, 3.0), 2)
        y_pose = round(random.uniform(-3.0, 3.0), 2)

        patched_sdf = load_sdf_with_namespace(sdf_path, namespace)

        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            remappings=remappings,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time == 'true',
                'robot_description': robot_desc,
                'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
            }])
        actions.append(robot_state_publisher)

        # Spawn robot
        spawner_node = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=[
                '-name', f'{namespace}',
                '-string', patched_sdf,
                '-x', str(x_pose),
                '-y', str(y_pose),
                '-z', '0.01',
            ],
            output='screen',
        )
        actions.append(spawner_node)

        # bridge_template = os.path.join(swarm_dir, 'params', f'{TURTLEBOT3_MODEL}_bridge.yaml')
        bridge_template = os.path.join(swarm_dir, 'params', 'minimal_bridge.yaml')
        namespaced_bridge = create_namespaced_bridge_yaml(bridge_template, namespace)

        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={namespaced_bridge}'],
            output='screen',
        )
        actions.append(bridge_node)

        ## Not needed for minimal setup
        # # Add image bridge if model has camera
        # image_bridge = None
        # if TURTLEBOT3_MODEL != 'burger':
        #     image_bridge = Node(
        #         package='ros_gz_image',
        #         executable='image_bridge',
        #         namespace=namespace,
        #         arguments=['/' + namespace + '/camera/image_raw'],
        #         output='screen',
        #     )
        # actions.append(image_bridge) if image_bridge else None

    # In a multi-robot setup using Gazebo Sim (Harmonic or later), each robot typically
    # requires a separate ROS-Gazebo bridge to relay topics such as sensor data, odometry,
    # and control commands between Gazebo and ROS 2.
    # However, some topics like `/clock` are *global* and should be published only once
    # to avoid conflicts or duplication. If multiple bridges publish `/clock`, it may lead
    # to inconsistent simulation time behavior across nodes or unnecessary topic traffic.
    # Therefore, the `/clock` topic is handled separately:
    # - It is excluded from the per-robot bridge configuration files (YAMLs).
    # - A dedicated, single bridge instance is launched to publish `/clock` from Gazebo to ROS 2.
    # This ensures consistent simulation time across the entire ROS 2 system while supporting
    # multiple robot instances with their own bridges.

    # Global clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        )
    actions.append(clock_bridge)

    return actions

def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_num_bots = DeclareLaunchArgument(
        'num_bots',
        default_value='3',
        description='Number of TurtleBot3 robots to spawn'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_num_bots,
        OpaqueFunction(function=launch_setup)
    ])