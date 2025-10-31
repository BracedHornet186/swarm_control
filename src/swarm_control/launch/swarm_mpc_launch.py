#!/usr/bin/env python3
"""
Launch file for MPC-enhanced swarm control system

This launch file starts:
1. The original swarm control system (kinematic nodes, graph observer, reference node)
2. MPC controllers for each robot
3. Gazebo simulation with multiple robots

Author: Assistant
"""

import os
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
    use_mpc = LaunchConfiguration('use_mpc', default='true').perform(context)

    # Load model and URDF
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    model_dir = f'turtlebot3_{TURTLEBOT3_MODEL}'
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    frame_prefix = LaunchConfiguration('frame_prefix', default='')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_path = os.path.join(swarm_dir, 'urdf', urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Spawn each bot
    for i in range(num_bots):
        namespace = f'bot{i + 1}'
        x_pose = round(random.uniform(-3.0, 3.0), 2)
        y_pose = round(random.uniform(-3.0, 3.0), 2)

        sdf_path = os.path.join(swarm_dir, 'models', model_dir, 'model.sdf')
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

        # Bridge configuration
        bridge_template = os.path.join(swarm_dir, 'params', f'{TURTLEBOT3_MODEL}_bridge.yaml')
        namespaced_bridge = create_namespaced_bridge_yaml(bridge_template, namespace)

        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={namespaced_bridge}'],
            output='screen',
        )
        actions.append(bridge_node)

        # Add image bridge if model has camera
        image_bridge = None
        if TURTLEBOT3_MODEL != 'waffle':
            image_bridge = Node(
                package='ros_gz_image',
                executable='image_bridge',
                namespace=namespace,
                arguments=['/' + namespace + '/camera/image_raw'],
                output='screen',
            )
        actions.append(image_bridge) if image_bridge else None

    # Global clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )
    actions.append(clock_bridge)

    # Launch swarm control nodes
    # Graph Observer
    graph_observer = Node(
        package='swarm_control',
        executable='graph_observer.py',
        name='graph_observer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time == 'true',
            'num_bots': num_bots,
            'delta_radius': 3.0
        }]
    )
    actions.append(graph_observer)

    # Reference Node
    reference_node = Node(
        package='swarm_control',
        executable='reference_node.py',
        name='reference_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time == 'true',
            'num_bots': num_bots
        }]
    )
    actions.append(reference_node)

    # Kinematic Nodes (one per robot)
    for i in range(num_bots):
        bot_id = f'bot{i + 1}'
        
        kinematic_node = Node(
            package='swarm_control',
            executable='kinematic_node.py',
            name=f'kinematic_node_{bot_id}',
            namespace=bot_id,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time == 'true',
                'bot_id': bot_id,
                'num_bots': num_bots,
                'delta_radius': 3.0,
                'role': 'agent'
            }]
        )
        actions.append(kinematic_node)

    # MPC Controllers (one per robot) - only if use_mpc is true
    if use_mpc == 'true':
        for i in range(num_bots):
            bot_id = f'bot{i + 1}'
            
            mpc_controller = Node(
                package='swarm_control',
                executable='mpc_controller_scipy.py',
                name=f'mpc_controller_{bot_id}',
                namespace=bot_id,
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time == 'true',
                    'bot_id': bot_id,
                    'num_bots': num_bots,
                    'mpc_horizon': 5,
                    'mpc_dt': 0.1,
                    'max_linear_vel': 0.5,
                    'max_angular_vel': 2.0,
                    'max_linear_acc': 1.0,
                    'max_angular_acc': 3.0,
                    'control_frequency': 10.0,
                    'tracking_weight': 10.0,
                    'control_weight': 1.0
                }]
            )
            actions.append(mpc_controller)

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
    
    declare_use_mpc = DeclareLaunchArgument(
        'use_mpc',
        default_value='true',
        description='Enable MPC controllers for robots'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_num_bots,
        declare_use_mpc,
        OpaqueFunction(function=launch_setup)
    ])


