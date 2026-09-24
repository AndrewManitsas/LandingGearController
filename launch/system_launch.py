import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    ws_dir = os.path.expanduser('~/repos/LandingGearController')
    urdf_path = os.path.join(ws_dir, 'models', 'urdf', 'landing_rig.urdf')
    bridge_config = os.path.join(ws_dir, 'config', 'bridge_config.yaml')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='alpha',
        description='Simulation testbed world: alpha, beta, or gamma'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI (true) or run headless server (false)'
    )

    world_choice = LaunchConfiguration('world')
    gui_choice = LaunchConfiguration('gui')

    world_file = PythonExpression([
        f"'{ws_dir}/worlds/testbed_' + '", world_choice, "'.strip() + '.sdf'"
    ])

    # 1a. Gazebo Sim with GUI (gui:=true)
    gz_sim_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        condition=IfCondition(gui_choice)
    )

    # 1b. Gazebo Sim Headless Server (gui:=false)
    gz_sim_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen',
        condition=UnlessCondition(gui_choice)
    )

    # 2. Spawn UAV Model after Gazebo initialization (2.5s)
    spawn_model = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', urdf_path,
                    '-name', 'landing_rig',
                    '-z', '1.0'
                ],
                output='screen'
            )
        ]
    )

    # 3. ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # 4. Domain Nodes
    attitude_estimator = Node(
        package='control_domain',
        executable='attitude_estimator',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    stabilator_controller = Node(
        package='control_domain',
        executable='stabilator_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    landing_gear_controller = Node(
        package='control_domain',
        executable='landing_gear_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    sensor_processor = Node(
        package='sensing_domain',
        executable='sensor_processor',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    terrain_classifier = Node(
        package='intelligence_domain',
        executable='terrain_classifier',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        gz_sim_gui,
        gz_sim_server,
        spawn_model,
        bridge,
        attitude_estimator,
        stabilator_controller,
        landing_gear_controller,
        sensor_processor,
        terrain_classifier
    ])