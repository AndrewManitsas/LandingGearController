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
        default_value='approach',
        description='Simulation testbed world: alpha, beta, gamma, or approach'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI (true) or run headless server (false)'
    )

    flight_arg = DeclareLaunchArgument(
        'flight',
        default_value='true',
        description='Enable active forward flight dynamics and supervisor FSM'
    )

    world_choice = LaunchConfiguration('world')
    gui_choice = LaunchConfiguration('gui')
    flight_choice = LaunchConfiguration('flight')

    world_file = PythonExpression([
        f"'{ws_dir}/worlds/testbed_' + '", world_choice, "'.strip() + '.sdf'"
    ])

    world_name_str = PythonExpression([
        f"'testbed_' + '", world_choice, "'.strip()"
    ])

    spawn_z = PythonExpression([
        "'1.6' if '", world_choice, "'.strip() == 'approach' else '1.0'"
    ])

    # 1a. Gazebo Sim with GUI
    gz_sim_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        condition=IfCondition(gui_choice)
    )

    # 1b. Gazebo Sim Headless Server
    gz_sim_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen',
        condition=UnlessCondition(gui_choice)
    )

    # 2. Spawn UAV Rig (Waits 4.0s for Gazebo to fully advertise /world/<name>/create)
    spawn_model = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', urdf_path,
                    '-name', 'uav_landing_rig',
                    '-world', world_name_str,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', spawn_z
                ],
                output='screen'
            )
        ]
    )

    # 3. Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # 4. Estimation & Control Domain
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

    # 5. Dynamic Flight & Autoland Supervisor Nodes
    flight_dynamics_node = Node(
        package='control_domain',
        executable='flight_dynamics_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(flight_choice)
    )

    flight_supervisor = Node(
        package='control_domain',
        executable='flight_supervisor',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(flight_choice)
    )

    # 6. Sensing Domain
    sensor_processor = Node(
        package='sensing_domain',
        executable='sensor_processor',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 7. Intelligence Domain
    terrain_classifier = Node(
        package='intelligence_domain',
        executable='terrain_classifier',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        flight_arg,
        gz_sim_gui,
        gz_sim_server,
        spawn_model,
        bridge,
        attitude_estimator,
        stabilator_controller,
        landing_gear_controller,
        flight_dynamics_node,
        flight_supervisor,
        sensor_processor,
        terrain_classifier
    ])