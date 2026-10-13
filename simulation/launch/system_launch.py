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
    flight_carrier_script = os.path.join(ws_dir, 'scripts', 'flight_carrier.py')

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

    # Spawn root at Z = 0.0 so flight_z_joint directly controls absolute physical AGL
    spawn_z = PythonExpression([
        "'0.0' if '", world_choice, "'.strip() == 'approach' else '0.7'"
    ])

    gz_sim_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        condition=IfCondition(gui_choice)
    )

    gz_sim_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen',
        condition=UnlessCondition(gui_choice)
    )

    spawn_model = TimerAction(
        period=3.5,
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

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

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

    flight_carrier_node = ExecuteProcess(
        cmd=['python3', flight_carrier_script, '--ros-args', '-p', 'use_sim_time:=true'],
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
        flight_arg,
        gz_sim_gui,
        gz_sim_server,
        spawn_model,
        bridge,
        attitude_estimator,
        stabilator_controller,
        landing_gear_controller,
        flight_carrier_node,
        flight_supervisor,
        sensor_processor,
        terrain_classifier
    ])