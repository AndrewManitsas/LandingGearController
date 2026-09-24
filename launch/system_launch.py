import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
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

    world_choice = LaunchConfiguration('world')
    world_file = PythonExpression([
        f"'{ws_dir}/worlds/testbed_' + '", world_choice, "'.strip() + '.sdf'"
    ])

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

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
        gz_sim,
        spawn_model,
        bridge,
        attitude_estimator,
        stabilator_controller,
        landing_gear_controller,
        sensor_processor,
        terrain_classifier
    ])