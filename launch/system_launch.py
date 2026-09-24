import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    ws_dir = os.path.expanduser('~/repos/LandingGearController')
    urdf_path = os.path.join(ws_dir, 'models', 'urdf', 'landing_rig.urdf')
    bridge_config = os.path.join(ws_dir, 'config', 'bridge_config.yaml')

    # 1. Enforce OGRE 2 across all Gazebo and sensor rendering systems
    set_render_engine = SetEnvironmentVariable(
        name='GZ_RENDERING_ENGINE',
        value='ogre2'
    )

    # 2. Launch Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='alpha',
        description='Simulation testbed world: alpha, beta, or gamma'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo visualization GUI in an isolated client process'
    )

    world_choice = LaunchConfiguration('world')
    gui_choice = LaunchConfiguration('gui')

    world_file = PythonExpression([
        f"'{ws_dir}/worlds/testbed_' + '", world_choice, "'.strip() + '.sdf'"
    ])

    # 3. Gazebo Physics & Sensor Server (-s = server only, -r = run immediately)
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_file],
        output='screen'
    )

    # 4. Decoupled Gazebo GUI Client (-g = GUI only)
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=IfCondition(gui_choice)
    )

    # 5. Model Spawner (waits 2.5s for gz_server to register entity services)
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

    # 6. ROS-Gazebo Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # 7. Domain Nodes
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

    return LaunchDescription([
        set_render_engine,
        world_arg,
        gui_arg,
        gz_server,
        gz_gui,
        spawn_model,
        bridge,
        attitude_estimator,
        stabilator_controller,
        landing_gear_controller,
        sensor_processor
    ])