import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Workspace paths
    ws_dir = os.path.expanduser('~/repos/LandingGearController')
    urdf_path = os.path.join(ws_dir, 'models', 'urdf', 'landing_rig.urdf')
    bridge_config = os.path.join(ws_dir, 'config', 'bridge_config.yaml')

    # 1. Start Gazebo Sim (unpaused with -r flag)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # 2. Spawn the UAV Model (delayed 2.0s to allow Gazebo to initialize)
    spawn_model = TimerAction(
        period=2.0,
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

    # 3. ROS-Gazebo Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # 4. Control Domain: Attitude Estimator Node
    attitude_estimator = Node(
        package='control_domain',
        executable='attitude_estimator',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 5. Control Domain: Stabilator PID Controller Node
    stabilator_controller = Node(
        package='control_domain',
        executable='stabilator_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 6. Control Domain: Landing Gear Actuator Controller Node
    landing_gear_controller = Node(
        package='control_domain',
        executable='landing_gear_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 7. Sensing Domain: Sensor Processor & Feature Extraction Node
    sensor_processor = Node(
        package='sensing_domain',
        executable='sensor_processor',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        spawn_model,
        bridge,
        attitude_estimator,
        stabilator_controller,
        landing_gear_controller,
        sensor_processor
    ])