"""Launch file for the ASV energy-aware planning simulation."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the ASV simulation."""
    package_dir = get_package_share_directory('asv_energy_aware_planning')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='maritime_regular.world')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='maritime_regular.world',
        description='Gazebo world file to use for simulation'
    )
    
    # Configure Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': [os.path.join(package_dir, 'simulation', 'worlds', world_file)],
            'gui': 'true',
            'verbose': 'true',
        }.items()
    )
    
    # Launch ASV model spawner
    asv_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'asv',
            '-file', os.path.join(package_dir, 'simulation', 'models', 'asv', 'model.sdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )
    
    # Launch TF broadcaster for ASV
    asv_tf_broadcaster = Node(
        package='asv_energy_aware_planning',
        executable='asv_tf_broadcaster',
        name='asv_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch environmental plugins
    hydrodynamics_plugin = Node(
        package='asv_energy_aware_planning',
        executable='hydrodynamics_plugin',
        name='hydrodynamics_plugin',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    current_plugin = Node(
        package='asv_energy_aware_planning',
        executable='current_plugin',
        name='current_plugin',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    wave_plugin = Node(
        package='asv_energy_aware_planning',
        executable='wave_plugin',
        name='wave_plugin',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch visualization
    rviz_config = os.path.join(package_dir, 'config', 'asv_simulation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_world_file,
        gazebo_launch,
        asv_spawner,
        asv_tf_broadcaster,
        hydrodynamics_plugin,
        current_plugin,
        wave_plugin,
        rviz_node
    ]) 