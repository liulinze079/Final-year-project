"""Launch file for the ASV hierarchical planning system."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the hierarchical planning system."""
    package_dir = get_package_share_directory('asv_energy_aware_planning')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Load parameter files
    global_planner_params = os.path.join(package_dir, 'config', 'global_planner_params.yaml')
    local_planner_params = os.path.join(package_dir, 'config', 'local_planner_params.yaml')
    mpc_controller_params = os.path.join(package_dir, 'config', 'mpc_controller_params.yaml')
    hierarchy_params = os.path.join(package_dir, 'config', 'hierarchy_params.yaml')
    
    # Global planner node (1 Hz)
    global_planner_node = Node(
        package='asv_energy_aware_planning',
        executable='global_planner_node',
        name='global_planner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            global_planner_params
        ]
    )
    
    # Local planner node (5 Hz)
    local_planner_node = Node(
        package='asv_energy_aware_planning',
        executable='local_planner_node',
        name='local_planner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            local_planner_params
        ]
    )
    
    # MPC controller node (10 Hz)
    mpc_controller_node = Node(
        package='asv_energy_aware_planning',
        executable='mpc_controller_node',
        name='mpc_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            mpc_controller_params
        ]
    )
    
    # Planning coordinator node
    planning_coordinator_node = Node(
        package='asv_energy_aware_planning',
        executable='planning_coordinator_node',
        name='planning_coordinator',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            hierarchy_params
        ]
    )
    
    # Environment representation node
    environment_node = Node(
        package='asv_energy_aware_planning',
        executable='environment_node',
        name='environment_representation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Path visualization node
    visualization_node = Node(
        package='asv_energy_aware_planning',
        executable='path_visualizer_node',
        name='path_visualizer',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time,
        global_planner_node,
        local_planner_node,
        mpc_controller_node,
        planning_coordinator_node,
        environment_node,
        visualization_node
    ]) 