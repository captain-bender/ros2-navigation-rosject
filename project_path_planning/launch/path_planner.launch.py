import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    recovery_yaml = os.path.join(
        get_package_share_directory('project_path_planning'),
        'config',
        'recovery.yaml'
    )

    bt_navigator_yaml = os.path.join(
        get_package_share_directory('project_path_planning'),
        'config',
        'bt_navigator.yaml'
    )

    controller_yaml = os.path.join(
        get_package_share_directory('project_path_planning'),
        'config',
        'controller.yaml'
    )

    nav2_yaml = os.path.join(
        get_package_share_directory('project_path_planning'),
        'config',
        'planner_server.yaml'
    )

    # Its main task is to perform reactive path planning 
    # from the current position to a few meters ahead 
    # (until the range of the sensors). Then, it builds 
    # a trajectory to avoid the dynamic obstacles 
    # (which do not appear on the map, but can be 
    # detected with the help of sensor data), while 
    # trying to follow the global plan. It is also in 
    # charge of generating the wheel's commands, to 
    # make the robot follow the trajectory.
    controller_node = Node(
        name='controller_server',
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[controller_yaml]
    )

    # Its task is to find a path for the robot from 
    # Point A to Point B. It computes the path while 
    # avoiding the known obstacles included on the map.
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_yaml]
    )

    # the node responsible for executing the recovery behaviors
    behaviour_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        output='screen',
        parameters=[recovery_yaml]
    )

    # node that coordinates the node that calls the path 
    # planner node asking for a path, and then calls the 
    # controller to move the robot along it
    navigation_coordinator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml]
    )

    nav2_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper_2',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': [
                    'planner_server', 
                    'controller_server',
                    'recoveries_server',
                    'bt_navigator'
                    ]}]
        )
    
    return LaunchDescription([
        behaviour_server_node,
        navigation_coordinator_node,
        controller_node,
        planner_node,
        nav2_lifecycle_manager_node
    ])