from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('ucus_bringup')
    kontrol_dir = get_package_share_directory('ucus_kontrol')
    mode_manager_dir = get_package_share_directory('ucus_mode_manager')
    
    # Launch arguments
    param_file_arg = DeclareLaunchArgument(
        'controller_params',
        default_value='moderate',
        description='Controller parameter preset: conservative, moderate, tuned, or default'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    use_gcs_arg = DeclareLaunchArgument(
        'use_gcs',
        default_value='false',
        description='Launch GCS monitor interface'
    )
    
    use_logging_arg = DeclareLaunchArgument(
        'use_logging',
        default_value='false',
        description='Enable telemetry logging'
    )
    
    # Parameter files
    moderate_params = os.path.join(kontrol_dir, 'params', 'controller_params_moderate.yaml')
    safety_params = os.path.join(mode_manager_dir, 'params', 'safety_params.yaml')
    rviz_config = os.path.join(bringup_dir, 'config', 'vtol_visualization.rviz')
    
    # Main system nodes
    nodes = [
        # Sensor nodes
        Node(
            package='ucus_sensorleri',
            executable='fake_imu_node',
            name='fake_imu',
            output='screen'
        ),
        Node(
            package='ucus_sensorleri',
            executable='fake_gps_node',
            name='fake_gps',
            output='screen'
        ),
        Node(
            package='ucus_sensorleri',
            executable='fake_barometer_node',
            name='fake_barometer',
            output='screen'
        ),
        Node(
            package='ucus_sensorleri',
            executable='fake_magnetometer_node',
            name='fake_magnetometer',
            output='screen'
        ),
        # State estimation
        Node(
            package='ucus_durum_tahmin',
            executable='ekf_node',
            name='ekf',
            parameters=[os.path.join(bringup_dir, 'params', 'ekf_params.yaml')],
            output='screen'
        ),
        Node(
            package='ucus_mode_manager',
            executable='safety_monitor_node',
            name='safety_monitor',
            parameters=[safety_params],
            output='screen'
        ),
        Node(
            package='ucus_mode_manager',
            executable='mode_manager_node',
            name='mode_mgr',
            output='screen'
        ),
        Node(
            package='ucus_kontrol',
            executable='vtol_controller_node',
            name='vtol_ctrl',
            parameters=[moderate_params],
            output='screen'
        ),
        # Visualization helper nodes
        Node(
            package='ucus_bringup',
            executable='odom_tf_broadcaster.py',
            name='odom_tf_broadcaster',
            output='screen'
        ),
        Node(
            package='ucus_bringup',
            executable='vtol_model_publisher.py',
            name='vtol_model_publisher',
            output='screen'
        ),
        Node(
            package='ucus_bringup',
            executable='flight_path_publisher.py',
            name='flight_path_publisher',
            output='screen'
        ),
        Node(
            package='ucus_bringup',
            executable='flight_mode_marker_publisher.py',
            name='flight_mode_marker_publisher',
            output='screen'
        ),
    ]
    
    # Optional nodes
    use_rviz = LaunchConfiguration('use_rviz')
    use_gcs = LaunchConfiguration('use_gcs')
    use_logging = LaunchConfiguration('use_logging')
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    # GCS Monitor
    gcs_node = Node(
        package='ucus_bringup',
        executable='vtol_gcs_monitor.py',
        name='vtol_gcs_monitor',
        condition=IfCondition(use_gcs),
        output='screen'
    )
    
    # Telemetry Logger
    logger_node = Node(
        package='ucus_bringup',
        executable='telemetry_logger_node.py',
        name='telemetry_logger',
        condition=IfCondition(use_logging),
        output='screen'
    )
    
    # Add optional nodes
    nodes.append(rviz_node)
    nodes.append(gcs_node)
    nodes.append(logger_node)
    
    return LaunchDescription([
        param_file_arg,
        use_rviz_arg,
        use_gcs_arg,
        use_logging_arg,
        *nodes
    ])

