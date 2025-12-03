from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('ucus_bringup')
    kontrol_dir = get_package_share_directory('ucus_kontrol')
    
    # Launch argument for parameter file selection
    param_file_arg = DeclareLaunchArgument(
        'controller_params',
        default_value='moderate',  # Options: conservative, moderate, tuned, or default
        description='Controller parameter preset: conservative, moderate, tuned, or default'
    )
    
    # Determine parameter file path
    param_file_name = LaunchConfiguration('controller_params')
    
    # Default parameter file paths
    default_params = os.path.join(kontrol_dir, 'params', 'controller_params.yaml')
    conservative_params = os.path.join(kontrol_dir, 'params', 'controller_params_conservative.yaml')
    moderate_params = os.path.join(kontrol_dir, 'params', 'controller_params_moderate.yaml')
    tuned_params = os.path.join(kontrol_dir, 'params', 'controller_params_tuned.yaml')
    
    # Get safety monitor parameters
    mode_manager_dir = get_package_share_directory('ucus_mode_manager')
    safety_params = os.path.join(mode_manager_dir, 'params', 'safety_params.yaml')
    
    return LaunchDescription([
        param_file_arg,
        
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
            # Note: Parameter file selection logic would need to be implemented
            # For now, using moderate preset as default
            parameters=[moderate_params],
            output='screen'
        ),
    ])
