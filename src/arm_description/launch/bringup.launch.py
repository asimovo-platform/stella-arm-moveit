import launch
import launch_ros
import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share_dir = launch_ros.substitutions.FindPackageShare(package='arm_description').find('arm_description') + '/'
    model_path = os.path.join(pkg_share_dir,'urdf', 'arm.urdf')
    
    robot_name = 'stella_arm'
    world_name = 'empty'
    ign_model = '/model/' + robot_name
    ign_world = '/world/' + world_name + ign_model
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch.substitutions.Command(['xacro ', model_path])}]
    ) 
                        
    static_tf = launch_ros.actions.Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'link1'])
                     
                     
    spawn_stella_arm = launch_ros.actions.Node(
        package='ros_ign_gazebo',
        executable='create',
        name = 'robot_spawner',
        arguments = ['-name', 'stella_arm',
                    '-x', '0',
                    '-y', '0.5',
                    '-z', '0',
                    '-R', '0.0',
                    '-P', '0.0',
                    '-Y', '0.0',
                    '-file', model_path],
        output='screen'
    )
    
    rviz_config_file = os.path.join(pkg_share_dir, 'config/config.rviz')
    rviz = launch_ros.actions.Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[{'robot_description': launch.substitutions.Command(['xacro ', model_path])}]
                     )
    
    gazebo_launch_node = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')
                ]),
                launch_arguments = {'ign_args': ['-r -v1 empty.sdf']}.items()
            )
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable(name = 'SDF_PATH', value = [launch.substitutions.EnvironmentVariable('SDF_PATH', default_value=''), pkg_share_dir]),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        gazebo_launch_node,
        spawn_stella_arm,
        robot_state_publisher_node,
        static_tf,
        #rviz
    ])
    
    
    
    
