from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.actions import Node
from os.path import join
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_node',
        output='screen',
        parameters = [{'use_sim_time': use_sim_time}],
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/world/empty/model/stella_arm/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',],
        remappings=[('/world/empty/model/stella_arm/joint_state', '/joint_states')]
    )
    
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', join(get_package_share_directory( 'arm_description' ), 'urdf', 'arm.urdf')])}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
	bridge_node,
	#robot_state_publisher_node
    ])
