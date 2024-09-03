import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro



import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)



def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # stella_arm =  IncludeLaunchDescription(
    #         AnyLaunchDescriptionSource([
    #             os.path.join(get_package_share_directory( 'arm_description' ), 'launch', 'bringup.launch.py')
    #         ]),
    # )
    #
    #
    # ros2_bridge =  IncludeLaunchDescription(
    #         AnyLaunchDescriptionSource([
    #             os.path.join(get_package_share_directory( 'ros2_bridge_stella_arm' ), 'launch', 'launch_bridge.launch.py')
    #         ]),
    # )
    #
	#
    # moveit2 = IncludeLaunchDescription(
    #         AnyLaunchDescriptionSource([
    #             os.path.join(get_package_share_directory( 'moveit_stella_arm' ), 'launch', 'demo.launch.py')
    #         ]),
    #
    # )
    # # moveit2 = IncludeLaunchDescription(
    # #     AnyLaunchDescriptionSource([
    # #         os.path.join(get_package_share_directory('new_moveit_config'), 'launch', 'demo.launch.py')
    # #     ]),
    # #
    # # )
    #
    # robot_description_config = xacro.process_file(
    #     os.path.join(
    #         get_package_share_directory("moveit_stella_arm"),
    #         "config",
    #         "arm.urdf.xacro",
    #     )
    # )
    # robot_description = {"robot_description": robot_description_config.toxml()}
    #
    # robot_description_semantic_config = load_file(
    #     "moveit_stella_arm", "config/arm.srdf"
    # )
    # robot_description_semantic = {
    #     "robot_description_semantic": robot_description_semantic_config
    # }
    #
    # kinematics_yaml = load_yaml(
    #     "moveit_stella_arm", "config/kinematics.yaml"
    # )
    # robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    #
    # joint_limits_yaml = load_yaml("moveit_stella_arm", "config/joint_limits.yaml")
    # joint_limits = {"joint_limits": joint_limits_yaml}
    #
    # ompl_planning_pipeline_config = {
    #     "move_group": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 0.1,
    #     }
    # }
    # ompl_planning_yaml = load_yaml(
    #     "moveit_stella_arm", "config/ompl_planning.yaml"
    # )
    # ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    #
    # # moveit_config = (
    # #     MoveItConfigsBuilder("stella_arm")
    # #     # .robot_description(
    # #     #     file_path="config/arm.urdf.xacro",
    # #     #     # mappings={
    # #     #     #     "ros2_control_hardware_type": LaunchConfiguration(
    # #     #     #         "ros2_control_hardware_type"
    # #     #     #     )
    # #     #     # },
    # #     # )
    # #     .robot_description_semantic(file_path="config/arm.srdf")
    # #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    # #     .to_moveit_configs()
    # # )
    #
    # # Trajectory Execution Functionality
    # moveit_simple_controllers_yaml = load_yaml(
    #     "moveit_stella_arm", "config/moveit_controllers.yaml"
    # )
    #
    # moveit_controllers = {
    #     "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    #     "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
    # }
    #
    # trajectory_execution = {
    #     "moveit_manage_controllers": True,
    #     "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    #     "trajectory_execution.allowed_goal_duration_margin": 0.5,
    #     "trajectory_execution.allowed_start_tolerance": 0.01,
    # }
    # planning_scene_monitor_parameters = {
    #     "publish_planning_scene": True,
    #     "publish_geometry_updates": True,
    #     "publish_state_updates": True,
    #     "publish_transforms_updates": True,
    # }
    #
    # # Start the actual move_group node/action server
    # move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     # parameters=[moveit_config.to_dict()],
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         kinematics_yaml,
    #         joint_limits,
    #         trajectory_execution,
    #         ompl_planning_pipeline_config,
    #         planning_scene_monitor_parameters,
    #         moveit_controllers
    #     ],
    #     arguments=["--ros-args", "--log-level", "info"],
    # )
    #
    # # RViz
    # rviz_base = os.path.join(get_package_share_directory("moveit_stella_arm"), "config")
    # rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_full_config],
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         ompl_planning_pipeline_config,
    #         robot_description_kinematics,
    #         joint_limits,
    #         moveit_controllers
    #     ]
    # )
    # #rviz_node = IncludeLaunchDescription(
    # #        AnyLaunchDescriptionSource([
    # #            os.path.join(get_package_share_directory( 'stella_arm_moveit_config' ), 'launch', 'moveit_rviz.launch.py')
    # #        ]),
    # #)
    #
    #
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[robot_description]
    # )
    #
    #
    # # ros2_control_node = Node(
    # #     package="controller_manager",
    # #     executable="ros2_control_node",
    # #     parameters=[robot_description,  os.path.join(get_package_share_directory("moveit_stella_arm"),"config","ros2_controllers.yaml")],
    # #     output={
    # #         "stdout": "screen",
    # #         "stderr": "screen",
    # #     },
    # # )
    #
    # # load_joint_state_broadcaster = ExecuteProcess(
    # #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    # #         'joint_state_broadcaster'],
    # #    output='screen'
    # # )
    # #
    # # load_arm_controller = ExecuteProcess(
    # #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    # #         'arm_controller'],
    # #    output='screen'
    # # )
    #
    #
    #
    # load_arm_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     output="log",
    #     arguments=['arm_controller', "--ros-args", "--log-level", 'info'],
    #     parameters=[{"use_sim_time": True}],
    # )
    # load_joint_state_broadcaster = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     output="log",
    #     arguments=['joint_state_broadcaster', "--ros-args", "--log-level", 'info'],
    #     parameters=[{"use_sim_time": True}],
    # )
    #
    #
    #
    # return LaunchDescription([
    #
	# SetParameter(name='use_sim_time', value=True),
	#
    #     stella_arm,
    #
    #     ros2_bridge,
    #
    #     # moveit2,
    #     rviz_node,
    #     move_group_node,
    #
    #     node_robot_state_publisher,
    #     load_joint_state_broadcaster,
    #     load_arm_controller
    #
    #
    # ])

    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    moveit_config_package_path = get_package_share_directory("stella_moveit_config")
    controller_package_path = get_package_share_directory("stella_arm_controllers")


    # stella_arm
    ld.add_action(IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                os.path.join(get_package_share_directory('arm_description'), 'launch', 'bringup.launch.py')
            ]),
    ))

    # ros2_bridge
    ld.add_action(IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros2_bridge_stella_arm'), 'launch', 'launch_bridge.launch.py')
            ]),
    ))

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_package_path, "launch", "move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_package_path, "launch", "moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(controller_package_path, "launch", "start_gripper_control.launch.py")
            ),
        )
    )

    return ld
