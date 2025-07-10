import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # 新增导入
from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():
#     declared_arguments = []

#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "description_package",
#             default_value="bbot_description",
#             description="Description package with robot URDF/xacro files. Usually the argument \
#         is not set, it enables use of a custom description.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "description_file",
#             default_value="bbot.urdf.xacro",
#             description="URDF/XACRO description file with the robot.",
#         )
#     )

#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "gui",
#             default_value="true",
#             description="Start Rviz2 and Joint State Publisher gui automatically \
#         with this launch file.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "control_package",
#             default_value="my_robot_bringup",
#             description="Description package with robot URDF/xacro files. Usually the argument \
#         is not set, it enables use of a custom description.",
#         )
#     )





#     description_package = LaunchConfiguration("description_package")
#     description_file = LaunchConfiguration("description_file")
#     control_package = LaunchConfiguration("control_package")

#     gui = LaunchConfiguration("gui")

#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution(
#                 [FindPackageShare(description_package), "urdf", description_file]
#             ),
#         ]
#     )
#     # robot_description = {"robot_description": robot_description_content}
#     robot_description = {
#     "robot_description": ParameterValue(robot_description_content, value_type=str)
# }

#         # 定义控制器配置文件路径（提前定义，避免作用域问题）
#     controller_config = PathJoinSubstitution(
#         [FindPackageShare(control_package), "config", "my_robot_controllers.yaml"]
#     )

#     joint_state_publisher_node = Node(
#         package="joint_state_publisher_gui",
#         executable="joint_state_publisher_gui",
#         condition=IfCondition(gui),
#     )
#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="both",
#         parameters=[robot_description],
#     )
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         condition=IfCondition(gui),
#     )
#     controller_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         output="both",
#         parameters=[controller_config,],
#     )
#     broadcaster_node = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster"],
#         output="both",
#     )
#     diff_drive_spawner_node = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["diff_drive_controller"],
#         output="both",
#     )
#     nodes = [
#         joint_state_publisher_node,
#         robot_state_publisher_node,
#         rviz_node,
#         controller_node,
#         diff_drive_spawner_node,
#         broadcaster_node,
#     ]

#     # Launch!
#     return LaunchDescription(declared_arguments + nodes)
