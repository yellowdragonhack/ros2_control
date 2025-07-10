# import os

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, RegisterEventHandler
# from launch.conditions import IfCondition
# from launch.event_handlers import OnProcessExit
# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

# from launch.substitutions import PathJoinSubstitution
# # from launch_ros.substitutions import FindPackageShare
# from launch_ros.parameter_descriptions import ParameterValue  # 新增导入
# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():

#     robot_description_path =get_package_share_directory("bbot_description")
#     robot_bringup_path = get_package_share_directory("my_robot_bringup")


#     urdf_path = os.path.join(robot_description_path, "urdf", "bbot.urdf.xacro")
#     robot_description = ParameterValue(Command(['xacro',  urdf_path]), value_type=str)
#     robot_controller_config = os.path.join(robot_bringup_path, "config", "my_robot_controllers.yaml")



#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[{"robot_description": robot_description}],
#         output="both",
#     )
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         # output="log",
#     )
#     controller_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         output="both",
#         parameters=[robot_controller_config],
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
#         robot_state_publisher_node,
#         rviz_node,
#         controller_node,
#         diff_drive_spawner_node,
#         broadcaster_node,
#     ]

#     # Launch!
#     return LaunchDescription(nodes)


import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration , FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 声明参数（可选，可直接使用默认包名）
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="bbot_description",
            description="包含机器人URDF的包"
        ),
        DeclareLaunchArgument(
            "control_package",
            default_value="my_robot_bringup",
            description="包含控制器配置的包"
        )
    ]

    # 获取参数值
    description_package = LaunchConfiguration("description_package")
    control_package = LaunchConfiguration("control_package")

    # 解析URDF/XACRO文件（使用ROS 2推荐的路径替代方案）
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "bbot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 控制器配置文件路径
    controller_config = PathJoinSubstitution(
        [FindPackageShare(control_package), "config", "my_robot_controllers.yaml"]
    )

    parameters=[controller_config],
    remappings=[
        ("~/robot_description", "/robot_description"),
    ],

    # 定义节点
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
    )

    controller_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controller_config],  # 传递两个参数
    )

    # 启动控制器
    broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="both",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="both",
    )

    nodes = [
        robot_state_publisher_node,
        rviz_node,
        controller_node,
        broadcaster_spawner,
        diff_drive_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
