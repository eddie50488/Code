from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("omni_drive_arduino"), "rviz", "omnibot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("omni_drive_arduino"), "urdf", "omnibot.urdf.xacro"]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="joint_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="both",
    )

    robot_controller = PathJoinSubstitution(
        [
            FindPackageShare("omni_drive_arduino"),
            "config",
            "omni_controller_config.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controller],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "omnidirectional_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="both",
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        control_node,
        robot_controller_spawner,
    ]

    return LaunchDescription(nodes)
