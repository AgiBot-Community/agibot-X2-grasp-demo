from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _mode_is(*values: str) -> IfCondition:
    return IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("mode"), "' in ", repr(list(values))]
        )
    )


def _command_backend_is(*values: str) -> IfCondition:
    return IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("command_backend"),
                "' in ",
                repr(list(values)),
            ]
        )
    )


def generate_launch_description() -> LaunchDescription:
    config = str(
        Path(get_package_share_directory("x2_grasp"))
        / "config"
        / "unified_grasp.yaml"
    )
    mode = LaunchConfiguration("mode")
    arm_side = LaunchConfiguration("arm_side")
    ik_backend = LaunchConfiguration("ik_backend")
    command_backend = LaunchConfiguration("command_backend")
    execute = ParameterValue(LaunchConfiguration("execute"), value_type=bool)

    actions = [
        DeclareLaunchArgument(
            "mode",
            default_value="grounding",
            description="apriltag, grounding, or automatic arbitration",
            choices=["apriltag", "grounding", "auto"],
        ),
        DeclareLaunchArgument(
            "execute",
            default_value="false",
            description="false only solves IK; true moves the real robot",
        ),
        DeclareLaunchArgument(
            "arm_side",
            default_value="auto",
            description="automatically select an arm or force left/right",
            choices=["auto", "left", "right"],
        ),
        DeclareLaunchArgument(
            "ik_backend",
            default_value="auto",
            description="prefer native IK, require native, or force Python",
            choices=["auto", "native", "python"],
        ),
        DeclareLaunchArgument(
            "command_backend",
            default_value="auto",
            description="prefer C++ command node, require it, or publish in Python",
            choices=["auto", "native", "python"],
        ),
        Node(
            package="x2_grasp",
            executable="rgbd_localizer",
            name="x2_rgbd_localizer",
            output="screen",
            parameters=[config],
            condition=_mode_is("grounding", "auto"),
        ),
        Node(
            package="x2_grasp",
            executable="grounding_node",
            name="x2_grounding",
            output="screen",
            parameters=[config],
            condition=_mode_is("grounding", "auto"),
        ),
        Node(
            package="x2_grasp",
            executable="apriltag_detector",
            name="x2_apriltag_detector",
            output="screen",
            parameters=[config],
            condition=_mode_is("apriltag", "auto"),
        ),
        Node(
            package="x2_grasp",
            executable="grasp_node",
            name="x2_grasp_node",
            output="screen",
            parameters=[
                config,
                {
                    "source": mode,
                    "arm_side": arm_side,
                    "ik_backend": ik_backend,
                    "command_backend": command_backend,
                    "execute": execute,
                },
            ],
        ),
    ]
    native_publisher = (
        Path(get_package_prefix("x2_grasp"))
        / "lib"
        / "x2_grasp"
        / "x2_command_publisher"
    )
    if native_publisher.is_file():
        actions.append(
            Node(
                package="x2_grasp",
                executable="x2_command_publisher",
                name="x2_command_publisher",
                output="screen",
                parameters=[config],
                condition=_command_backend_is("auto", "native"),
            )
        )
    return LaunchDescription(actions)
