from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def _mode_is(*values: str) -> IfCondition:
    return IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("mode"), "' in ", repr(list(values))]
        )
    )


def generate_launch_description() -> LaunchDescription:
    config = str(
        Path(get_package_share_directory("x2_grasp"))
        / "config"
        / "unified_grasp.yaml"
    )
    mode = LaunchConfiguration("mode")
    execute = ParameterValue(LaunchConfiguration("execute"), value_type=bool)

    return LaunchDescription(
        [
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
                parameters=[config, {"source": mode, "execute": execute}],
            ),
        ]
    )
