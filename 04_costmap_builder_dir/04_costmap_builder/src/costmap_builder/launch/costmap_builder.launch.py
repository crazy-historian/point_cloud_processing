from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="costmap_builder",
                executable="costmap_builder",
                name="costmap_builder",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["-d", "/src/costmap_builder/rviz/default.rviz"],
            ),
        ]
    )
