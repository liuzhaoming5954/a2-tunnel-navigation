"""Start the mock platform and mock-only mission node as one bounded group."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from a2_bringup.configuration import parse_mock_configuration


def _setup(context):
    config = parse_mock_configuration(
        profile=LaunchConfiguration("profile").perform(context),
        robot_id=LaunchConfiguration("robot_id").perform(context),
        database_path=LaunchConfiguration("database_path").perform(context),
        mock_plan_path=LaunchConfiguration("mock_plan_path").perform(context),
        platform_drop_after_s=LaunchConfiguration("platform_drop_after_s").perform(context),
    )
    platform = Node(
        package="a2_platform", executable="readonly_node", output="screen",
        parameters=[{
            "source": "mock",
            "robot_id": config.robot_id,
            "publish_period_ms": 100,
            "stale_timeout_ms": 500,
            "mock_drop_after_s": config.platform_drop_after_s,
        }],
    )
    mission = Node(
        package="a2_mission", executable="mission_node", output="screen",
        parameters=[{
            "profile": "mock",
            "robot_id": config.robot_id,
            "database_path": config.database_path,
            "plan_path": config.mock_plan_path,
            "authorization_ttl_s": 30.0,
            "platform_timeout_s": 1.0,
        }],
    )
    # A surviving half-system must not look like a complete running demo.
    # Any process exit shuts the group down; there is no automatic respawn.
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=platform,
            on_exit=[EmitEvent(event=Shutdown(reason="mock platform process exited"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=mission,
            on_exit=[EmitEvent(event=Shutdown(reason="mock mission process exited"))],
        )),
        platform,
        mission,
    ]


def generate_launch_description():
    share = get_package_share_directory("a2_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="mock"),
        DeclareLaunchArgument("robot_id", default_value="mock-a2"),
        DeclareLaunchArgument(
            "database_path",
            default_value=os.path.join(os.getcwd(), "data", "mock", "mission.sqlite"),
            description="Absolute persistent mock database path; parent is created by the mission node",
        ),
        DeclareLaunchArgument(
            "mock_plan_path", default_value=os.path.join(share, "config", "mock-plan.json"),
        ),
        DeclareLaunchArgument("platform_drop_after_s", default_value="-1.0"),
        OpaqueFunction(function=_setup),
    ])
