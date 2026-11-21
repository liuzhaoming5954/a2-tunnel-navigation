"""用 PC2 的 Humble Nav2 启动单点导航。可加载机载栅格，外部提供定位、障碍观测和机身 TF。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


NAMESPACE = "a2_single_goal"
CANDIDATE = "/a2/navigation/cmd_vel_candidate"
# Humble navigation_launch.py 将 /tf 和 /tf_static 改为相对名。本组全局规则优先于
# 子 launch 的节点规则，显式保留外部 TF；costmap 自身还可能位于嵌套命名空间。
# costmap 默认地图名是“父命名空间/map”，所以同时涵盖绝对的带前缀地图名与相对名。
REMAPS = (
    ("/tf", "/tf"), ("/tf_static", "/tf_static"),
    ("tf", "/tf"), ("tf_static", "/tf_static"),
    (f"/{NAMESPACE}/map", "/map"), ("map", "/map"),
    (f"/{NAMESPACE}/odom", "/odom"), ("odom", "/odom"),
    # 上游控制器已发到 cmd_vel_nav，恢复行为也先经同一个平滑器。
    ("behavior_server:cmd_vel", "cmd_vel_nav"),
    ("cmd_vel_smoothed", CANDIDATE), ("/cmd_vel", CANDIDATE),
)


def include_navigation(context):
    # 不使用上游演示机器人的速限、外廓或雷达主题。实机 YAML 由部署时指定。
    params = Path(LaunchConfiguration("params_file").perform(context)).expanduser().resolve(strict=True)
    if not params.is_file():
        raise RuntimeError("请传入 PC2 已核对的 Nav2 参数文件")
    share = Path(get_package_share_directory("nav2_bringup"))
    source = PythonLaunchDescriptionSource(str(share / "launch" / "navigation_launch.py"))
    arguments = {"namespace": NAMESPACE, "params_file": str(params),
                 "use_sim_time": LaunchConfiguration("use_sim_time"),
                 "use_composition": "False", "use_respawn": "False", "autostart": "True"}
    map_nodes = []
    map_file = LaunchConfiguration("map_yaml").perform(context)
    if map_file:
        grid = Path(map_file).expanduser().resolve(strict=True)
        if not grid.is_file():
            raise RuntimeError("请传入与 PC2 导航网关配置相同的栅格 YAML")
        # navigation_launch 只启动导航；上游 localization_launch 还会启动 AMCL。
        # 这里按需独立启动原生地图服务器，保留 ODIN1 的定位和外部机身 TF。
        sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
        map_nodes = [
            Node(package="nav2_map_server", executable="map_server", name="map_server",
                 output="screen", parameters=[{"yaml_filename": str(grid), "topic_name": "/map",
                                                "frame_id": "map", "use_sim_time": sim_time}]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_manager_map", output="screen",
                 parameters=[{"node_names": ["map_server"], "autostart": True,
                              "use_sim_time": sim_time}]),
        ]
    return [GroupAction(actions=[
        PushRosNamespace(NAMESPACE),
        *(SetRemap(src=src, dst=dst) for src, dst in REMAPS),
        *map_nodes,
        IncludeLaunchDescription(source, launch_arguments=arguments.items()),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", description="已核对的 Humble Nav2 参数 YAML；按上游格式不套命名空间"),
        DeclareLaunchArgument("map_yaml", default_value="",
                              description="与 --goal-grid-yaml 相同的地图 YAML；已有外部 /map 时留空"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"],
                              description="仅在提供 /clock 的仿真环境中使用 true"),
        OpaqueFunction(function=include_navigation),
    ])
