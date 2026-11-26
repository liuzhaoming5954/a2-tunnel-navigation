"""在开发机回环 ROS 域验证 Humble 单点 launch 的重映射规则，不启动任何导航或底盘组件。"""

import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


@unittest.skipUnless(os.environ.get("A2_SINGLE_GOAL_ROS_TEST") == "1", "需显式开启回环 ROS 测试")
class Nav2RemapsTest(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / "pc2_single_goal_nav2.launch.py"
        spec = importlib.util.spec_from_file_location("single_goal_launch", path)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.module.generate_launch_description()

    def test_map_server_is_optional_and_uses_same_map_and_frame(self):
        from launch import LaunchContext
        module = self.module
        with tempfile.TemporaryDirectory() as work:
            grid = Path(work) / "map.yaml"
            grid.touch()
            context = LaunchContext()
            context.launch_configurations.update(params_file=str(grid), map_yaml="", use_sim_time="false")
            # 此处检查 launch 生成的节点和参数，不依赖本机安装导航服务器。
            with patch.object(module, "get_package_share_directory", return_value=work), \
                    patch.object(module, "Node", wraps=module.Node) as node:
                module.include_navigation(context)
                node.assert_not_called()
                context.launch_configurations["map_yaml"] = str(grid)
                module.include_navigation(context)
                self.assertEqual(node.call_count, 2)
                server, manager = [call.kwargs for call in node.call_args_list]
                self.assertEqual((server["package"], server["executable"]), ("nav2_map_server", "map_server"))
                self.assertEqual(server["parameters"][0], dict(yaml_filename=str(grid), topic_name="/map",
                                                               frame_id="map", use_sim_time=False))
                self.assertEqual(manager["parameters"][0]["node_names"], [server["name"]])
                self.assertTrue(manager["parameters"][0]["autostart"])
                context.launch_configurations["map_yaml"] = str(grid.with_name("missing.yaml"))
                with self.assertRaises(FileNotFoundError):
                    module.include_navigation(context)

    def test_node_topics_share_external_inputs_and_isolate_velocity(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.signals import SignalHandlerOptions

        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"), "1")
        module = self.module

        ns = "/" + module.NAMESPACE
        context = Context()
        rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
        self.addCleanup(context.shutdown)
        def make_node(name, extra=(), suffix=""):
            # 按 Humble launch_ros 顺序，先放 group 全局规则，再附带上游节点的局部规则。
            ros_args = ["--ros-args"]
            upstream_tf = (("/tf", "tf"), ("/tf_static", "tf_static"))
            for src, dst in (*module.REMAPS, *upstream_tf, *extra):
                ros_args.extend(("-r", f"{src}:={dst}"))
            node = rclpy.create_node(name, namespace=ns+suffix, context=context, cli_args=ros_args)
            self.addCleanup(node.destroy_node)
            return node

        controller = make_node("controller_server", (("cmd_vel", "cmd_vel_nav"),))
        behavior = make_node("behavior_server")
        smoother = make_node("velocity_smoother", (("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")))
        costmap = make_node("global_costmap", suffix="/global_costmap")
        for node in (controller, behavior, smoother, costmap):
            for name in ("tf", "tf_static", "map", "odom"):
                self.assertEqual(node.resolve_topic_name(name), "/" + name)
                self.assertEqual(node.resolve_topic_name("/" + name), "/" + name)
            self.assertEqual(node.resolve_topic_name("/cmd_vel"), module.CANDIDATE)
        self.assertEqual(costmap.resolve_topic_name(ns + "/map"), "/map")
        self.assertEqual(controller.resolve_topic_name("cmd_vel"), ns+"/cmd_vel_nav")
        self.assertEqual(behavior.resolve_topic_name("cmd_vel"), ns+"/cmd_vel_nav")
        self.assertEqual(smoother.resolve_topic_name("cmd_vel"), ns+"/cmd_vel_nav")
        self.assertEqual(smoother.resolve_topic_name("cmd_vel_smoothed"), module.CANDIDATE)


if __name__ == "__main__":
    unittest.main()
