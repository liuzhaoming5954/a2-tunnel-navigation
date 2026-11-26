"""开发机本地 ROS 动作冒烟。使用唯一动作名和假的 Nav2 服务，仅交换目标、反馈和结果。"""

import math
import os
import threading
import time
import unittest
from uuid import UUID, uuid4


@unittest.skipUnless(os.environ.get("A2_SINGLE_GOAL_ROS_TEST") == "1",
                     "需要显式启用开发机回环 ROS 测试")
class RosActionTests(unittest.TestCase):
    def test_goal_feedback_success_and_cancel(self):
        import rclpy
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionServer, CancelResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from a2_navigation.nav2_goal_client import Nav2GoalBackend
        from a2_navigation.single_goal import SingleGoalController

        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"), "1")
        context = Context()
        rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
        self.addCleanup(context.shutdown)
        node = rclpy.create_node("test_single_goal_server", context=context)
        self.addCleanup(node.destroy_node)
        received = []
        ros_ids = []
        release = threading.Event()
        name = "/a2_test_" + uuid4().hex + "/navigate_to_pose"

        def execute(handle):
            # 取消回调和执行回调可并行；停留在假服务中直到用例读到反馈或发出取消请求。
            received.append(handle.request.pose)
            ros_ids.append(str(UUID(bytes=bytes(handle.goal_id.uuid))))
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    handle.canceled()
                    break
                if release.is_set():
                    handle.succeed()
                    break
                msg = NavigateToPose.Feedback()
                msg.distance_remaining = 1.25
                handle.publish_feedback(msg)
                time.sleep(0.03)
            else:
                handle.abort()
            return NavigateToPose.Result()

        server = ActionServer(node, NavigateToPose, name, execute,
                              cancel_callback=lambda _: CancelResponse.ACCEPT,
                              callback_group=ReentrantCallbackGroup())
        self.addCleanup(server.destroy)
        executor = MultiThreadedExecutor(num_threads=2, context=context)
        executor.add_node(node)
        worker = threading.Thread(target=executor.spin, daemon=True)
        worker.start()
        self.addCleanup(worker.join, 3)
        self.addCleanup(executor.shutdown)
        client = Nav2GoalBackend(name)
        digest = "c" * 64
        controller = SingleGoalController(digest, client)
        self.addCleanup(controller.close)

        def wait_for(check):
            until = time.monotonic() + 5
            while time.monotonic() < until:
                if check():
                    return
                time.sleep(0.02)
            self.fail(f"未观察到预期 ROS 事件，当前状态 {controller.status()}")

        fields = dict(frame_id="map", map_sha256=digest, x_m=2., y_m=-1., yaw_rad=1.1)
        wait_for(client.available)
        controller.submit(dict(fields, goal_id="first"))
        wait_for(lambda: controller.status()[1]["goal"]["distance_remaining_m"] == 1.25)
        self.assertEqual(ros_ids, [controller.status()[1]["goal"]["nav2_goal_uuid"]])
        pose = received[0]
        self.assertEqual(pose.header.frame_id, "map")
        self.assertGreater(pose.header.stamp.sec, 0)
        self.assertEqual((pose.pose.position.x, pose.pose.position.y), (2., -1.))
        self.assertAlmostEqual(pose.pose.orientation.z, math.sin(.55))
        self.assertAlmostEqual(pose.pose.orientation.w, math.cos(.55))
        release.set()
        wait_for(lambda: controller.status()[1]["goal"]["phase"] == "succeeded")
        release.clear()
        controller.submit(dict(fields, goal_id="second"))
        wait_for(lambda: controller.status()[1]["goal"]["distance_remaining_m"] == 1.25)
        controller.cancel({"goal_id": "second"})
        wait_for(lambda: controller.status()[1]["goal"]["phase"] == "canceled")
        self.assertEqual(len(received), 2)


if __name__ == "__main__":
    unittest.main()
