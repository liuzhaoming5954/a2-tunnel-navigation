"""开发机回环双端验证。只替换 Nav2 动作端，PC2 目标控制器、HTTP 和站端转发均走正常入口。"""

from http.client import HTTPConnection
import json
import math
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event, Thread
import time
import unittest
from unittest.mock import Mock, patch

from a2_gateway.server import GatewayServer
from a2_gateway.station import StationSource
from a2_navigation.single_goal import SingleGoalController

DIGEST = "c" * 64
PATH = "/api/navigation/goal"
TARGET = {"goal_id": "test-1", "frame_id": "map", "map_sha256": DIGEST,
          "x_m": 1.25, "y_m": -0.75, "yaw_rad": 0.3}


class ActionFixture:
    """测试主动送入各阶段回调，因此可以区分已受理、导航成功和收到取消结果。"""

    def __init__(self):
        self.sent = []
        self.canceled = []
        self.notify = None

    def available(self):
        return True

    def send(self, point, notify):
        self.sent.append(point)
        self.notify = notify
        notify("accepted")

    def cancel(self, goal_id):
        self.canceled.append(goal_id)

    def close(self):
        pass


class GoalEndpoint:
    """与 PC2 网关的三个方法对接，沿用生产控制器的幂等规则和锁。"""

    def __init__(self, controller):
        self.goal_status = controller.status
        self.submit_goal = controller.submit
        self.cancel_goal = controller.cancel
        self.close = controller.close


class HttpLinkCase(unittest.TestCase):
    def start_server(self, role, source, endpoint):
        server = GatewayServer(("127.0.0.1", 0), source, mode="robot_readonly",
                               role=role, single_goal=endpoint)
        worker = Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
        worker.start()
        self.addCleanup(worker.join, 2)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return server

    def request(self, method="GET", path=PATH, body=None, port=None):
        connection = HTTPConnection("127.0.0.1", port or self.web.server_port, timeout=2)
        try:
            connection.request(method, path, json.dumps(body) if body is not None else None,
                               {"Content-Type": "application/json"} if body is not None else {})
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()


class SingleGoalLinkTest(HttpLinkCase):
    def setUp(self):
        self.actions = ActionFixture()
        controller = SingleGoalController(DIGEST, self.actions)
        self.addCleanup(controller.close)
        self.endpoint = GoalEndpoint(controller)
        source = Mock(spec=["snapshot", "close"])
        source.snapshot.return_value = {"robot": {"id": "fixture", "online": True}}
        pc2 = self.start_server("pc2", source, self.endpoint)
        self.pc2_url = f"http://127.0.0.1:{pc2.server_port}"
        self.station = StationSource(self.pc2_url, goal_map_sha256=DIGEST)
        self.web = self.start_server("station", self.station, self.station)

    def test_submission_feedback_and_terminal_result_across_both_servers(self):
        self.assertEqual(self.request()[1]["goal"]["phase"], "idle")
        self.assertTrue(self.request(path="/api/state")[1]["single_goal_enabled"])
        status, receipt = self.request("POST", body=TARGET)
        self.assertLess(status, 300)
        goal = receipt["goal"]
        self.assertEqual(goal["phase"], "active")
        self.assertEqual({key: goal[key] for key in TARGET}, TARGET)
        self.actions.notify("feedback", 0.31)
        self.assertEqual(self.request()[1]["goal"]["distance_remaining_m"], 0.31)
        self.assertLess(self.request("POST", body=TARGET)[0], 300)
        self.assertEqual(self.request("POST", body={**TARGET, "x_m": 4.})[0], 409)
        self.assertEqual(self.request("POST", body={**TARGET, "goal_id": "different"})[0], 409)
        self.actions.notify("succeeded")
        self.assertEqual(self.request()[1]["goal"]["phase"], "succeeded")
        self.assertEqual(self.actions.sent, [TARGET])

    def test_cancel_only_current_goal_and_wait_for_action_result(self):
        self.request("POST", body=TARGET)
        self.assertEqual(self.request("POST", PATH + "/cancel", {"goal_id": "other"})[0], 409)
        self.assertFalse(self.actions.canceled)
        self.assertEqual(self.request("POST", PATH + "/cancel", {"goal_id": "test-1"})[1]["goal"]["phase"],
                         "canceling")
        self.assertEqual(self.actions.canceled, ["test-1"])
        self.actions.notify("canceled")
        self.assertEqual(self.request()[1]["goal"]["phase"], "canceled")

    def test_station_rejects_wrong_map_and_inconsistent_receipt(self):
        with patch.object(self.station, "_pc2_request", wraps=self.station._pc2_request) as upstream:
            self.assertEqual(self.request("POST", body={**TARGET, "map_sha256": "b"*64})[0], 409)
            self.assertEqual(self.request("POST", body={**TARGET, "frame_id": "odom"})[0], 400)
            upstream.assert_not_called()
        # 即使上游返回 HTTP 成功，也必须对应预期地图、同一请求标识和坐标。
        goal = self.request()[1]["goal"]
        foreign = {**goal, "map_sha256": "f" * 64}
        with patch.object(self.endpoint, "goal_status", return_value=(200, {"ok": True, "goal": foreign})):
            self.assertEqual(self.request(), (409, {"ok": False, "reason": "map_mismatch"}))
        for fields in ({"goal_id": "other"}, {"yaw_rad": 0.0}, {"map_sha256": "e"*64}):
            wrong = {**goal, **TARGET, "phase": "active", **fields}
            with self.subTest(fields=fields), patch.object(self.endpoint, "submit_goal",
                    return_value=(200, {"ok": True, "goal": wrong})):
                self.assertEqual(self.request("POST", body=TARGET)[0], 502)

    def test_lost_ack_can_be_queried_without_resending(self):
        # PC2 已成功调用动作端，站端读取回执后发生传输故障；查询应恢复同一目标。
        relay = self.station._pc2_request
        def lose_ack(method, path, payload):
            relay(method, path, payload)
            raise ConnectionResetError
        with patch.object(self.station, "_pc2_request", side_effect=lose_ack) as upstream:
            self.assertEqual(self.request("POST", body=TARGET), (503, {"ok": False, "reason": "pc2_unavailable"}))
            upstream.assert_called_once()
        self.assertEqual(self.request()[1]["goal"]["goal_id"], TARGET["goal_id"])
        self.assertLess(self.request("POST", body=TARGET)[0], 300)
        self.assertEqual(len(self.actions.sent), 1)
        self.actions.notify("succeeded")

    def test_default_station_and_pc2_have_no_goal_writes(self):
        for role in ("station", "pc2"):
            with self.subTest(role=role):
                readonly = self.start_server(role, Mock(spec=["close"]), None)
                self.assertEqual(self.request("POST", body=TARGET, port=readonly.server_port)[0], 405)
                self.assertEqual(self.request(port=readonly.server_port)[0], 404)
        self.assertFalse(self.actions.sent)


@unittest.skipUnless(os.environ.get("A2_RUN_LOCAL_NAV2_LINK") == "1", "按需启用开发机回环 ROS 2 验证")
class RosSingleGoalLinkTest(HttpLinkCase):
    def test_nav2_wire_goal_feedback_success_and_cancel(self):
        """使用隔离 ROS 域和唯一的假动作名，检查正常 HTTP 双跳直到 ROS 消息序列化的全链。"""
        import rclpy
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionServer, CancelResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.signals import SignalHandlerOptions

        from a2_gateway.odin1_pose import BOOT_ID_FILE, SOURCE
        from a2_gateway.pc2_goal import Pc2GoalPort, bin_digest
        from a2_navigation.nav2_goal_client import Nav2GoalBackend

        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"), "1")
        self.assertGreater(int(os.environ.get("ROS_DOMAIN_ID", 0)), 0)
        finish = Event()
        requests = []
        def execute(handle):
            # 在测试明确放行或收到 ROS 取消之前持续发布反馈；Web 查询的值须经过整条链路。
            pose = handle.request.pose
            requests.append((pose.header.frame_id, pose.pose.position.x, pose.pose.position.y,
                             pose.pose.orientation.z, pose.pose.orientation.w))
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    handle.canceled()
                    return NavigateToPose.Result()
                if finish.is_set():
                    handle.succeed()
                    return NavigateToPose.Result()
                feedback = NavigateToPose.Feedback()
                feedback.distance_remaining = 0.375
                handle.publish_feedback(feedback)
                time.sleep(0.02)
            handle.abort()
            return NavigateToPose.Result()

        ctx = Context()
        rclpy.init(context=ctx, args=[], signal_handler_options=SignalHandlerOptions.NO)
        self.addCleanup(ctx.try_shutdown)
        node = rclpy.create_node("a2_goal_wire_fixture", context=ctx)
        self.addCleanup(node.destroy_node)
        name = f"/a2_local_fixture_{os.getpid()}/navigate_to_pose"
        action = ActionServer(node, NavigateToPose, name, execute_callback=execute,
                              cancel_callback=lambda _: CancelResponse.ACCEPT,
                              callback_group=ReentrantCallbackGroup())
        self.addCleanup(action.destroy)
        executor = MultiThreadedExecutor(num_threads=3, context=ctx)
        executor.add_node(node)
        thread = Thread(target=executor.spin, daemon=True)
        thread.start()
        self.addCleanup(thread.join, 3)
        self.addCleanup(executor.shutdown)

        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        folder = Path(tmp.name)
        yaml = folder / "test-map.yaml"
        (folder / "grid.pgm").write_bytes(b"P5\n8 8\n255\n" + bytes([255])*64)
        yaml.write_text(json.dumps(dict(image="grid.pgm", resolution=1.0, origin=[-3., -3., 0.],
                                        occupied_thresh=0.65, free_thresh=0.25)))
        fixture_bin = folder / "fixture.bin"
        fixture_bin.write_bytes(b"synthetic-map-for-loopback-test")
        digest = bin_digest(fixture_bin)
        pose_file = folder / "pose.json"
        observation = dict(source=SOURCE, status="live", frame="map", child_frame="imu", map_sha256=digest,
                           boot_id=BOOT_ID_FILE.read_text().strip(), position_m=[0.,0.,0.], yaw_rad=0.)
        done = Event()
        def write_pose():
            # 按生产格式刷新一个同图、同一系统启动批次的样本，读者不会看见半写的 JSON。
            stamp = time.monotonic_ns()
            stage = pose_file.with_suffix(".tmp")
            stage.write_text(json.dumps(dict(observation, stamp_ns=stamp, received_monotonic_ns=stamp)))
            stage.replace(pose_file)

        def refresh_pose():
            while not done.wait(0.1):
                write_pose()
        write_pose()
        publisher = Thread(target=refresh_pose, daemon=True)
        publisher.start()
        self.addCleanup(publisher.join, 1)
        self.addCleanup(done.set)

        grid = dict(id="grid-1", frame_id="map", width=8, height=8, resolution_m=1.,
                    origin_x_m=-3., origin_y_m=-3., origin_yaw_rad=0.)
        state = Mock(spec=["snapshot", "navigation_grid", "close"])
        state.navigation_grid.return_value = grid, bytes([1])*64
        state.snapshot.return_value = {"robot":{"online":True}, "map":grid,
                                       "pose":{"map_id":"grid-1", "x_m":0., "y_m":0., "yaw_rad":0.}}
        backend = Nav2GoalBackend(action_name=name)
        pc2_nav = Pc2GoalPort(state, pose_file, digest, yaml, backend, folder / "journal.sqlite")
        pc2 = self.start_server("pc2", state, pc2_nav)
        station = StationSource(f"http://127.0.0.1:{pc2.server_port}", goal_map_sha256=digest)
        self.web = self.start_server("station", station, station)

        def wait_for(probe, description):
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                result = probe()
                if result:
                    return result
                time.sleep(0.025)
            self.fail(description)
        def status():
            code, msg = self.request()
            self.assertEqual(code, 200, msg)
            return msg["goal"]

        wait_for(backend.available, "未发现隔离域的测试动作服务")
        first = {**TARGET, "map_sha256":digest}
        for ident in ("wire-success", "wire-cancel"):
            finish.clear()
            point = {**first, "goal_id":ident}
            code, ack = self.request("POST", body=point)
            self.assertLess(code, 300, ack)
            self.assertEqual(ack["goal"]["goal_id"], ident)
            wait_for(lambda: status()["distance_remaining_m"] == 0.375, "未收到 ROS 动作反馈")
            if ident == "wire-success":
                finish.set()
                phase = "succeeded"
            else:
                code, ack = self.request("POST", PATH + "/cancel", {"goal_id":ident})
                self.assertLess(code, 300, ack)
                phase = "canceled"
            wait_for(lambda: status()["phase"] == phase, f"等待动作结果 {phase} 超时")
            self.assertEqual(status()["goal_id"], ident)
        self.assertEqual(len(requests), 2)
        for frame, x, y, qz, qw in requests:
            self.assertEqual((frame,x,y), ("map",TARGET["x_m"],TARGET["y_m"]))
            self.assertAlmostEqual(qz, math.sin(TARGET["yaw_rad"]/2))
            self.assertAlmostEqual(qw, math.cos(TARGET["yaw_rad"]/2))


if __name__ == "__main__":
    unittest.main()
