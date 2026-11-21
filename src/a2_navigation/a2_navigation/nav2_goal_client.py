"""将单点导航请求交给已有的 Nav2 Action 服务，控制器仅处理明确关联的 Action 回执。"""

import math
import threading

from .single_goal import nav2_uuid


class Nav2GoalBackend:
    def __init__(self, action_name="/navigate_to_pose", use_sim_time=False):
        from nav2_msgs.action import NavigateToPose
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions

        # 独立上下文与线程，机载只读遥测和 HTTP 无需等待导航回调，也不会一同 shutdown。
        self._action = NavigateToPose
        self._context = Context()
        self._node = self._executor = self._client = self._thread = None
        self._records = {}
        self._lock = threading.Lock()
        self._done = threading.Event()
        rclpy.init(args=[], context=self._context, signal_handler_options=SignalHandlerOptions.NO)
        try:
            self._node = rclpy.create_node(
                "a2_pc2_single_goal", context=self._context, start_parameter_services=False,
                parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)])
            self._client = ActionClient(self._node, NavigateToPose, action_name)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._spin, name="a2-nav2-action", daemon=True)
            self._thread.start()
        except BaseException:
            self.close()
            raise

    def _spin(self):
        try:
            self._executor.spin()
        finally:
            self._done.set()

    def available(self):
        return not self._done.is_set() and self._context.ok() and self._client.server_is_ready()

    def send(self, point, notify):
        from unique_identifier_msgs.msg import UUID
        message = self._action.Goal()
        message.pose.header.frame_id = point["frame_id"]
        message.pose.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.pose.position.x = point["x_m"]
        message.pose.pose.position.y = point["y_m"]
        # 地图中的水平航向绕 +Z 旋转；角度输入单位为弧度。
        yaw = point["yaw_rad"]
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        ident = point["goal_id"]
        # 显式填写 ROS UUID，机载收据与动作状态消息中的 goal_id 可直接核对。
        ros_uuid = UUID(uuid=list(nav2_uuid(point).bytes))
        future = self._client.send_goal_async(
            message, goal_uuid=ros_uuid, feedback_callback=lambda m: notify("feedback", m.feedback.distance_remaining))
        future.add_done_callback(lambda f: self._receive_handle(ident, f, notify))

    def _receive_handle(self, ident, future, notify):
        try:
            handle = future.result()
            if not handle.accepted:
                notify("rejected")
                return
            # 在通知上层前登记已受理句柄，支持「先取消、稍后收到受理」的竞态。
            with self._lock:
                self._records[ident] = (handle, notify)
            notify("accepted")
            handle.get_result_async().add_done_callback(lambda f: self._receive_result(ident, f, notify))
        except Exception:
            notify("uncertain", "nav2_response_unknown")

    def _receive_result(self, ident, future, notify):
        from action_msgs.msg import GoalStatus
        try:
            result = future.result()
            event = {GoalStatus.STATUS_SUCCEEDED: "succeeded",
                     GoalStatus.STATUS_CANCELED: "canceled",
                     GoalStatus.STATUS_ABORTED: "aborted"}.get(result.status)
            if event is None:
                notify("uncertain", "nav2_outcome_unknown")
                return
            with self._lock:
                self._records.pop(ident, None)
            notify(event)
        except Exception:
            notify("uncertain", "nav2_result_unknown")

    def cancel(self, ident):
        with self._lock:
            handle, notify = self._records[ident]
        future = handle.cancel_goal_async()

        def cancel_response(done):
            try:
                # 服务受理取消并不表示目标已取消；终态始终由 get_result 返回。
                response = done.result()
                if not any(tuple(info.goal_id.uuid) == tuple(handle.goal_id.uuid)
                           for info in response.goals_canceling):
                    notify("uncertain", "cancel_not_confirmed")
            except Exception:
                notify("uncertain", "cancel_outcome_unknown")

        future.add_done_callback(cancel_response)

    def close(self):
        if self._executor:
            self._executor.shutdown()
        if self._thread and self._thread.is_alive():
            self._thread.join()
        if self._client:
            self._client.destroy()
        if self._node:
            self._node.destroy_node()
        if self._context.ok():
            self._context.shutdown()
