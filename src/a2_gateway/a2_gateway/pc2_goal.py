"""把 PC2 的已加载地图、定位观测和机身状态接到单点导航动作。"""

import hashlib
import math
import struct

from a2_navigation.offline_planner import load_trinary_grid, world_to_cell
from a2_navigation.single_goal import SingleGoalController

from .odin1_pose import pc2_pose_snapshot


def bin_digest(path):
    """分块读取本轮配置的设备地图；大地图不整体载入内存。"""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        first = stream.read(1024 * 1024)
        if not first:
            raise ValueError("设备重定位地图为空")
        digest.update(first)
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class Pc2GoalPort:
    def __init__(self, telemetry, pose_file, map_sha256, grid_yaml, backend, journal_path):
        self.telemetry = telemetry
        self.pose_file = pose_file
        self.backend = backend
        self.map_sha256 = map_sha256
        self.grid = load_trinary_grid(grid_yaml)
        # RosTelemetry 用 0/1/101 编码未知/空闲/占据。把配置图一次转换成相同编码，
        # 后续只在订阅到新地图时逐格比较，同时校验几何，防止地图同名却内容不同。
        self.cells = self.grid.cells.translate(bytes((n + 1) % 256 for n in range(256)))
        self.grid_check = (None, False)
        self.controller = SingleGoalController(map_sha256, backend, self.readiness,
                                               journal_path=journal_path, check_target=self.check_target)

    def check_target(self, target):
        """拒绝地图外、未知或占据格，路径可达性及机器人外廓由 Nav2 的代价地图检查。"""
        grid = self.grid
        cell = world_to_cell((target["x_m"], target["y_m"]), grid.origin,
                             grid.resolution, grid.width, grid.height)
        if cell is None:
            return "goal_outside_map"
        x, y = cell
        return None if grid.cells[y * grid.width + x] == 0 else "goal_cell_not_free"

    def _map_matches(self, meta, cells):
        grid = self.grid
        z_rotation = meta["origin_yaw_rad"] - grid.origin[2]
        # OccupancyGrid 的分辨率是 float32，YAML 解析后是 float64。
        resolution = struct.unpack("!f", struct.pack("!f", grid.resolution))[0]
        return (meta["frame_id"] == "map" and meta["width"] == grid.width and meta["height"] == grid.height
                and meta["resolution_m"] == resolution and meta["origin_x_m"] == grid.origin[0]
                and meta["origin_y_m"] == grid.origin[1]
                and abs(math.remainder(z_rotation, 2 * math.pi)) < 1e-9 and cells == self.cells)

    def readiness(self):
        """状态过期、地图版本变化或机身位姿缺失时，停止派发；运行中触发撤销。"""
        pose = pc2_pose_snapshot(self.pose_file, self.map_sha256)
        if pose["status"] != "live":
            return "odin1_" + pose["status"]
        state = self.telemetry.snapshot()
        if not state["robot"]["online"]:
            return "platform_offline"
        meta, cells = self.telemetry.navigation_grid()
        if meta is None or state["map"] is None or state["map"]["id"] != meta["id"]:
            return "navigation_grid_unavailable"
        cached_id, matches = self.grid_check
        if cached_id != meta["id"]:
            matches = self._map_matches(meta, cells)
            self.grid_check = (meta["id"], matches)
        if not matches:
            return "navigation_grid_mismatch"
        if state["pose"] is None or state["pose"]["map_id"] != meta["id"]:
            return "body_pose_unavailable"
        return None

    def goal_status(self):
        status, body = self.controller.status()
        # 查询用于下午部署后的静止核对，不会向 Nav2 发送目标或申请底盘控制。
        reason = self.readiness()
        if reason is None and not self.backend.available():
            reason = "nav2_unavailable"
        return status, {**body, "readiness": {"ready": reason is None, "reason": reason}}

    def submit_goal(self, payload):
        return self.controller.submit(payload)

    def cancel_goal(self, payload):
        return self.controller.cancel(payload)

    def close(self):
        self.controller.close()
