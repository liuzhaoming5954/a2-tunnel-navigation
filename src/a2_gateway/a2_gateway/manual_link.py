"""上位机通过固定的回环维护通道，向 PC2 转发机身遥控请求。"""

from hmac import compare_digest
from http.client import HTTPException
import ipaddress
import math
import os
import re
import stat

from .motion import (_action_request, _command_receipt, _control_action_fields, _finite_number,
                     _session_id, _velocity_request)


# 与受管接收器分别限速。网关拒绝越界或混合轴请求，不能通过滑块或手写 JSON 提速。
# linear_mps 是后退上限，也是旧版 PC2 缺少 forward_mps 时的双向上限。
LINEAR_LIMIT = 0.20
FORWARD_LIMIT = 0.20
ANGULAR_LIMIT = math.radians(10)
KEY_PATTERN = re.compile(r"[A-Za-z0-9_-]{32,128}")
BROWSER_COMMANDS = {
    "/api/control/session": "acquire",
    "/api/control/velocity": "velocity",
    "/api/control/stop": "stop",
    "/api/control/action/start": "action_start",
    "/api/control/action/release": "action_release",
}
# 机间端点与原模拟器分开，错误转发到模拟器也不会误领会话或送出速度。
PC2_COMMANDS = {path.replace("/control/", "/manual/"): method for path, method in BROWSER_COMMANDS.items()}
PROTOCOL = "a2-manual-v1"
STOP_COMMANDS = frozenset(("stop", "action_release"))
# 复位只通过独立维护通道转发，不加入速度和会话命令表。
PC2_STOP_RESET_PATH = "/api/manual/stop-reset"
BROWSER_STOP_RESET_PATH = "/api/control/stop-reset"
_STOP_RESET_REASON = re.compile(r"[a-z][a-z0-9_]{0,95}")


def valid_stop_reset_confirmation(payload):
    return isinstance(payload, dict) and set(payload) == {"confirmed"} and payload["confirmed"] is True


def valid_stop_reset_reason(reason):
    return isinstance(reason, str) and _STOP_RESET_REASON.fullmatch(reason) is not None


def loopback(host):
    if host.lower().rstrip(".") == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def read_key(path):
    """仅从部署用户或管理员的私有普通文件读取站机凭据，凭据不经 URL 和网页。"""
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, "rb") as source:
        info = os.fstat(fd)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid not in (0, os.geteuid())
                or info.st_mode & 0o077):
            raise ValueError("遥控密钥文件须为当前账户或管理员的私有普通文件")
        key = source.read(131).rstrip(b"\r\n").decode("ascii")
        if info.st_size > 130 or not KEY_PATTERN.fullmatch(key):
            raise ValueError("遥控密钥须为 32 至 128 个 URL 安全的字母、数字、下划线或连字符")
        return key


def authenticated(headers, key):
    fields = headers.get_all("Authorization", [])
    return (len(fields) == 1 and fields[0].isascii()
            and compare_digest(fields[0], "Bearer " + key))


def unavailable(reason, source=None):
    return {"available": False, "armed": False, "source": source, "reason": reason,
            "limits": {"linear_mps": 0., "angular_radps": 0.}}


def control_state(control):
    """仅透出页面使用的状态；缺失来源、非法限速或协议混淆均关闭遥控。"""
    if not isinstance(control, dict):
        return unavailable("manual_control_disabled")
    source = control.get("source")
    if source not in ("mock", "unitree"):
        # 锁存或守护失效时机载网关无需接触底盘套接字。只透传这些独立判定的禁用原因，
        # 不猜测一个来源，不允许未知接收器宣称可用或签发会话。
        reason = control.get("reason")
        if (source is None and control.get("available") is False and control.get("armed") is False
                and reason in ("manual_auth_failed", "stop_latched", "stop_guard_not_ready",
                               "manual_receiver_source_mismatch")):
            return unavailable(reason)
        return unavailable("motion_arbiter_unavailable")
    available, armed = control.get("available"), control.get("armed")
    reason = control.get("reason")
    if type(available) is not bool or type(armed) is not bool or not isinstance(reason, str):
        return unavailable("manual_invalid_response", source)
    if not available:
        return unavailable(reason[:160], source)
    limits = control.get("limits")
    if not isinstance(limits, dict):
        return unavailable("manual_invalid_limits", source)
    caps = {"linear_mps": LINEAR_LIMIT, "angular_radps": ANGULAR_LIMIT}
    if "forward_mps" in limits:
        caps["forward_mps"] = FORWARD_LIMIT
    if not all(_finite_number(limits.get(k)) and 0 < limits[k] <= cap for k, cap in caps.items()):
        return unavailable("manual_invalid_limits", source)
    try:
        action = _control_action_fields(control)
    except ValueError:
        return unavailable("manual_invalid_response", source)
    return {"available": True, "armed": armed, "source": source, "reason": reason[:160],
            "limits": {k: limits[k] for k in caps}, **action}


def valid_command(operation, payload):
    if not isinstance(payload, dict):
        return False
    if operation == "acquire":
        return not payload
    if operation == "stop":
        return set(payload) == {"session_id"} and _session_id(payload["session_id"])
    if operation in ("action_start", "action_release"):
        return _action_request(operation, payload)
    if operation != "velocity" or not _velocity_request(payload):
        return False
    vx, wz = payload["linear_mps"], payload["angular_radps"]
    return -LINEAR_LIMIT <= vx <= FORWARD_LIMIT and abs(wz) <= ANGULAR_LIMIT and (vx == 0 or wz == 0)


def forward_command(request, path, payload):
    """请求只发一次。响应丢失时交给接收器的独立超时停车，不能补发旧速度。"""
    operation = PC2_COMMANDS[path]
    if not valid_command(operation, payload):
        return 400, {"ok": False, "reason": "invalid_manual_command"}
    try:
        status, reply = request("POST", path, payload)
        if 400 <= status < 600 and reply.get("ok") is False and isinstance(reply.get("reason"), str):
            return status, {"ok": False, "reason": reply["reason"][:160]}
        if status != 200 or reply.get("ok") is not True or reply.get("protocol") != PROTOCOL:
            raise ValueError("机载遥控回执无效")
        # 先校验 HTTP 传输标识，再用机载套接字共用的解析器校验回执正文。
        receipt = {key: value for key, value in reply.items() if key != "protocol"}
        return 200, _command_receipt(operation, payload, receipt)
    except (OSError, HTTPException):
        return 503, {"ok": False, "reason": "pc2_unavailable"}
    except (KeyError, TypeError, ValueError, RecursionError, OverflowError):
        return 502, {"ok": False, "reason": "pc2_invalid_response"}
