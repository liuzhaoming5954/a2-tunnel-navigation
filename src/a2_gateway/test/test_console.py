import http.client
import json
from pathlib import Path
import socket
import tempfile
import threading
import time
import unittest

from a2_gateway.motion import LocalMotionClient
from a2_gateway.server import GatewayServer
from a2_gateway.simulator import DEADMAN_SECONDS, Simulator


class ConsoleGatewayTest(unittest.TestCase):
    def test_session_deadman_stops_motion_and_rejects_delayed_commands(self):
        simulator = Simulator()
        self.addCleanup(simulator.close)
        initial = simulator.snapshot()["pose"]
        status, session = simulator.acquire({})
        self.assertEqual(status, 200)
        command = {"session_id": session["session_id"], "sequence": 1,
                   "linear_mps": 0.2, "angular_radps": 0.0}
        self.assertEqual(simulator.velocity(command)[0], 200)
        self.assertEqual(simulator.velocity(command)[0], 409)
        time.sleep(DEADMAN_SECONDS + 0.08)
        stopped = simulator.snapshot()
        self.assertGreater(stopped["pose"]["x_m"], initial["x_m"] + 0.03)
        self.assertEqual(stopped["robot"]["linear_mps"], 0.0)
        self.assertFalse(stopped["control"]["armed"])
        self.assertEqual(stopped["control"]["reason"], "deadman_expired")
        self.assertEqual(simulator.velocity({**command, "sequence": 2})[0], 409)

    def test_http_map_and_state_agree_and_cross_origin_cannot_arm(self):
        simulator = Simulator()
        server = GatewayServer(("127.0.0.1", 0), simulator, mode="simulator", control=simulator)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()

        def cleanup():
            server.shutdown()
            server.server_close()
            worker.join(timeout=2)

        self.addCleanup(cleanup)
        port = server.server_address[1]

        def request(method, path, payload=None, origin=None):
            connection = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
            self.addCleanup(connection.close)
            headers = {"Content-Type": "application/json"} if payload is not None else {}
            if origin:
                headers["Origin"] = origin
            connection.request(method, path, json.dumps(payload) if payload is not None else None, headers)
            response = connection.getresponse()
            return response.status, json.loads(response.read())

        status, state = request("GET", "/api/state")
        self.assertEqual(status, 200)
        status, grid = request("GET", "/api/map")
        self.assertEqual(status, 200)
        self.assertEqual(grid["id"], state["pose"]["map_id"])
        self.assertEqual(len(grid["cells"]), grid["width"] * grid["height"])
        self.assertEqual(set(grid["cells"]), {-1, 0, 100})
        status, _ = request("POST", "/api/control/session", {}, "https://elsewhere.test")
        self.assertEqual(status, 403)

        origin = f"http://127.0.0.1:{port}"
        status, session = request("POST", "/api/control/session", {}, origin)
        self.assertEqual(status, 200)
        payload = {"session_id": session["session_id"], "sequence": 1, "linear_mps": 0.2, "angular_radps": 0.0}
        self.assertEqual(request("POST", "/api/control/velocity", payload, origin)[0], 200)
        self.assertEqual(request("POST", "/api/control/stop", {"session_id": payload["session_id"]}, origin)[0], 200)
        self.assertFalse(request("GET", "/api/state")[1]["control"]["armed"])
        self.assertEqual(request("POST", "/api/control/velocity", {**payload, "sequence": 2}, origin)[0], 409)

    def test_local_motion_service_must_confirm_receiver_stop_before_acquire(self):
        with tempfile.TemporaryDirectory(prefix="a2-motion-") as work:
            path = Path(work) / "arbiter.sock"
            client = LocalMotionClient(path)
            self.assertFalse(client.state()["available"])
            received = []
            session = "safe-lease-for-test"

            def response(request):
                if request["op"] == "status":
                    return {"ok": True, "control": {"available": True, "armed": False,
                            "receiver_stop_verified": len(received) > 1,
                            "limits": {"linear_mps": 0.2, "angular_radps": 0.4}}}
                if request["op"] == "acquire":
                    return {"ok": True, "session_id": session, "expires_in_ms": 1500}
                return {"ok": True}

            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
                listener.bind(str(path))
                listener.listen()
                listener.settimeout(2)

                def serve():
                    for _ in range(5):
                        connection, _ = listener.accept()
                        with connection, connection.makefile("rb") as request_stream:
                            request = json.loads(request_stream.readline())
                            received.append(request)
                            connection.sendall(json.dumps(response(request)).encode() + b"\n")

                worker = threading.Thread(target=serve, daemon=True)
                worker.start()
                self.assertEqual(client.acquire({})[1]["reason"], "receiver_stop_unverified")
                self.assertEqual(client.acquire({})[1]["session_id"], session)
                command = {"session_id": session, "sequence": 1, "linear_mps": 0.1, "angular_radps": 0.0}
                self.assertEqual(client.velocity({**command, "linear_mps": float("nan")})[0], 400)
                self.assertEqual(client.velocity(command)[0], 200)
                self.assertEqual(client.stop({"session_id": session})[0], 200)
                worker.join(timeout=3)
                self.assertFalse(worker.is_alive())

        self.assertEqual([item["op"] for item in received], ["status", "status", "acquire", "velocity", "stop"])
        self.assertTrue(all(item["schema_version"] == 1 for item in received))


if __name__ == "__main__":
    unittest.main()
