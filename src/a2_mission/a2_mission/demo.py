"""Run a synthetic mission against a temporary SQLite database; no robot I/O."""

import json
import tempfile
from pathlib import Path

from a2_storage import SQLiteJournal

from .controller import MissionController, PRECHECK_FIELDS


def run_demo():
    output = []

    def step(controller, request, command, **params):
        reply = controller.command(request, command, "demo-mission", **params)
        mission = reply["current"]["snapshot"]["mission"]
        output.append({"step": request, "receipt_code": reply["receipt"]["code"],
                       "receipt_state": reply["receipt"]["state"],
                       "current_state": mission["state"],
                       "completed_visits": mission["completed_visits"]})
        return reply["receipt"]

    with tempfile.TemporaryDirectory(prefix="a2-mission-demo-") as directory:
        database = Path(directory) / "mission.sqlite"
        with SQLiteJournal(database) as journal:
            controller = MissionController(journal, "demo-robot", "demo-boot", authorization_ttl_s=30)
            step(controller, "prepare", "prepare", bundle_id="synthetic-bundle",
                 route_id="synthetic-route", map_version="synthetic-map",
                 config_version="synthetic-config", bundle_verified=True)
            step(controller, "unauthorized-start", "start", authorization_id="missing")
            auth = step(controller, "authorize", "authorize", operator_id="mock-operator", local_confirmation=True)
            token = auth["authorization_id"]
            step(controller, "start", "start", authorization_id=token)
            step(controller, "precheck", "precheck", checks={field: True for field in PRECHECK_FIELDS})
            step(controller, "localized", "localized", valid=True)
            step(controller, "visit", "record_visit", visit_id="synthetic-visit-01")
        # Simulate a controller process restart on the same OS boot.
        with SQLiteJournal(database) as journal:
            controller = MissionController(journal, "demo-robot", "demo-boot", authorization_ttl_s=30)
            step(controller, "start", "start", authorization_id=token)
            assert controller.current()["snapshot"]["mission"]["state"] == "PAUSED_FAULT"
            step(controller, "clear", "clear_fault", operator_id="mock-operator",
                 local_confirmation=True, health_confirmed=True, stopped_confirmed=True)
            auth = step(controller, "authorize-again", "authorize", operator_id="mock-operator",
                        local_confirmation=True)
            step(controller, "resume", "resume", authorization_id=auth["authorization_id"],
                 health_confirmed=True, stopped_confirmed=True)
            step(controller, "cancel", "cancel", stopped_confirmed=True)
    return output


def main():
    print(json.dumps({"mode": "synthetic_no_hardware", "steps": run_demo()},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
