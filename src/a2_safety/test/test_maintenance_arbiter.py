from dataclasses import replace
from threading import Event
import unittest

from maintenance_arbiter import (
    Direction, Feedback, Inputs, Limits, LocalGrant, MaintenanceArbiter, Owner,
    ReceiverStopEvidence, Sample,
)


class MockOutput:
    receiver_id = "simulated-receiver"

    def __init__(self):
        self.commands = []
        self.stops = []
        self.stop_seen = Event()
        self.accept_velocity = True

    def velocity(self, request):
        self.commands.append(request)
        return self.accept_velocity

    def stop(self, session, reason):
        self.stops.append((session, reason))
        self.stop_seen.set()
        return True


class MaintenanceArbiterTest(unittest.TestCase):
    def setUp(self):
        self.now = 1.
        self.output = MockOutput()
        self.inputs = Inputs()
        self.arbiter = MaintenanceArbiter(Limits(.2, .4, 3., .3, .5, .01),
                                          lambda: self.inputs, self.output, clock=lambda: self.now)
        self.addCleanup(self.arbiter.close)
        self.now += .01
        self.directions = frozenset(Direction)
        self.inputs = Inputs(Sample(Owner.RELEASED, self.now), Sample(self.directions, self.now),
                             Sample(Feedback.STOPPED, self.now), self.grant(1),
                             ReceiverStopEvidence(self.output.receiver_id, "simulated-report", self.now + 10.))

    def grant(self, serial):
        return LocalGrant(serial, self.now, self.now + 5., frozenset(Direction))

    def acquire(self):
        status, reply = self.arbiter.acquire({})
        self.assertEqual(status, 200, reply)
        return reply["session_id"]

    def send(self, session, sequence=1, linear=.1, angular=0.):
        return self.arbiter.velocity({"session_id": session, "sequence": sequence,
                                      "linear_mps": linear, "angular_radps": angular})

    def test_receiver_evidence_owner_and_local_grant_required(self):
        complete = self.inputs
        for evidence in (None, replace(complete.receiver_stop, receiver_id="other"),
                         replace(complete.receiver_stop, expires_at=self.now)):
            self.inputs = replace(complete, receiver_stop=evidence)
            state = self.arbiter.state()
            self.assertFalse(state["available"])
            self.assertFalse(state["receiver_stop_verified"])
            self.assertEqual(self.arbiter.acquire({})[1]["reason"], "receiver_stop_unverified")
        self.inputs = replace(complete, owner=Sample(Owner.OWNED, self.now))
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "control_owned")
        self.inputs = replace(complete, grant=replace(complete.grant, issued_at=1.))
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "local_authorization_required")
        self.assertEqual(self.output.commands, [])

    def test_exclusive_lease_limits_sequence_and_stop_feedback(self):
        session = self.acquire()
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "session_busy")
        self.assertEqual(self.send(session, linear=.3)[0], 400)
        self.assertEqual(self.send(session, angular=.1)[0], 400)
        self.assertEqual(self.send(session, sequence=2)[0], 200)
        self.assertEqual(self.output.commands[-1].grant_serial, 1)
        self.assertEqual(self.send(session)[1]["reason"], "stale_sequence")
        self.assertEqual(self.arbiter.stop({"session_id": session})[0], 200)
        self.assertEqual(self.arbiter.state()["reason"], "stop_unconfirmed")
        self.assertEqual(self.arbiter.stop({"session_id": session})[0], 200)
        self.assertEqual(self.output.stops, [(session, "operator_stop")])
        self.assertEqual(self.send(session, sequence=3)[1]["reason"], "invalid_session")
        self.now += .02
        self.inputs = replace(self.inputs, feedback=Sample(Feedback.STOPPED, self.now))
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "local_authorization_required")
        self.inputs = replace(self.inputs, grant=self.grant(2))
        self.assertNotEqual(self.acquire(), session)
        self.assertEqual(self.arbiter.stop({"session_id": session})[1]["reason"], "invalid_session")
        self.assertTrue(self.arbiter.state()["armed"])

    def test_background_watchdog_rejects_late_request_after_deadman(self):
        session = self.acquire()
        self.assertEqual(self.send(session)[0], 200)
        self.now += .31
        self.assertTrue(self.output.stop_seen.wait(.5), "watchdog must run without requests")
        self.assertEqual(self.output.stops, [(session, "deadman_expired")])
        self.assertEqual(self.send(session, sequence=2)[1]["reason"], "invalid_session")

    def test_directional_guard_revokes_and_does_not_forward_blocked_command(self):
        session = self.acquire()
        self.inputs = replace(self.inputs, guard=Sample(frozenset({Direction.LEFT}), self.now))
        self.assertEqual(self.send(session)[1]["reason"], "obstacle")
        self.assertEqual(self.output.commands, [])
        self.assertEqual(self.output.stops, [(session, "obstacle")])

    def test_control_takeover_revokes_without_sending_chassis_commands(self):
        session = self.acquire()
        self.assertEqual(self.send(session)[0], 200)
        self.now += .02
        self.inputs = replace(self.inputs, owner=Sample(Owner.OWNED, self.now))
        self.arbiter.tick()
        self.assertFalse(self.arbiter.state()["armed"])
        self.assertEqual(self.output.stops, [])
        self.assertEqual(self.arbiter.stop({"session_id": session})[1]["reason"], "stop_unconfirmed")
        self.assertEqual(len(self.output.commands), 1)

    def test_stale_guard_revokes_even_with_fresh_chassis_feedback(self):
        # Start with a still-fresh guard sample that expires before the first command.
        self.inputs = replace(self.inputs, guard=Sample(self.directions, self.now - .45))
        session = self.acquire()
        self.assertEqual(self.send(session)[0], 200)
        self.now += .06
        self.arbiter.tick()
        self.assertEqual(self.output.stops, [(session, "guard_stale")])

    def test_output_rejection_requests_stop_and_latches_until_feedback(self):
        session = self.acquire()
        self.output.accept_velocity = False
        self.assertEqual(self.send(session)[1]["reason"], "output_rejected")
        self.assertEqual(self.output.stops, [(session, "output_rejected")])
        self.assertEqual(self.arbiter.state()["reason"], "stop_unconfirmed")


if __name__ == "__main__":
    unittest.main()
