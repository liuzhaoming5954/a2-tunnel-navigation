from dataclasses import replace
from threading import Event
import unittest
from unittest.mock import patch

from maintenance_arbiter import (
    Direction, Feedback, HoldReleasedBeforeOutput, Inputs, Limits, LocalGrant, MaintenanceArbiter,
    OperatorHold, Owner, Sample,
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


class OperatorHoldArbiterTest(unittest.TestCase):
    def test_repeated_action_binds_hold_at_start_before_first_speed_packet(self):
        now = 20.
        output = MockOutput()
        inputs = Inputs()
        arbiter = MaintenanceArbiter(Limits(.20, .15, 30., .40, .20, .02), lambda: inputs, output,
                                     clock=lambda: now, idle_deadman=True, operator_supervised=True,
                                     require_operator_hold=True)
        self.addCleanup(arbiter.close)
        now += 1.
        grant = LocalGrant(1, now, None, frozenset({Direction.FORWARD}), repeated_actions=True)
        inputs = Inputs(owner=Sample(Owner.NEUTRAL, now), feedback=Sample(Feedback.STOPPED, now), grant=grant)
        session = arbiter.acquire({})[1]['session_id']
        start = {'session_id': session, 'sequence': 1, 'direction': 'forward'}
        self.assertEqual(arbiter.action_start(start)[1]['reason'], 'operator_hold_stale')
        inputs = replace(inputs, operator_hold=Sample(OperatorHold(True, 'first'), now))
        self.assertEqual(arbiter.action_start(start)[1]['expires_in_ms'], 30000)
        now += .05
        inputs = replace(inputs, operator_hold=Sample(OperatorHold(True, 'second'), now))
        arbiter.tick()
        self.assertEqual(output.stops, [(session, 'operator_hold_changed')])
        self.assertEqual(output.commands, [])


class ReusableHoldArbiterTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.output = MockOutput()
        self.inputs = Inputs()
        self.arbiter = MaintenanceArbiter(Limits(.20, .15, 30., .40, .20, .02),
                                          lambda: self.inputs, self.output, clock=lambda: self.now,
                                          idle_deadman=True, operator_supervised=True,
                                          require_hold_direction=True, allow_continuous_hold=True)
        self.addCleanup(self.arbiter.close)
        self.refresh(.125, epoch="initial-neutral")
        self.inputs = replace(self.inputs, grant=LocalGrant(
            1, self.now, None, frozenset(Direction), continuous_hold=True))
        self.assertIs(self.arbiter.state()["reusable_hold"], True)
        status, reply = self.arbiter.acquire({"reusable_hold": True})
        self.assertEqual(status, 200, reply)
        self.session = reply["session_id"]
        self.assertEqual(reply, {"ok": True, "session_id": self.session,
                                 "continuous_hold": True, "reusable_hold": True})

    def refresh(self, elapsed, *, epoch, direction=None, feedback=Feedback.STOPPED, owner=Owner.NEUTRAL):
        self.now += elapsed
        sample = OperatorHold(direction is not None, epoch, direction)
        self.inputs = replace(self.inputs, owner=Sample(owner, self.now),
                              feedback=Sample(feedback, self.now), operator_hold=Sample(sample, self.now))

    def start(self, sequence, direction=Direction.FORWARD):
        return self.arbiter.action_start({"session_id": self.session, "sequence": sequence,
                                          "direction": direction.value})

    def release(self, sequence, action_id):
        return self.arbiter.action_release({"session_id": self.session, "sequence": sequence,
                                            "action_id": action_id})

    def send(self, sequence, linear=0., angular=0., *, action_id=None):
        payload = {"session_id": self.session, "sequence": sequence, "linear_mps": linear, "angular_radps": angular}
        if action_id is not None:
            payload["action_id"] = action_id
        return self.arbiter.velocity(payload)

    def move_forward(self):
        self.refresh(.0625, epoch="first-press", direction=Direction.FORWARD)
        self.assertEqual(self.start(1), (200, {"ok": True, "action_id": 1}))
        self.assertEqual(self.send(2, .20, action_id=1)[0], 200)

    def test_independent_release_stops_then_allows_each_new_direction(self):
        self.assertEqual(self.arbiter.state()["action_state"], "ready")
        self.assertEqual(self.start(1)[1]["reason"], "operator_hold_released")
        self.move_forward()
        self.refresh(.0625, epoch="second-neutral", feedback=Feedback.MOVING)
        # 没有网页释放时，机载独立键盘已停车，旧速度包不能续期。
        self.assertEqual(self.send(3, .20, action_id=1)[1]["reason"], "action_expired")
        self.assertEqual(self.output.stops, [(self.session, "action_hold_released")])
        self.assertEqual(self.release(3, 1)[0], 200)
        self.assertEqual(self.send(4)[0], 200)
        self.assertEqual(self.start(5, Direction.LEFT)[1]["reason"], "action_stopping")
        self.refresh(.0625, epoch="second-neutral")
        self.assertEqual(self.arbiter.state()["action_state"], "ready")
        self.assertEqual(self.start(5, Direction.LEFT)[1]["reason"], "operator_hold_released")

        self.refresh(.0625, epoch="second-press", direction=Direction.LEFT)
        self.assertEqual(self.start(5, Direction.LEFT), (200, {"ok": True, "action_id": 5}))
        self.assertEqual(self.send(6, angular=.15, action_id=5)[0], 200)
        self.assertEqual([(c.session_id, c.linear_mps, c.angular_radps) for c in self.output.commands],
                         [(self.session, .20, 0.), (self.session, 0., .15)])

    def test_idle_waits_for_standstill_without_revoking_and_rejects_motion(self):
        self.refresh(.0625, epoch="initial-neutral", feedback=Feedback.MOVING)
        self.assertEqual(self.arbiter.state()["action_state"], "standstill_pending")
        self.assertTrue(self.arbiter.state()["armed"])
        self.assertEqual(self.send(1)[0], 200)

        self.refresh(.0625, epoch="before-standstill", direction=Direction.FORWARD,
                     feedback=Feedback.SETTLING)
        self.assertEqual(self.start(2)[1]["reason"], "standstill_required")
        self.assertEqual(self.output.commands, [])

        self.refresh(.0625, epoch="released")
        self.assertEqual(self.arbiter.state()["action_state"], "ready")
        self.refresh(.0625, epoch="new-press", direction=Direction.FORWARD)
        self.assertEqual(self.start(3)[0], 200)
        self.assertEqual(self.send(4, .20, action_id=3)[0], 200)
        self.assertTrue(self.output.commands[-1].require_standstill)
        self.assertEqual(self.send(5, .20, action_id=3)[0], 200)
        self.assertFalse(self.output.commands[-1].require_standstill)

    def test_fresh_settling_keeps_stopped_action_pending_but_lost_feedback_revokes(self):
        self.move_forward()
        self.refresh(.0625, epoch="second-neutral", feedback=Feedback.MOVING)
        self.assertEqual(self.release(3, 1)[0], 200)
        self.refresh(.0625, epoch="second-neutral", feedback=Feedback.SETTLING)
        self.assertEqual(self.send(4)[0], 200)
        self.assertEqual(self.arbiter.state()["action_state"], "stopping")
        self.assertEqual(self.start(5)[1]["reason"], "action_stopping")

        self.inputs = replace(self.inputs, feedback=None)
        state = self.arbiter.state()
        self.assertFalse(state["armed"])
        self.assertEqual(state["reason"], "feedback_stale")
        self.assertEqual(len(self.output.commands), 1)

    def test_browser_release_first_requires_observed_neutral_and_fences_old_action(self):
        self.move_forward()
        self.refresh(.0625, epoch="first-press", direction=Direction.FORWARD)
        self.assertEqual(self.release(3, 1)[0], 200)
        # 停车请求之前的停稳样本不能放行，之后仍要观察到实体键回中。
        self.assertEqual(self.arbiter.state()["action_state"], "stopping")
        self.refresh(.0625, epoch="first-press", direction=Direction.FORWARD)
        self.assertEqual(self.start(4)[1]["reason"], "action_release_required")
        self.assertEqual(self.send(4, .20, action_id=1)[1]["reason"], "action_expired")
        self.refresh(.0625, epoch="second-neutral")
        self.assertEqual(self.arbiter.state()["action_state"], "ready")

        # 即使重新按同一个方向，旧动作编号也不能下发速度。
        self.refresh(.0625, epoch="second-press", direction=Direction.FORWARD)
        self.assertEqual(self.start(4)[0], 200)
        self.assertEqual(self.send(5, .20, action_id=1)[1]["reason"], "invalid_action")
        self.assertEqual(self.send(5, .20, action_id=4)[0], 200)
        self.assertEqual(self.output.stops, [(self.session, "action_release")])
        self.assertEqual([command.hold_epoch for command in self.output.commands], ["first-press", "second-press"])

    def test_missing_browser_release_cannot_keep_session_alive(self):
        self.move_forward()
        self.refresh(.0625, epoch="second-neutral", feedback=Feedback.MOVING)
        self.assertEqual(self.send(3, .20, action_id=1)[1]["reason"], "action_expired")
        self.refresh(.0625, epoch="second-neutral")
        self.assertEqual(self.arbiter.state()["action_state"], "release_required")
        self.assertEqual(self.send(4)[1]["reason"], "action_release_required")
        self.refresh(.3125, epoch="second-neutral")
        self.assertEqual(self.arbiter.state()["reason"], "deadman_expired")
        self.refresh(.0625, epoch="second-press", direction=Direction.FORWARD)
        self.assertEqual(self.start(5)[1]["reason"], "invalid_session")
        self.assertEqual(self.send(6, .20, action_id=1)[1]["reason"], "invalid_session")
        self.assertEqual(len(self.output.commands), 1)

    def test_release_in_output_recheck_stops_without_revoking_when_confirmed(self):
        self.move_forward()

        def release_before_sdk(request):
            self.refresh(.0625, epoch="second-neutral", feedback=Feedback.MOVING)
            raise HoldReleasedBeforeOutput

        # 键盘恰好在仲裁通过和底盘输出复核之间松开，确认不发送这一包也不误撤权。
        with patch.object(self.output, "velocity", side_effect=release_before_sdk):
            self.assertEqual(self.send(3, .20, action_id=1)[1]["reason"], "action_expired")
        self.assertEqual(len(self.output.commands), 1)
        self.assertEqual(self.output.stops, [(self.session, "action_hold_released")])
        self.assertEqual(self.release(4, 1)[0], 200)
        self.refresh(.0625, epoch="second-neutral")
        self.assertEqual(self.arbiter.state()["action_state"], "ready")

    def test_failed_stop_output_blocks_next_action_until_stop_is_accepted(self):
        self.move_forward()
        self.refresh(.0625, epoch="second-neutral", feedback=Feedback.MOVING)
        with patch.object(self.output, "stop", return_value=False):
            self.assertEqual(self.release(3, 1)[0], 200)
            self.refresh(.0625, epoch="second-neutral")
            self.assertEqual(self.arbiter.state()["action_state"], "stopping")
        # 停车回执重试成功后，消化新一帧停稳反馈，再开放下一次按键。
        self.refresh(.0625, epoch="second-neutral")
        self.arbiter.tick()
        self.refresh(.0625, epoch="second-neutral")
        self.assertEqual(self.arbiter.state()["action_state"], "ready")
        self.assertEqual(len(self.output.stops), 1)

    def test_explicit_close_consumes_the_session_even_at_rest(self):
        self.assertEqual(self.send(1)[0], 200)
        self.assertEqual(self.arbiter.stop({"session_id": self.session})[0], 200)
        self.refresh(.0625, epoch="initial-neutral")
        self.assertFalse(self.arbiter.state()["armed"])
        self.assertEqual(self.output.stops, [(self.session, "operator_stop")])
        self.assertEqual(self.arbiter.acquire({"reusable_hold": True})[1]["reason"], "local_authorization_required")
        self.assertEqual(self.start(2)[1]["reason"], "invalid_session")
        self.assertEqual(self.output.commands, [])

    def test_keyboard_evidence_expiry_stops_despite_browser_sending(self):
        self.move_forward()
        first_hold = self.inputs.operator_hold
        self.refresh(.125, epoch="first-press", direction=Direction.FORWARD, feedback=Feedback.MOVING)
        self.inputs = replace(self.inputs, operator_hold=first_hold)
        self.assertEqual(self.send(3, .20, action_id=1)[0], 200)
        self.refresh(.125, epoch="first-press", direction=Direction.FORWARD, feedback=Feedback.MOVING)
        self.inputs = replace(self.inputs, operator_hold=first_hold)
        self.assertEqual(self.send(4, .20, action_id=1)[1]["reason"], "invalid_session")
        self.assertEqual(self.output.stops, [(self.session, "operator_hold_stale")])

    def test_new_physical_epoch_without_confirmed_release_revokes(self):
        self.move_forward()
        self.refresh(.0625, epoch="second-press", direction=Direction.BACKWARD, feedback=Feedback.MOVING)
        self.assertEqual(self.send(3, .20, action_id=1)[1]["reason"], "invalid_session")
        self.assertEqual(self.output.stops, [(self.session, "operator_hold_changed")])

    def test_held_action_has_no_30_second_cap_but_browser_deadman_still_applies(self):
        self.move_forward()
        started_at = self.now
        for sequence in range(3, 253):
            self.refresh(.125, epoch="first-press", direction=Direction.FORWARD, feedback=Feedback.MOVING)
            self.assertEqual(self.send(sequence, .20, action_id=1)[0], 200)
        self.assertGreater(self.now - started_at, 30.)
        self.assertEqual(self.arbiter.state()["action_state"], "active")
        self.assertEqual(self.output.stops, [])

        self.refresh(.4375, epoch="first-press", direction=Direction.FORWARD, feedback=Feedback.MOVING)
        self.assertEqual(self.send(253, .20, action_id=1)[1]["reason"], "invalid_session")
        self.assertEqual(self.output.stops, [(self.session, "deadman_expired")])


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
                             Sample(Feedback.STOPPED, self.now), self.grant(1))

    def grant(self, serial):
        return LocalGrant(serial, self.now, self.now + 5., frozenset(Direction))

    def acquire(self):
        status, reply = self.arbiter.acquire({})
        self.assertEqual(status, 200, reply)
        return reply["session_id"]

    def send(self, session, sequence=1, linear=.1, angular=0.):
        return self.arbiter.velocity({"session_id": session, "sequence": sequence,
                                      "linear_mps": linear, "angular_radps": angular})

    def test_owner_and_local_grant_required(self):
        complete = self.inputs
        for owner, reason in [(Owner.OWNED, "control_owned"), (Owner.UNKNOWN, "control_unknown")]:
            self.inputs = replace(complete, owner=Sample(owner, self.now))
            self.assertFalse(self.arbiter.state()["available"])
            self.assertEqual(self.arbiter.acquire({})[1]["reason"], reason)
        self.inputs = replace(complete, grant=replace(complete.grant, issued_at=1.))
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "local_authorization_required")
        self.assertEqual(self.output.commands, [])

    def test_exclusive_lease_limits_sequence_and_stop_feedback(self):
        status, reply = self.arbiter.acquire({})
        self.assertEqual(status, 200, reply)
        self.assertEqual(reply["expires_in_ms"], 3000)
        session = reply["session_id"]
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
        self.inputs = replace(self.inputs, feedback=None, site_stop_feedback=Sample(Feedback.STOPPED, self.now))
        self.assertEqual(self.arbiter.state()["reason"], "stop_unconfirmed")
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
        # 起始时守卫样本仍在有效期内，且会在首条命令前过期。
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


class SiteFrontArbiterTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.output = MockOutput()
        self.inputs = Inputs()
        self.arbiter = MaintenanceArbiter(Limits(.20, .15, 30., .4, .2, .02),
                                          lambda: self.inputs, self.output, clock=lambda: self.now,
                                          idle_deadman=True)
        self.addCleanup(self.arbiter.close)
        self.now += .125
        self.refresh()
        self.inputs = replace(self.inputs, grant=self.site_grant(1))

    def site_grant(self, serial):
        return LocalGrant(serial, self.now, self.now + 5., frozenset({Direction.FORWARD}),
                          site_front_confirmed=True)

    def refresh(self, *, owner=Owner.NEUTRAL, feedback=Feedback.STOPPED):
        self.inputs = replace(self.inputs, owner=Sample(owner, self.now),
                              feedback=Sample(feedback, self.now))

    def acquire(self):
        status, reply = self.arbiter.acquire({})
        self.assertEqual(status, 200, reply)
        return reply["session_id"]

    def send(self, session, sequence=1, linear=.05, angular=0.):
        return self.arbiter.velocity({"session_id": session, "sequence": sequence,
                                      "linear_mps": linear, "angular_radps": angular})

    def test_missing_automatic_guard_only_accepts_local_site_forward_grant(self):
        grant = self.inputs.grant
        self.inputs = replace(self.inputs, grant=None)
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "guard_stale")
        self.inputs = replace(self.inputs, grant=replace(grant, site_front_confirmed=False))
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "guard_stale")
        self.inputs = replace(self.inputs, grant=grant)
        session = self.acquire()
        self.assertEqual(self.send(session)[0], 200)
        self.assertEqual([(r.linear_mps, r.angular_radps) for r in self.output.commands], [(.05, 0.)])

    def test_backward_and_both_turns_are_rejected_before_any_velocity_output(self):
        for serial, (linear, angular) in enumerate([(-.20, 0.), (0., .15), (0., -.15)], 1):
            with self.subTest(linear=linear, angular=angular):
                self.now += .025
                self.refresh()
                self.inputs = replace(self.inputs, grant=self.site_grant(serial))
                session = self.acquire()
                self.assertEqual(self.send(session, linear=linear, angular=angular),
                                 (409, {"ok": False, "reason": "obstacle"}))
                self.assertFalse(self.arbiter.state()["armed"])
        self.assertEqual(self.output.commands, [])

    def test_fresh_blocking_or_stale_automatic_guard_vetoes_site_confirmation(self):
        self.inputs = replace(self.inputs, guard=Sample(frozenset(), self.now))
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "obstacle")
        old_clear = Sample(frozenset({Direction.FORWARD}), self.now - .18)
        self.inputs = replace(self.inputs, guard=old_clear)
        session = self.acquire()
        self.assertEqual(self.send(session)[0], 200)
        self.now += .03
        self.refresh(feedback=Feedback.MOVING)
        self.assertEqual(self.arbiter.state()["reason"], "guard_stale")
        self.assertEqual(self.output.stops, [(session, "guard_stale")])
        self.assertEqual(self.send(session, sequence=2)[1]["reason"], "invalid_session")
        self.assertEqual(len(self.output.commands), 1)

    def test_site_confirmation_requires_fresh_standstill_and_keeps_feedback_watchdog(self):
        self.inputs = replace(self.inputs, feedback=Sample(Feedback.STOPPED, self.now - .201))
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "feedback_stale")
        self.refresh(feedback=Feedback.MOVING)
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "standstill_required")
        self.refresh()
        session = self.acquire()
        self.assertEqual(self.send(session)[0], 200)
        self.inputs = replace(self.inputs, feedback=None)
        self.assertEqual(self.arbiter.state()["reason"], "feedback_stale")
        self.assertEqual(self.output.stops, [(session, "feedback_stale")])
        self.assertEqual(self.send(session, sequence=2)[1]["reason"], "invalid_session")

    def test_five_second_site_grant_expires_despite_continued_forward_packets(self):
        deadline = self.inputs.grant.expires_at
        status, reply = self.arbiter.acquire({})
        self.assertEqual(status, 200, reply)
        self.assertEqual(reply["expires_in_ms"], 5000)
        session = reply["session_id"]
        for sequence in range(1, 21):
            self.refresh(feedback=Feedback.STOPPED if sequence == 1 else Feedback.MOVING)
            self.assertEqual(self.send(session, sequence=sequence)[0], 200)
            self.assertLessEqual(self.output.commands[-1].expires_at, deadline)
            self.now += .25
        self.assertEqual(self.now, deadline)
        self.refresh(feedback=Feedback.MOVING)
        self.assertEqual(self.send(session, sequence=21)[1]["reason"], "invalid_session")
        self.assertEqual(len(self.output.commands), 20)
        self.assertEqual(self.output.stops[0][0], session)
        self.assertFalse(self.arbiter.state()["armed"])

    def test_handset_takeover_retracts_site_session_without_competing_stop(self):
        session = self.acquire()
        self.assertEqual(self.send(session)[0], 200)
        self.now += .025
        self.refresh(owner=Owner.OWNED, feedback=Feedback.MOVING)
        self.assertEqual(self.arbiter.state()["reason"], "control_owned")
        self.assertEqual(self.arbiter.stop({"session_id": session})[1]["reason"], "stop_unconfirmed")
        self.assertEqual(self.send(session, sequence=2)[1]["reason"], "invalid_session")
        self.assertEqual(self.output.stops, [])
        self.assertEqual(len(self.output.commands), 1)


class RepeatedActionsTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.output = MockOutput()
        self.inputs = Inputs()
        self.arbiter = MaintenanceArbiter(Limits(.20, .15, .75, .25, .20, .02),
                                          lambda: self.inputs, self.output, clock=lambda: self.now,
                                          idle_deadman=True, operator_supervised=True)
        self.addCleanup(self.arbiter.close)
        self.now += .125
        self.inputs = replace(self.inputs, grant=LocalGrant(
            1, self.now, None, frozenset({Direction.FORWARD, Direction.BACKWARD}), repeated_actions=True))
        self.refresh()
        code, response = self.arbiter.acquire({})
        self.assertEqual(code, 200, response)
        self.assertEqual(response["action_limit_ms"], 750)
        self.assertIs(response["repeated_actions"], True)
        self.assertNotIn("expires_in_ms", response)
        self.session = response["session_id"]

    def refresh(self, elapsed=0., *, feedback=Feedback.STOPPED, owner=Owner.NEUTRAL):
        self.now += elapsed
        self.inputs = replace(self.inputs, owner=Sample(owner, self.now),
                              feedback=None if feedback is None else Sample(feedback, self.now))

    def start(self, sequence, direction="forward"):
        return self.arbiter.action_start({"session_id": self.session, "sequence": sequence, "direction": direction})

    def release(self, sequence, action_id):
        return self.arbiter.action_release({"session_id": self.session, "sequence": sequence, "action_id": action_id})

    def send(self, sequence, linear=0., action_id=None):
        payload = {"session_id": self.session, "sequence": sequence, "linear_mps": linear, "angular_radps": 0.}
        if action_id is not None:
            payload["action_id"] = action_id
        return self.arbiter.velocity(payload)

    def test_release_waits_for_a_fresh_stop_before_next_direction(self):
        self.assertEqual(self.start(1, "left")[1]["reason"], "direction_not_authorized")
        self.assertEqual(self.send(1, .20)[1]["reason"], "invalid_action")
        self.assertEqual(self.start(1), (200, {"ok": True, "action_id": 1, "expires_in_ms": 750}))
        self.assertEqual(self.output.commands, [])
        self.assertEqual(self.send(2, .20, 1)[0], 200)
        self.refresh(.125, feedback=Feedback.MOVING)
        self.assertEqual(self.release(3, 1)[0], 200)
        self.assertEqual(self.release(3, 1)[0], 200)
        self.assertEqual(self.output.stops, [(self.session, "action_release")])
        self.assertEqual(self.start(4, "backward")[1]["reason"], "action_stopping")
        self.assertEqual(self.send(4, .20, 1)[1]["reason"], "action_expired")
        self.refresh(feedback=Feedback.STOPPED)
        self.assertEqual(self.arbiter.state()["action_state"], "stopping")
        self.refresh(.125)
        self.assertEqual(self.arbiter.state()["action_state"], "ready")
        self.assertEqual(self.start(5, "backward")[0], 200)
        self.assertEqual(self.send(6, -.20, 1)[1]["reason"], "invalid_action")
        self.assertEqual(self.send(6, .20, 5)[1]["reason"], "direction_not_authorized")
        self.assertEqual(self.send(6, -.20, 5)[0], 200)
        self.assertEqual([r.linear_mps for r in self.output.commands], [.20, -.20])

    def test_ready_can_outlast_motion_cap_but_held_action_cannot(self):
        for sequence in range(1, 9):
            self.refresh(.125)
            self.assertEqual(self.send(sequence)[0], 200)
        self.assertEqual(self.start(9)[0], 200)
        expires_at = self.now + .75
        for sequence in range(10, 16):
            self.assertEqual(self.send(sequence, .20, 9)[0], 200)
            self.assertLessEqual(self.output.commands[-1].expires_at, expires_at)
            self.refresh(.125, feedback=Feedback.MOVING)
        self.assertEqual(self.now, expires_at)
        self.assertEqual(self.send(16, .20, 9)[1]["reason"], "action_expired")
        self.assertEqual(self.output.stops, [(self.session, "action_expired")])
        self.assertEqual(self.arbiter.state()["action_state"], "stopping")
        self.assertEqual(self.send(17)[0], 200)
        self.refresh(.125)
        self.assertEqual(self.arbiter.state()["action_state"], "release_required")
        self.assertEqual(self.start(18, "backward")[1]["reason"], "action_release_required")
        self.assertEqual(self.release(18, 9)[0], 200)
        self.assertEqual(self.start(19, "backward")[0], 200)

    def test_release_reorders_before_start_and_fences_delayed_packets(self):
        self.assertEqual(self.release(2, 1)[0], 200)
        self.assertEqual(self.start(1)[1]["reason"], "stale_sequence")
        self.assertEqual(self.send(3, .20, 1)[1]["reason"], "invalid_action")
        self.assertEqual(self.output.commands, [])
        self.assertEqual(self.start(4)[0], 200)
        self.assertEqual(self.release(5, 1)[0], 200)
        self.assertEqual(self.arbiter.state()["action_id"], 4)

    def test_late_release_after_heartbeat_does_not_extend_the_watchdog(self):
        self.assertEqual(self.send(3)[0], 200)
        self.refresh(.125)
        self.assertEqual(self.release(2, 1)[0], 200)
        self.assertEqual(self.start(1)[1]["reason"], "stale_sequence")
        self.refresh(.125)
        self.assertEqual(self.arbiter.state()["reason"], "deadman_expired")
        self.assertEqual(self.output.commands, [])

    def test_zero_does_not_extend_motion_and_deadman_consumes_ready_grant(self):
        self.assertEqual(self.start(1)[0], 200)
        self.assertEqual(self.send(2, .20, 1)[0], 200)
        self.refresh(.125, feedback=Feedback.MOVING)
        self.assertEqual(self.send(3)[1]["reason"], "idle_heartbeat_after_motion")
        self.refresh(.125, feedback=Feedback.MOVING)
        self.assertEqual(self.send(4, .20, 1)[1]["reason"], "invalid_session")
        self.assertEqual(self.output.stops, [(self.session, "deadman_expired")])
        self.refresh(.125)
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "local_authorization_required")

    def test_handset_takeover_during_stop_revokes_ready_session(self):
        self.assertEqual(self.start(1)[0], 200)
        self.assertEqual(self.send(2, .20, 1)[0], 200)
        self.refresh(.125, feedback=Feedback.MOVING)
        self.assertEqual(self.release(3, 1)[0], 200)
        self.refresh(.125, feedback=None, owner=Owner.OWNED)
        state = self.arbiter.state()
        self.assertEqual(state["reason"], "control_owned")
        self.assertFalse(state["armed"])
        self.assertEqual(self.output.stops, [(self.session, "action_release")])
        self.refresh(.125)
        self.assertEqual(self.arbiter.acquire({})[1]["reason"], "local_authorization_required")

    def test_lost_feedback_during_stop_revokes_ready_session(self):
        self.assertEqual(self.start(1)[0], 200)
        self.assertEqual(self.send(2, .20, 1)[0], 200)
        self.refresh(.125, feedback=Feedback.MOVING)
        self.assertEqual(self.release(3, 1)[0], 200)
        self.refresh(.0625, feedback=None)
        state = self.arbiter.state()
        self.assertFalse(state["armed"])
        self.assertEqual(state["reason"], "feedback_stale")
        self.assertEqual(self.output.stops, [(self.session, "action_release")])


if __name__ == "__main__":
    unittest.main()
