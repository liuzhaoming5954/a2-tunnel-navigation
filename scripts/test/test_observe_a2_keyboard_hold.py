"""用合成事件验证键盘采样状态。无需打开实体设备。"""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from observe_a2_keyboard_hold import EV_KEY, EV_SYN, SYN_DROPPED, SYN_REPORT, KeyboardHoldState


UP = 103
DOWN = 108


class KeyboardHoldTest(unittest.TestCase):
    def setUp(self):
        self.state = KeyboardHoldState()

    def test_boot_held_and_chord_require_neutral_before_next_press(self):
        self.assertFalse(self.state.snapshot([UP])['pressed'])
        self.assertFalse(self.state.snapshot([])['pressed'])
        self.state.event(EV_KEY, UP, 1)
        self.assertEqual(self.state.snapshot([UP])['direction'], 'forward')
        self.state.event(EV_KEY, DOWN, 1)
        self.assertFalse(self.state.snapshot([UP, DOWN])['pressed'])
        self.state.event(EV_KEY, DOWN, 0)
        self.assertFalse(self.state.snapshot([UP])['pressed'])

    def test_release_and_repress_change_epoch_without_relying_on_repeat(self):
        self.state.snapshot([])
        self.state.event(EV_KEY, UP, 1)
        pressed = self.state.snapshot([UP])
        self.state.event(EV_KEY, UP, 2)
        self.assertEqual(self.state.snapshot([UP]), pressed)
        self.state.event(EV_KEY, UP, 0)
        self.state.event(EV_KEY, UP, 1)
        self.assertNotEqual(self.state.snapshot([UP])['epoch'], pressed['epoch'])

    def test_dropped_events_or_kernel_state_mismatch_invalidate_hold(self):
        self.state.snapshot([])
        self.state.event(EV_KEY, UP, 1)
        self.assertTrue(self.state.snapshot([UP])['pressed'])
        self.assertFalse(self.state.snapshot([])['pressed'])
        self.state.event(EV_KEY, UP, 1)
        self.state.event(EV_SYN, SYN_DROPPED, 0)
        self.state.event(EV_KEY, UP, 0)
        self.assertFalse(self.state.snapshot([])['pressed'])
        self.state.event(EV_SYN, SYN_REPORT, 0)
        self.assertFalse(self.state.snapshot([])['pressed'])
        self.state.event(EV_KEY, UP, 1)
        self.assertTrue(self.state.snapshot([UP])['pressed'])
