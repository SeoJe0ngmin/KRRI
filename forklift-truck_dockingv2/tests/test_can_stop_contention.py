import unittest
from unittest.mock import Mock
from docking.canbus import Can

class StopContentionTests(unittest.TestCase):
    def make(self, previous):
        can=Can.__new__(Can)
        can.last_requested_command=previous
        can.abort_event=Mock();can.abort_event.is_set.return_value=False
        can.process=Mock();can.process.is_alive.return_value=True
        can.shared=Mock();can.shared.get_lock.return_value.acquire.return_value=False
        return can
    def test_repeated_stop_does_not_latch(self):
        can=self.make('stop');can.set('stop')
        can.abort_event.set.assert_not_called()
    def test_stop_after_motion_still_latches_when_lock_busy(self):
        can=self.make('forward');can.set('stop')
        can.abort_event.set.assert_called_once()
