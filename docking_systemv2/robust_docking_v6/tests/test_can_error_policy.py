import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from enum import IntFlag
from docking.canhealth import Diagnostics
from docking.runtime import CAN_CONFIG

class Flags(IntFlag):
    ERROR_FRAME=32;TXACK=64;OVERRUN=1536
class NoMsg(Exception):pass
class ErrorPolicyTests(unittest.TestCase):
    def make(self,bits):
        ch=Mock();ch.read.side_effect=[NS(id=0,data=[],flags=bits),NoMsg()]
        ch.readStatus.return_value=0
        ch.read_error_counters.return_value=NS(tx=0,rx=0,overrun=0)
        api=NS(MessageFlag=Flags,CanNoMsg=NoMsg,Stat=NS(BUS_OFF=1,ERROR_PASSIVE=2))
        return Diagnostics(ch,api,CAN_CONFIG['diagnostics']),ch
    def test_error_frame_is_recorded_without_single_frame_abort(self):
        d,ch=self.make(Flags.ERROR_FRAME);r=d.poll()
        self.assertEqual(r['error_frame_count'],1)
        self.assertEqual(r['last_error_frame']['flags'],32)
        self.assertIsNone(r['fault'])
    def test_overrun_and_busoff_still_stop(self):
        d,ch=self.make(Flags.OVERRUN)
        with self.assertRaisesRegex(RuntimeError,'can_rx_overrun'):d.poll()
        d,ch=self.make(Flags.ERROR_FRAME);ch.readStatus.return_value=1
        with self.assertRaisesRegex(RuntimeError,'bus off'):d.poll()
