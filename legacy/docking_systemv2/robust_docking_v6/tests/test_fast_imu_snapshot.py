import threading
import unittest
from legacy.docking_systemv2.robust_docking_v6.docking.runtime import read_fast_imu,fast_imu_available

class Shared(list):
    def __init__(self,value):super().__init__(value);self.lock=threading.Lock()
    def get_lock(self):return self.lock

class FastImuTests(unittest.TestCase):
    def test_collision_keeps_fresh_cache_and_next_read_recovers(self):
        old=[0,10.,0,0,0,0,0,1];shared=Shared([0,10.02,0,0,0,0,0,1])
        shared.lock.acquire()
        try:cached=read_fast_imu(shared,old)
        finally:shared.lock.release()
        self.assertIs(cached,old)
        self.assertTrue(fast_imu_available(cached,10.03,.1))
        self.assertFalse(fast_imu_available(cached,10.2,.1))
        self.assertEqual(read_fast_imu(shared,cached)[1],10.02)
    def test_no_initial_snapshot_and_fault_remain_invalid(self):
        self.assertFalse(fast_imu_available([0]*8,10,.1))
        self.assertFalse(fast_imu_available([0,10.,0,0,0,0,1,1],10.01,.1))
