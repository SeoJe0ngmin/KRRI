"""CAN payload guard and transmit-only bus status snapshot.

2026-10-18 decision: transmit only, like apriltag_v4 that drove the truck on
2026-10-01. The receive queue is never read and TXACK is not enabled, so no
received frame (error frame, overrun, foreign sender) can stop the vehicle.
Bus status and error counters are read every status_period_s for the log only.
"""
import math
import resource
import time


def validate_diagnostics(cfg, live=False):
    c = cfg.get('diagnostics', {})
    if not c.get('enabled'):
        if live:
            raise ValueError('drive requires CAN diagnostics')
        return
    v = c.get('status_period_s')
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
        raise ValueError('invalid CAN diagnostics.status_period_s')


def guard(frame):
    cid, data = int(frame.id), list(frame.data)
    if int(frame.flags) != 0:
        raise RuntimeError('CAN requires standard data frame flags=0')
    if cid == 0x1E3:
        from .canbus import PAYLOADS
        if tuple(data) not in PAYLOADS.values():
            raise RuntimeError("unapproved drive payload")
        if len(data) != 8 or any(data[i] != 127 for i in (0, 3, 4, 5, 6, 7)) or (data[1] != 127 and data[2] != 127):
            raise RuntimeError('CAN drive-only byte guard')
    elif cid == 0x2E3:
        if len(data) != 8 or data[:4]+data[5:] != [0x42, 0, 0, 0x0A, 0x40, 0x69, 0x93] or not 0 <= data[4] < 16:
            raise RuntimeError('CAN driving mode byte guard')
    elif cid != 0x764 or data != [0]:
        raise RuntimeError('CAN identifier/payload not allowed')


class BusStatus:
    """Log-only snapshot: v4 read the same two values every 5 s and never acted on them."""

    def __init__(self, channel, backend, cfg):
        self.ch, self.api, self.c = channel, backend, cfg
        self.next_status = 0.
        self.usage0 = resource.getrusage(resource.RUSAGE_SELF)
        self.started = time.monotonic()
        self.report = dict(tx_count=0, bus_status=None, bus_status_names=[], error_counters=None,
                           status_error=None, worker_cpu_percent=None, transmission_is_not_actuation=True,
                           receive_queue='not_read_by_design')

    def wrote(self):
        self.report['tx_count'] += 1
        self.report['last_write_s'] = time.monotonic()

    def poll(self, now):
        if now < self.next_status:
            return None
        self.next_status = now+self.c['status_period_s']
        try:
            state = int(self.ch.readStatus())
            self.report['bus_status'] = state
            stat = getattr(self.api, 'Stat', None)
            self.report['bus_status_names'] = [n for n in ('ERROR_PASSIVE', 'BUS_OFF', 'ERROR_WARNING', 'ERROR_ACTIVE',
                                                           'TX_PENDING', 'RX_PENDING', 'TXERR', 'RXERR', 'HW_OVERRUN', 'SW_OVERRUN')
                                               if stat is not None and getattr(stat, n, 0) and state & int(getattr(stat, n))]
            counts = self.ch.read_error_counters()
            self.report['error_counters'] = dict(tx=int(counts.tx), rx=int(counts.rx), overrun=int(counts.overrun))
            self.report['status_error'] = None
        except Exception as exc:
            self.report['status_error'] = str(exc)
        usage = resource.getrusage(resource.RUSAGE_SELF)
        elapsed = max(1e-6, time.monotonic()-self.started)
        self.report['worker_cpu_percent'] = 100.*(usage.ru_utime+usage.ru_stime-self.usage0.ru_utime-self.usage0.ru_stime)/elapsed
        return dict(self.report)
