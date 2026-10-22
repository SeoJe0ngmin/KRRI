"""Bounded CANlib diagnostics. No additional CAN identifiers or payloads.

TXACK measures host write to receipt of the transmission notification, not
vehicle actuation latency. Lack of application feedback remains explicit.
"""
import collections
import math
import time


def validate_diagnostics(cfg, live=False):
    c = cfg.get('diagnostics', {})
    if not c.get('enabled'):
        if live:
            raise ValueError('drive requires CAN diagnostics')
        return
    for name in ('max_ack_age_s', 'max_write_gap_s', 'poll_budget_s', 'status_period_s', 'max_rx_per_poll', 'max_pending'):
        v = c.get(name)
        if v is None and not live and not c.get('verified'):
            continue
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError('invalid CAN diagnostics.'+name)
    for name in ('max_rx_per_poll', 'max_pending'):
        if not isinstance(c.get(name), int):
            raise ValueError('CAN diagnostic count must be an integer')


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


class Diagnostics:
    def __init__(self, channel, backend, cfg):
        self.ch, self.api, self.c = channel, backend, cfg
        channel.iocontrol.txack = 1
        channel.iocontrol.local_txecho = False
        self.pending = collections.deque()
        self.next_status = 0.
        self.last_write = {}
        self.report = dict(tx_count=0, txack_count=0, rx_count=0, foreign_drive_count=0,
                           max_notification_delay_s=0., max_write_gap_s=0., fault=None,
                           transmission_is_not_actuation=True)

    def write(self, frame):
        guard(frame)
        now = time.monotonic()
        cid = int(frame.id)
        if len(self.pending) >= self.c['max_pending']:
            raise RuntimeError('CAN pending diagnostic queue overflow')
        old = self.last_write.get(cid)
        if old is not None and cid in (0x1E3, 0x2E3):
            gap = now-old
            self.report['max_write_gap_s'] = max(self.report['max_write_gap_s'], gap)
            if gap > self.c['max_write_gap_s']:
                raise RuntimeError('CAN write interval exceeded')
        self.ch.write(frame)
        self.last_write[cid] = now
        self.pending.append((now, cid, tuple(frame.data)))
        self.report['tx_count'] += 1
        self.report['last_write_s'] = now

    def stop_transition(self):
        # Purge pending drive commands before neutral; do not replay after recovery.
        self.ch.iocontrol.flush_tx_buffer()
        self.pending.clear()

    def poll(self):
        started = time.monotonic()
        flags = self.api.MessageFlag
        for _ in range(self.c['max_rx_per_poll']):
            if time.monotonic()-started >= self.c['poll_budget_s']:
                break
            try:
                f = self.ch.read(timeout=0)
            except self.api.CanNoMsg:
                break
            now = time.monotonic()
            if f.flags & (flags.ERROR_FRAME | flags.OVERRUN):
                raise RuntimeError('CAN error frame or RX overrun')
            if f.flags & flags.TXACK:
                for i, (t, cid, data) in enumerate(self.pending):
                    if cid == f.id and data == tuple(f.data):
                        del self.pending[i]
                        self.report['txack_count'] += 1
                        self.report['last_txack_s'] = now
                        self.report['max_notification_delay_s'] = max(self.report['max_notification_delay_s'], now-t)
                        break
            else:
                self.report['rx_count'] += 1
                if f.id == 0x1E3:
                    self.report['foreign_drive_count'] += 1
                    raise RuntimeError('CAN foreign drive sender detected')
        now = time.monotonic()
        if now >= self.next_status:
            state = self.ch.readStatus()
            self.report['bus_status'] = int(state)
            if state & (self.api.Stat.BUS_OFF | self.api.Stat.ERROR_PASSIVE):
                raise RuntimeError('CAN bus off or error passive')
            counts = self.ch.read_error_counters()
            self.report['error_counters'] = dict(tx=counts.tx, rx=counts.rx, overrun=counts.overrun)
            if counts.overrun:
                raise RuntimeError('CAN RX overrun counter')
            self.ch.requestChipStatus()  # Async request; inspect most recent status without waiting.
            self.next_status = now+self.c['status_period_s']
        if self.pending and now-self.pending[0][0] > self.c['max_ack_age_s']:
            raise RuntimeError('CAN transmission acknowledgement timeout')
        self.report['pending'] = len(self.pending)
        return dict(self.report)
