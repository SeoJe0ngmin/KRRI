"""Bounded, nonblocking recording. Storage failures never drive the vehicle.

Only this thread serializes/writes records. Producers hand over immutable
snapshots; queue overflow, storage errors and limits are visible counters.
"""
import json
import os
import queue
import threading
import time


class Recorder:
    def __init__(self, folder, cfg, stream='observations'):
        self.folder, self.cfg, self.stream = folder, cfg, stream
        self.q = queue.Queue(cfg.get('queue_size', 4096))
        self.done = threading.Event()
        self.counts = {'enqueued': 0, 'written': 0, 'dropped': 0, 'bytes': 0}
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def put(self, value, filename=None):
        if self.error or self.done.is_set():
            self.counts['dropped'] += 1
            return False
        try:
            self.q.put_nowait((filename, value))
            self.counts['enqueued'] += 1
            return True
        except queue.Full:
            self.counts['dropped'] += 1
            return False

    def status(self):
        return dict(self.counts, pending=self.q.qsize(), error=self.error,
                    recording=not self.error and not self.done.is_set())

    def _run(self):
        handle = None
        flushed = time.monotonic()
        interval = self.cfg.get('flush_s', 1.)
        cap = self.cfg.get('max_bytes', 500*1024*1024)
        try:
            handle = open(os.path.join(self.folder, self.stream+'.jsonl'), 'w')
            while not self.done.is_set() or not self.q.empty():
                try:
                    filename, value = self.q.get(timeout=min(interval, .1))
                except queue.Empty:
                    filename = value = None
                if value is not None and not self.error:
                    data = json.dumps(value, ensure_ascii=False, allow_nan=False)+'\n'
                    size = len(data.encode('utf8'))
                    if self.counts['bytes']+size > cap:
                        self.error = 'recording_size_limit; control continues'
                        self.counts['dropped'] += 1
                    elif filename is None:
                        handle.write(data)
                        self.counts['bytes'] += size
                        self.counts['written'] += 1
                    else:
                        if os.path.basename(filename) != filename:
                            raise ValueError('recording snapshot must be a basename')
                        temp = os.path.join(self.folder, '.'+filename+'.tmp')
                        with open(temp, 'w') as f:
                            f.write(data)
                        os.replace(temp, os.path.join(self.folder, filename))
                        self.counts['bytes'] += size
                        self.counts['written'] += 1
                elif value is not None:
                    self.counts['dropped'] += 1
                if time.monotonic()-flushed >= interval:
                    handle.flush()
                    flushed = time.monotonic()
        except Exception as e:
            self.error = 'recording_io_error: '+str(e)
        finally:
            if handle:
                try:
                    handle.close()
                except OSError as e:
                    self.error = 'recording_close_error: '+str(e)

    def close(self, timeout=2.):
        self.done.set()
        self.thread.join(timeout)
        if self.thread.is_alive():
            self.error = 'recording_close_timeout; pending records may be lost'
        return self.status()
