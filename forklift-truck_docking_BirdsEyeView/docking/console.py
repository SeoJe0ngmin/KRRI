"""Bounded console writer: a stalled SSH stdout cannot block control."""
import queue
import threading

class Console:
    def __init__(self):
        self.queue=queue.Queue(8);self.done=threading.Event();self.dropped=0
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()
    def write(self,message):
        try:self.queue.put_nowait(message)
        except queue.Full:self.dropped+=1
    def _run(self):
        while not self.done.is_set() or not self.queue.empty():
            try:message=self.queue.get(timeout=.1)
            except queue.Empty:continue
            try:print(message,flush=True)
            except (OSError,ValueError):return
    def close(self):
        self.done.set();self.thread.join(.2)
