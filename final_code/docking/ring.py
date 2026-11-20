"""Shared-memory frame ring between the camera worker and a measurement process (2026-11-01).

The supervisor owns the ring (`link`) because the camera worker is a daemon process and cannot spawn children. The camera worker
writes a frame into the next slot (`write_slot`: sequence number cleared, frame copied, sequence number set) and queues
(slot, seq, *meta); the process (`run`) re-checks the sequence number before and after copying the frame out, so a slot
overwritten by a newer frame is skipped instead of being read torn. The worker object is built in the process from a settings
dict that arrives on its own queue (the camera worker knows intrinsics and scales only once the device streams; frames queued
before the process is ready can never evict it).
"""

import gc
import queue
import traceback
import numpy as np

HEADER_BYTES = 8  # one int64 sequence number per slot


def link(ctx, frame_shape, dtype, slots=4, results_depth=16):
    from multiprocessing import shared_memory

    shape = tuple(int(v) for v in frame_shape)
    size = slots * HEADER_BYTES + slots * int(np.prod(shape)) * np.dtype(dtype).itemsize
    shm = shared_memory.SharedMemory(create=True, size=size)
    np.ndarray((slots,), np.int64, shm.buf)[:] = -1
    return dict(
        shm=shm,
        shm_name=shm.name,
        slots=slots,
        frame_shape=shape,
        dtype=np.dtype(dtype).str,
        frames=ctx.Queue(2),
        results=ctx.Queue(results_depth),
        settings=ctx.Queue(1),
        stop=ctx.Event(),
    )


def handoff(lk):
    """The picklable part of a link, for the camera worker's arguments."""
    return {k: lk[k] for k in ('shm_name', 'slots', 'frame_shape', 'dtype', 'frames', 'results', 'settings')}


def views(buf, slots, frame_shape, dtype):
    header = np.ndarray((slots,), np.int64, buf)
    frames = np.ndarray((slots,) + tuple(frame_shape), np.dtype(dtype), buf, offset=slots * HEADER_BYTES)
    return header, frames


def write_slot(header, frames, seq, frame):
    slot = seq % len(header)
    header[slot] = -1
    frames[slot] = frame
    header[slot] = seq
    return slot


def put_latest(q, item):
    """Queue an item; when the process is behind, the waiting item is replaced (the newest frame matters, not the backlog)."""
    try:
        q.put_nowait(item)
        return
    except queue.Full:
        pass
    try:
        q.get_nowait()
    except queue.Empty:
        pass
    try:
        q.put_nowait(item)
    except queue.Full:
        pass


def drain(q):
    """All results waiting in q, oldest first."""
    out = []
    while True:
        try:
            out.append(q.get_nowait())
        except queue.Empty:
            return out


def run(factory, shm_name, slots, frame_shape, dtype, frames_q, results_q, stop, settings_q=None):
    """Process body: build the worker from the settings dict (settings_q, or a dict item on frames_q), then
    worker.frame(raw, *meta) per (slot, seq, *meta) item. Frames that arrive before the settings are skipped.
    Never raises: a failure is reported once as dict(error=...) and the loop ends."""
    from multiprocessing import shared_memory

    shm = shared_memory.SharedMemory(name=shm_name)
    try:
        header, frames = views(shm.buf, slots, frame_shape, dtype)
        worker = None
        while not stop.is_set():
            if worker is None and settings_q is not None:
                try:
                    item = settings_q.get(timeout=0.2)
                except queue.Empty:
                    continue
                if isinstance(item, dict):
                    worker = factory(item)
                    gc.collect()
                    gc.freeze()
                continue
            try:
                item = frames_q.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break
            if isinstance(item, dict):
                worker = factory(item)
                gc.collect()
                gc.freeze()
                continue
            if worker is None:
                continue
            slot, seq = item[0], item[1]
            if header[slot] != seq:
                continue
            raw = np.array(frames[slot], copy=True)
            if header[slot] != seq:
                continue
            out = worker.frame(raw, *item[2:])
            try:
                results_q.put(out, timeout=1.0)
            except queue.Full:
                pass
    except Exception:
        try:
            results_q.put(dict(error=traceback.format_exc()), timeout=1.0)
        except Exception:
            pass
    finally:
        shm.close()


def close(lk, process, join_s=3.0):
    """Supervisor side: stop the process and release the ring."""
    if lk is None:
        return
    lk['stop'].set()
    try:
        lk['frames'].put_nowait(None)
    except Exception:
        pass
    if process is not None and process.pid:
        process.join(join_s)
        if process.is_alive():
            process.terminate()
            process.join(1)
    for q in (lk['frames'], lk['results'], lk.get('settings')):
        if q is None:
            continue
        try:
            q.close()
            q.cancel_join_thread()
        except Exception:
            pass
    try:
        lk['shm'].close()
        lk['shm'].unlink()
    except Exception:
        pass
