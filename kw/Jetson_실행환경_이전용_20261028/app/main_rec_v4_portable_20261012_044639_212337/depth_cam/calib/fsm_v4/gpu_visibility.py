"""Bounded float64 CUDA visibility batches for the numeric planning worker.

Uses the installed CUDA driver/NVRTC directly; never imports camera, CAN,
OpenCV or PyTorch. Near-boundary decisions use the original CPU evaluator.
"""
from __future__ import annotations

import ctypes as C
import ctypes.util
import math
from pathlib import Path
import sys
import time

import numpy as np

from . import config as cfg


class GPUUnavailable(RuntimeError):
    pass


class GPUVisibility:
    capacity = 128

    def __init__(self, device=0):
        self.device = int(device)
        self.context = C.c_void_p()
        self.module = C.c_void_p()
        self.stream = C.c_void_p()
        self.input_device, self.output_device = C.c_uint64(), C.c_uint64()
        self.input_host, self.output_host = C.c_void_p(), C.c_void_p()
        self.driver = None
        self.stats = dict(batches=0, candidates=0, boundary_cpu=0, gpu_wall_sec=0.)
        self.stats.update(timing_batches=0, h2d_stream_ms=0., kernel_stream_ms=0.,
                          d2h_stream_ms=0., host_sync_wait_ms=0., host_sync_cpu_ms=0.)
        self.timing_events = []
        self.profile_timing = False
        self.timing_error = None
        started = time.monotonic()
        try:
            self._initialize()
        except Exception:
            self.close()
            raise
        self.init_sec = time.monotonic()-started

    @staticmethod
    def _bind(library, name, arguments):
        function = getattr(library, name)
        function.argtypes, function.restype = arguments, C.c_int
        return function

    def _call(self, name, *args):
        code = getattr(self.driver, name)(*args)
        if code:
            raise GPUUnavailable(f'CUDA {name} failed ({code})')

    def _initialize(self):
        if (not .05 <= cfg.VISIBILITY_PATH_SAMPLE_STEP_DEG <= 30.
                or not 1 <= cfg.VISIBILITY_FORWARD_SEARCH_ITERATIONS <= 32):
            raise GPUUnavailable('Sampling exceeds bounded CUDA kernel domain')
        void, pv = C.c_void_p, C.POINTER(C.c_void_p)
        self.driver = C.CDLL(C.util.find_library('cuda') or 'libcuda.so.1')
        functions = {
            'cuInit': [C.c_uint], 'cuDevicePrimaryCtxRetain': [pv, C.c_int],
            'cuDevicePrimaryCtxRelease_v2': [C.c_int], 'cuCtxSetCurrent': [void],
            'cuDeviceComputeCapability': [C.POINTER(C.c_int), C.POINTER(C.c_int), C.c_int],
            'cuStreamCreateWithPriority': [pv, C.c_uint, C.c_int],
            'cuStreamDestroy_v2': [void], 'cuStreamSynchronize': [void],
            'cuMemAlloc_v2': [C.POINTER(C.c_uint64), C.c_size_t],
            'cuMemFree_v2': [C.c_uint64], 'cuMemAllocHost_v2': [pv, C.c_size_t],
            'cuMemFreeHost': [void], 'cuModuleLoadData': [pv, void],
            'cuModuleUnload': [void], 'cuModuleGetFunction': [pv, void, C.c_char_p],
            'cuMemcpyHtoDAsync_v2': [C.c_uint64, void, C.c_size_t, void],
            'cuMemcpyDtoHAsync_v2': [void, C.c_uint64, C.c_size_t, void],
            'cuLaunchKernel': [void, C.c_uint, C.c_uint, C.c_uint, C.c_uint,
                               C.c_uint, C.c_uint, C.c_uint, void, pv, pv],
        }
        for name, arguments in functions.items():
            self._bind(self.driver, name, arguments)
        self._call('cuInit', 0)
        self._call('cuDevicePrimaryCtxRetain', C.byref(self.context), self.device)
        self._call('cuCtxSetCurrent', self.context)
        major, minor = C.c_int(), C.c_int()
        self._call('cuDeviceComputeCapability', C.byref(major), C.byref(minor), self.device)
        ptx = self._compile(major.value, minor.value)
        self._call('cuModuleLoadData', C.byref(self.module), ptx)
        self.function = void()
        self._call('cuModuleGetFunction', C.byref(self.function), self.module, b'evaluate')
        # Nonblocking, normal/lowest-priority stream; batches are bounded.
        self._call('cuStreamCreateWithPriority', C.byref(self.stream), 1, 0)
        for device_ptr, host_ptr, columns in (
            (self.input_device, self.input_host, 64),
            (self.output_device, self.output_host, 4),
        ):
            size = self.capacity*columns*8
            self._call('cuMemAlloc_v2', C.byref(device_ptr), size)
            self._call('cuMemAllocHost_v2', C.byref(host_ptr), size)
        self.host_input = np.ctypeslib.as_array(C.cast(self.input_host, C.POINTER(C.c_double)), shape=(self.capacity*64,)).reshape(-1,64)
        self.host_output = np.ctypeslib.as_array(C.cast(self.output_host, C.POINTER(C.c_double)), shape=(self.capacity*4,)).reshape(-1,4)

    def _compile(self, major, minor):
        libraries = [C.util.find_library('nvrtc'), '/usr/local/cuda/lib64/libnvrtc.so']
        libraries += [str(p) for p in Path(sys.prefix).glob('lib/python*/site-packages/nvidia/cuda_nvrtc/lib/libnvrtc.so*')]
        nv = None
        for name in libraries:
            if not name:
                continue
            try:
                nv = C.CDLL(name)
                break
            except OSError:
                pass
        if nv is None:
            raise GPUUnavailable('NVRTC library is unavailable')
        void, pv = C.c_void_p, C.POINTER(C.c_void_p)
        for name, args in {
            'nvrtcCreateProgram': [pv,C.c_char_p,C.c_char_p,C.c_int,C.POINTER(C.c_char_p),C.POINTER(C.c_char_p)],
            'nvrtcCompileProgram': [void,C.c_int,C.POINTER(C.c_char_p)],
            'nvrtcGetProgramLogSize': [void,C.POINTER(C.c_size_t)],
            'nvrtcGetProgramLog': [void,void],
            'nvrtcGetPTXSize': [void,C.POINTER(C.c_size_t)],
            'nvrtcGetPTX': [void,void], 'nvrtcDestroyProgram': [pv],
        }.items():
            self._bind(nv,name,args)
        constants = {
            'CENTER_X': cfg.CAMERA_TO_ROT_CENTER_X_M,
            'CENTER_Z': cfg.CAMERA_TO_ROT_CENTER_Z_M,
            'CAMERA_YAW': cfg.CAMERA_YAW_IN_VEHICLE_DEG,
            'HALF_WIDTH': cfg.PALLET_FRONT_VISIBILITY_WIDTH_M*.5,
            'MIN_DEPTH': cfg.VISIBILITY_MIN_CORNER_DEPTH_M,
            'SAMPLE_STEP': cfg.VISIBILITY_PATH_SAMPLE_STEP_DEG,
        }
        source = ''.join(f'#define {key} {value:.17g}\n' for key,value in constants.items())
        source += f'#define FORWARD_ITERATIONS {int(cfg.VISIBILITY_FORWARD_SEARCH_ITERATIONS)}\n'
        source += Path(__file__).with_name('visibility_gpu.cu').read_text()
        program = void()
        code = nv.nvrtcCreateProgram(C.byref(program), source.encode(), b'visibility_gpu.cu', 0, None, None)
        if code:
            raise GPUUnavailable(f'NVRTC create failed ({code})')
        try:
            options = (C.c_char_p*3)(b'--std=c++11', f'--gpu-architecture=compute_{major}{minor}'.encode(), b'--fmad=false')
            code = nv.nvrtcCompileProgram(program,3,options)
            size = C.c_size_t()
            if code:
                nv.nvrtcGetProgramLogSize(program,C.byref(size))
                log = C.create_string_buffer(size.value)
                nv.nvrtcGetProgramLog(program,log)
                raise GPUUnavailable(f'NVRTC compile failed: {log.value.decode()}')
            if nv.nvrtcGetPTXSize(program,C.byref(size)):
                raise GPUUnavailable('NVRTC PTX size failed')
            ptx = C.create_string_buffer(size.value)
            if nv.nvrtcGetPTX(program,ptx):
                raise GPUUnavailable('NVRTC PTX export failed')
            return ptx
        finally:
            nv.nvrtcDestroyProgram(C.byref(program))

    def close(self):
        if self.driver is None:
            return
        if self.context.value:
            self.driver.cuCtxSetCurrent(self.context)
        for event in self.timing_events:
            self.driver.cuEventDestroy_v2(event)
        self.timing_events = []
        for name, value in [('cuStreamDestroy_v2',self.stream),
                            ('cuMemFree_v2',self.input_device),('cuMemFree_v2',self.output_device),
                            ('cuMemFreeHost',self.input_host),('cuMemFreeHost',self.output_host),
                            ('cuModuleUnload',self.module)]:
            if value.value:
                getattr(self.driver,name)(value)
                value.value = 0
        if self.context.value:
            self.driver.cuDevicePrimaryCtxRelease_v2(self.device)
            self.context.value = None

    def enable_timing(self, enabled=True):
        """Optional events on the existing stream; no new synchronization."""
        self.profile_timing = bool(enabled) and self.timing_error is None
        if not self.profile_timing or self.timing_events:
            return
        try:
            for name, args in {
                'cuEventCreate': [C.POINTER(C.c_void_p), C.c_uint],
                'cuEventDestroy_v2': [C.c_void_p],
                'cuEventRecord': [C.c_void_p, C.c_void_p],
                'cuEventElapsedTime': [C.POINTER(C.c_float), C.c_void_p, C.c_void_p],
            }.items():
                self._bind(self.driver, name, args)
            for _ in range(4):
                event = C.c_void_p()
                self._call('cuEventCreate', C.byref(event), 0)
                self.timing_events.append(event)
        except Exception as exc:
            self.timing_error = repr(exc)
            self.profile_timing = False

    def _timing_mark(self, index):
        if self.profile_timing:
            code = self.driver.cuEventRecord(self.timing_events[index], self.stream)
            if code:
                self.timing_error = f'cuEventRecord failed: {code}'
                self.profile_timing = False

    def _timing_read(self):
        if not self.profile_timing:
            return
        for i, key in enumerate(('h2d_stream_ms', 'kernel_stream_ms', 'd2h_stream_ms')):
            elapsed = C.c_float()
            code = self.driver.cuEventElapsedTime(C.byref(elapsed), self.timing_events[i], self.timing_events[i+1])
            if code:
                self.timing_error = f'cuEventElapsedTime failed: {code}'
                self.profile_timing = False
                return
            self.stats[key] += elapsed.value
        self.stats['timing_batches'] += 1

    def _evaluate(self, rows, kind):
        if not np.isfinite(rows).all() or np.max(np.abs(rows), initial=0)>1e9:
            raise GPUUnavailable('Nonfinite/out-of-range batch; use original CPU planner')
        answers = []
        started = time.monotonic()
        self._call('cuCtxSetCurrent', self.context)
        for first in range(0,len(rows),self.capacity):
            batch = rows[first:first+self.capacity]
            n = len(batch)
            self.host_input[:n] = batch
            self._timing_mark(0)
            self._call('cuMemcpyHtoDAsync_v2',self.input_device,self.input_host,n*64*8,self.stream)
            self._timing_mark(1)
            values = [C.c_uint64(self.input_device.value),C.c_uint64(self.output_device.value),C.c_int(n),C.c_int(kind)]
            arguments = (C.c_void_p*4)(*(C.addressof(v) for v in values))
            self._call('cuLaunchKernel',self.function,(n+127)//128,1,1,128,1,1,0,self.stream,arguments,None)
            self._timing_mark(2)
            self._call('cuMemcpyDtoHAsync_v2',self.output_host,self.output_device,n*4*8,self.stream)
            self._timing_mark(3)
            wait_wall, wait_cpu = time.monotonic(), time.thread_time()
            self._call('cuStreamSynchronize',self.stream)
            self.stats['host_sync_wait_ms'] += (time.monotonic()-wait_wall)*1000
            self.stats['host_sync_cpu_ms'] += (time.thread_time()-wait_cpu)*1000
            self._timing_read()
            answers.append(self.host_output[:n].copy())
            self.stats['batches'] += 1
            self.stats['candidates'] += n
        self.stats['gpu_wall_sec'] += time.monotonic()-started
        return np.concatenate(answers) if answers else np.zeros((0,4))

    @staticmethod
    def _metadata(meta):
        # Match the CPU pixel -> intrinsic-only -> angular-only fallback order.
        if not meta:
            return 0, (0.,)*6, ()
        try:
            fx,ppx,width = (float(meta[k]) for k in ('fx','ppx','frame_width'))
            if min(fx,width)<=0:
                return 0, (0.,)*6, ()
        except (KeyError,TypeError,ValueError):
            return 0, (0.,)*6, ()
        intrinsic = (fx,0.,ppx,0.,width,1.)
        try:
            corners = [tuple(map(float,p[:3])) for p in meta['face_corners_camera_m']]
            fy,ppy,height = (float(meta[k]) for k in ('fy','ppy','frame_height'))
            if len(corners)!=4 or any(len(p)!=3 for p in corners) or min(fy,height)<=0:
                return 1,intrinsic,()
            return 2,(fx,fy,ppx,ppy,width,height),tuple(v for p in corners for v in p)
        except (KeyError,TypeError,ValueError,IndexError):
            return 1,intrinsic,()

    def forward_limits(self, pose, meta, prepared, starts_safe):
        from . import planner
        rows = np.zeros((len(prepared),64),dtype=np.float64)
        mode,intrinsics,corners = self._metadata(meta)
        half = planner.configured_visible_half_angle_deg()
        initial_half = half if starts_safe else cfg.CAMERA_HORIZONTAL_FOV_DEG*.5
        initial_edge = cfg.IMAGE_EDGE_MARGIN_NORM if starts_safe else 0.
        for row,(turn,turned,turned_meta,requested) in zip(rows,prepared):
            if abs(turn)>180 or not 0<=requested<=10:
                raise GPUUnavailable('Action exceeds bounded GPU batch domain')
            turned_mode,_,turned_corners = self._metadata(turned_meta)
            if turned_mode != mode:
                raise GPUUnavailable('Metadata representation changed during prediction')
            row[:10] = (pose.pallet_x_m,pose.pallet_z_m,pose.yaw_deg,turn,requested,
                         initial_half,initial_edge,turned.pallet_x_m,turned.pallet_z_m,turned.yaw_deg)
            row[10:16],row[16] = intrinsics,mode
            if mode==2:
                row[17:29],row[29:41] = corners,turned_corners
            row[41:43] = half,cfg.IMAGE_EDGE_MARGIN_NORM
        results = self._evaluate(rows,0)
        if not np.isfinite(results[:,:3]).all():
            raise GPUUnavailable('Invalid GPU forward-limit result')
        answer = []
        for result,(turn,turned,turned_meta,requested) in zip(results,prepared):
            if (result[0] not in (0.,1.) or result[2] not in (0.,1.)
                    or not 0. <= result[1] <= requested):
                raise GPUUnavailable('GPU forward limit exceeds candidate domain')
            recovery = (not starts_safe and not bool(result[0])
                        and planner.visibility_recovery_rotation_safe(pose, turn, meta))
            if result[2] or recovery:
                # Original CUDA path remains unchanged for ordinary starts.
                # The rare recovery exception uses the exact parent CPU rule.
                self.stats['boundary_cpu'] += 1
                valid = (planner.waypoint_rotation_visibility_safe(
                    pose, turn, meta, starts_safe=starts_safe)
                    and planner.action_keeps_front_visible(turned,0.,0.,turned_meta))
                forward = planner._maximum_visible_forward_m(turned,0.,requested,turned_meta)[0] if valid else 0.
                answer.append((bool(valid),forward))
            else:
                answer.append((bool(result[0]),float(result[1])))
        return answer

    def angular_margins(self, actions):
        from . import planner
        rows = np.zeros((len(actions),64),dtype=np.float64)
        for row,(pose,turn,forward) in zip(rows,actions):
            if abs(turn)>180 or abs(forward)>10:
                raise GPUUnavailable('Action exceeds bounded GPU angular domain')
            row[:6]=(pose.pallet_x_m,pose.pallet_z_m,pose.yaw_deg,turn,forward,planner.configured_visible_half_angle_deg())
        result = self._evaluate(rows,1)
        if np.isnan(result[:,:2]).any() or np.isposinf(result[:,0]).any():
            raise GPUUnavailable('Invalid GPU angular-margin result')
        margins=[]
        for answer,(pose,turn,forward) in zip(result,actions):
            if answer[1]:
                self.stats['boundary_cpu']+=1
                margins.append(planner.action_visibility_margin_deg(pose,turn,forward))
            else:
                margins.append(float(answer[0]))
        return margins
