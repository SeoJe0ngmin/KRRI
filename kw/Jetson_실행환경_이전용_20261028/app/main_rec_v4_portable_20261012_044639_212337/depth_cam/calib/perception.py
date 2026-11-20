# calib/perception.py
# YOLO26 pose 추론으로 파렛트의 전체 키포인트와 전면 4모서리를 뽑는다.
# 전체 키포인트는 visible-point PnP에, 전면 4점은 시각화/호환에 사용한다.

import numpy as np
import cv2
import torch
from ultralytics import YOLO
from typing import Optional, Tuple, List
from .config import (
    MODEL_PATH, FRONT_CLASS_NAME, CONF_THR, USE_GPU, CUDA_DEVICE,
    POSE_FACE_KPTS, PALLET_FACE_W, PALLET_FACE_H, VERBOSE_PERCEPTION,
    POSE_REFLECT_PADDING_PX, POSE_INFERENCE_IMGSZ, POSE_INSTANCE_SELECTION,
)
from .geometry import clamp_bbox
from .gpu_inference import configure_fp32_device, single_frame_result
from model_catalog import configured_clahe
from .performance import span, profiled


class Perception:
    def __init__(self,
                 model_path: str = MODEL_PATH,
                 front_class_name: str = FRONT_CLASS_NAME,
                 conf_thr: float = CONF_THR):
        # 1) 디바이스 선택
        cuda_available = torch.cuda.is_available()
        self.device = configure_fp32_device(USE_GPU, CUDA_DEVICE)
        self._fast_predictor = None
        self.backend_name = "pytorch-fp32"

        # ---- 상태 출력 (초기화) ----
        print("==========[Perception / CUDA Status]==========")
        print(f"[Perception] torch.__version__           : {torch.__version__}")
        try:
            print(f"[Perception] torch.version.cuda         : {torch.version.cuda}")
        except Exception:
            print(f"[Perception] torch.version.cuda         : (unavailable)")
        print(f"[Perception] USE_GPU (config)           : {USE_GPU}")
        print(f"[Perception] CUDA available             : {cuda_available}")
        print(f"[Perception] Requested CUDA_DEVICE      : {CUDA_DEVICE}")
        print(f"[Perception] Selected device            : {self.device}")

        if str(self.device).startswith("cuda") and cuda_available:
            try:
                dev_idx = int(str(self.device).split(":")[1])
            except Exception:
                try:
                    dev_idx = torch.cuda.current_device()
                except Exception:
                    dev_idx = 0
            try:
                print(f"[Perception] CUDA device name          : {torch.cuda.get_device_name(dev_idx)}")
            except Exception:
                print(f"[Perception] CUDA device name          : (unavailable)")
            try:
                print(f"[Perception] torch.cuda.current_device : {torch.cuda.current_device()}")
            except Exception:
                print(f"[Perception] torch.cuda.current_device : (unavailable)")
        else:
            print("[Perception] Running on CPU (no CUDA).")
        print("==============================================")

        # Keep original checkpoint and architecture; require full FP32 on the selected device.
        self.model = YOLO(model_path)
        self.model.model.to(device=self.device, dtype=torch.float32)
        print(f"[Perception] task={getattr(self.model, 'task', '?')} names={getattr(self.model, 'names', None)}")

        self.front_class_name = front_class_name
        self.conf_thr = conf_thr
        self.front_idx: Optional[int] = None
        self.clahe_enabled = configured_clahe()
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)) if self.clahe_enabled else None
        print(f'[Perception] Inference-only CLAHE: {self.clahe_enabled}')
        # Initialize CUDA/backend and select convolution algorithms before live frames.
        warmup = np.zeros((480, 640, 3), dtype=np.uint8)
        for _ in range(3):
            self.infer_front(warmup)
        actual = self.model.predictor.model
        if actual.fp16 or (USE_GPU and actual.device.type != 'cuda'):
            raise RuntimeError('Inference backend did not honor GPU FP32 policy')
        if USE_GPU:
            self._fast_predictor = self.model.predictor
            self.backend_name = 'pytorch-fp32-single-frame'
        print(f'[Perception] Active backend: {self.backend_name}; device={actual.device}; precision=FP32')


    def preprocessing_metadata(self):
        return dict(clahe_enabled=self.clahe_enabled, scope='inference_only',
                    color_channel='LAB L', clip_limit=2.0, tile_grid_size=[8, 8],
                    backend=self.backend_name, device=self.device, precision='float32',
                    quantization=False, pruning=False, tf32=False)

    @profiled('inference.photometric_preprocess')
    def _inference_image(self, color_img):
        """Photometric preprocessing on a separate buffer; retain optical coordinates."""
        if self._clahe is None:
            return color_img
        lab = cv2.cvtColor(color_img, cv2.COLOR_BGR2LAB)
        lab[:, :, 0] = self._clahe.apply(lab[:, :, 0])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def _resolve_front_idx(self, names) -> Optional[int]:
        """names에서 FRONT_CLASS_NAME의 인덱스를 찾는다. 클래스가 1개뿐이면 그것을 쓴다."""
        if self.front_idx is not None:
            return self.front_idx
        if not isinstance(names, dict) or not names:
            return None
        for k, v in names.items():
            if v == self.front_class_name:
                self.front_idx = int(k)
                return self.front_idx
        for k, v in names.items():
            if v in ('item', 'pallet'):
                self.front_idx = int(k)
                return self.front_idx
        if len(names) == 1:
            # 단일 클래스 모델이면 이름이 달라도 그 클래스가 대상이다
            self.front_idx = int(next(iter(names.keys())))
            return self.front_idx
        return None

    @profiled('perception.total', 'host_mixed')
    @torch.inference_mode()
    def infer_front(self, color_img) -> Tuple[bool, Optional[np.ndarray], Optional[tuple], Optional[np.ndarray]]:
        """
        color_img: (H, W, 3) BGR uint8
        return: (det_ok, kpts4 (4,2) float32 [좌상,우상,우하,좌하] 픽셀좌표, bbox_xyxy or None,
                 kpts_all (K,3) float32 전체 키포인트[x,y,vis] or None — visible PnP/시각화용)
        """
        H, W = color_img.shape[:2]

        # ---- 상태 출력 (추론 직전) — 고 fps 에서 터미널 I/O 가 병목이라 기본 off ----
        if VERBOSE_PERCEPTION:
            print("-----[Perception / Inference]-----")
            print(f"[Perception] Inference device : {self.device}")
            print("[Perception] Precision        : FP32")
            print("----------------------------------")

        pad = POSE_REFLECT_PADDING_PX
        if pad and color_img.shape != (480, 640, 3):
            raise ValueError('Cleanlabel requires a 640x480 BGR frame')
        inference_img = self._inference_image(color_img)
        with span('inference.reflect_padding', 'host_work'):
            inference_img = cv2.copyMakeBorder(inference_img, pad, pad, pad, pad, cv2.BORDER_REFLECT_101) if pad else inference_img
        if self._fast_predictor is not None:
            res = single_frame_result(self._fast_predictor, inference_img, self.conf_thr)
        else:
            with span('inference.legacy_predict', 'host_mixed'):
                res = self.model.predict(
                    source=inference_img,
                    imgsz=POSE_INFERENCE_IMGSZ,
                    augment=False,
                    device=self.device,
                    verbose=False,
                    conf=self.conf_thr,
                    quantize=None,
                )[0]

        return self._select_result(res, H, W, pad)

    @profiled('perception.selection_and_transfer', 'host_mixed')
    def _select_result(self, res, H, W, pad):
        det_ok = False
        kpts4 = None
        kpts_all = None
        bbox_now = None

        # 유효성 검사
        if res.boxes is None or len(res.boxes) == 0:
            if VERBOSE_PERCEPTION:
                print("[Perception] No boxes in result.")
            return det_ok, kpts4, bbox_now, kpts_all
        if res.keypoints is None or res.keypoints.data is None or len(res.keypoints.data) == 0:
            if VERBOSE_PERCEPTION:
                print("[Perception] No keypoints in result.")
            return det_ok, kpts4, bbox_now, kpts_all

        names = getattr(res, "names", None) or getattr(self.model, "names", None)
        front_idx = self._resolve_front_idx(names)
        if front_idx is None:
            print(f"[Perception] Could not resolve class index for '{self.front_class_name}'.")
            return det_ok, kpts4, bbox_now, kpts_all

        # One GPU-to-CPU transfer instead of four separate synchronizations.
        with span('inference.result_pack_launch', 'host_mixed'):
            packed_device = torch.cat((res.boxes.xyxy, res.boxes.conf[:, None], res.boxes.cls[:, None],
                                       res.keypoints.data.flatten(1)), dim=1).detach()
        with span('inference.results_to_cpu', 'device_transfer_wait'):
            packed = packed_device.cpu().numpy()
        classes = packed[:, 5].astype(int)
        confs = packed[:, 4].astype(float)
        xyxy = packed[:, :4].astype(float)
        kpts = packed[:, 6:].reshape(res.keypoints.data.shape).astype(np.float32)
        # Restore original optical coordinates BEFORE PnP, display and bbox clamp.
        xyxy -= pad
        kpts[:, :, :2] -= pad

        idxs: List[int] = [i for i, (c, p) in enumerate(zip(classes, confs))
                           if (c == front_idx and p >= self.conf_thr)]
        if not idxs:
            if VERBOSE_PERCEPTION:
                print(f"[Perception] No detection for class '{self.front_class_name}' over conf {self.conf_thr}.")
            return det_ok, kpts4, bbox_now, kpts_all

        # === 선택 기준 ===
        TARGET_AR = PALLET_FACE_W / PALLET_FACE_H
        def ar_distance(i: int) -> tuple:
            x1, y1, x2, y2 = xyxy[i]
            w = max(1.0, (x2 - x1))
            h = max(1.0, (y2 - y1))
            ar = w / h
            # 1순위: PnP 전면부 종횡비와의 차이 최소, 2순위: conf 큰 것
            return (abs(ar - TARGET_AR), -float(confs[i]))

        best_idx = (max(idxs, key=lambda i: float(confs[i]))
                    if POSE_INSTANCE_SELECTION == 'max_box_conf' else min(idxs, key=ar_distance))

        # 전면부 외곽 4모서리 좌표 추출.
        # visibility로 여기서 탈락시키지 않고 최종 성공 여부는 PnP가 결정한다.
        k = kpts[best_idx]
        kpts_all = k.astype(np.float32)   # (K,3) 전체 키포인트 — visible PnP/표시용
        if k.shape[0] <= max(POSE_FACE_KPTS):
            print(f"[Perception] Model has only {k.shape[0]} keypoints; need index {max(POSE_FACE_KPTS)}.")
            return det_ok, kpts4, bbox_now, kpts_all

        sel = k[list(POSE_FACE_KPTS)]                      # (4, 3)
        kpts4 = sel[:, :2].astype(np.float32)

        x1f, y1f, x2f, y2f = xyxy[best_idx]
        bbox_now = clamp_bbox(int(x1f), int(y1f), int(x2f), int(y2f), W, H)

        det_ok = True
        if VERBOSE_PERCEPTION:
            print(f"[Perception] Selected detection idx={best_idx}, conf={float(confs[best_idx]):.4f}")
        return det_ok, kpts4, bbox_now, kpts_all
