"""Filesystem model selection before importing FSM/camera/CAN modules."""
import hashlib
import json
import os
from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parent / 'models'
MODEL_ENV = 'LIFTER_POSE_MODEL'
CLAHE_ENV = 'LIFTER_INFERENCE_CLAHE'
DEMO_ENV = 'LIFTER_DEMO_MODE'


def configured_demo_mode():
    return os.environ.get(DEMO_ENV, '0') == '1'


def configured_clahe():
    return os.environ.get(CLAHE_ENV, '0') == '1'

DEFAULT_MODEL = 'green_pallet_finetune_60epochs (1).pt'


def list_models(folder=MODEL_DIR):
    return sorted((p.resolve() for p in Path(folder).glob('*.pt')
                   if p.is_file() and p.stat().st_size > 0), key=lambda p:p.name.casefold())


def resolve_model(name, folder=MODEL_DIR):
    path = Path(name)
    if not path.is_absolute():
        path = Path(folder) / path
    path = path.resolve()
    if path.parent != Path(folder).resolve() or path.suffix.lower() != '.pt':
        raise ValueError('Select a .pt file directly inside the models folder')
    if not path.is_file() or path.stat().st_size <= 0:
        raise ValueError(f'Model is missing or empty: {path}')
    return path


def configured_model(legacy_root):
    name = os.environ.get(MODEL_ENV)
    return resolve_model(name) if name else Path(legacy_root)/'pallet_yolo26n_pose_cleanlabel.pt'


def validate_pose_model(path):
    # Shape/class validation before device startup; never download missing weights.
    os.environ.setdefault('YOLO_AUTOINSTALL', 'false')
    from ultralytics import YOLO
    path = resolve_model(path)
    model = YOLO(str(path))
    shape = list(getattr(model.model.model[-1], 'kpt_shape', []))
    if model.task != 'pose' or shape != [9, 3]:
        raise ValueError(f'FSM requires a pose model with 9x3 keypoints; got {model.task}, {shape}')
    names = model.names
    if len(names) != 1 and not any(n in ('item', 'pallet') for n in names.values()):
        raise ValueError('Model has no item/pallet class')
    return dict(name=path.name,path=str(path),sha256='sha256:'+hashlib.sha256(path.read_bytes()).hexdigest(),
                task=model.task,keypoint_shape=shape,classes=names)


def choose_model(*, allow_demo=True):
    import tkinter as tk
    from tkinter import ttk, messagebox
    root=tk.Tk();root.title('FSM v4 - 추론 모델 선택');root.geometry('720x280')
    selected=tk.StringVar();result=[]
    root.geometry('720x400')
    clahe_enabled=tk.BooleanVar(value=configured_clahe())
    demo_enabled=tk.BooleanVar(value=configured_demo_mode() if allow_demo else False)
    ttk.Label(root,text='실행할 파렛트 인식 모델을 선택하세요',font=('',13)).pack(pady=14)
    ttk.Label(root,text=str(MODEL_DIR),wraplength=690).pack(pady=5)
    combo=ttk.Combobox(root,textvariable=selected,state='readonly',width=76);combo.pack(pady=12)
    ttk.Checkbutton(root, text='CLAHE 사용 (모델 추론 입력에만 적용)',
                    variable=clahe_enabled).pack(pady=6)
    if allow_demo:
        ttk.Checkbutton(root, text='시연 모드 (실행 화면·녹화·파일 로그 끄기)',
                        variable=demo_enabled).pack(pady=6)
        ttk.Label(root, text='시연 모드: 자동 제어 실행 / 완료·실패 시 종료').pack()
    else:
        ttk.Label(root, text='독립 실행: 연결이 끊겨도 젯슨에서 계속 실행하며 녹화·로그를 저장합니다.').pack(pady=6)
    def refresh():
        values=[p.name for p in list_models()];combo['values']=values
        if selected.get() not in values:
            selected.set(DEFAULT_MODEL if DEFAULT_MODEL in values else (values[0] if values else ''))
    def launch():
        try:path=resolve_model(selected.get())
        except ValueError as exc:messagebox.showerror('모델 선택',str(exc));return
        os.environ[CLAHE_ENV] = '1' if clahe_enabled.get() else '0'
        os.environ[DEMO_ENV] = '1' if demo_enabled.get() else '0'
        result.append(path);root.destroy()
    buttons=ttk.Frame(root);buttons.pack(pady=8)
    ttk.Button(buttons,text='목록 새로고침',command=refresh).pack(side='left',padx=8)
    ttk.Button(buttons,text='모델 폴더 열기',command=lambda:os.startfile(MODEL_DIR)).pack(side='left',padx=8)
    ttk.Button(buttons,text='선택한 모델로 실행',command=launch).pack(side='left',padx=8)
    ttk.Button(buttons,text='취소',command=root.destroy).pack(side='left',padx=8)
    ttk.Label(root,text='동일한 9개 키포인트 순서의 Pose .pt 모델을 이 폴더에 추가하면 목록에 나타납니다.').pack(pady=8)
    refresh();root.mainloop()
    return result[0] if result else None
