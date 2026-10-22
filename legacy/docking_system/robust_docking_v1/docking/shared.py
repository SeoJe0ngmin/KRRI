"""Locate the shared configuration implementation alongside all versions."""
import importlib.util
from pathlib import Path
import sys


def manager():
    name = '_docking_system_common'
    if name not in sys.modules:
        path = Path(__file__).resolve().parents[2]/'common_config.py'
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules[name] = module
    return sys.modules[name]
