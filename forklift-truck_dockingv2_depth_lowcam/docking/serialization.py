"""Explicit JSON conversion for NumPy sensor values; unknown objects still fail."""
import numpy as np


def numpy_json(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f'Object of type {type(value).__name__} is not JSON serializable')
