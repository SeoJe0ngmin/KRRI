from typing import Optional

__all__ = ['__version__', 'debug', 'cuda', 'git_version', 'hip', 'xpu']
__version__ = '2.8.0'
debug = False
cuda: Optional[str] = '12.6'
git_version = '3a67bf9c620e8958a1677e68779be08eb34dafa3'
hip: Optional[str] = None
xpu: Optional[str] = None
