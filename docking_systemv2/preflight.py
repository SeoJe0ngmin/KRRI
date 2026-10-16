"""Convenience entry point; never opens CAN."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent/'robust_docking_v5'))
if __name__=='__main__':
    from docking.preflight import main
    raise SystemExit(main())
