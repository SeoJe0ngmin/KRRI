"""Keep old measurements intact; new results are grouped by system/version."""
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SYSTEM_ROOT = os.path.dirname(PROJECT_ROOT)
WORK_ROOT = os.path.join(os.path.dirname(SYSTEM_ROOT), 'work', 'docking_system', os.path.basename(PROJECT_ROOT))
