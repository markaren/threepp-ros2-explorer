"""Let pytest find the package without building the workspace first.

After `colcon build` the package is installed and importable anyway. This
makes `python -m pytest test` work straight from the source tree as well.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
