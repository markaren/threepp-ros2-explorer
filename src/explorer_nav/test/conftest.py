"""pytest reads this file before it collects the tests.

It puts the package directory (the parent of this folder) on the import path, so that
`import explorer_nav` works straight from the source tree, without building the workspace.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
