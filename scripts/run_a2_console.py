#!/usr/bin/env python3
"""Run the A2 operator console from the source workspace."""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "a2_gateway"))

from a2_gateway.server import main


if __name__ == "__main__":
    sys.exit(main())
