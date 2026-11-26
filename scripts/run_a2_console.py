#!/usr/bin/env python3
"""Run the A2 operator console from the source workspace."""

from pathlib import Path
import sys


# 离线包沿用工作区目录结构。上位机加载可选二维图时复用已有的栅格解析器。
source = Path(__file__).resolve().parents[1] / "src"
sys.path[:0] = [str(source / "a2_gateway"), str(source / "a2_navigation")]

from a2_gateway.server import main


if __name__ == "__main__":
    sys.exit(main())
