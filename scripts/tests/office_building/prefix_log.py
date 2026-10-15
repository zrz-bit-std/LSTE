#!/usr/bin/env python3
"""Prefix tmux pane output with the benchmark logging contract."""

from __future__ import annotations

import argparse
import datetime
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--process", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("a", encoding="utf-8", buffering=1) as stream:
        for raw_line in sys.stdin:
            line = raw_line.rstrip("\r\n")
            timestamp = datetime.datetime.now().astimezone().isoformat(timespec="milliseconds")
            stream.write(
                "%s [INFO] process=%s event=stdout line=%s\n"
                % (timestamp, args.process, line)
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
