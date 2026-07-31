#!/usr/bin/env python3
"""Ping script used by pyrunner tests to verify Go->Python invocation."""

import json
import os
import sys


def main() -> None:
    print(json.dumps({"ok": True, "pid": os.getpid()}))
    sys.exit(0)


if __name__ == "__main__":
    main()
