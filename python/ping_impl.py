"""Ping impl. Reusable function returning process info as a dict.

Structured as impl + CLI (see ping.py) to keep future migration to a
persistent per-calibration subprocess cheap — the dispatcher there just
imports ping() directly instead of exec-ing ping.py.
"""

import os


def ping() -> dict:
    return {"ok": True, "pid": os.getpid()}
