#!/usr/bin/env python3
"""Ping CLI. Thin wrapper: calls ping_impl.ping() and prints as JSON."""

import json
import sys

from ping_impl import ping


def main() -> None:
    print(json.dumps(ping()))
    sys.exit(0)


if __name__ == "__main__":
    main()
