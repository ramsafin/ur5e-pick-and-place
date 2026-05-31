#!/usr/bin/env python3
"""CLI wrapper for static robot/MoveIt configuration validation."""

import sys

from ur5e_pick_and_place.config_validation import main

if __name__ == '__main__':
    sys.exit(main())
