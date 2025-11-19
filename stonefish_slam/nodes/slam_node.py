#!/usr/bin/env python3

import sys
from stonefish_slam.core.slam_ros import main

def main_wrapper():
    sys.exit(main())

if __name__ == "__main__":
    main_wrapper()
