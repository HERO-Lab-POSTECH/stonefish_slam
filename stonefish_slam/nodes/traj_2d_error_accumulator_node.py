#!/usr/bin/env python3

import sys
from stonefish_slam.core.traj_2d_error_accumulator import main

def main_wrapper():
    sys.exit(main())

if __name__ == "__main__":
    main_wrapper()
