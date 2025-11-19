#!/usr/bin/env python3

import sys
from stonefish_slam.feature_extraction_sim import main

def main_wrapper():
    sys.exit(main())

if __name__ == "__main__":
    main_wrapper()
