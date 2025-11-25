"""
I/O and performance utilities for stonefish_slam.

Provides:
- add_lock: Callback locking for offline bag replay
- CodeTimer: Performance measurement context manager
"""
import timeit
from functools import wraps
from threading import Event


# Offline mode flag (for bag file replay)
offline = False
callback_lock_event = Event()
callback_lock_event.set()


def add_lock(callback):
    """
    Lock decorator for callback functions, useful for running ROS offline with bag files.

    Forces callback functions to execute sequentially, allowing matplotlib plots
    or other blocking operations during bag replay.

    Args:
        callback: Function to decorate

    Returns:
        Wrapped callback with locking mechanism
    """
    @wraps(callback)
    def lock_callback(*args, **kwargs):
        if not offline:
            callback(*args, **kwargs)
        else:
            callback_lock_event.wait()
            callback_lock_event.clear()
            callback(*args, **kwargs)
            callback_lock_event.set()

    return lock_callback


class CodeTimer(object):
    """Timer class used with `with` statement

    - Disable output by setting CodeTimer.silent = False
    - Change log_func to print/tqdm.write/rospy.loginfo/etc

    with CodeTimer("Some function"):
        some_func()

    """

    silent = False

    def __init__(self, name="Code block"):
        self.name = name

    def __enter__(self):
        """Start measuring at the start of indent"""
        if not CodeTimer.silent:
            self.start = timeit.default_timer()

    def __exit__(self, exc_type, exc_value, traceback):
        """
            Stop measuring at the end of indent. This will run even
            if the indented lines raise an exception.
        """
        if not CodeTimer.silent:
            self.took = timeit.default_timer() - self.start
            msg = "{} : {:.5f} s".format(self.name, float(self.took))
            print(f"[DEBUG] {msg}")
