# File: /Omega-Code/servers/robot_controller_backend/utils/threading_control.py

"""
Threading Control Script

Provides utilities to safely stop threads by raising exceptions within them.
Use with caution: forcibly killing threads can leave resources in an uncertain state.

Functions:
- async_raise: Raises an exception in a thread by its thread ID.
- stop_thread: Requests a thread to exit.
- test: Test function to demonstrate continuous output and stopping.
"""

import threading
import time
import inspect
import ctypes
from typing import Type

def async_raise(tid: int, exctype: Type[BaseException]) -> None:
    """
    Raises an exception in the threads with id tid.

    Args:
        tid (int): Thread ID.
        exctype (Exception): The Exception type to be raised.
    Raises:
        ValueError: If tid is not a valid thread id.
        SystemError: If the exception couldn't be raised.
    """
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(tid),
        ctypes.py_object(exctype)
    )
    if res == 0:
        raise ValueError("Invalid thread id")
    elif res != 1:
        # if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect
        ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(tid), None)
        raise SystemError("PyThreadState_SetAsyncExc failed")

def stop_thread(thread: threading.Thread, attempts: int = 5) -> None:
    """
    Stops the given thread by raising SystemExit in it.

    Args:
        thread (threading.Thread): The thread to stop.
        attempts (int): How many times to try raising the exception.
    """
    for _ in range(attempts):
        async_raise(thread.ident, SystemExit)

def test_worker() -> None:
    """
    A simple worker function that prints every second.
    """
    try:
        while True:
            print('-------')
            time.sleep(1)
    except SystemExit:
        print("[Thread] Received SystemExit. Exiting cleanly.")

def _test() -> None:
    """
    Demonstrates starting and forcibly stopping a thread.
    """
    t = threading.Thread(target=test_worker)
    t.start()
    time.sleep(5)
    print("[Main] Stopping worker thread now.")
    stop_thread(t)
    print("[Main] Worker thread stopped.")

if __name__ == "__main__":
    _test()
