"""
Threading Control Script

This script provides utilities to safely stop threads by raising exceptions in them.

Functions:
- _async_raise: Raises an exception in a thread.
- stop_thread: Stops a given thread.
- test: A test function that prints continuously.
"""

import threading
import time
import inspect
import ctypes

def _async_raise(tid, exctype):
    """
    Raises an exception in the given thread.

    Parameters:
    tid (int): Thread ID.
    exctype (Exception): Exception type to be raised.
    """
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")

def stop_thread(thread):
    """
    Stops the given thread.

    Parameters:
    thread (Thread): Thread to be stopped.
    """
    for i in range(5):
        _async_raise(thread.ident, SystemExit)

def test():
    """
    A test function that prints continuously.
    """
    while True:
        print('-------')
        time.sleep(1)

if __name__ == "__main__":
    t = threading.Thread(target=test)
    t.start()
    time.sleep(5)
    print("main thread sleep finish")
    stop_thread(t)
