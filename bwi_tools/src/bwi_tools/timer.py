#!/usr/bin/env python

import time
import threading

class Timer:

    def __init__(self, period = 1.0):
        self.timer_lock = threading.Lock()
        self.total_time = 0.0
        self.time_remaining = 0.0
        self.callback = None
        self.start_time = 0.0
        self.timer = None
        self.period = period

    def start(self, total_time, callback):
        self.timer_lock.acquire()
        success = False
        if not self.timer:
            self.timer = threading.Timer(self.period, self.tick)
            self.start_time = time.time()
            self.total_time = total_time
            self.time_remaining = total_time
            self.callback = callback
            self.timer.start()
            success = True
        self.timer_lock.release()
        return success

    def tick(self):
        callback = None
        self.timer_lock.acquire()
        if self.timer: # check in case cancel timer has been called
            current_time = time.time() 
            self.time_remaining = \
                    self.total_time - (current_time - self.start_time)
            if self.time_remaining <= 0.0:
                callback = self.callback
                self.reset_vars()
            else:
                self.timer = threading.Timer(self.period, self.tick)
                self.timer.start()
        self.timer_lock.release()
        if callback:
            callback()

    def cancel(self):
        self.timer_lock.acquire()
        success = False
        if self.timer:
            self.timer.cancel()
            self.reset_vars()
            success = True
        self.timer_lock.release()
        return success

    def time(self):
        return round(self.time_remaining)

    def reset_vars(self):
        self.total_time = 0.0
        self.time_remaining = 0.0
        self.callback = None
        self.start_time = 0.0
        self.timer = None

