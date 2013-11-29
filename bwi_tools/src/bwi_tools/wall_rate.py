#!/usr/bin/env python

import time

class WallRate():

    def __init__(self, rate):
        self.rate = rate
        self.period = 1.0 / rate if rate > 0.0 else 0.0
        self.recorded_time = time.time() 

    def sleep(self):
        current_time = time.time()
        elapsed = current_time - self.recorded_time
        if self.period - elapsed > 0:
            time.sleep(self.period - elapsed)
        self.recorded_time = time.time()
