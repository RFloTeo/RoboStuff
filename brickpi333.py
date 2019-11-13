#!/usr/bin/env python

from brickpi3 import *

class BrickPi333(BrickPi3):
    def __init__(self):
        super().__init__()

    def __del__(self):
        self.reset_all()
