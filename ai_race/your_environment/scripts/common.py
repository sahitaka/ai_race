#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

class CmdManager:

    def __init__(self, max_val, min_val, n):
        self.max_val = max_val
        self.min_val = min_val
        self.resolution = (max_val - min_val)/(n-1)
        self.discretization = n
        self.cmd_list = np.arange(self.min_val, self.max_val+self.resolution, self.resolution)
        print self.cmd_list

    def resample_cmd(self, cmd):
        enum = self.cmd_to_enum(cmd)
        return self.enum_to_cmd(enum)

    def cmd_to_enum(self, cmd):
        return int(cmd*((self.discretization-1)/2)+((self.discretization-1)/2))
        # return np.argmin(abs(self.cmd_list - cmd))

    def resample_to_enum(self, resample):
        return self.cmd_to_enum(resample)

    def enum_to_cmd(self, enum):
        return float(float(enum-1)-((self.discretization-1)/2))/((self.discretization-1)/2)
        # return self.cmd_list[enum]


