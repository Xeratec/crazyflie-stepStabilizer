#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# step_detection.py
# Philip Wiese <wiesep@student.ethz.ch>
# Luca Rufer <lrufer@student.ethz.ch>
#
# Copyright (C) 2021 ETH Zurich
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. 
#

import collections
import math
import numpy as np
import logging
import time

from scipy.stats import linregress

logger = logging.getLogger(__name__)

class StepDetector():
    def __init__(self, num_points=8, step_delay = 1, step_agressivity = 100):
        # Number of datapoints to consider
        self.num_points = num_points

        self.z_range_data       = collections.deque(maxlen=num_points)
        self.z_range_timestamps = collections.deque(maxlen=num_points)
        self.z_gyro_data        = collections.deque(maxlen=num_points)
        self.z_gyro_timestamps  = collections.deque(maxlen=num_points)
        self.z_acc_data         = collections.deque(maxlen=num_points)
        self.z_acc_timestamps   = collections.deque(maxlen=num_points)

        self.z_offset      = 0
        self.z_range_slope = 0
        self.z_gyro_slope  = 0
        self.z_acc_slope   = 0
        self.step_detector = 0

        self.step_time    = time.time()
        self.step_delay   = step_delay
        self.step_agressivity = step_agressivity

    def get_offset(self): 
        if self.z_acc_slope != 0:
            self.step_detector = self.z_range_slope/(abs(self.z_acc_slope)+1)
        else:
            self.step_detector = 0
        
        if self.step_detector < -self.step_agressivity and self.z_offset >= 0 and (time.time() - self.step_time) >= self.step_delay:
            self.step_time = time.time() 
            self.z_offset = -0.30
            logger.warning("Step Detected: %f (Offset: %f)", self.step_detector, self.z_offset)

        if self.step_detector > self.step_agressivity and self.z_offset < 0 and (time.time() - self.step_time) >= self.step_delay:
            self.step_time = time.time() 
            self.z_offset = 0
            logger.warning("Step Detected: %f (Offset: %f)", self.step_detector, self.z_offset)

        
        return self.z_offset, self.step_detector
    
    def update_z_range(self, timestamp, value):     
        self.z_range_data.append(value)
        self.z_range_timestamps.append(timestamp)
        if len(self.z_range_data) < self.num_points:
            return 0
        self.z_range_slope = 1E3*linregress(np.array(self.z_range_timestamps), np.array(self.z_range_data))[0]
        
        return self.z_range_slope

    def update_z_acc(self, timestamp, value):     
        self.z_acc_data.append(value)
        self.z_acc_timestamps.append(timestamp)
        if len(self.z_range_data) < self.num_points:
            return 0
        self.z_acc_slope = 1E3*linregress(np.array(self.z_acc_timestamps), np.array(self.z_acc_data))[0]
        
        return self.z_acc_slope

    def update_z_gyro(self, timestamp, value):
        self.z_gyro_data.append(value)
        self.z_gyro_timestamps.append(timestamp)

        if len(self.z_gyro_data) < self.num_points:
            return 0
        self.z_gyro_slope  = 1E3*linregress(np.array(self.z_gyro_timestamps), np.array(self.z_gyro_data))[0]

        return self.z_gyro_slope     
       
