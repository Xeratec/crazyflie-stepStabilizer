import collections
import math
import numpy as np
import logging
import time

from scipy.stats import linregress

logger = logging.getLogger(__name__)

class StepDetector():
    def __init__(self, num_points=8, step_delay = 2, step_agressivity = 1.5):
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
            self.step_detector = self.z_range_slope/(abs(self.z_acc_slope)+20)
        else:
            self.step_detector = 0
        
        if self.step_detector < -self.step_agressivity and self.z_offset >= 0 and (time.time() - self.step_time) >= self.step_delay:
            self.step_time = time.time() 
            logger.warning("Step Detected: %f", self.step_detector)
            self.z_offset = -0.11

        if self.step_detector > self.step_agressivity and self.z_offset < 0 and (time.time() - self.step_time) >= self.step_delay:
            self.step_time = time.time() 
            logger.warning("Step Detected: %f", self.step_detector)
            self.z_offset = 0
        
        logger.debug(np.array(self.z_range_timestamps), np.array(self.z_range_data), np.array(self.z_gyro_timestamps), np.array(self.z_gyro_data))

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
       
