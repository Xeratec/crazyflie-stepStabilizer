import collections
import math
import numpy as np

from scipy.stats import linregress

VERBOSE = False

class StepDetector():
    def __init__(self, num_points=8):
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
        self.z_acc_slope  = 0
        self.step_detector = 0

    def get_offset(self): 
        if self.z_acc_slope != 0:
            self.step_detector = 1E6*self.z_range_slope/(abs(self.z_acc_slope)+20)
        else:
            self.step_detector = 0
        
        # if self.step_detector > 5000 and self.z_offset >= 0:
        #     self.z_offset = -0.15

        # if self.step_detector < -5000 and self.z_offset < 0:
        #     self.z_offset = 0

        if VERBOSE:
            print(np.array(self.z_range_timestamps), np.array(self.z_range_data), np.array(self.z_gyro_timestamps), np.array(self.z_gyro_data))

        return self.z_offset, self.step_detector
    
    def update_z_range(self, timestamp, value):     
        self.z_range_data.append(value)
        self.z_range_timestamps.append(timestamp)
        if len(self.z_range_data) < self.num_points:
            return 0
        self.z_range_slope = linregress(np.array(self.z_range_timestamps), np.array(self.z_range_data))[0]
        
        return self.z_range_slope

    def update_z_acc(self, timestamp, value):     
        self.z_acc_data.append(value)
        self.z_acc_timestamps.append(timestamp)
        if len(self.z_range_data) < self.num_points:
            return 0
        self.z_acc_slope = linregress(np.array(self.z_acc_timestamps), np.array(self.z_acc_data))[0]
        
        return self.z_acc_slope

    def update_z_gyro(self, timestamp, value):
        self.z_gyro_data.append(value)
        self.z_gyro_timestamps.append(timestamp)

        if len(self.z_gyro_data) < self.num_points:
            return 0
        self.z_gyro_slope  = linregress(np.array(self.z_gyro_timestamps), np.array(self.z_gyro_data))[0]

        return self.z_gyro_slope     
       
