import collections
import math
import numpy as np

from scipy.stats import linregress

VERBOSE = False

class StepDetector():
    def __init__(self, num_points=5):
        # Number of datapoints to consider
        self.num_points = 5

        self.data = collections.deque(maxlen=num_points)
        self.timestamps =  collections.deque(maxlen=num_points)

        self.offset = 0

    def get_offset(self): 
        return self.offset
    
    def update_offset(self, timestamp, value):
        z_slope = self._update_model(timestamp, value)

        return self.offset, z_slope

    def _update_model(self, timestamp, value):
        if len(self.data) == 0:
            for x in range(self.num_points):
                self.data.append(value)
                self.timestamps.append(timestamp)
            return 0

        self.timestamps.append(timestamp)
        self.data.append(value)

        if VERBOSE:
            print(np.array(self.timestamps), np.array(self.data))

        slope = linregress(np.array(self.timestamps), np.array(self.data))[0]

        # if slope > 0.4:
        #     self.offset = -0.3
        #     print("[SD] ", slope)
        
        if slope < -0.1:
             self.offset = -0.1
             print("[SD] ", slope)

        return slope
