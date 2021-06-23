import sys
import os
sys.path.append("../extern/pyvicon/")

import json
import keyboard
import time
import simpleaudio as sa

from datetime import datetime
from threading import Thread
from numpy import arctan2, arcsin

from stepStabilizer.stable_motion_commander import StableMotionCommander
from stepStabilizer.crazyflie_wrapper import CrazyFlieWrapper
from stepStabilizer.vicon_wrapper import ViconWrapper

import cflib

class Logger(Thread):
    def __init__(self, filename, log_config_vicon, log_config_crazyflie, log_period_vicon=20, log_period_crazyflie=50):
        Thread.__init__(self)
        self.zero_time = datetime.now()
        self.log_config_vicon = log_config_vicon
        self.log_config_crazyflie = log_config_crazyflie
        self.is_running = False
        self.cf = None
        self.mc = None
        self.vicon = None
        self.state = "TAKEOFF"
       
        cflib.crtp.init_drivers(enable_debug_driver=False)
        drones = cflib.crtp.scan_interfaces()
        print("[LOG] Found ", drones)

        if not drones:
            print("[LOG] No drone found!")
            return

        # Create vicon and crazyflie object
        if log_config_vicon is not None:
            self.vicon = ViconWrapper(ip="192.168.10.1", period=log_period_vicon, subjects=log_config_vicon, time0=self.zero_time, filename=filename)
        if log_config_crazyflie is not None:
            self.cf = CrazyFlieWrapper("radio://0/80/2M", log_list=log_config_crazyflie, sampling_period=log_period_crazyflie, time0=self.zero_time, filename=filename, logger = self)
            
        if log_config_crazyflie is not None:
            self.cf.start()
        if log_config_vicon is not None:
            self.vicon.start()

        self.start()

    def run(self):
       
        while(self.cf.is_connected != True):
            pass
    
        print("[LOG] Logging starting...")

        if self.log_config_vicon is not None:
            self.vicon.logging_enabled(1)

        if self.log_config_crazyflie is not None:
            self.is_running = True
            self.cf.logging_enabled(1)
            try:
                print('[LOG] Taking off!')
                self.cf.mc.take_off()
                time.sleep(1)
                self.state = "FLY"

                print('[LOG] Moving forward')
                self.cf.mc.forward(1.5)
                self.state = "LAND"

            except Exception:
                print("[LOG] Illegal command")
                pass
           
        if self.is_running:
            self.is_running = False
            self.stop()

    def stop(self):
        if self.cf is not None:
            print('[LOG] Landing!')
            self.is_running = False
            while self.cf.mc._is_flying:
                self.cf.mc.land()
                time.sleep(1)

if __name__ == '__main__':
    #log_config_vicon = ["Drone", "Box"]
    log_config_vicon = None

    # Variables to log from CF
    log_config_crazyflie = ["stateEstimate.z", "stateEstimate.vz", "gyro.z", "posCtl.targetZ", "range.zrange"]

    ## run function in the background
    logger = Logger(filename=os.path.join("logs", sys.argv[1]), log_config_vicon=log_config_vicon, log_config_crazyflie=log_config_crazyflie)
    
    ## will not exit if function finishes, only when
    ## "q" is entered, but this is just a simple example

    # Wait for esablished connection
    while(logger.cf.is_connected != True):
        pass
    
    # Wait for start of flight
    while(logger.is_running != True):
            pass

    time.sleep(0.1)

    while logger.cf.mc._is_flying:
        if keyboard.is_pressed('q'):
            print("Aborting...")
            logger.stop()
            time.sleep(1)
            break

    if log_config_vicon is not None:
        logger.vicon.logging_enabled(0)
        logger.vicon.save_log()

    if log_config_crazyflie is not None and logger.cf is not None:
        logger.cf.logging_enabled(0)     
        logger.cf.save_log()
