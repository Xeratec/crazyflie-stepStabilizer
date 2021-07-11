import sys
import os
sys.path.append("../extern/pyvicon/")

import json
import logging
import keyboard
import time
import signal
import simpleaudio as sa

from datetime import datetime
from threading import Thread
from numpy import arctan2, arcsin

from stepStabilizer.stable_motion_commander import StableMotionCommander
from stepStabilizer.crazyflie_wrapper import CrazyFlieWrapper
from stepStabilizer.vicon_wrapper import ViconWrapper

import cflib

import coloredlogs
coloredlogs.install()

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Logger(Thread):
    def __init__(self, filename, log_config_vicon, log_config_crazyflie, log_period_vicon=20, log_period_crazyflie=100):
        Thread.__init__(self)

        self.zero_time = datetime.now()
        self.log_config_vicon = log_config_vicon
        self.log_config_crazyflie = log_config_crazyflie
        self.is_running = False
        self.cf = None
        self.mc = None
        self.vicon = None
        self.state = "TAKEOFF"

        signal.signal(signal.SIGINT, signal.SIG_DFL)
       
        cflib.crtp.init_drivers(enable_debug_driver=False)
        drones = cflib.crtp.scan_interfaces()
        logger.info("Found %s", drones)

        if not drones:
            logger.warning("No drone found!")
            return

        # Create vicon and crazyflie object
        if log_config_vicon is not None:
            self.vicon = ViconWrapper(ip="192.168.10.1", period=log_period_vicon, subjects=log_config_vicon, time0=self.zero_time, filename=filename)
        if log_config_crazyflie is not None:
            self.cf = CrazyFlieWrapper("radio://0/80/2M", log_list=log_config_crazyflie, sampling_period=log_period_crazyflie, time0=self.zero_time, filename=filename, data_logger = self)
        
        # Start threads
        if log_config_crazyflie is not None:
            self.cf.start()
        if log_config_vicon is not None:
            self.vicon.start()

        self.start()

    def run(self):
        if self.cf is None:
            return

        while(self.cf.is_connected != True):
            time.sleep(0.1)
            pass
    
        logger.info("Logging starting...")

        if self.log_config_vicon is not None:
            self.vicon.logging_enabled(1)

        if self.log_config_crazyflie is not None:
            self.is_running = True
            self.cf.logging_enabled(1)
            try:
                logger.info('Taking off!')
                self.cf.mc.take_off(0.5)
                time.sleep(2)
                self.state = "FLY"
                time.sleep(2)

                logger.info('Moving forward')
                self.cf.mc.forward(1, velocity=0.1)
                time.sleep(1)
                self.state = "LAND"

                self.cf.mc.turn_left(180)

            except Exception:
                logger.exception("Illegal command")
                pass
           
        if self.is_running:
            self.is_running = False
            self.stop()

    def stop(self):
        if self.cf is not None:
            logger.info('Landing!')
            self.is_running = False
            while self.cf.mc._is_flying:
                self.cf.mc.land()
                time.sleep(1)

def main():

    # Variables to log from CF
    # log_config_crazyflie = ["stateEstimate.z", "acc.z", "stateEstimate.vz", "posCtl.targetZ", "range.zrange", "stepstabilizer.TOFslope", "stepstabilizer.ACCZslope"]
    log_config_crazyflie = ["stateEstimate.z", "stateEstimate.vz", "posCtl.targetZ", "acc.z", "range.zrange"]

    ## run function in the background
    data_logger = Logger(filename=os.path.join("logs", sys.argv[1]+"_"), log_config_vicon=log_config_vicon, log_config_crazyflie=log_config_crazyflie)
    
    if data_logger.cf is None:
        exit(1)

    # Wait for esablished connection
    while(data_logger.cf.is_connected != True):
        time.sleep(0.1)
        pass
    
    # Wait for start of flight
    while(data_logger.is_running != True):
        time.sleep(0.1)
        pass

    while data_logger.cf.mc._is_flying:
        time.sleep(0.1)
        if keyboard.is_pressed('q'):
            logger.warning("Aborting...")
            data_logger.stop()
            time.sleep(10)
            break

    time.sleep(.5)
    
    if log_config_vicon is not None:
        data_logger.vicon.logging_enabled(0)
        data_logger.vicon.save_log()

    if log_config_crazyflie is not None and data_logger.cf is not None:
        data_logger.cf.logging_enabled(0)     
        data_logger.cf.save_log()


if __name__ == '__main__':
    main()
