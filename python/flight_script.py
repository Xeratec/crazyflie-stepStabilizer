import logging
import sys
sys.path.append("../extern/pyvicon/")

import os
import json
import argparse
import coloredlogs
import keyboard
import signal
import time
import simpleaudio as sa
import cflib

from datetime import datetime
import threading
from threading import Thread
from numpy import arctan2, arcsin

from stepStabilizer.stable_motion_commander import StableMotionCommander
from stepStabilizer.crazyflie_wrapper import CrazyFlieWrapper
from stepStabilizer.vicon_wrapper import ViconWrapper


import cfclient
from cfclient.utils.config import Config
from cfclient.utils.config_manager import ConfigManager

coloredlogs.install()
logger = logging.getLogger(__name__)

class Logger(Thread):
    def __init__(self, filename, log_config_vicon, log_config_crazyflie, uri="radio://0/80/2M",  log_period_vicon=20, log_period_crazyflie=100):
        Thread.__init__(self)
        self.name = "Logger"
        self.zero_time = datetime.now()
        self.log_config_vicon = log_config_vicon
        self.log_config_crazyflie = log_config_crazyflie
        self.log_period_vicon = log_period_vicon
        self.log_period_crazyflie = log_period_crazyflie
        self.filename = filename 
        self.flight_script = None
        self.uri = uri
        self.is_running = False
        self.cf = None
        self.mc = None
        self.vicon = None
        self.state = "TAKEOFF"

        # Create vicon and crazyflie object
        self.vicon = ViconWrapper(ip="192.168.10.1", period=self.log_period_vicon, subjects=self.log_config_vicon, time0=self.zero_time, filename=self.filename)
        self.cf = CrazyFlieWrapper(self.uri, log_list=self.log_config_crazyflie, sampling_period=self.log_period_crazyflie, time0=self.zero_time, filename=self.filename, data_logger = self)

    def set_flight_script(self, script):
        self.flight_script = script

    def run(self):
         # Start threads
        if self.log_config_crazyflie is not None:
            self.cf.start()
        if self.log_config_vicon is not None:
            self.vicon.start()

        while(self.cf.is_connected != True):
            time.sleep(0.1)
            pass
    
        logger.info("Logging starting...")

        self.is_running = True

        self.vicon.logging_enabled(1)
        self.cf.logging_enabled(1)

        if self.flight_script:
            try:
                self.flight_script(self.cf.mc, self.state)
                self.cf.logging_enabled(0)
                self.vicon.logging_enabled(0)
                self.cf.save_log()
                self.vicon.save_log()
            except Exception:
                logger.exception("Illegal command")
                pass
        else:
            while self.is_running:
                time.sleep(0.1)

            logging.warning("Exit")
            self.cf.logging_enabled(0)
            self.vicon.logging_enabled(0)
            self.cf.save_log()
            self.vicon.save_log()

def main():
    parser = argparse.ArgumentParser(prog="flight_script")
    parser.add_argument("log_path", action="store", type=str,
                        default="./logs/test",
                        help="Path to save log file, "
                             "defaults to ./logs/test")
    parser.add_argument("--sv", action="extend", nargs="+", type=str, dest="log_config_vicon",
                        default=None,
                        help="Vicon log entry to save, "
                             "defaults to None")
    parser.add_argument("--sc", action="extend", nargs="+", type=str, dest="log_config_crazyflie",
                        default=["stateEstimate.z", "stateEstimate.vz", "posCtl.targetZ", "acc.z", "range.zrange"],
                        help="Crazyflie log entry to save, "
                             "defaults to stateEstimate.z, stateEstimate.vz, posCtl.targetZ, acc.z, range.zrange")
    parser.add_argument("-u", "--uri", action="store", dest="uri", type=str,
                        default="radio://0/80/2M",
                        help="URI to use for connection to the Crazyradio"
                             " dongle, defaults to radio://0/80/2M")
    parser.add_argument("-i", "--input", action="store", dest="input",
                        type=str, default="PS3_Mode_1",
                        help="Input mapping to use for the controller,"
                             "defaults to PS3_Mode_1")
    parser.add_argument("-d", "--debug", action="store_true", dest="debug",
                        help="Enable debug output")
    parser.add_argument("-c", "--controller", action="store", type=int,
                        dest="controller", default=-1,
                        help="Use controller with specified id,"
                             " id defaults to 0")
    parser.add_argument("--controllers", action="store_true",
                        dest="list_controllers",
                        help="Only display available controllers and exit")
    parser.add_argument("--stepStabilizer", action="store_true",
                        dest="enableStepStabilizer",
                        help="Enable python step stabilizer algorithm")

    (args, unused) = parser.parse_known_args()

    if args.debug:
        logging.basicConfig(level=logging.INFO)
        logger.setLevel(logging.DEBUG)
        logger.info(args)
    else:
        logging.basicConfig(level=logging.WARN)
        logger.setLevel(logging.WARN)

    ## run function in the background
    data_logger = Logger(filename=args.log_path, 
        log_config_vicon=args.log_config_vicon, 
        log_config_crazyflie=args.log_config_crazyflie,
        uri=args.uri)
    
    if (args.list_controllers):
        data_logger.cf.list_controllers()
    else:
        if args.controller != -1:
            if data_logger.cf.controller_connected():
                data_logger.cf.setup_controller(input_config=args.input, input_device=args.controller)
                data_logger.start()
                while data_logger.isAlive:
                    try:
                        data_logger.join(1)
                    except KeyboardInterrupt:
                        # Ctrl-C handling and send kill to threads
                        data_logger.is_running = False
                        time.sleep(1)
                        os.kill(os.getpid(), signal.SIGTERM)
                
                   
            else:
                print("No input-device connected, using flight script!")
        else:
            data_logger.set_flight_script(flight_script)
            data_logger.start()

        
       
            
def flight_script(cf_commander, state):
    logger.info('Taking off!')
    state = "TAKEOFF"
    cf_commander.take_off(0.5)
    
    time.sleep(2)

    logger.info('Moving forward')
    state = "FLY"
    time.sleep(2)
    cf_commander.forward(1, velocity=0.1)
    time.sleep(1)

    logger.info('Landing!')
    state = "LAND"
    cf_commander.land()
    time.sleep(1)


if __name__ == '__main__':
    main()
