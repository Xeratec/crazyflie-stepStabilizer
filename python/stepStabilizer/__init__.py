#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
The Crazyflie Micro Quadcopter stepStabilizer.
"""

import logging
import time
import sys

from threading import Thread
from datetime import datetime

from .crazyflie_wrapper import CrazyFlieWrapper
from .stable_motion_commander import StableMotionCommander
from .vicon_wrapper import ViconWrapper
from .step_detecition import StepDetector

logger = logging.getLogger(__name__)

class MainCommunication(Thread):
    def __init__(self, 
        filename, 
        log_config_vicon, 
        log_config_crazyflie,
        algorithm = 0,
        uri="radio://0/80/2M",  
        log_period_vicon=50, 
        log_period_crazyflie=50
    ):

        Thread.__init__(self)

        self.name                   = "MainCommunication"

        # Parameters
        self.filename               = filename 
        self.log_config_vicon       = log_config_vicon
        self.log_config_crazyflie   = log_config_crazyflie
        self.algorithm              = algorithm
        self.uri                    = uri
        self.log_period_crazyflie   = log_period_crazyflie
        self.log_period_vicon       = log_period_vicon

        self.zero_time              = datetime.now()
        self.flight_script          = None

        # States
        self.is_running             = False
        self.state                  = "TAKEOFF"

        # Objects
        self.cf = CrazyFlieWrapper(
            self.uri, 
            algorithm = self.algorithm,
            log_list=self.log_config_crazyflie, 
            sampling_period=self.log_period_crazyflie, 
            time0=self.zero_time, filename=self.filename, 
            data_logger = self
        )

        self.vicon = None
        if self.log_config_vicon is not None:
            self.vicon = ViconWrapper(
                ip="192.168.10.1", 
                period=self.log_period_vicon, 
                subjects=self.log_config_vicon, 
                time0=self.zero_time, 
                filename=self.filename
            )

    def set_flight_script(self, script):
        self.flight_script = script

    def run(self):
        # Start threads
        if self.cf is not None:
            self.cf.start()
        if self.vicon is not None:
            self.vicon.start()

        # Wait for crazyflie to connect
        while(self.cf.is_connected != True):
            time.sleep(0.1)
            pass
        
        # Start logging
        self.is_running = True
        logger.info("Logging starting...")
        self.cf.logging_enabled(1)
        if (self.vicon is not None): 
            self.vicon.logging_enabled(1)

        # Use flight script
        if self.flight_script:
            try:
                # Run flight script
                self.flight_script(self.cf.mc, self.state)

                # Stop logging and save logs
                self.cf.logging_enabled(0)
                self.cf.save_log()
                if (self.vicon is not None):
                    self.vicon.logging_enabled(0)
                    self.vicon.save_log()
            except Exception:
                logger.exception("Illegal command")
                pass

        # Use manual control
        else:
            while self.is_running:
                time.sleep(0.1)
                # Optionally save logs periodically
                # self.cf.save_log_noExit()
                # self.vicon.save_log_noExit()

            # Stop logging and save logs
            self.cf.logging_enabled(0)
            self.cf.save_log()
            if (self.vicon is not None):
                self.vicon.logging_enabled(0)
                self.vicon.save_log()
