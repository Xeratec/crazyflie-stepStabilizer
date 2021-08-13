#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# vicon_wrapper.py
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

import sys
import logging
import time
import math
import numpy as np
from datetime import datetime
from threading import Thread


import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

logger = logging.getLogger(__name__)

try:
    sys.path.append("../extern/pyvicon/")
    from pyvicon.pyvicon import PyVicon, StreamMode, Direction, Result
except Exception:
    logger.exception("No PyVicon available!")
    pass


class ViconWrapper(Thread):
    def __init__(self, ip, period, subjects, time0, filename, exposeVicon = 'None', sendDataToCF=None):
        Thread.__init__(self)
        self.name = "ViconWrapper"
        self.ip = ip
        self.period = period
        self.vicon = []
        self.subjects = subjects
        self.exposeVicon = None if exposeVicon == 'None' else exposeVicon
        self.send_data = sendDataToCF
        self.data_log = np.array([])
        self.data_logging_en = False
        self.text = " "
        self.position = dict([])
        self.quaternions = dict([])
        self.t0 = time0
        self.is_running = False
        self.filename = filename

    def run(self):
        self.connect()
        self.loop()

    def connect(self):
        self.vicon = PyVicon()
        logger.info("SDK version : {}".format(self.vicon.__version__))
        logger.info("{}".format(self.vicon.connect(self.ip)))
        logger.info("Vicon connection status : {}".format(self.vicon.is_connected()))
        self.vicon.set_stream_mode(StreamMode.ServerPush)
        self.vicon.enable_segment_data()
        self.vicon.enable_marker_data()
        self.vicon.enable_unlabeled_marker_data()
        self.vicon.enable_device_data()
        self.vicon.set_axis_mapping(Direction.Forward, Direction.Left, Direction.Up)

    def loop(self):
        self.is_running = True
        while self.is_running:
            while self.vicon.get_frame() != Result.Success:
                time.sleep(self.period)
            subj_count = self.vicon.get_subject_count()
            for i in range(0, subj_count):
                name = self.vicon.get_subject_name(i)
                if name in self.subjects:
                    timestamp = round(1000 * (datetime.now() - self.t0).total_seconds(), 3)
                    pos = self.vicon.get_segment_global_translation(name, name)
                    quat = self.vicon.get_segment_global_quaternion(name, name)
                    if quat is not None:
                        pos = pos / 1000.0
                        self.position[name] = pos
                        self.quaternions[name] = quat
                        if self.data_logging_en:
                            self.log(timestamp, name + "_" + "posx", pos[0])
                            self.log(timestamp, name + "_" + "posy", pos[1])
                            self.log(timestamp, name + "_" + "posz", pos[2])
                            self.log(timestamp, name + "_" + "qw", quat[0])
                            self.log(timestamp, name + "_" + "qx", quat[1])
                            self.log(timestamp, name + "_" + "qy", quat[2])
                            self.log(timestamp, name + "_" + "qz", quat[3])
                        if (self.exposeVicon is not None and self.send_data):
                            if name == self.exposeVicon:
                                self.send_data(pos, 0)

            time.sleep(self.period / 1000.0)
            
    def log(self, timestamp, id_var, value):
        data_row = np.array([timestamp, id_var, value]).reshape(1, -1)
        if self.data_log.shape[0] == 0:
            self.data_log = data_row
        else:
            self.data_log = np.append(self.data_log, data_row, axis=0)

    def save_log(self):
        if self.filename != "":
            np.savetxt(self.filename + "_vicon.csv", self.data_log, fmt='%s', delimiter=',')
            logger.info("Log saved to {}".format(self.filename + "_vicon.csv"))
        self.is_running = False

    def save_log_noExit(self): 
        if self.filename != "":
            np.savetxt(self.filename + "_vicon.csv", self.data_log, fmt='%s', delimiter=',')
            logger.info("Log saved to {}".format(self.filename + "_vicon.csv"))
    
    def logging_enabled(self, val):
        if val == 0:
            self.data_logging_en = False
        else:
            self.data_logging_en = True