import math
import sys
import os
import signal
import time
import logging
import numpy as np
from datetime import datetime
from threading import Thread

import cflib.crtp
import cfclient.utils
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cfclient.utils.input import JoystickReader

from .stable_motion_commander import StableMotionCommander
from .step_detecition import StepDetector

logger = logging.getLogger(__name__)

class CrazyFlieWrapper(Thread):
    def __init__(self, uri, log_list, sampling_period, time0, filename, data_logger):
        Thread.__init__(self)
        self.name = "CrazyFlieWrapper"
        self.uri = uri
        self.data_logger = data_logger
        self.period = sampling_period
        self.data_log = np.array([])
        self.data_logging_en = False
        self.log_list = log_list
        self.is_connected = False
        self.is_running = False
        self.t0 = time0
        self.filename = filename
        self.current_log = dict([])
        self.devs = []

        cflib.crtp.init_drivers(enable_debug_driver=False)

        self.jr = JoystickReader(do_device_discovery=False)

        # Instantiate Crazyflie interface
        self.scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))
        # Instantiate motion commander
        self.mc = StableMotionCommander(self.scf, default_height = 0.5, time0 = time0)
        # Instantiate custom step detector
        self.sd = StepDetector()

        for d in self.jr.available_devices():
            self.devs.append(d.name)

    def setup_controller(self, input_config, input_device=0):
        """Set up the device reader"""
        # Set up the joystick reader
        self.devs = self.jr.available_devices()
        logger.info("Will use [{}] for input with [{}] config".format(self.devs[input_device].name, input_config))
        self.jr.start_input(self.devs[input_device].name)
        self.jr.set_input_map(self.devs[input_device].name, input_config)

    def controller_connected(self):
        """ Return True if a controller is connected"""
        return True if (len(self.jr.available_devices()) > 0) else False
    
    def list_controllers(self):
        """List the available controllers and input mapping"""
        print("\nAvailable controllers:")
        for i, dev in enumerate(self.devs):
            print(" - Controller #{}: {}".format(i, dev))
        print("\nAvailable input mapping:")
        for map in os.listdir(cfclient.config_path + '/input'):
            print(" - " + map.split(".json")[0])

    def run(self):
        self.connect_tocf()

    def connect_tocf(self):
        # Connect some callbacks from the Crazyflie API
        self.scf.cf.connected.add_callback(self._connected)
        self.scf.cf.disconnected.add_callback(self._disconnected)
        self.scf.cf.connection_failed.add_callback(self._connection_failed)
        self.scf.cf.connection_lost.add_callback(self._connection_lost)

        self.jr.set_alt_hold_available(True)


        self.jr.assisted_control_updated.add_callback(
            cb=(
                lambda enabled: (
                    self.scf.cf.param.set_value("flightmode.althold", enabled)
                )
            )
        )

        logger.info('Connecting to %s', self.uri)

        # Try to connect to the Crazyflie
        self.scf.cf.open_link(self.uri)
        self.jr.input_updated.add_callback(self.scf.cf.commander.send_setpoint)
        self.jr.heighthold_input_updated.add_callback(self.scf.cf.commander.send_hover_setpoint)


        self.jr.input_updated.add_callback(self.scf.cf.commander.send_setpoint)
        self.jr.assisted_input_updated.add_callback(self.scf.cf.commander.send_velocity_world_setpoint)
        self.jr.heighthold_input_updated.add_callback(self.scf.cf.commander.send_zdistance_setpoint)
        self.jr.hover_input_updated.add_callback(self.scf.cf.commander.send_hover_setpoint)

    def _connected(self, link_uri):
        logger.info('Connected to %s' % link_uri)
        self.config()
        self.is_connected = True
        if len(self.log_list) > 0:
            self.logging()

    def logging(self):
        N = len(self.log_list)
        logger.info("Logging %s", str(self.log_list))
        logs_nr = math.ceil(N / 6.0)

        logs = []
        for i in range(logs_nr):
            logs.append(LogConfig(name="log" + str(i), period_in_ms=self.period))
        logger.info("Logs added: %d", logs_nr)

        for i in range(N):
            logs[i//6].add_variable(self.log_list[i], 'float')

        try:
            for i in range(logs_nr):
                current_log = logs[i]
                self.scf.cf.log.add_config(current_log)
                # This callback will receive the data
                current_log.data_received_cb.add_callback(self._stab_log_data)
                # Start the logging
                current_log.start()
        except KeyError as e:
            logger.exception('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            logger.exception('Could not add Stabilizer log config, bad configuration.')

    def config(self):
        time.sleep(0.2)
        # Enable kalmann filter
        self.scf.cf.param.set_value('stabilizer.estimator', '2')

        # self.scf.cfparam.set_value('locSrv.extQuatStdDev', 0.05)
        # self.scf.cfparam.set_value('stabilizer.controller', '2')
        
        # Set PID Controller
        # self.scf.cf.param.set_value('commander.enHighLevel', '1')
        logger.info("CF configured!")
        
        logger.info("Reset estimator")
        self.scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.scf.cf.param.set_value('kalman.resetEstimation', '0')

    def _stab_log_data(self, timestamp_cf, data, logconf):
        out = np.fromiter(data.values(), dtype=float).reshape(1, -1)
        names = list(data.keys())
       
        if self.data_logging_en:    
            timestamp = round(1000 * (datetime.now() - self.t0).total_seconds(), 3)
            for i in range(len(names)):
                self.current_log[names[i]] = out[0, i]
                # if self.data_logger.state == "FLY":
                if True:
                    self.log(timestamp, timestamp_cf, names[i], out[0, i])
                    if names[i] == "range.zrange":
                        slope = self.sd.update_z_range(timestamp_cf, out[0,i])
                        self.log(timestamp, timestamp_cf, "range.zslope", slope)
                    if names[i] == "gyro.z":
                        slope = self.sd.update_z_gyro(timestamp_cf, out[0,i])
                        self.log(timestamp, timestamp_cf, "gyro.zslope", slope)
                    if names[i] == "acc.z":
                        slope = self.sd.update_z_acc(timestamp_cf, out[0,i])
                        self.log(timestamp, timestamp_cf, "acc.zslope", slope)
                    
                    z_offset, z_slope = self.sd.get_offset()
                    # self.mc.set_z_offset(z_offset)
                    self.log(timestamp, timestamp_cf, "zslope", z_slope)
                    self.log(timestamp, timestamp_cf, "z_offset", z_offset)
                
    def _connection_failed(self, link_uri, msg):
        logger.info('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False
        sys.exit(-1)

    def _connection_lost(self, link_uri, msg):
        logger.info('Connection to %s lost: %s' % (link_uri, msg))
        sys.exit(-1)

    def _disconnected(self, link_uri):
        logger.info('Disconnected from %s' % link_uri)
        self.is_connected = False

    def send_extpose(self, cf, pos, quat):
        x = pos[0]
        y = pos[1]
        z = pos[2]
        # qw = quat[0]
        # qx = quat[1]
        # qy = quat[2]
        # qz = quat[3]

        # scf.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
        self.scf.cf.extpos.send_extpos(x, y, z)

    def log(self, timestamp, timestamp_cf, id_var, value):
        data_row = np.array([timestamp, timestamp_cf, id_var, value]).reshape(1, -1)
        if self.data_log.shape[0] == 0:
            self.data_log = data_row
        else:
            self.data_log = np.append(self.data_log, data_row, axis=0)

    def save_log(self):
        self.is_running = False
        np.savetxt(self.filename + "cf.csv", self.data_log, fmt='%s', delimiter=',')
        logger.info("Log Saved!")
        self.scf.cf.close_link()

    def logging_enabled(self, val):
        if val == 0:
            self.data_logging_en = False
        else:
            self.data_logging_en = True